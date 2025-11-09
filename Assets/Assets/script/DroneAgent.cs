using System.Collections.Generic;
using UnityEngine;

using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

[RequireComponent(typeof(DroneController))]
public class DroneAgent : Agent
{
    [Header("Energy scaling")]
    [SerializeField] bool drainBySteps = true;
    [SerializeField] float secondsPerStepForEnergy = 0.02f;

    [Header("Elimination (no instant respawn)")]
    public bool eliminateOnFail = true;
    public bool freezeRigidbodyOnElim = true;
    public bool disableColliderOnElim = true;
    public bool disableControllerAndSensorOnElim = true;

    private bool _isEliminated = false;
    private Collider[] _allColliders;
    private Renderer[] _allRenderers;

    [Header("Episode control")]
    [Tooltip("그룹 에피소드(멀티에이전트)를 사용할 때 체크. 체크 시 이 스크립트는 EndEpisode를 스스로 호출하지 않습니다.")]
    public bool useGroupEpisodes = true;
    public int episodeMaxSteps = 50000;   // useGroupEpisodes=false일 때만 사용
    private int episodeStep = 0;

    private Rigidbody _rb;
    int stepCount;
    public float yawCmdDegPerSec = 0f;
    private DroneController ctrl;

    [SerializeField] private RadioReceiver sensor;

    private Vector3 prevPos;
    public float LastStepDistance { get; private set; }

    [Header("QoE reward")]
    public float totalWeightDenom = 1f;
    public float eps = 1e-6f;

    public bool IsEliminated => _isEliminated;
    int GetSrcId() => gameObject.GetInstanceID();

    // ===== 에너지 모델 =====
    private float energyWhInit;
    private float energyWh;

    [Header("Energy model (Table III)")]
    public float rho = 1.225f;
    public float s = 0.0157f;
    public float R = 0.40f;
    public float Omega = 300f;
    public float k_induced = 0.10f;
    public float delta_profile = 0.012f;
    public float d0_parasite = 0.0161f;
    public float massKg = 1.375f;
    public float hoverSpeedEps = 0.2f;

    public float batteryVolt = 15.2f;
    public float battery_mAh = 5870f;

    float DiscAreaA => Mathf.PI * R * R;
    float WeightN => massKg * 9.81f;
    float Vtip => Omega * R;
    float v0_hover => Mathf.Sqrt(WeightN / (2f * rho * DiscAreaA));

    float PowerHoverW()
    {
        float Po = (delta_profile / 8f) * rho * s * DiscAreaA * Mathf.Pow(Omega, 3f) * Mathf.Pow(R, 3f);
        float Pi = (1f + k_induced) * Mathf.Pow(WeightN, 1.5f) / Mathf.Sqrt(2f * rho * DiscAreaA);
        return Po + Pi;
    }

    float PowerForwardW(float v)
    {
        v = Mathf.Max(0f, v);
        float Po = (delta_profile / 8f) * rho * s * DiscAreaA * Mathf.Pow(Omega, 3f) * Mathf.Pow(R, 3f);
        float Pi_hover = (1f + k_induced) * Mathf.Pow(WeightN, 1.5f) / Mathf.Sqrt(2f * rho * DiscAreaA);

        float V2 = v * v;
        float v0 = v0_hover;
        float inside = Mathf.Sqrt(1f + (V2 * V2) / (4f * Mathf.Pow(v0, 4f))) - (V2 / (2f * v0 * v0));
        float induced = Pi_hover * Mathf.Sqrt(Mathf.Max(0f, inside));
        float profile = Po * (1f + 3f * V2 / (Vtip * Vtip));
        float parasite = 0.5f * d0_parasite * rho * s * DiscAreaA * v * V2;

        return profile + induced + parasite;
    }

    void UpdateEnergyQueueByPaperModel()
    {
        if (_isEliminated) return;

        float dt = drainBySteps
            ? Mathf.Max(secondsPerStepForEnergy, 1e-4f)
            : (Time.inFixedTimeStep ? Time.fixedDeltaTime : Time.deltaTime);

        float distanceScale = 10f;

        float V;
        if (_rb != null)
            V = _rb.velocity.magnitude * distanceScale;
        else
            V = ((transform.position - prevPos).magnitude * distanceScale) / dt;

        float P_hover = PowerHoverW();
        float P = (V <= hoverSpeedEps) ? P_hover : Mathf.Max(P_hover * 3.0f, PowerForwardW(V));

        float usedWh = Mathf.Max(0f, P) * dt / 3600f;
        energyWh = Mathf.Max(0f, energyWh - usedWh);

        if (energyWh <= 0f && eliminateOnFail)
            Eliminate("battery");
    }

    // ===== 경계/충돌 =====
    [Header("Collision & Boundary")]
    public float collisionPenalty = -0.5f;
    public bool endOnCollision = true;
    public float xMin = -650f, xMax = 60f;
    public float zMin = -1100f, zMax = -50f;
    public Vector2 yLimit = new Vector2(0f, 300f);
    public float boundaryPenalty = -0.2f;
    public bool endOnBoundary = true;

    public LayerMask obstacleLayers;
    public string[] obstacleTags = new string[] { "Drone", "Building" };

    // ===== 고정 스폰 설정 =====
    [Header("Fixed Spawn (per agent)")]
    [Tooltip("이 에이전트가 에피소드 시작 시 위치/회전을 가져올 Transform")]
    public Transform spawnPoint;
    [Tooltip("spawnPoint의 회전을 사용할지 여부 (끄면 현재 회전 유지)")]
    public bool useSpawnRotation = true;
    [Tooltip("spawnPoint가 없을 때 높이를 yLimit 범위로 클램프할지 여부")]
    public bool clampYToLimitIfNoSpawnPoint = true;

    void Awake()
    {
        ctrl = GetComponent<DroneController>();
        if (sensor == null) sensor = GetComponentInChildren<RadioReceiver>();
        _rb = GetComponent<Rigidbody>();
        prevPos = transform.position;

        _allColliders = GetComponentsInChildren<Collider>(true);
        _allRenderers = GetComponentsInChildren<Renderer>(true);
    }

    public override void OnEpisodeBegin()
    {
        energyWhInit = (batteryVolt * battery_mAh) / 1000f;
        energyWh = energyWhInit;

        // --- 고정 스폰 로직 ---
        if (spawnPoint != null)
        {
            transform.position = spawnPoint.position;
            if (useSpawnRotation) transform.rotation = spawnPoint.rotation;
        }
        else
        {
            if (clampYToLimitIfNoSpawnPoint)
            {
                var p = transform.position;
                p.y = Mathf.Clamp(p.y, yLimit.x, yLimit.y);
                transform.position = p;
            }
        }

        prevPos = transform.position;

        int totalWeight = 0;
        foreach (var da in FindObjectsOfType<DemandArea>(true))
            if (da.kind == AreaKind.Building)
                totalWeight += Mathf.Max(0, da.demand);
        totalWeightDenom = Mathf.Max(1f, totalWeight);

        RecoverFromElimination();
        episodeStep = 0;
    }

    public override void CollectObservations(VectorSensor s)
    {
        s.AddObservation(transform.position / 500f);
        s.AddObservation(ctrl.Altitude() / 200f);
        s.AddObservation(ctrl.CurrentVelocity() / 20f);

        float qoeHint = (sensor != null) ? sensor.LastQoE : 0f;
        s.AddObservation(Mathf.Clamp(qoeHint / 10f, -1f, 1f));
    }

    public bool debugReward = false;

    public override void OnActionReceived(ActionBuffers actions)
    {
        if (_isEliminated) return;

        var a = actions.ContinuousActions;
        Vector3 cmdLocal = new Vector3(a[0], a[1], a[2]);
        ctrl.SetCommand(new Vector3(cmdLocal.x * ctrl.maxHorizontalSpeed,
                                    cmdLocal.y * ctrl.maxClimbRate,
                                    cmdLocal.z * ctrl.maxHorizontalSpeed),
                        yawCmdDegPerSec);

        LastStepDistance = Vector3.Distance(prevPos, transform.position);
        prevPos = transform.position;
        UpdateEnergyQueueByPaperModel();

        float qoe = ComputeQoEReward_Aggregated();
        float ene = ComputeEnergyReward();

        // === 개별 보상만 더한다 (공통 보상 cov는 매니저가 그룹 보상으로 처리) ===
        float stepR = qoe * ene;
        AddReward(stepR);

        if (debugReward)
        {
            Debug.Log($"[Agent {gameObject.name}] QoE={qoe:F3}  Ene={ene:F3}  stepR={stepR:F4}");
        }
    }

    // ===== Reward terms =====

    float ComputeQoEReward_Aggregated()
    {
        int srcId = GetSrcId();
        float num = 0f;
        float denom = 0.7f * totalWeightDenom;

        foreach (var rr in RadioReceiver.All)
        {
            if (rr == null) continue;

            var area = rr.GetComponentInParent<DemandArea>();
            if (area == null || area.kind != AreaKind.Building) continue;

            int demand = Mathf.Max(0, area.demand);

            if (rr.IsConnectedTo(srcId))
            {
                float qoe = Mathf.Max(0f, rr.GetQoEFor(srcId));
                num += qoe * demand;
            }
        }

        if (denom <= 0f) return 0f;
        return Mathf.Clamp01(num / denom);
    }

    public static float ComputeCoverageRewardForScene()
    {
        float totalDemand = 0f;
        float coveredDemand = 0f;
        float sumOver = 0f;
        int overCount = 0;

        foreach (var rr in RadioReceiver.All)
        {
            if (rr == null) continue;

            var area = rr.GetComponentInParent<DemandArea>();
            if (area == null || area.kind != AreaKind.Building) continue;

            int demand = Mathf.Max(0, area.demand);
            totalDemand += demand;

            int k = rr.ConnectedSourceCount;
            if (k > 0) coveredDemand += demand;

            int over = Mathf.Max(0, k - 1);
            if (over > 0)
            {
                sumOver += over;
                overCount++;
            }
        }

        float tau = (totalDemand > 0f) ? (coveredDemand / totalDemand) : 0f;
        float w = (overCount > 0) ? (sumOver / overCount) : 0f;

        float cov = (2.0f * tau) / (1f + (0.5f * w));
        return Mathf.Clamp01(cov);
    }

    float ComputeEnergyReward()
    {
        if (energyWhInit <= 0f) return 0f;
        return Mathf.Clamp01(energyWh / energyWhInit);
    }

    // ===== 경계 처리 =====
    void FixedUpdate()
    {
        if (!useGroupEpisodes)
        {
            episodeStep++;
            if (episodeStep >= episodeMaxSteps)
            {
                EndEpisode();
                return;
            }
        }

        if (_isEliminated) return;

        var p = transform.position;

        bool outX = (p.x < xMin) || (p.x > xMax);
        bool outZ = (p.z < zMin) || (p.z > zMax);
        bool outY = (p.y < yLimit.x) || (p.y > yLimit.y);

        if (outX || outZ || outY)
        {
            AddReward(boundaryPenalty);
            if (eliminateOnFail) Eliminate("boundary");
            else if (!useGroupEpisodes && endOnBoundary) EndEpisode();
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (_isEliminated) return;

        bool isObstacleTag = obstacleTags != null && System.Array.Exists(obstacleTags, t => collision.collider.CompareTag(t));
        bool isObstacleLayer = ((1 << collision.collider.gameObject.layer) & obstacleLayers.value) != 0;

        if (isObstacleTag || isObstacleLayer)
        {
            AddReward(collisionPenalty);
            Eliminate("collision");
        }
    }

    void Eliminate(string reason)
    {
        if (_isEliminated) return;
        _isEliminated = true;

        if (disableControllerAndSensorOnElim)
        {
            if (ctrl != null) ctrl.enabled = false;
            if (sensor != null) sensor.enabled = false;
        }

        if (freezeRigidbodyOnElim && _rb != null)
        {
            _rb.velocity = Vector3.zero;
            _rb.angularVelocity = Vector3.zero;
            _rb.isKinematic = true;
            _rb.useGravity = false;
        }

        if (disableColliderOnElim && _allColliders != null)
        {
            foreach (var c in _allColliders) if (c) c.enabled = false;
        }

        int src = gameObject.GetInstanceID();
        foreach (var rr in RadioReceiver.All)
            if (rr != null) rr.ForceDisconnect(src);
    }

    void RecoverFromElimination()
    {
        _isEliminated = false;
        if (disableControllerAndSensorOnElim)
        {
            if (ctrl != null) ctrl.enabled = true;
            if (sensor != null) sensor.enabled = true;
        }

        if (freezeRigidbodyOnElim && _rb != null)
        {
            _rb.isKinematic = false;
            _rb.useGravity = true;
            _rb.velocity = Vector3.zero;
            _rb.angularVelocity = Vector3.zero;
        }

        if (disableColliderOnElim && _allColliders != null)
        {
            foreach (var c in _allColliders) if (c) c.enabled = true;
        }
    }

    // 호환용 no-op
    public void BeginStepAggregation() { }
    public void ReportQoEAndOverlap(float perDroneQoENumerator, int overconnect) { }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var a = actionsOut.ContinuousActions;

        a[0] = Input.GetAxis("Horizontal");
        a[2] = Input.GetAxis("Vertical");

        float up = Input.GetKey(KeyCode.E) ? 1f : 0f;
        float down = Input.GetKey(KeyCode.Q) ? -1f : 0f;
        a[1] = up + down;
    }
}
