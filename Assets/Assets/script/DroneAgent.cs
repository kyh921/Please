using System.Collections.Generic;
using UnityEngine;

using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

[RequireComponent(typeof(DroneController))]
public class DroneAgent : Agent
{   
    [Header("Energy scaling")]
    [SerializeField] float energyDrainScale = 5f;   // 1보다 크면 더 빨리 닳음 (예: 3f)
    [SerializeField] bool drainBySteps = true;          // true면 스텝 기반
    [SerializeField] float secondsPerStepForEnergy = 0.02f; // 1 스텝을 몇 초로 간주할지(예: fixedDeltaTime)

    // ===== Elimination (multi-agent async) =====
    [Header("Elimination (no instant respawn)")]
    [Tooltip("충돌/경계/배터리 0 시 즉시 리스폰 대신 해당 드론만 제외하고 나머지는 계속 학습")]
    public bool eliminateOnFail = true;
    [Tooltip("제외 시 Rigidbody를 고정(중력/속도 정지)")]
    public bool freezeRigidbodyOnElim = true;
    [Tooltip("제외 시 Collider를 꺼서 추가 충돌 방지")]
    public bool disableColliderOnElim = true;
    [Tooltip("제외 시 시각/센서/컨트롤러 비활성화")]
    public bool disableControllerAndSensorOnElim = true;

    private bool _isEliminated = false;
    private Collider[] _allColliders;
    private Renderer[] _allRenderers;

    // ===== Episode step limit (per-agent) =====
    [Header("Per-Agent Episode")]
    [Tooltip("이 에이전트의 에피소드 최대 스텝(도달 시 이 에이전트만 EndEpisode)")]
    public int episodeMaxSteps = 5000;
    private int episodeStep = 0;

    private Rigidbody _rb;
    int stepCount;
    // ===== 이동 제어 =====
    public float yawCmdDegPerSec = 0f;
    private DroneController ctrl;

    [SerializeField] private RadioReceiver sensor;

    private Vector3 prevPos;
    public float LastStepDistance { get; private set; }

    // ===== QoE 보상 =====
    [Header("QoE reward")]
    [Tooltip("분모 = Σ(전체 건물 가중치). 에피소드 시작 시 계산")]
    public float totalWeightDenom = 1f;

    [Tooltip("보상 안정화를 위한 작은 값")]
    public float eps = 1e-6f;

    // 집계 입력(프레임당)
    private float _qoeNumeratorThisStep = 0f; // 내 드론의 Σ(A_l × UE가중치)
    private int   _overconnectThisStep   = 0; // 유효 링크 수 - 1

    public bool IsEliminated => _isEliminated;  // 외부에서 조회

    // DroneAgent.cs (도움 메서드 추가)
    int GetSrcId() => gameObject.GetInstanceID();

    // DroneAgent.cs (Eliminate 끝에 추가)
    void Eliminate(string reason)
    {
        if (_isEliminated) return;
        _isEliminated = true;

        // 움직임/제어 정지
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

        // ★ UE에서 이 드론 연결 강제 제거 (통신 완전 차단)
        int src = gameObject.GetInstanceID();
        foreach (var rr in RadioReceiver.All)
        {
            if (rr != null) rr.ForceDisconnect(src);
        }
    }

    public void BeginStepAggregation()
    {
        _qoeNumeratorThisStep = 0f;
        _overconnectThisStep  = 0;
    }

    public void ReportQoEAndOverlap(float perDroneQoENumerator, int overconnect)
    {
        _qoeNumeratorThisStep += Mathf.Max(0f, perDroneQoENumerator);
        _overconnectThisStep   = Mathf.Max(_overconnectThisStep, Mathf.Max(0, overconnect));
    }

    // ===== 에너지 모델 (Table III) =====

    // --- Battery state (Q(t), Wh) ---
    private float energyWhInit;   // 초기 충전량(Wh), 에피소드 시작 시 계산
    private float energyWh;       // 현재 잔량 Q(t) (Wh)

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
    float WeightN   => massKg * 9.81f;
    float Vtip      => Omega * R;
    float v0_hover  => Mathf.Sqrt(WeightN / (2f * rho * DiscAreaA));

    float PowerHoverW()
    {
        float Po = (delta_profile / 8f) * rho * s * DiscAreaA
                 * Mathf.Pow(Omega, 3f) * Mathf.Pow(R, 3f);
        float Pi = (1f + k_induced) * Mathf.Pow(WeightN, 1.5f) / Mathf.Sqrt(2f * rho * DiscAreaA);
        return Po + Pi;
    }

    float PowerForwardW(float v)
    {
        v = Mathf.Max(0f, v);
        float Po = (delta_profile / 8f) * rho * s * DiscAreaA
                 * Mathf.Pow(Omega, 3f) * Mathf.Pow(R, 3f);
        float Pi_hover = (1f + k_induced) * Mathf.Pow(WeightN, 1.5f) / Mathf.Sqrt(2f * rho * DiscAreaA);

        float V2 = v * v;
        float v0 = v0_hover;
        float inside  = Mathf.Sqrt(1f + (V2 * V2) / (4f * Mathf.Pow(v0, 4f))) - (V2 / (2f * v0 * v0));
        float induced = Pi_hover * Mathf.Sqrt(Mathf.Max(0f, inside));
        float profile = Po * (1f + 3f * V2 / (Vtip * Vtip));
        float parasite= 0.5f * d0_parasite * rho * s * DiscAreaA * v * V2;

        return profile + induced + parasite;
    }

    // 논문 식(12): Q(t+1) = max{0, Q(t) - μ(t)}
    // 여기서 μ(t)는 전력[W] * Δt[s] / 3600 = Wh 로 환산
    void UpdateEnergyQueueByPaperModel()
    {
        if (_isEliminated) return; // 제외된 드론은 배터리 더 이상 소모하지 않음

        // Δt: FixedUpdate와 OnActionReceived 혼용 대비
        float dt = drainBySteps
            ? Mathf.Max(secondsPerStepForEnergy, 1e-4f)     // 스텝 고정
            : (Time.inFixedTimeStep ? Time.fixedDeltaTime : Time.deltaTime);

        // 3D 속도: Rigidbody가 있으면 그것을 신뢰, 없으면 위치 변화로 추정
        float V;
        if (_rb != null)
            V = _rb.velocity.magnitude;
        else
            V = (transform.position - prevPos).magnitude / dt;

        // hover/forward 분류만 정확히 (hoverSpeedEps=0.2 유지)
        float P_hover = PowerHoverW();
        float P = (V <= hoverSpeedEps)
            ? P_hover
            : Mathf.Max(P_hover * 3.0f, PowerForwardW(V));

        float usedWh = energyDrainScale * Mathf.Max(0f, P) * dt / 3600f;  // μ(t) [Wh]
        energyWh = Mathf.Max(0f, energyWh - usedWh);

        // ★ 배터리 0이면 그 드론만 탈락
        if (energyWh <= 0f && eliminateOnFail)
        {
            Eliminate("battery");
        }
    }

    // ===== 충돌 & 경계 =====
    [Header("Collision & Boundary")]
    public float collisionPenalty = -0.5f;
    public bool endOnCollision = true;

    [Tooltip("작업영역 X 최소/최대, Z 최소/최대 (비대칭 지원)")]
    public float xMin = -650f, xMax = 60f;
    public float zMin = -1100f, zMax = -50f;

    [Tooltip("허용 고도 범위 (min,max)")]
    public Vector2 yLimit  = new Vector2(0f, 200f);

    public float boundaryPenalty = -0.2f;
    public bool endOnBoundary = true;

    [Tooltip("커버리지 중복(오버커넥트) 1단위당 추가 패널티")]
    public float overlapPenaltyPerLink = 0.05f;

    public LayerMask obstacleLayers;
    public string[] obstacleTags = new string[] { "Drone", "Building" };

    // ====== 스폰 범위 ======
    [Header("Spawn Range (per episode)")]
    [Tooltip("OnEpisodeBegin에서 사용할 스폰 범위 X")]
    public float spawnXMin = 180f, spawnXMax = 270f;

    [Tooltip("OnEpisodeBegin에서 사용할 스폰 범위 Z")]
    public float spawnZMin = -540f, spawnZMax = -440f;

    // ===== Unity lifecycle =====
    void Awake()
    {
        ctrl = GetComponent<DroneController>();
        if (sensor == null) sensor = GetComponentInChildren<RadioReceiver>();
        _rb = GetComponent<Rigidbody>();
        prevPos = transform.position;

        _allColliders = GetComponentsInChildren<Collider>(includeInactive: true);
        _allRenderers = GetComponentsInChildren<Renderer>(includeInactive: true);
    }

    public override void OnEpisodeBegin()
    {
        // 에피소드 시작 시에만 완충
        energyWhInit = (batteryVolt * battery_mAh) / 1000f;
        energyWh = energyWhInit;

        // == 스폰 범위 사용 ==
        float rx = Random.Range(spawnXMin, spawnXMax);
        float rz = Random.Range(spawnZMin, spawnZMax);
        float ry = Random.Range(yLimit.x, yLimit.y);
        transform.position = new Vector3(rx, ry, rz);
        transform.rotation = Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);

        prevPos = transform.position;
        LastStepDistance = 0f;

        int totalWeight = 0;
        foreach (var da in FindObjectsOfType<DemandArea>(true))
            if (da.kind == AreaKind.Building)
                totalWeight += Mathf.Max(0, da.demand);
        totalWeightDenom = Mathf.Max(1f, totalWeight);

        _qoeNumeratorThisStep = 0f;
        _overconnectThisStep  = 0;

        // 제외 상태 초기화 및 재활성화
        RecoverFromElimination();

        // 에피소드 스텝 카운터 리셋 (이 에이전트 전용)
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
        if (_isEliminated)
        {
            // 제외된 동안에는 행동/보상/배터리 소모 없음
            return;
        }

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
        float cov = ComputeCoverageReward_Aggregated();
        float ene = ComputeEnergyReward();

        int overlap = Mathf.Max(0, _overconnectThisStep);
        float overlapPenalty = overlap * overlapPenaltyPerLink;

        float stepR = qoe * cov * ene - overlapPenalty;
        AddReward(stepR);

        const int LOG_EVERY_N_STEPS = 0;
        stepCount++;

        if (debugReward)
        {
            bool nonZeroReward = Mathf.Abs(stepR) > 1e-6f;
            bool periodic = (LOG_EVERY_N_STEPS > 0) && (stepCount % LOG_EVERY_N_STEPS == 0);

            if (nonZeroReward || periodic)
            {
                Debug.Log(
                    $"[Agent {gameObject.name}] QoE={qoe:F3}  Cov={cov:F3}  Ene={ene:F3}  Overlap={overlap}  pen_o={overlapPenalty:F3} " +
                    $"=> stepR={stepR:F4} num={_qoeNumeratorThisStep:F3} denom={totalWeightDenom:F1} batt={(energyWhInit > 0 ? (energyWh / energyWhInit * 100f) : 0f):F2} %"
                );
            }
        }

        _qoeNumeratorThisStep = 0f;
        _overconnectThisStep  = 0;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var a = actionsOut.ContinuousActions;
        a[0] = Input.GetAxis("Horizontal");
        a[2] = Input.GetAxis("Vertical");
        a[1] = (Input.GetKey(KeyCode.E) ? 1f : 0f) + (Input.GetKey(KeyCode.Q) ? -1f : 0f);
    }

    // ===== Reward terms =====
    float ComputeQoEReward_Aggregated()
    {
        float qoe = _qoeNumeratorThisStep / (1.5f*totalWeightDenom);
        return Mathf.Clamp01(qoe);
    }

    float ComputeCoverageReward_Aggregated()
    {
        return 1f / (1f + Mathf.Max(0, _overconnectThisStep));
    }

    float ComputeEnergyReward()
    {
        if (energyWhInit <= 0f) return 0f;
        float rem = Mathf.Clamp01(energyWh / energyWhInit); // em
        return rem;
    }

    // ===== 충돌 & 경계 처리 =====
    void OnCollisionEnter(Collision other)
    {
        if (!IsObstacle(other.collider)) return;

        AddReward(collisionPenalty);

        if (eliminateOnFail)
        {
            Eliminate("collision");
        }
        else if (endOnCollision)
        {
            EndEpisode();
        }
    }

    void OnTriggerEnter(Collider other)
    {
        if (!IsObstacle(other)) return;

        AddReward(collisionPenalty);

        if (eliminateOnFail)
        {
            Eliminate("trigger");
        }
        else if (endOnCollision)
        {
            EndEpisode();
        }
    }

    bool IsObstacle(Collider col)
    {
        if (obstacleTags != null && obstacleTags.Length > 0)
        {
            foreach (var t in obstacleTags)
                if (!string.IsNullOrEmpty(t) && col.CompareTag(t))
                    return true;
        }
        if (obstacleLayers.value != 0)
            return ((1 << col.gameObject.layer) & obstacleLayers.value) != 0;

        return true;
    }
 
    void FixedUpdate()
    {
        // 에피소드 스텝 카운트는 제외 상태와 관계없이 증가 (비동기 종료 트리거용)
        episodeStep++;
        if (episodeStep >= episodeMaxSteps)
        {
            EndEpisode();   // 이 에이전트만 리셋 (다른 드론은 계속)
            return;
        }

        if (_isEliminated) return;

        var p = transform.position;

        bool outX = (p.x < xMin) || (p.x > xMax);
        bool outZ = (p.z < zMin) || (p.z > zMax);
        bool outY = (p.y < yLimit.x) || (p.y > yLimit.y);

        if (outX || outZ || outY)
        {
            AddReward(boundaryPenalty);

            if (eliminateOnFail)
            {
                Eliminate("boundary");
            }
            else if (endOnBoundary)
            {
                EndEpisode();
            }
        }
    }

    // ===== Elimination helpers =====
    

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

#if UNITY_EDITOR
    void OnDrawGizmosSelected()
    {
        float y = (Application.isPlaying ? transform.position.y : 50f);

        // 작업영역(노란색)
        Vector3 center = new Vector3((xMin + xMax) * 0.5f, y, (zMin + zMax) * 0.5f);
        Vector3 size   = new Vector3(Mathf.Abs(xMax - xMin), 0.1f, Mathf.Abs(zMax - zMin));
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireCube(center, size);

        // 스폰 영역(청록색)
        Vector3 spawnCenter = new Vector3((spawnXMin + spawnXMax) * 0.5f, y, (spawnZMin + spawnZMax) * 0.5f);
        Vector3 spawnSize   = new Vector3(Mathf.Abs(spawnXMax - spawnXMin), 0.1f, Mathf.Abs(spawnZMax - spawnZMin));
        Gizmos.color = Color.cyan;
        Gizmos.DrawWireCube(spawnCenter, spawnSize);

        // 제외 상태 표시(빨간 X)
        if (_isEliminated)
        {
            Gizmos.color = Color.red;
            Vector3 p = Application.isPlaying ? transform.position : Vector3.zero;
            float s = 5f;
            Gizmos.DrawLine(p + new Vector3(-s, 0, -s), p + new Vector3(s, 0, s));
            Gizmos.DrawLine(p + new Vector3(-s, 0, s),  p + new Vector3(s, 0, -s));
        }
    }
#endif
}
