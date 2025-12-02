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

    [Header("Debug metrics (for logging)")]
    public float debugQoE;
    public float debugCov;
    public float debugEne;

    [Header("Logging")]
    public bool logTrainingStats = true;

    const int UncoveredSectorCount = 8;
    float[] _uncoveredSectors = new float[UncoveredSectorCount];

    void GetUncoveredDemandSectors(int sectorCount, float maxDist, float[] sectorValues)
    {
        // 배열 초기화
        for (int i = 0; i < sectorCount; i++)
            sectorValues[i] = 0f;

        Vector3 myPos = transform.position;
        float sectorAngle = Mathf.PI * 2f / sectorCount;

        foreach (var rr in RadioReceiver.All)
        {
            if (rr == null) continue;

            var area = rr.GetComponentInParent<DemandArea>();
            if (area == null || area.kind != AreaKind.Building) continue;

            int demand = Mathf.Max(0, area.demand);
            if (demand <= 0) continue;

            // "미커버 UE"만 사용 (아무 드론과도 연결 안 된 UE)
            if (rr.ConnectedSourceCount > 0)
                continue;

            Vector3 delta = rr.transform.position - myPos;
            // 수평면만 고려
            delta.y = 0f;

            float dist = delta.magnitude;
            if (dist < 1e-3f || dist > maxDist)
                continue;

            // [-PI, PI] → [0, 2PI)
            float angle = Mathf.Atan2(delta.z, delta.x);
            if (angle < 0f) angle += Mathf.PI * 2f;

            int idx = (int)(angle / sectorAngle);
            if (idx >= sectorCount) idx = sectorCount - 1;

            // 거리 가중치를 줄 수도 있음 (가까울수록 더 중요하게)
            // float weight = 1f / (1f + dist); // 선택 사항
            float weight = 1f;

            sectorValues[idx] += demand * weight;
        }

        // 정규화: 가장 큰 값 기준으로 0~1 스케일링
        float maxVal = 0f;
        for (int i = 0; i < sectorCount; i++)
            if (sectorValues[i] > maxVal) maxVal = sectorValues[i];

        if (maxVal > 0f)
        {
            for (int i = 0; i < sectorCount; i++)
                sectorValues[i] = Mathf.Clamp01(sectorValues[i] / maxVal);
        }
    }

    [Header("Elimination (no instant respawn)")]
    public bool eliminateOnFail = true;
    public bool freezeRigidbodyOnElim = true;
    public bool disableColliderOnElim = true;
    public bool disableControllerAndSensorOnElim = true;

    private bool _isEliminated = false;
    private Collider[] _allColliders;
    private Renderer[] _allRenderers;

    // DroneAgent.cs (상단 필드)
    [Header("Separation penalty")]
    public float minSeparation = 120f;           // 이 거리보다 가까워지면 패널티 시작
    public float hardSeparation = 60f;           // 매우 가까우면 더 큰 패널티
    public float proximityPenaltyScale = 0.05f;  // 연속 패널티 강도(스텝당)
    public float hardProximityPenalty = -0.5f;   // 하드 근접(붙음) 시 추가 패널티

    float ComputeMinNeighborDistance()
    {
        var agents = FindObjectsOfType<DroneAgent>();
        if (agents == null || agents.Length == 0) return float.PositiveInfinity;

        Vector3 myPos = transform.position;
        float best = float.PositiveInfinity;

        for (int i = 0; i < agents.Length; i++)
        {
            var a = agents[i];
            if (a == null || a == this || a.IsEliminated) continue;

            float d = Vector3.Distance(myPos, a.transform.position);
            if (d < best) best = d;
        }
        return best;
    }

    [Header("Episode control")]
    [Tooltip("그룹 에피소드(멀티에이전트)를 사용할 때 체크. 체크 시 이 스크립트는 EndEpisode를 스스로 호출하지 않습니다.")]
    public bool useGroupEpisodes = true;
    public int episodeMaxSteps = 50000;   // useGroupEpisodes=false일 때만 사용
    private int episodeStep = 0;

    private Rigidbody _rb;
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

        float distanceScale = 10.0f;

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
    public float collisionPenalty = -5f;
    public bool endOnCollision = true;
    public float xMin = -650f, xMax = 60f;
    public float zMin = -1100f, zMax = -50f;
    public Vector2 yLimit = new Vector2(0f, 300f);
    public float boundaryPenalty = -5f;
    public bool endOnBoundary = true;

    public LayerMask obstacleLayers;
    public string[] obstacleTags = new string[] { "Drone", "Building" };

    // ===== 초반 안정화(그레이스 + 생존 소액 보상) =====
    [Header("Survival shaping")]
    public int graceSteps = 1000;          // 초반 보호 구간
    public float aliveTinyReward = 0.002f; // 살아있기만 해도 매 스텝 주는 소액 보상
    private int globalStep = 0;

    // ===== 고정 스폰 설정 =====
    [Header("Fixed Spawn (per agent)")]
    public Transform spawnPoint;
    public bool useSpawnRotation = true;
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

        if (spawnPoint != null)
        {
            transform.position = spawnPoint.position;
            if (useSpawnRotation) transform.rotation = spawnPoint.rotation;
        }
        else if (clampYToLimitIfNoSpawnPoint)
        {
            var p = transform.position;
            p.y = Mathf.Clamp(p.y, yLimit.x, yLimit.y);
            transform.position = p;
        }

        prevPos = transform.position;

        int totalWeight = 0;
        foreach (var da in FindObjectsOfType<DemandArea>(true))
            if (da.kind == AreaKind.Building)
                totalWeight += Mathf.Max(0, da.demand);
        totalWeightDenom = Mathf.Max(1f, totalWeight);

        RecoverFromElimination();
        episodeStep = 0;
        globalStep = 0;   // 그레이스 리셋
    }

    public override void CollectObservations(VectorSensor s)
    {
        // 기존 관측: 위치/고도/속도/로컬 QoE 힌트
        s.AddObservation(transform.position / 500f);
        s.AddObservation(ctrl.Altitude() / 200f);
        s.AddObservation(ctrl.CurrentVelocity() / 20f);

        float qoeHint = (sensor != null) ? sensor.LastQoE : 0f;
        s.AddObservation(Mathf.Clamp(qoeHint / 10f, -1f, 1f));

        // === 추가 관측 항목들 ===

        // 1) 남은 에너지 비율
        float energyRatio = (energyWhInit > 0f) ? (energyWh / energyWhInit) : 0f;
        s.AddObservation(Mathf.Clamp01(energyRatio));

        // 2) 글로벌 커버리지(팀 스칼라)
        float covteam = ComputeUniqueCoverageRewardForScene();
        s.AddObservation(covteam);

        // 3) 내 UE demand 기여도
        s.AddObservation(ComputeMyDemandRatio());

        // 4) 이웃 드론 밀집도
        s.AddObservation(ComputeNeighborDensity());

        // 5) 팀 생존율
        s.AddObservation(ComputeSurvivalRatio());

        GetUncoveredDemandSectors(UncoveredSectorCount, 500f, _uncoveredSectors);
        for (int i = 0; i < UncoveredSectorCount; i++)
            s.AddObservation(_uncoveredSectors[i]);
    }

    public bool debugReward = false;

    public static float ComputeUniqueCoverageRewardForScene()
    {
        float totalDemand = 0f;
        float uniqueCovered = 0f;

        foreach (var rr in RadioReceiver.All)
        {
            if (rr == null) continue;

            var area = rr.GetComponentInParent<DemandArea>();
            if (area == null || area.kind != AreaKind.Building) continue;

            int demand = Mathf.Max(0, area.demand);
            totalDemand += demand;

            // ★ 오직 1대 드론만 연결된 UE만 커버리지로 인정
            if (rr.ConnectedSourceCount == 1)
                uniqueCovered += demand;
        }

        return (totalDemand > 0f) ? (uniqueCovered / totalDemand) : 0f;
    }
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
        float cov = ComputeMyDemandRatio();

        debugQoE = qoe;
        debugCov = cov;
        debugEne = ene;

        // === 개별 보상: qoe * ene * cov
        float indiv = qoe * cov * ene ;
        AddReward(indiv);
        if (logTrainingStats)
        {
            var stats = Academy.Instance.StatsRecorder;

            // 각 구성 보상
            stats.Add("Drone/QoE", qoe);
            stats.Add("Drone/Cov", cov);
            stats.Add("Drone/Ene", ene);

            // 개별 보상(팀 보상에 기여하는 값)
            stats.Add("Drone/IndivReward", indiv);
        }
        // === 생존 소액 보상 ===
        AddReward(aliveTinyReward);

        // === 드론 간 근접(붙어있음) 패널티 ===
        // 아래 파라미터들은 클래스 상단에 직렬화 필드로 두면 인스펙터에서 조절 가능합니다.
        // public float minSeparation = 120f;
        // public float hardSeparation = 60f;
        // public float proximityPenaltyScale = 0.01f;
        // public float hardProximityPenalty = -0.2f;

        float dMin = float.PositiveInfinity;
        var agents = FindObjectsOfType<DroneAgent>();
        if (agents != null && agents.Length > 0)
        {
            Vector3 myPos = transform.position;
            for (int i = 0; i < agents.Length; i++)
            {
                var other = agents[i];
                if (other == null || other == this || other.IsEliminated) continue;
                float d = Vector3.Distance(myPos, other.transform.position);
                if (d < dMin) dMin = d;
            }
        }

        if (dMin < minSeparation)
        {
            // 선형 패널티: minSeparation 밖에서는 0, 안으로 들어올수록 커짐
            float denom = Mathf.Max(1f, minSeparation - hardSeparation);
            float ratio = Mathf.Clamp01((minSeparation - dMin) / denom);
            float softPen = -proximityPenaltyScale * ratio;   // 스텝당 작은 페널티
            AddReward(softPen);

            // 하드 근접(붙음) 시 추가 큰 페널티 1회
            if (dMin < hardSeparation)
            {
                AddReward(hardProximityPenalty);
                if (debugReward)
                    Debug.Log($"[Agent {gameObject.name}] HARD proximity penalty {hardProximityPenalty:F3} (dMin={dMin:F1})");
            }

            if (debugReward)
                Debug.Log($"[Agent {gameObject.name}] proximity penalty {softPen:F4} (dMin={dMin:F1})");
        }

        if (debugReward)
        {
            float stepR = indiv + aliveTinyReward; // 근접 패널티는 AddReward에 누적됨
            Debug.Log($"[Agent {gameObject.name}] QoE={qoe:F3}  Ene={ene:F3}  indiv={indiv:F3}  stepR~={(stepR):F4}");
        }
    }

    // ===== Reward terms =====

    float ComputeQoEReward_Aggregated()
    {
        int srcId = GetSrcId();
        float num = 0f;
        float denom = totalWeightDenom;

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
        return Mathf.Max(0f, num / denom);
    }

    public static float ComputeCoverageRewardForScene()
    {
        float totalDemand = 0f;
        float coveredDemand = 0f;

        foreach (var rr in RadioReceiver.All)
        {
            if (rr == null) continue;

            var area = rr.GetComponentInParent<DemandArea>();
            if (area == null || area.kind != AreaKind.Building) continue;

            int demand = Mathf.Max(0, area.demand);
            totalDemand += demand;

            // "turn on" = 임계 이상으로 연결된 드론이 하나라도 있는 경우
            if (rr.ConnectedSourceCount > 0)
                coveredDemand += demand;
        }

        return (totalDemand > 0f) ? (coveredDemand / totalDemand) : 0f;
    }

    float ComputeEnergyReward()
    {
        if (energyWhInit <= 0f) return 0f;
        return Mathf.Clamp01(energyWh / energyWhInit);
    }

    // ===== Observation helpers =====

    float ComputeMyDemandRatio()
    {
        int srcId = GetSrcId();
        float myUnique = 0f;
        float myCovered = 0f;   // ★ 추가: 내가 커버하는 demand 합

        foreach (var rr in RadioReceiver.All)
        {
            if (rr == null) continue;
            var area = rr.GetComponentInParent<DemandArea>();
            if (area == null || area.kind != AreaKind.Building) continue;

            int d = Mathf.Max(0, area.demand);

            // 내가 커버하고 있는 UE면 분모에 포함
            if (rr.IsConnectedTo(srcId))
            {
                myCovered += d;

                // 그 중에서 나만 연결된 UE만 분자에 포함
                if (rr.ConnectedSourceCount == 1)
                    myUnique += d;
            }
        }

        if (myCovered <= 0f) return 0f;
        return myUnique / myCovered;   // ★ 0~1, 사실상 90/100 같은 비율
    }


    float ComputeNeighborDensity()
    {
        var agents = FindObjectsOfType<DroneAgent>();
        if (agents == null || agents.Length == 0) return 0f;

        Vector3 myPos = transform.position;
        int near = 0;

        for (int i = 0; i < agents.Length; i++)
        {
            var a = agents[i];
            if (a == null || a == this || a.IsEliminated) continue;

            float dist = Vector3.Distance(myPos, a.transform.position);
            if (dist < 150f) // 맵 스케일에 맞게 조정 가능
                near++;
        }

        // 최대 5대 기준 정규화 (필요 시 조정)
        return Mathf.Clamp01(near / 5f);
    }

    float ComputeSurvivalRatio()
    {
        var agents = FindObjectsOfType<DroneAgent>();
        if (agents == null || agents.Length == 0) return 0f;

        int total = 0;
        int alive = 0;

        for (int i = 0; i < agents.Length; i++)
        {
            var a = agents[i];
            if (a == null) continue;

            total++;
            if (!a.IsEliminated) alive++;
        }

        if (total == 0) return 0f;
        return Mathf.Clamp01((float)alive / total);
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

        globalStep++;

        var p = transform.position;

        bool outX = (p.x < xMin) || (p.x > xMax);
        bool outZ = (p.z < zMin) || (p.z > zMax);
        bool outY = (p.y < yLimit.x) || (p.y > yLimit.y);

        if (outX || outZ || outY)
        {
            if (globalStep < graceSteps)
            {
                AddReward(boundaryPenalty * 0.5f);

                var p2 = transform.position;
                p2.x = Mathf.Clamp(p2.x, xMin + 2f, xMax - 2f);
                p2.y = Mathf.Clamp(p2.y, yLimit.x + 2f, yLimit.y - 2f);
                p2.z = Mathf.Clamp(p2.z, zMin + 2f, zMax - 2f);
                transform.position = p2;
            }
            else
            {
                AddReward(boundaryPenalty);
                if (eliminateOnFail) Eliminate("boundary");
                else if (!useGroupEpisodes && endOnBoundary) EndEpisode();
            }
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (_isEliminated) return;

        bool isObstacleTag = obstacleTags != null && System.Array.Exists(obstacleTags, t => collision.collider.CompareTag(t));
        bool isObstacleLayer = ((1 << collision.collider.gameObject.layer) & obstacleLayers.value) != 0;

        if (isObstacleTag || isObstacleLayer)
        {
            // ★ 충돌한 "해당 드론"에게만 패널티
            AddReward(collisionPenalty);

            // 탈락 및 연결 해제
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

    // 호환용 no-op (다른 코드에서 호출해도 에러 방지)
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
