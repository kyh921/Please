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

    // ===== 역할 분리 기반 Observation =====
    [Header("Agent Role Index")]
    public int agentIndex = 0;   // DroneTeamManager에서 할당됨

    float Altitude()
    {
        return transform.position.y;
    }

    // ===== Helper: QoE LastStep (관찰용이 아니므로 삭제 가능) =====
    float ComputeQoEReward_LastStep()
    {
        int srcId = GetSrcId();
        float qoeVal = 0f;

        foreach (var rr in RadioReceiver.All)
        {
            if (rr == null) continue;

            var area = rr.GetComponentInParent<DemandArea>();
            if (area == null || area.kind != AreaKind.Building) continue;

            if (rr.IsConnectedTo(srcId))
            {
                qoeVal = Mathf.Max(qoeVal, rr.GetQoEFor(srcId));
            }
        }
        return Mathf.Clamp01(qoeVal);
    }

    // ===== Helper: Energy normalized =====
    float EnergyRewardNormalized()
    {
        if (energyWhInit <= 0f) return 0f;
        return Mathf.Clamp01(energyWh / energyWhInit);
    }

    // ===== Helper: Global coverage (매니저로 이동) =====
    // ★ 삭제됨 ★
    // float ComputeGlobalCoverageFraction() { ... }

    // ===== Helper: My demand coverage =====
    float ComputeAgentDemandCoverage()
    {
        return ComputeMyDemandRatio();
    }

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

        float distanceScale = 8.0f;

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
    // Agent 쪽에서는 패널티를 주지 않고 Manager에서만 처리한다.
    public float collisionPenalty = 0f;
    public bool endOnCollision = true;
    public float xMin = -650f, xMax = 60f;
    public float zMin = -1100f, zMax = -50f;
    public Vector2 yLimit = new Vector2(0f, 300f);
    public float boundaryPenalty = 0f;
    public bool endOnBoundary = true;
    public LayerMask obstacleLayers;
    public string[] obstacleTags = new string[] { "Drone", "Building" };

    // ===== 초반 안정화(그레이스 + 생존 소액 보상) =====
    [Header("Survival shaping")]
    public int graceSteps = 1000;          // 초반 보호 구간
    public float aliveTinyReward = 0.0005f; // 살아있기만 해도 매 스텝 주는 소액 보상
    private int globalStep = 0;

    // ===== 고정 스폰 설정 =====
    [Header("Fixed Spawn (per agent)")]
    public Transform spawnPoint;
    public bool useSpawnRotation = true;
    public bool clampYToLimitIfNoSpawnPoint = true;

    [Header("QoE scaling")]
    [Tooltip("QoE 전체 스케일 (자살 전략이 이득 안 되도록 줄이는 용도)")]
    [Range(0f, 1f)]
    public float qoeScale = 0.2f;   // ★ 변경: 0.2f -> 1.0f

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

        // === 에피소드 요약 변수 리셋 ===
        epiIndivReward = 0f;
        epiTotalReward = 0f;
        epiSteps = 0;
    }

    public override void CollectObservations(VectorSensor s)
    {
        // 1) 위치 및 고도 관련 정보
        Vector3 p = transform.position;
        float alt = Altitude();
        s.AddObservation(p / 500f);       // 기존 그대로
        s.AddObservation(alt / 200f);     // 기존 그대로

        // ===== 새로 추가: 경계까지 정규화 거리 =====
        // xMin ~ xMax → 0~1
        float normX = Mathf.InverseLerp(xMin, xMax, p.x);
        float normZ = Mathf.InverseLerp(zMin, zMax, p.z);
        float normY = Mathf.InverseLerp(yLimit.x, yLimit.y, p.y);

        s.AddObservation(normX);
        s.AddObservation(normZ);
        s.AddObservation(normY);

        // 2) 속도 관련
        if (_rb != null)
        {
            s.AddObservation(_rb.velocity / 20f); // 기존 그대로
        }
        else
        {
            s.AddObservation(Vector3.zero);
        }

        // 3) QoE 정보 ★ 변경: 불일치 해결
        // s.AddObservation(ComputeQoEReward_LastStep()); // (X)
        s.AddObservation(ComputeQoEReward_Aggregated()); // (O) 보상 함수와 동일한 값

        // 4) 에너지 정보
        s.AddObservation(EnergyRewardNormalized());
        s.AddObservation(energyWh / (energyWhInit + 1e-6f));

        // 5) 커버리지 및 demand 관련
        // ★ 변경: 매니저의 캐시된 값 사용
        if (DroneTeamManager.Instance != null)
        {
            s.AddObservation(DroneTeamManager.Instance.CurrentGlobalCoverage);
        }
        else
        {
            s.AddObservation(0f); // 매니저가 없는 경우 (안전 장치)
        }
        s.AddObservation(ComputeAgentDemandCoverage());

        // 6) 주변 에이전트 밀집도
        float nd = ComputeNeighborDensity();
        s.AddObservation(nd);

        // ===== 새로 추가: agentIndex 역할 힌트 =====
        // 0~4 index → 0~1 정규화
        s.AddObservation(agentIndex / 4f);

        // 7) 팀 생존율
        // ★ 변경: 매니저의 캐시된 값 사용
        if (DroneTeamManager.Instance != null)
        {
            s.AddObservation(DroneTeamManager.Instance.CurrentSurvivalRatio);
        }
        else
        {
            s.AddObservation(0f); // 매니저가 없는 경우 (안전 장치)
        }
    }


    public bool debugReward = false;

    // 에피소드 요약용
    [HideInInspector] public float epiIndivReward;  // qoe*ene + alive + crowd 누적
    [HideInInspector] public float epiTotalReward;  // 최종 cumulative reward
    [HideInInspector] public int   epiSteps;        // 이 에이전트가 밟은 step 수

    [Header("Crowding penalty")]
    public bool useCrowdingPenalty = true;

    [Range(0f, 0.2f)]
    public float crowdPenaltyWeight = 0.05f;

    public override void OnActionReceived(ActionBuffers actions)
    {
        if (_isEliminated) return;

        // 0) 액션 → 드론 제어
        var a = actions.ContinuousActions;
        Vector3 cmdLocal = new Vector3(a[0], a[1], a[2]);
        ctrl.SetCommand(
            new Vector3(
                cmdLocal.x * ctrl.maxHorizontalSpeed,
                cmdLocal.y * ctrl.maxClimbRate,
                cmdLocal.z * ctrl.maxHorizontalSpeed),
            yawCmdDegPerSec
        );

        // 이동 거리 및 에너지 업데이트
        LastStepDistance = Vector3.Distance(prevPos, transform.position);
        prevPos = transform.position;
        UpdateEnergyQueueByPaperModel();

        // 1) QoE + 에너지 보상
        float rawQoe = ComputeQoEReward_Aggregated();   // 원래 쓰던 qoe
        float qoe    = qoeScale * rawQoe;               // ★ qoeScale이 1.0이 됨

        float ene    = ComputeEnergyReward();
        float indiv  = qoe * ene;                       // qoe * ene 그대로 유지

        AddReward(indiv);

        // 2) 살아있기 보상
        float alive = aliveTinyReward;
        AddReward(alive);

        // 3) 군집 패널티 (이전처럼 그대로, 필요 없으면 나중에 끄면 됨)
        float crowd = 0f;
        if (useCrowdingPenalty) // (useCrowdingPenalty 플래그 사용)
        {
            float nd = ComputeNeighborDensity();
            crowd = -crowdPenaltyWeight * nd; // (가중치 사용)
            AddReward(crowd);
        }

        // 4) 디버그 로그
        if (debugReward)
        {
            float stepNoGroup = indiv + alive + crowd;

            // 현재 시점 전체 맵 기준 cov (그룹 보상으로 쓰이는 값)
            float cov = (DroneTeamManager.Instance != null) 
                ? DroneTeamManager.Instance.CurrentGlobalCoverage 
                : 0f;

            Debug.Log(
                $"[Agent {gameObject.name}] " +
                $"qoeRaw={rawQoe:F3}, qoeScaled={qoe:F3}, ene={ene:F3}, " +
                $"indiv={indiv:F4}, alive={alive:F4}, crowd={crowd:F4}, " +
                $"stepNoGroup={stepNoGroup:F4}, cov={cov:F3}, epiCum={GetCumulativeReward():F4}"
            );
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
                // 새로 추가: 같은 UE를 여러 드론이 잡으면 demand를 나눠 가지기
                int k = Mathf.Max(1, rr.ConnectedSourceCount);
                float shareDemand = (float)demand / k;

                num += qoe * shareDemand;
            }
        }

        if (denom <= 0f) return 0f;
        return Mathf.Clamp01(num / denom);
    }

    /// <summary>
    /// ★ Static으로 변경: 매니저에서도 호출해야 함
    /// </summary>
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

        float cov = tau / (1f + w);
        return Mathf.Clamp01(cov);
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
        float myDemand = 0f;

        foreach (var rr in RadioReceiver.All)
        {
            if (rr == null) continue;

            var area = rr.GetComponentInParent<DemandArea>();
            if (area == null || area.kind != AreaKind.Building) continue;

            if (rr.IsConnectedTo(srcId))
                myDemand += Mathf.Max(0, area.demand);
        }

        if (totalWeightDenom <= 0f) return 0f;
        return Mathf.Clamp01(myDemand / totalWeightDenom);
    }

    float ComputeNeighborDensity()
    {
        // ★ 변경: FindObjectsOfType -> DroneTeamManager.Instance.agents
        var agents = DroneTeamManager.Instance?.agents;
        if (agents == null || agents.Count == 0) return 0f;

        Vector3 myPos = transform.position;
        int near = 0;

        for (int i = 0; i < agents.Count; i++)
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

    // ★ 삭제됨 ★ (매니저로 이동)
    // float ComputeSurvivalRatio() { ... }

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

        // === 수정: 초반/이후 구분 없이, 경계 밖으로 나가면 항상 죽음 ===
        if (outX || outZ || outY)
        {
            AddReward(boundaryPenalty);

            if (eliminateOnFail)
            {
                // 바로 사망 (패널티는 Manager에서 처리)
                Eliminate("boundary");

                // 그룹 에피소드 안 쓰는 경우에만 개별 에피소드 종료
                if (!useGroupEpisodes && endOnBoundary)
                    EndEpisode();
            }
        }
    }

    

    private void OnCollisionEnter(Collision collision)
    {
        if (_isEliminated) return;

        bool isObstacleTag = obstacleTags != null &&
                            System.Array.Exists(obstacleTags, t => collision.collider.CompareTag(t));
        bool isObstacleLayer = ((1 << collision.collider.gameObject.layer) & obstacleLayers.value) != 0;

        if (isObstacleTag || isObstacleLayer)
        {
            // === 수정: 초반/이후 상관 없이 항상 같은 페널티 + 즉시 사망 ===
            AddReward(collisionPenalty);
            Eliminate("collision");

            // 그룹 에피소드 안 쓰는 경우에만 개별 에피소드 종료
            if (!useGroupEpisodes && endOnCollision)
                EndEpisode();
        }
    }


    void Eliminate(string reason)
    {
        if (_isEliminated) return;
        _isEliminated = true;

        // 1) 컨트롤러/센서 비활성화
        if (disableControllerAndSensorOnElim)
        {
            if (ctrl != null) ctrl.enabled = false;
            if (sensor != null) sensor.enabled = false;
        }

        // 2) 리지드바디 정지 및 중력/키네마틱 설정
        if (freezeRigidbodyOnElim && _rb != null)
        {
            _rb.velocity = Vector3.zero;
            _rb.angularVelocity = Vector3.zero;
            _rb.isKinematic = true;
            _rb.useGravity = false;
        }

        // 3) 콜라이더 비활성화
        if (disableColliderOnElim && _allColliders != null)
        {
            foreach (var c in _allColliders)
                if (c != null) c.enabled = false;
        }

        // 4) 통신 링크 정리
        int src = gameObject.GetInstanceID();
        foreach (var rr in RadioReceiver.All)
            if (rr != null) rr.ForceDisconnect(src);

        // 5) 마지막에 한 번만 Manager에 알림 (패널티/그룹 종료 결정)
        if (DroneTeamManager.Instance != null)
            DroneTeamManager.Instance.NotifyAgentEliminated(this, reason);
    }


    public void RecoverFromElimination()
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