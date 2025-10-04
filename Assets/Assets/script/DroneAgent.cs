using System.Collections.Generic;
using UnityEngine;

using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

[RequireComponent(typeof(DroneController))]
public class DroneAgent : Agent
{
    private Rigidbody _rb;
    int stepCount;
    // ===== �대룞 �쒖뼱 =====
    public float yawCmdDegPerSec = 0f;
    private DroneController ctrl;

    [SerializeField] private RadioReceiver sensor;

    private Vector3 prevPos;
    public float LastStepDistance { get; private set; }

    // ===== QoE 蹂댁긽 =====
    [Header("QoE reward")]
    [Tooltip("遺꾨え = 誇(�꾩껜 嫄대Ъ 媛�以묒튂). �먰뵾�뚮뱶 �쒖옉 �� 怨꾩궛")]
    public float totalWeightDenom = 1f;

    [Tooltip("蹂댁긽 �덉젙�붾� �꾪븳 �묒� 媛�")]
    public float eps = 1e-6f;

    // 吏묎퀎 �낅젰(�꾨젅�꾨떦)
    private float _qoeNumeratorThisStep = 0f; // �� �쒕줎�� 誇(A_l 횞 UE媛�以묒튂)
    private int   _overconnectThisStep   = 0; // �좏슚 留곹겕 �� - 1

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

    // ===== �먮꼫吏� 紐⑤뜽 (Table III) =====

    // --- Battery state (Q(t), Wh) ---
    private float energyWhInit;   // 珥덇린 異⑹쟾��(Wh), �먰뵾�뚮뱶 �쒖옉 �� 怨꾩궛
    private float energyWh;       // �꾩옱 �붾웾 Q(t) (Wh)


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

    // �쇰Ц ��(12): Q(t+1) = max{0, Q(t) - 關(t)}
    // �ш린�� 關(t)�� �꾨젰[W] * �t[s] / 3600 = Wh 濡� �섏궛
    void UpdateEnergyQueueByPaperModel()
    {
        // �t: FixedUpdate�� OnActionReceived �쇱슜 ��鍮�
        float dt = Time.inFixedTimeStep ? Time.fixedDeltaTime : Time.deltaTime;
        dt = Mathf.Max(dt, 1e-4f);

        // 3D �띾룄: Rigidbody媛� �덉쑝硫� 洹멸쾬�� �좊ː, �놁쑝硫� �꾩튂 蹂��붾줈 異붿젙
        float V;
        if (_rb != null)
            V = _rb.velocity.magnitude;
        else
            V = (transform.position - prevPos).magnitude / dt;

        // hover/forward 遺꾨쪟留� �뺥솗�� (hoverSpeedEps=0.2 �좎�)
        float P_hover = PowerHoverW();
        float P = (V <= hoverSpeedEps)
            ? P_hover
            : Mathf.Max(P_hover * 2.00f, PowerForwardW(V));

        float usedWh = Mathf.Max(0f, P) * dt / 3600f;  // 關(t) [Wh]
        energyWh = Mathf.Max(0f, energyWh - usedWh);
    }

    // ===== 異⑸룎 & 寃쎄퀎 =====
    [Header("Collision & Boundary")]
    public float collisionPenalty = -0.5f;
    public bool endOnCollision = true;

    // ---------- 작업영역(경계) ----------
    [Tooltip("작업영역 X 최소/최대, Z 최소/최대 (드론은 이 범위를 벗어나면 패널티/종료)")]
    public float xMin = -620f, xMax = 60f;
    public float zMin = -540f, zMax = 550f;

    [Tooltip("허용 고도 범위 (min,max)")]
    public Vector2 yLimit  = new Vector2(20f, 150f);

    public float boundaryPenalty = -0.2f;
    public bool endOnBoundary = true;

    [Tooltip("而ㅻ쾭由ъ� 以묐났(�ㅻ쾭而ㅻ꽖��) 1�⑥쐞�� 異붽� �⑤꼸��")]
    public float overlapPenaltyPerLink = 0.05f;

    public LayerMask obstacleLayers;
    public string[] obstacleTags = new string[] { "Drone", "Obstacle", "Building" };

    // ---------- 스폰 범위(에피소드 시작 시 위치 뽑기) ----------
    [Header("Spawn Range (per episode)")]
    [Tooltip("OnEpisodeBegin에서 사용할 스폰 범위 X")]
    public float spawnXMin = -65f, spawnXMax = 43f;

    [Tooltip("OnEpisodeBegin에서 사용할 스폰 범위 Z")]
    public float spawnZMin = -13f, spawnZMax = 125f;

    // ===== Unity lifecycle =====
    void Awake()
    {
        ctrl = GetComponent<DroneController>();
        if (sensor == null) sensor = GetComponentInChildren<RadioReceiver>();
        _rb = GetComponent<Rigidbody>();
        prevPos = transform.position;
    }

    public override void OnEpisodeBegin()
    {
        energyWhInit = (batteryVolt * battery_mAh) / 1000f;
        energyWh = energyWhInit;

        // 스폰 범위 사용 (요청 반영)
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
                    $"[Agent {gameObject.name}] QoE={{qoe:F3}}  Cov={{cov:F3}}  Ene={{ene:F3}}  Overlap={overlap}  pen_o={{overlapPenalty:F3}} " +
                    $"=> stepR={{stepR:F4}} num={{_qoeNumeratorThisStep:F3}} denom={{totalWeightDenom:F1}} batt={{(energyWhInit > 0 ? (energyWh / energyWhInit * 100f) : 0f):F2}} %"
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
        float qoe = _qoeNumeratorThisStep / (3.5f*totalWeightDenom);
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

    // ===== 異⑸룎 & 寃쎄퀎 泥섎━ =====
    void OnCollisionEnter(Collision other)
    {
        if (!IsObstacle(other.collider)) return;

        AddReward(collisionPenalty);
        if (endOnCollision) EndEpisode();
    }

    void OnTriggerEnter(Collider other)
    {
        if (!IsObstacle(other)) return;

        AddReward(collisionPenalty);
        if (endOnCollision) EndEpisode();
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
        var p = transform.position;

        // 작업영역(경계) 체크
        bool outX = (p.x < xMin) || (p.x > xMax);
        bool outZ = (p.z < zMin) || (p.z > zMax);
        bool outY = (p.y < yLimit.x) || (p.y > yLimit.y);

        if (outX || outZ || outY)
        {
            AddReward(boundaryPenalty);
            if (endOnBoundary) EndEpisode();
        }
    }

#if UNITY_EDITOR
    void OnDrawGizmosSelected()
    {
        float y = (Application.isPlaying ? transform.position.y : 50f);

        // 작업영역(경계) - 노란색
        Vector3 boundaryCenter = new Vector3((xMin + xMax) * 0.5f, y, (zMin + zMax) * 0.5f);
        Vector3 boundarySize   = new Vector3(Mathf.Abs(xMax - xMin), 0.1f, Mathf.Abs(zMax - zMin));
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireCube(boundaryCenter, boundarySize);

        // 스폰 영역 - 청록색
        Vector3 spawnCenter = new Vector3((spawnXMin + spawnXMax) * 0.5f, y, (spawnZMin + spawnZMax) * 0.5f);
        Vector3 spawnSize   = new Vector3(Mathf.Abs(spawnXMax - spawnXMin), 0.1f, Mathf.Abs(spawnZMax - spawnZMin));
        Gizmos.color = Color.cyan;
        Gizmos.DrawWireCube(spawnCenter, spawnSize);
    }
#endif
}
