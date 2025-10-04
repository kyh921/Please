using System.Collections.Generic;
using UnityEngine;

using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

[RequireComponent(typeof(DroneController))]
public class DroneAgent : Agent
{
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

    // ===== 충돌 & 경계 =====
    [Header("Collision & Boundary")]
    public float collisionPenalty = -0.5f;
    public bool endOnCollision = true;

    [Tooltip("작업영역 X 최소/최대, Z 최소/최대 (비대칭 지원)")]
    public float xMin = -623f, xMax = 44f;
    public float zMin = -531f, zMax = -2.5f;

    [Tooltip("허용 고도 범위 (min,max)")]
    public Vector2 yLimit  = new Vector2(20f, 150f);

    public float boundaryPenalty = -0.2f;
    public bool endOnBoundary = true;

    [Tooltip("커버리지 중복(오버커넥트) 1단위당 추가 패널티")]
    public float overlapPenaltyPerLink = 0.05f;

    public LayerMask obstacleLayers;
    public string[] obstacleTags = new string[] { "Drone", "Obstacle", "Building" };

    // ===== Unity lifecycle =====
    void Awake()
    {
        ctrl = GetComponent<DroneController>();
        if (sensor == null) sensor = GetComponentInChildren<RadioReceiver>();
        prevPos = transform.position;
    }

    public override void OnEpisodeBegin()
    {
        float rx = Random.Range(xMin, xMax);
        float rz = Random.Range(zMin, zMax);
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

        float qoe = ComputeQoEReward_Aggregated();
        float cov = ComputeCoverageReward_Aggregated();
        float ene = ComputeEnergyReward();

        int overlap = Mathf.Max(0, _overconnectThisStep);
        float overlapPenalty = overlap * overlapPenaltyPerLink;

        float stepR = qoe * cov * ene - overlapPenalty;
        AddReward(stepR);

        if (debugReward)
        {
            Debug.Log($"[Agent {name}] QoE={qoe:F3}  Cov={cov:F3}  Ene={ene:F3}  Overlap={overlap}  " +
                      $"pen_o={overlapPenalty:F3}  => stepR={stepR:F4}, cumR={GetCumulativeReward():F4}  " +
                      $"num={_qoeNumeratorThisStep:F3} denom={totalWeightDenom:F1}");
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
        float qoe = _qoeNumeratorThisStep / Mathf.Max(eps, totalWeightDenom);
        return Mathf.Clamp01(qoe);
    }

    float ComputeCoverageReward_Aggregated()
    {
        return 1f / (1f + Mathf.Max(0, _overconnectThisStep));
    }

    float ComputeEnergyReward()
    {
        Vector3 v = ctrl.CurrentVelocity();
        float V = new Vector2(v.x, v.z).magnitude;

        float P  = (V < hoverSpeedEps) ? PowerHoverW() : PowerForwardW(V);
        float mu = Mathf.Max(0f, P) * Mathf.Max(Time.deltaTime, 1e-3f);

        return 1f / (1f + mu);
    }

    // ===== 충돌 & 경계 처리 =====
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
        Vector3 center = new Vector3((xMin + xMax) * 0.5f, y, (zMin + zMax) * 0.5f);
        Vector3 size   = new Vector3(Mathf.Abs(xMax - xMin), 0.1f, Mathf.Abs(zMax - zMin));
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireCube(center, size);
    }
#endif
}
