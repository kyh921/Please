using System.Collections.Generic;
using UnityEngine;

#if UNITY_MLAGENTS
using Unity.MLAgents;
using Unity.MLAGENTS.Actuators;
using Unity.MLAGENTS.Sensors;
#endif

[RequireComponent(typeof(DroneController))]
[RequireComponent(typeof(RadioTransceiver))]
[RequireComponent(typeof(CommsSensor))]
public class DroneAgent :
#if UNITY_MLAGENTS
    Agent
#else
    MonoBehaviour
#endif
{
    // ===== 이동 제어 =====
    public float yawCmdDegPerSec = 0f;
    DroneController ctrl;
    RadioTransceiver radio;
    CommsSensor sensor;

    Vector3 prevPos;
    public float LastStepDistance { get; private set; }

    // ===== QoE 보상 =====
    [Header("QoE reward")]
    [Tooltip("분모 = Σ(전체 건물 가중치). 에피소드 시작 시 계산")]
    public float totalWeightDenom = 1f;

    [Tooltip("보상 안정화를 위한 작은 값")]
    public float eps = 1e-6f;

    // 이번 스텝 집계 입력(센서/리시버가 채워줌)
    float _qoeNumeratorThisStep = 0f; // 내 드론의 Σ(A_l × UE가중치)
    int _overconnectThisStep = 0;  // 유효링크수 - 1 (음수면 0으로)

    /// <summary>프레임 시작에 센서가 호출해서 누산기 초기화</summary>
    public void BeginStepAggregation()
    {
        _qoeNumeratorThisStep = 0f;
        _overconnectThisStep = 0;
    }

    /// <summary>
    /// 센서/리시버가 이번 프레임의 내 드론 QoE 분자와 오버커넥트를 보고.
    /// perDroneQoENumerator = Σ(A_l × UE가중치) for THIS drone.
    /// overconnect = 유효 링크 수 - 1.
    /// </summary>
    public void ReportQoEAndOverlap(float perDroneQoENumerator, int overconnect)
    {
        _qoeNumeratorThisStep += Mathf.Max(0f, perDroneQoENumerator);
        _overconnectThisStep = Mathf.Max(_overconnectThisStep, Mathf.Max(0, overconnect));
    }

    // ===== 에너지 모델 (Table III) =====
    [Header("Energy model (Table III)")]
    public float rho = 1.225f;        // air density [kg/m^3]
    public float s = 0.0157f;         // rotor solidity [-]
    public float R = 0.40f;           // rotor radius [m]
    public float Omega = 300f;        // blade angular velocity [rad/s]
    public float k_induced = 0.10f;   // induced power increment factor [-]
    public float delta_profile = 0.012f; // profile drag coeff δ [-]
    public float d0_parasite = 0.0161f;  // fuselage drag ratio d0 [-]
    public float massKg = 1.375f;     // aircraft mass incl. battery [kg]
    public float hoverSpeedEps = 0.2f;   // [m/s] 이하면 호버 파워 사용

    // (선택) 배터리 표기용
    public float batteryVolt = 15.2f;   // [V]
    public float battery_mAh = 5870f;   // [mAh]

    // --- 파생 값 ---
    float DiscAreaA => Mathf.PI * R * R;                           // A = πR^2
    float WeightN => massKg * 9.81f;                             // W [N]
    float Vtip => Omega * R;                                  // U_tip
    float v0_hover => Mathf.Sqrt(WeightN / (2f * rho * DiscAreaA)); // v0

    float PowerHoverW()
    {
        float Po = (delta_profile / 8f) * rho * s * DiscAreaA
                 * Mathf.Pow(Omega, 3f) * Mathf.Pow(R, 3f);
        float Pi = (1f + k_induced) * Mathf.Pow(WeightN, 1.5f) / Mathf.Sqrt(2f * rho * DiscAreaA);
        return Po + Pi; // [W]
    }

    float PowerForwardW(float v)
    {
        v = Mathf.Max(0f, v);
        float Po = (delta_profile / 8f) * rho * s * DiscAreaA
                 * Mathf.Pow(Omega, 3f) * Mathf.Pow(R, 3f);
        float Pi_hover = (1f + k_induced) * Mathf.Pow(WeightN, 1.5f) / Mathf.Sqrt(2f * rho * DiscAreaA);

        float V2 = v * v;
        float v0 = v0_hover;
        float inside = Mathf.Sqrt(1f + (V2 * V2) / (4f * Mathf.Pow(v0, 4f))) - (V2 / (2f * v0 * v0));
        float induced = Pi_hover * Mathf.Sqrt(Mathf.Max(0f, inside));
        float profile = Po * (1f + 3f * V2 / (Vtip * Vtip));
        float parasite = 0.5f * d0_parasite * rho * s * DiscAreaA * v * V2;

        return profile + induced + parasite; // [W]
    }

    // ===== Unity lifecycle =====
    void Awake()
    {
        ctrl = GetComponent<DroneController>();
        radio = GetComponent<RadioTransceiver>();
        sensor = GetComponent<CommsSensor>();
        prevPos = transform.position;
    }

#if UNITY_MLAGENTS
    public override void OnEpisodeBegin()
    {
        // 간단 리셋
        Vector3 p = new Vector3(Random.Range(-200f, 200f), Random.Range(40f, 120f), Random.Range(-200f, 200f));
        transform.position = p;
        transform.rotation = Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);
        prevPos = transform.position;
        LastStepDistance = 0f;

        // 분모 = Σ(전체 건물 가중치)
        int totalWeight = 0;
        foreach (var da in FindObjectsOfType<DemandArea>(true))
            if (da.kind == AreaKind.Building)
                totalWeight += Mathf.Max(0, da.demand);

        totalWeightDenom = Mathf.Max(1f, totalWeight);

        // 누산기 초기화
        _qoeNumeratorThisStep = 0f;
        _overconnectThisStep  = 0;
    }

    public override void CollectObservations(VectorSensor s)
    {
        s.AddObservation(transform.position / 500f);
        s.AddObservation(ctrl.Altitude() / 200f);
        s.AddObservation(ctrl.CurrentVelocity() / 20f);

        int linkable = sensor ? sensor.linkableCount : 0;
        s.AddObservation(Mathf.Clamp01(linkable / 50f));
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        var a = actions.ContinuousActions;
        Vector3 cmdLocal = new Vector3(a[0], a[1], a[2]);
        ctrl.SetCommand(new Vector3(cmdLocal.x * ctrl.maxHorizontalSpeed,
                                    cmdLocal.y * ctrl.maxClimbRate,
                                    cmdLocal.z * ctrl.maxHorizontalSpeed),
                        yawCmdDegPerSec);

        // 이동 추적
        LastStepDistance = Vector3.Distance(prevPos, transform.position);
        prevPos = transform.position;

        // ===== 보상 =====
        float qoe = ComputeQoEReward_Aggregated();
        float cov = ComputeCoverageReward_Aggregated();
        float ene = ComputeEnergyReward();

        AddReward(qoe * cov * ene);

        // 다음 스텝 대비
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
#else
    void Update()
    {
        float strafe = Input.GetAxis("Horizontal");
        float forward = Input.GetAxis("Vertical");
        float climb = 0f; if (Input.GetKey(KeyCode.E)) climb += 1f; if (Input.GetKey(KeyCode.Q)) climb -= 1f;

        ctrl.SetCommand(new Vector3(strafe * ctrl.maxHorizontalSpeed,
                                    climb * ctrl.maxClimbRate,
                                    forward * ctrl.maxHorizontalSpeed),
                        yawCmdDegPerSec);

        LastStepDistance = Vector3.Distance(prevPos, transform.position);
        prevPos = transform.position;

        float finalReward = ComputeQoEReward_Aggregated()
                          * ComputeCoverageReward_Aggregated()
                          * ComputeEnergyReward();

        // 디버그 모드에선 누산기 초기화만
        _qoeNumeratorThisStep = 0f;
        _overconnectThisStep = 0;
    }
#endif

    // ===== Reward terms =====

    // QoE = ( 내 드론의 Σ(A_l * UE가중치) ) / ( Σ(전체 건물 가중치) )
    float ComputeQoEReward_Aggregated()
    {
        float qoe = _qoeNumeratorThisStep / Mathf.Max(eps, totalWeightDenom);
        return Mathf.Clamp01(qoe);
    }

    // Coverage = 1 / (1 + overconnect)
    float ComputeCoverageReward_Aggregated()
    {
        return 1f / (1f + Mathf.Max(0, _overconnectThisStep));
    }

    // Energy reward = 1 / (1 + μ(t)), μ = P * Δt
    float ComputeEnergyReward()
    {
        Vector3 v = ctrl.CurrentVelocity();
        float V = new Vector2(v.x, v.z).magnitude;

        float P = (V < hoverSpeedEps) ? PowerHoverW() : PowerForwardW(V); // [W]
        float mu = Mathf.Max(0f, P) * Mathf.Max(Time.deltaTime, 1e-3f);    // [J]

        return 1f / (1f + mu);
    }
}
