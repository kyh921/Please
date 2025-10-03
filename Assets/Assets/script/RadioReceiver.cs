using System;
using System.Collections.Generic;
using UnityEngine;

public enum ReceiverMetric
{
    AlThroughputMbps, // A_l : Mbps
    QoE_Log           // log(237*A_l - 216.6)
}

[DisallowMultipleComponent]
public class RadioReceiver : MonoBehaviour
{
    // ===== Static registry (옵션: 씬에서 UE 빠르게 모으기) =====
    static readonly HashSet<RadioReceiver> s_all = new HashSet<RadioReceiver>();
    public static IReadOnlyCollection<RadioReceiver> All => s_all;
    public bool debugLog = false;


    [Header("Identity")]
    [SerializeField] private int _nodeId = 0;
    public int NodeId => _nodeId;

    [Header("Threshold (dB)")]
    [Tooltip("이 값 이상일 때 '연결'로 간주 (overconnect 계산에 사용)")]
    public float rxThresholdSinrDb = 0f; // 기본: 0 dB

    [Header("Metric Mode")]
    [Tooltip("드론으로 넘길 지표 선택: A_l(Mbps) 또는 QoE=log(237*A_l-216.6)")]
    public ReceiverMetric metric = ReceiverMetric.AlThroughputMbps;

    [Header("Runtime (readonly)")]
    public int recvPackets = 0;

    // 수신 이벤트(시각화/깜빡임 등)
    public event Action<int, byte[], float> OnReceive;  // (srcId, payload, sinrDb)

    // --- 디버그 노출값 ---
    public int LastCQI { get; private set; } = 0;
    public int LastQm { get; private set; } = 0;
    public int LastITbs { get; private set; } = -1;
    public int LastTbsBits { get; private set; } = 0;
    public int LastAl_bps { get; private set; } = 0;
    public float LastAl_Mbps { get; private set; } = 0f;
    public float LastQoE { get; private set; } = 0f;

    DemandArea _area;

    // ===== LTE CQI 임계 (SINR dB) =====
    static readonly float[] _cqiThresholdDb = {
        float.NegativeInfinity,
        -9.478f,-6.658f,-4.098f,-1.798f, 0.399f,
         2.424f, 4.489f, 6.367f, 8.456f,10.266f,
        12.218f,14.122f,15.849f,17.786f,19.809f
    };

    // CQI → (Qm, iTBS)
    static readonly (int Qm, int iTbs)[] _cqiToQmItbs = new (int, int)[32] {
        (2,0),(2,1),(2,2),(2,3),(2,4),(2,5),(2,6),
        (2,7),(2,8),(2,9),
        (4,9),(4,10),(4,11),(4,12),(4,13),(4,14),(4,15),
        (6,16),(6,17),(6,18),(6,19),(6,20),(6,21),(6,22),
        (6,23),(6,24),(6,25),(6,26),(6,26),
        (2,-1),(4,-1),(6,-1) // 29~31 reserved
    };

    // iTBS → TBS(bits) @ N_PRB=50 (iTBS=0..26)
    [Header("TBS(bits) for N_PRB=50 (index=iTBS 0..26)")]
    [SerializeField]
    int[] TBS_bits_for_PRB50 = new int[27] {
        /* 0*/1384, /* 1*/1800, /* 2*/2216, /* 3*/2856, /* 4*/3624, /* 5*/4392, /* 6*/5160,
        /* 7*/6200, /* 8*/6968, /* 9*/7992, /*10*/8760, /*11*/9912, /*12*/11448,
        /*13*/12960, /*14*/14112, /*15*/15264, /*16*/16416, /*17*/18336, /*18*/19848, /*19*/21384,
        /*20*/22920, /*21*/25456, /*22*/27376, /*23*/28336, /*24*/30576, /*25*/31704, /*26*/36696
    };

    // === 프레임 집계: 드론ID별 누산 버킷 ===
    // droneId -> Σ(metric * demand),  droneId -> 연결여부 카운트
    readonly Dictionary<int, float> _sumWeighted = new Dictionary<int, float>(64);
    readonly HashSet<int> _connectedSrc = new HashSet<int>(); // 이 UE와 연결된 드론 집합

    void OnEnable()
    {
        if (_nodeId == 0) _nodeId = GetInstanceID();
        _area = GetComponent<DemandArea>() ?? GetComponentInParent<DemandArea>() ?? GetComponentInChildren<DemandArea>();
        s_all.Add(this);
    }

    void OnDisable()
    {
        s_all.Remove(this);
        _sumWeighted.Clear();
        _connectedSrc.Clear();
    }

    /// 링크모델이 계산한 SINR(dB) 주입 → A_l(Mbps) → (선택)QoE → demand 곱 → 누산
    /// 연결 기준(sinr ≥ rxThresholdSinrDb) 충족 시만 누산 및 연결 집합 반영
    public void AcceptSinrFromModel(int srcId, float sinrDb, byte[] payload = null)
    {
        // 1) SINR → CQI
        int cqi = 0;
        for (int i = 15; i >= 1; --i)
        {
            if (sinrDb >= _cqiThresholdDb[i]) { cqi = i; break; }
        }

        int Qm = 0, iTbs = -1, tbsBits = 0;
        int Al_bps = 0;
        float Al_Mbps = 0f;
        float QoE = 0f;

        if (cqi > 0)
        {
            // 2) CQI → (Qm, iTBS)
            (Qm, iTbs) = _cqiToQmItbs[cqi];

            // 3) iTBS → TBS(bits) @ 1ms, N_PRB=50 → A_l
            if ((uint)iTbs < (uint)TBS_bits_for_PRB50.Length)
            {
                tbsBits = Mathf.Max(0, TBS_bits_for_PRB50[iTbs]);
                Al_bps = tbsBits * 1000;        // 1ms 전송 → bps 환산
                Al_Mbps = Al_bps / 1_000_000f;   // Mbps
                // QoE 모델(옵션)
                QoE = Mathf.Log(Mathf.Max(1e-6f, 237f * Al_Mbps - 216.6f)); // ln()
            }
        }

        // 디버그 저장
        LastCQI = cqi; LastQm = Qm; LastITbs = iTbs; LastTbsBits = tbsBits; LastAl_bps = Al_bps; LastAl_Mbps = Al_Mbps; LastQoE = QoE;

        if (debugLog)
        {
            Debug.Log($"[UE {name}] src={srcId} SINR={sinrDb:F2} dB, CQI={cqi}, iTBS={iTbs}, " +
                      $"A_l={Al_Mbps:F3} Mbps, QoE={QoE:F3}");
        }

        // 4) 연결 판정 + 누산
        if (sinrDb >= rxThresholdSinrDb)
        {
            _connectedSrc.Add(srcId);

            int demand = (_area && _area.kind == AreaKind.Building) ? Mathf.Max(0, _area.demand) : 0;
            float value = (metric == ReceiverMetric.AlThroughputMbps) ? Al_Mbps : QoE;
            float weighted = Mathf.Max(0f, value) * demand;

            if (debugLog)
            {
                // prevTmp 라는 다른 이름으로 사용
                float nowSum = (_sumWeighted.TryGetValue(srcId, out var prevTmp) ? prevTmp + weighted : weighted);
                Debug.Log($"[UE {name}] CONNECT src={srcId} demand={demand}  addWeighted={weighted:F3}  nowSum={nowSum:F3}");
            }

            // 여기서는 prevVal 로 사용 (prev 라는 이름 재사용 금지)
            if (_sumWeighted.TryGetValue(srcId, out float prevVal))
                _sumWeighted[srcId] = prevVal + weighted;
            else
                _sumWeighted[srcId] = weighted;

            recvPackets++;
            OnReceive?.Invoke(srcId, payload, sinrDb);
        }
    }

    /// 드론(srcId)이 이 UE에서 가져갈 값:
    ///   sumWeighted = Σ( (A_l 또는 QoE) × demand )  (pop)
    ///   overconnect = max(0, 연결드론수 - 1)  — srcId가 이 UE와 연결되어 있을 때만 반영
    public void PopQoeAndOverlapFor(int srcId, out float sumWeighted, out int overconnect)
    {
        _sumWeighted.TryGetValue(srcId, out sumWeighted);
        // pop: 키 제거하여 누수/증식을 방지
        _sumWeighted.Remove(srcId);

        if (_connectedSrc.Contains(srcId))
            overconnect = Mathf.Max(0, _connectedSrc.Count - 1);
        else
            overconnect = 0;

        if (debugLog)
        {
            Debug.Log($"[UE {name}] POP by src={srcId}  sumWeighted={sumWeighted:F3}  overconnect={overconnect}");
        }
    }

    /// 프레임 경계에서 전체 리셋(여러 드론이 모두 Pop한 뒤 매니저/센서가 호출)
    public void ResetAggregation()
    {
        _sumWeighted.Clear();
        _connectedSrc.Clear();
    }

    public Vector3 GetAntennaPosition()
    {
        var rn = GetComponent<ReceiverNode>();
        return rn ? rn.GetRxPosition() : transform.position;
    }
}
