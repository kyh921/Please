using System;
using System.Collections.Generic;
using UnityEngine;

[DisallowMultipleComponent]
public class RadioReceiver : MonoBehaviour
{
    [Header("Identity")]
    [SerializeField] private int _nodeId = 0;
    public int NodeId => _nodeId;

    [Header("Threshold (dB)")]
    public float rxThresholdSinrDb = 0f;  // sinr ≥ 0 dB이면 연결로 간주

    [Header("Runtime (readonly)")]
    public int recvPackets = 0;

    // 수신 이벤트(깜빡임/커버) 그대로 사용
    public event Action<int, byte[], float> OnReceive;  // (srcId, payload, sinrDb)

    // --- 디버그 노출용 최종 계산값 ---
    public int   LastCQI      { get; private set; } = 0;
    public int   LastQm       { get; private set; } = 0;
    public int   LastITbs     { get; private set; } = -1;
    public int   LastTbsBits  { get; private set; } = 0;
    public int   LastAl_bps   { get; private set; } = 0;
    public float LastAl_Mbps  { get; private set; } = 0f;
    public float LastQoE      { get; private set; } = 0f;

    DemandArea _area;

    // ===== LTE 매핑(내부 보관) =====
    static readonly float[] _cqiThresholdDb = {
        float.NegativeInfinity,
        -9.478f,-6.658f,-4.098f,-1.798f, 0.399f,
         2.424f, 4.489f, 6.367f, 8.456f,10.266f,
        12.218f,14.122f,15.849f,17.786f,19.809f
    };

    static readonly (int Qm, int iTbs)[] _cqiToQmItbs = new (int,int)[32] {
        (2,0),(2,1),(2,2),(2,3),(2,4),(2,5),(2,6),
        (2,7),(2,8),(2,9),
        (4,9),(4,10),(4,11),(4,12),(4,13),(4,14),(4,15),
        (6,16),(6,17),(6,18),(6,19),(6,20),(6,21),(6,22),
        (6,23),(6,24),(6,25),(6,26),(6,26),
        (2,-1),(4,-1),(6,-1) // 29~31 reserved
    };

    // iTBS → TBS(bits) @ N_PRB=50 (iTBS=0..26)
    [Header("TBS(bits) for N_PRB=50 (index=iTBS 0..26)")]
    [SerializeField] int[] TBS_bits_for_PRB50 = new int[27] {
        /* 0*/1384, /* 1*/1800, /* 2*/2216, /* 3*/2856, /* 4*/3624, /* 5*/4392, /* 6*/5160,
        /* 7*/6200, /* 8*/6968, /* 9*/7992, /*10*/8760, /*11*/9912, /*12*/11448,
        /*13*/12960, /*14*/14112, /*15*/15264, /*16*/16416, /*17*/18336, /*18*/19848, /*19*/21384,
        /*20*/22920, /*21*/25456, /*22*/27376, /*23*/28336, /*24*/30576, /*25*/31704, /*26*/36696
    };

    // === 프레임 집계: srcId별 가중 QoE & 연결 집합 ===
    readonly Dictionary<int, float> _pendingWeightedQoe = new();
    readonly HashSet<int> _connectedSrc = new();

    void OnEnable()
    {
        if (_nodeId == 0) _nodeId = GetInstanceID();
        _area = GetComponent<DemandArea>() ?? GetComponentInParent<DemandArea>() ?? GetComponentInChildren<DemandArea>();
    }

    /// 링크모델이 계산한 SINR(dB) 주입 → A_l(Mbps) → QoE = log(237*A_l-216.6)
    /// 연결 기준(sinr ≥ rxThresholdSinrDb) 만족 시, 가중 QoE를 srcId에 누적.
    public void AcceptSinrFromModel(int srcId, float sinrDb, byte[] payload = null)
    {
        // 1) SINR → CQI
        int cqi = 0;
        for (int i = 15; i >= 1; --i)
            if (sinrDb >= _cqiThresholdDb[i]) { cqi = i; break; }

        int Qm = 0, iTbs = -1, tbsBits = 0;
        int Al_bps = 0; float Al_Mbps = 0f; float QoE = 0f;

        if (cqi > 0)
        {
            // 2) CQI → (Qm, iTBS)
            (Qm, iTbs) = _cqiToQmItbs[cqi];

            // 3) iTBS → TBS(bits) @ PRB50 → A_l
            if (iTbs >= 0 && iTbs < TBS_bits_for_PRB50.Length)
            {
                tbsBits  = Mathf.Max(0, TBS_bits_for_PRB50[iTbs]);
                Al_bps   = tbsBits * 1000;               // 1 ms → ×1000
                Al_Mbps  = Al_bps / 1_000_000f;          // Mbps
                QoE      = Mathf.Log(Mathf.Max(1e-6f, 237f * Al_Mbps - 216.6f));
            }
        }

        // 디버그 저장
        LastCQI=cqi; LastQm=Qm; LastITbs=iTbs; LastTbsBits=tbsBits; LastAl_bps=Al_bps; LastAl_Mbps=Al_Mbps; LastQoE=QoE;

        // 4) 연결 판정 + 집계
        if (sinrDb >= rxThresholdSinrDb)
        {
            _connectedSrc.Add(srcId);

            int weight = (_area && _area.kind == AreaKind.Building) ? Mathf.Max(0, _area.demand) : 0;
            float weightedQoe = Mathf.Max(0f, QoE * weight);

            if (_pendingWeightedQoe.TryGetValue(srcId, out float prev))
                _pendingWeightedQoe[srcId] = prev + weightedQoe;
            else
                _pendingWeightedQoe[srcId] = weightedQoe;

            // 기존 이벤트(깜빡임/커버 처리) 유지
            recvPackets++;
            OnReceive?.Invoke(srcId, payload, sinrDb);
        }
    }

    /// 드론(srcId)이 이 UE에서 가져갈 값:
    ///   weightedQoe = Σ(QoE × demand)  (pop)
    ///   overconnect = max(0, 연결수-1)  — 이 UE 기준, srcId가 연결돼 있을 때만 계산
    public void PopQoeAndOverlapFor(int srcId, out float weightedQoe, out int overconnect)
    {
        _pendingWeightedQoe.TryGetValue(srcId, out weightedQoe);
        _pendingWeightedQoe[srcId] = 0f;

        if (_connectedSrc.Contains(srcId))
            overconnect = Mathf.Max(0, _connectedSrc.Count - 1);
        else
            overconnect = 0;
    }

    /// 프레임 경계에서 전체 리셋이 필요하면 호출(선택사항)
    public void ResetAggregation()
    {
        _pendingWeightedQoe.Clear();
        _connectedSrc.Clear();
    }

    public Vector3 GetAntennaPosition()
    {
        var rn = GetComponent<ReceiverNode>();
        return rn ? rn.GetRxPosition() : transform.position;
    }
}
