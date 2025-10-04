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
    // ===== Static registry (占쎈벊��: 占싼딅퓠占쏙옙 UE ��쥓�ㅵ칰占� 筌뤴뫁�앮묾占�) =====
    static readonly HashSet<RadioReceiver> s_all = new HashSet<RadioReceiver>();
    public static IReadOnlyCollection<RadioReceiver> All => s_all;
    public bool debugLog = false;

    [Header("Identity")]
    [SerializeField] private int _nodeId = 0;
    public int NodeId => _nodeId;

    [Header("Threshold (dB)")]
    [Tooltip("占쏙옙 揶쏉옙 占쎈똻湲쏙옙占� 占쏙옙 '占쎄퀗猿�'嚥∽옙 揶쏄쑴竊� (overconnect �④쑴沅쏉옙占� 占싼딆뒠)")]
    public float rxThresholdSinrDb = 0f; // 疫꿸퀡��: 0 dB

    [Header("Metric Mode")]
    [Tooltip("占쎌뮆以롳옙�곗쨮 占쎌꼵留� 筌욑옙占쏙옙 占쎌쥚源�: A_l(Mbps) 占쎈Ŧ�� QoE=log(237*A_l-216.6)")]
    public ReceiverMetric metric = ReceiverMetric.AlThroughputMbps;

    [Header("Runtime (readonly)")]
    public int recvPackets = 0;

    public event Action<int, byte[], float> OnReceive;  // (srcId, payload, sinrDb)

    // --- 占쎈뗀苡�뉩占� 占쎈챷�㎩첎占� ---
    public int LastCQI { get; private set; } = 0;
    public int LastQm { get; private set; } = 0;
    public int LastITbs { get; private set; } = -1;
    public int LastTbsBits { get; private set; } = 0;
    public int LastAl_bps { get; private set; } = 0;
    public float LastAl_Mbps { get; private set; } = 0f;
    public float LastQoE { get; private set; } = 0f;

    DemandArea _area;

    // ===== LTE CQI 占쎄쑨�� (SINR dB) =====
    static readonly float[] _cqiThresholdDb = {
        float.NegativeInfinity,
        -9.478f,-6.658f,-4.098f,-1.798f, 0.399f,
         2.424f, 4.489f, 6.367f, 8.456f,10.266f,
        12.218f,14.122f,15.849f,17.786f,19.809f
    };

    // CQI 占쏙옙 (Qm, iTBS)
    static readonly (int Qm, int iTbs)[] _cqiToQmItbs = new (int, int)[32] {
        (2,0),(2,1),(2,2),(2,3),(2,4),(2,5),(2,6),
        (2,7),(2,8),(2,9),
        (4,9),(4,10),(4,11),(4,12),(4,13),(4,14),(4,15),
        (6,16),(6,17),(6,18),(6,19),(6,20),(6,21),(6,22),
        (6,23),(6,24),(6,25),(6,26),(6,26),
        (2,-1),(4,-1),(6,-1) // 29~31 reserved
    };

    // iTBS 占쏙옙 TBS(bits) @ N_PRB=50 (iTBS=0..26)
    [Header("TBS(bits) for N_PRB=50 (index=iTBS 0..26)")]
    [SerializeField]
    int[] TBS_bits_for_PRB50 = new int[27] {
        /* 0*/1384, /* 1*/1800, /* 2*/2216, /* 3*/2856, /* 4*/3624, /* 5*/4392, /* 6*/5160,
        /* 7*/6200, /* 8*/6968, /* 9*/7992, /*10*/8760, /*11*/9912, /*12*/11448,
        /*13*/12960, /*14*/14112, /*15*/15264, /*16*/16416, /*17*/18336, /*18*/19848, /*19*/21384,
        /*20*/22920, /*21*/25456, /*22*/27376, /*23*/28336, /*24*/30576, /*25*/31704, /*26*/36696
    };

    // === 占쎄쑬�낉옙占� 筌욌쵌��: 占쎌뮆以랪D癰귨옙 占쎄쑴沅� 甕곌쑵沅� ===
    // droneId -> 沃�(metric * demand),  droneId -> 占쎄퀗猿먲옙�占� 燁삳똻�ワ옙占�
    readonly Dictionary<int, float> _sumWeighted = new Dictionary<int, float>(64);
    readonly HashSet<int> _connectedSrc = new HashSet<int>(); // 占쏙옙 UE占쏙옙 占쎄퀗猿먲옙占� 占쎌뮆以� 筌욌쵑鍮�

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

    public void ForceDisconnect(int srcId)
    {
        _sumWeighted.Remove(srcId);
        _connectedSrc.Remove(srcId);
    }
    public void AcceptSinrFromModel(int srcId, float sinrDb, byte[] payload = null)
    {
        // 1) SINR 占쏙옙 CQI
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
            // 2) CQI 占쏙옙 (Qm, iTBS)
            (Qm, iTbs) = _cqiToQmItbs[cqi];

            // 3) iTBS 占쏙옙 TBS(bits) @ 1ms, N_PRB=50 占쏙옙 A_l
            if ((uint)iTbs < (uint)TBS_bits_for_PRB50.Length)
            {
                tbsBits = Mathf.Max(0, TBS_bits_for_PRB50[iTbs]);
                Al_bps = tbsBits * 1000;        // 1ms 占쎄쑴�� 占쏙옙 bps 占쎌꼷沅�
                Al_Mbps = Al_bps / 1_000_000f;  // Mbps
                QoE = Mathf.Log(Mathf.Max(1e-6f, Al_Mbps - 1.0f));
            }
        }

        // 占쎈뗀苡�뉩占� 占쏙옙占쏙옙
        LastCQI = cqi; LastQm = Qm; LastITbs = iTbs; LastTbsBits = tbsBits; LastAl_bps = Al_bps; LastAl_Mbps = Al_Mbps; LastQoE = QoE;

        if (debugLog)
        {
            Debug.Log($"[UE {name}] src={srcId} SINR={sinrDb:F2} dB, CQI={cqi}, iTBS={iTbs}, A_l={Al_Mbps:F3} Mbps, QoE={QoE:F3}");
        }

        // 4) 占쎄퀗猿� 占쎈Ŋ�� + 占쎄쑴沅�
        if (sinrDb >= rxThresholdSinrDb)
        {
            _connectedSrc.Add(srcId);

            int demand = (_area && _area.kind == AreaKind.Building) ? Mathf.Max(0, _area.demand) : 0;
            float value = (metric == ReceiverMetric.AlThroughputMbps) ? Al_Mbps : QoE;
            float weighted = Mathf.Max(0f, value) * demand;

            if (debugLog)
            {
                float nowSum = (_sumWeighted.TryGetValue(srcId, out var prevTmp) ? prevTmp + weighted : weighted);
                Debug.Log($"[UE {name}] CONNECT src={srcId} demand={demand}  addWeighted={weighted:F3}  nowSum={nowSum:F3}");
            }

            if (_sumWeighted.TryGetValue(srcId, out float prevVal))
                _sumWeighted[srcId] = prevVal + weighted;
            else
                _sumWeighted[srcId] = weighted;

            recvPackets++;
            OnReceive?.Invoke(srcId, payload, sinrDb);
        }
    }

    public void PopQoeAndOverlapFor(int srcId, out float sumWeighted, out int overconnect)
    {
        _sumWeighted.TryGetValue(srcId, out sumWeighted);
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

#if UNITY_EDITOR
    // 占싼딅퓠占쏙옙 占쎌쥚源� 占쏙옙 占쎈뜇�믭옙占� 占쎄쑴�귞몴占� 占쎌뮄而뽳옙占� (占쎈뗀苡�틦占� 占쎈챷��)
    void OnDrawGizmosSelected()
    {
        Vector3 p = GetAntennaPosition();
        Gizmos.color = Color.cyan;
        Gizmos.DrawWireSphere(p, 0.6f);
        Gizmos.DrawLine(transform.position, p);
    }
#endif
}