using System.Linq;
using UnityEngine;

public enum AreaKind { Building, Road }

[DisallowMultipleComponent]
public class DemandArea : MonoBehaviour
{
    [Header("Type & Demand")]
    public AreaKind kind = AreaKind.Building;
    public int demand;
    public bool covered = false;

    [Header("Colors")]
    public Color colorLow = Color.yellow;                 // 0~14
    public Color colorMid = new Color(1f, 0.55f, 0f, 1f); // 15~30
    public Color colorHigh = Color.red;                   // 30+
    public Color colorCovered = Color.green;              // covered

    [Header("Exclusions")]
    public string excludeTag = "UEViz"; // UE 마커(시각화)는 색 변경에서 제외

    [Header("Auto Coverage")]
    public bool setCoveredOnReceive = true;      // 수신 이벤트에 반응하여 covered 처리
    public bool findReceiverInHierarchy = true;  // 부모/자식에서 RadioReceiver 자동 탐색

    Renderer[] rends;
    RadioReceiver rr; // ⬅ UE(=건물)에 붙은 수신기 참조

    void Awake()
    {
        EnsureInit();
        AutoFindReceiver();
    }

    void OnEnable()
    {
        if (setCoveredOnReceive && rr != null)
            rr.OnReceive += HandleOnReceive; // (srcId, payload, sinrDb)
    }

    void OnDisable()
    {
        if (rr != null)
            rr.OnReceive -= HandleOnReceive;
    }

    void Start()
    {
        // 초기 demand 자동 세팅
        if (demand == 0 && kind == AreaKind.Building) demand = Random.Range(10, 51);
        if (demand == 0 && kind == AreaKind.Road)     demand = Random.Range(0, 31);
        ApplyColor();
    }

    void EnsureInit()
    {
        if (rends == null || rends.Length == 0)
            rends = GetComponentsInChildren<Renderer>(true);
    }

    void AutoFindReceiver()
    {
        if (!findReceiverInHierarchy) return;

        rr = GetComponent<RadioReceiver>();
        if (!rr) rr = GetComponentInParent<RadioReceiver>();
        if (!rr) rr = GetComponentInChildren<RadioReceiver>();
    }

    // RadioReceiver가 SINR 임계치(예: 0 dB) 이상일 때만 OnReceive를 발행하므로,
    // 여기서는 단순히 covered 처리만 하면 된다.
    void HandleOnReceive(int srcId, byte[] payload, float sinrDb)
    {
        SetCovered(true);
    }

    public void RandomizeDemand()
    {
        demand = (kind == AreaKind.Building)
            ? Random.Range(10, 51)   // [10,50]
            : Random.Range(0, 31);   // [0,30]
    }

    public void SetCovered(bool on)
    {
        covered = on;
        ApplyColor();
    }

    public void ApplyColor()
    {
        EnsureInit();
        if (rends == null || rends.Length == 0) return;

        foreach (var r in rends)
        {
            if (!r) continue;

            // UEViz 태그는 건드리지 않음(UE 마커 시각화 보호)
            if (!string.IsNullOrEmpty(excludeTag) && r.CompareTag(excludeTag)) continue;

            var mat = r.sharedMaterial;
            if (!mat) continue;

            Color c = covered
                ? colorCovered
                : (demand <= 14) ? colorLow : (demand <= 30) ? colorMid : colorHigh;

            var mpb = new MaterialPropertyBlock();
            r.GetPropertyBlock(mpb);
            if (mat.HasProperty("_BaseColor")) mpb.SetColor("_BaseColor", c);
            else if (mat.HasProperty("_Color")) mpb.SetColor("_Color", c);
            else continue;

            r.SetPropertyBlock(mpb);
        }
    }

#if UNITY_EDITOR
    void OnValidate()
    {
        EnsureInit();
        if (rr == null) AutoFindReceiver();
        ApplyColor(); // 인스펙터 값 변경 시 즉시 반영
    }
#endif
}
