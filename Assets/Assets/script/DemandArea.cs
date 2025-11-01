// DemandArea.cs
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
    [Tooltip("이 태그를 가진 Renderer는 색 변경에서 제외")]
    public string excludeTag = "UEViz";

    [Header("Auto Coverage")]
    [Tooltip("수신 이벤트가 오면 즉시 covered 처리")]
    public bool setCoveredOnReceive = true;

    [Tooltip("부모/자식/자기 자신에서 RadioReceiver를 자동 탐색")]
    public bool findReceiverInHierarchy = true;

    [Header("Auto Uncover")]
    [Tooltip("이 시간(초) 동안 추가 수신이 없으면 covered 해제(원래 색으로 복귀)")]
    public float loseCoverAfter = 0.5f;

    Renderer[] rends;
    RadioReceiver rr;

    // 마지막 임계 이상 수신 시각(Time.time)
    float _lastReceiveTime = -1f;

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
        // 초기 demand 자동 셋업
        if (demand == 0 && kind == AreaKind.Building) demand = Random.Range(10, 51);
        if (demand == 0 && kind == AreaKind.Road)     demand = Random.Range(0, 31);
        ApplyColor();
    }

    void Update()
    {
        // 수신 기반으로 covered 유지하는 경우에만 자동 해제 수행
        if (!setCoveredOnReceive) return;

        if (covered && loseCoverAfter > 0f)
        {
            // 아직 수신한 적이 없거나, 일정 시간 동안 추가 수신이 없으면 covered 해제
            if (_lastReceiveTime < 0f || (Time.time - _lastReceiveTime) > loseCoverAfter)
            {
                SetCovered(false); // demand 기반 색으로 복귀
            }
        }
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

    // RadioReceiver에서 SINR 임계 이상일 때만 발생하는 OnReceive를 받아 covered=true 처리
    void HandleOnReceive(int srcId, byte[] payload, float sinrDb)
    {
        _lastReceiveTime = Time.time;
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

            // excludeTag를 가진 Renderer는 색 변경 제외(UE 아이콘 등)
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
