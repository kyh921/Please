// DemandArea.cs
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public enum AreaKind { Building, Road }

[DisallowMultipleComponent]
public class DemandArea : MonoBehaviour
{
    // === Global uniform controller state (scene-wide) ===
    static readonly HashSet<DemandArea> s_all = new HashSet<DemandArea>();
    static bool s_useUniform = false;
    static int  s_uniformValue = 10;

    [Header("Type & Demand")]
    public AreaKind kind = AreaKind.Building;
    [Tooltip("UE의 가중치")]
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

    // === Global uniform override (scene-wide switch) ===
    [Header("Uniform Demand (Global Override)")]
    [Tooltip("씬에서 단 하나만 체크하세요. 체크 시 모든 UE demand를 동일값으로 강제합니다.")]
    public bool enableGlobalUniformDemandController = false;

    [Tooltip("모든 UE에 적용할 공통 demand 값")]
    public int uniformDemandForAll = 10;

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
        s_all.Add(this);

        if (setCoveredOnReceive && rr != null)
            rr.OnReceive += HandleOnReceive; // (srcId, payload, sinrDb)

        // 컨트롤러로 지정된 인스턴스가 켜질 때 전역 상태 갱신 + 즉시 반영
        if (enableGlobalUniformDemandController)
        {
            s_useUniform = true;
            s_uniformValue = Mathf.Max(0, uniformDemandForAll);
            ApplyUniformToAll();
        }
    }

    void OnDisable()
    {
        if (rr != null)
            rr.OnReceive -= HandleOnReceive;

        s_all.Remove(this);

        // 컨트롤러가 꺼지면 전역 강제 해제(값은 유지되지만 이후부터는 강제 적용 안 함)
        if (enableGlobalUniformDemandController)
        {
            s_useUniform = false;
        }
    }

    void Start()
    {
        // 초기 demand 자동 셋업 (기존 동작 유지)
        if (demand == 0 && kind == AreaKind.Building) demand = Random.Range(10, 51);
        if (demand == 0 && kind == AreaKind.Road)     demand = Random.Range(0, 31);

        // 전역 uniform이 켜져 있으면 덮어쓰기
        if (s_useUniform)
            demand = s_uniformValue;

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

    // === 기존 API: 랜덤 설정 (전역 uniform이 켜져 있으면 무시하고 동일값 적용) ===
    public void RandomizeDemand()
    {
        if (s_useUniform)
        {
            demand = s_uniformValue;
        }
        else
        {
            demand = (kind == AreaKind.Building)
                ? Random.Range(10, 51)   // [10,50]
                : Random.Range(0, 31);   // [0,30]
        }
        ApplyColor();
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

        // 전역 uniform이 켜져 있으면 항상 강제
        int shownDemand = s_useUniform ? s_uniformValue : demand;

        foreach (var r in rends)
        {
            if (!r) continue;

            // excludeTag를 가진 Renderer는 색 변경 제외(UE 아이콘 등)
            if (!string.IsNullOrEmpty(excludeTag) && r.CompareTag(excludeTag)) continue;

            var mat = r.sharedMaterial;
            if (!mat) continue;

            Color c = covered
                ? colorCovered
                : (shownDemand <= 14) ? colorLow : (shownDemand <= 30) ? colorMid : colorHigh;

            var mpb = new MaterialPropertyBlock();
            r.GetPropertyBlock(mpb);
            if (mat.HasProperty("_BaseColor")) mpb.SetColor("_BaseColor", c);
            else if (mat.HasProperty("_Color")) mpb.SetColor("_Color", c);
            else continue;

            r.SetPropertyBlock(mpb);
        }
    }

    // === Scene-wide helper ===
    void ApplyUniformToAll()
    {
        foreach (var da in s_all)
        {
            if (!da) continue;
            da.demand = s_uniformValue;
            da.ApplyColor();
        }
    }

#if UNITY_EDITOR
    void OnValidate()
    {
        EnsureInit();
        if (rr == null) AutoFindReceiver();

        // 에디터에서 값 바꿀 때도 즉시 반영
        if (enableGlobalUniformDemandController)
        {
            s_useUniform = true;
            s_uniformValue = Mathf.Max(0, uniformDemandForAll);
            ApplyUniformToAll();
        }

        ApplyColor(); // 인스펙터 값 변경 시 즉시 반영
    }
#endif
}
