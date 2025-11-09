using UnityEngine;
using System.Collections.Generic;
using Unity.MLAgents;

public class CoverageManager : MonoBehaviour
{
    public static CoverageManager Instance { get; private set; }

    [Range(0f, 1f)]
    [SerializeField] private float currentCov = 1f;   // 인스펙터 확인용
    public float CurrentCov => currentCov;            // 읽기 전용

    private readonly List<RadioReceiver> _allReceivers = new();

    private void Awake()
    {
        if (Instance != null && Instance != this) { Destroy(gameObject); return; }
        Instance = this;

        if (RadioReceiver.All != null)
            _allReceivers.AddRange(RadioReceiver.All);
    }

    private void LateUpdate()
    {
        float totalDemand = 0f, coveredDemand = 0f;
        float sumOver = 0f; int overCount = 0;

        var list = (RadioReceiver.All != null && RadioReceiver.All.Count > 0) ? RadioReceiver.All : _allReceivers;

        foreach (var rr in list)
        {
            if (rr == null) continue;
            var area = rr.GetComponentInParent<DemandArea>();
            if (area == null || area.kind != AreaKind.Building) continue;

            int demand = Mathf.Max(0, area.demand);
            totalDemand += demand;

            int k = rr.ConnectedSourceCount;     // 이 UE를 덮는 드론 수
            if (k > 0) coveredDemand += demand;

            int over = Mathf.Max(0, k - 1);
            if (over > 0) { sumOver += over; overCount++; }
        }

        float tau = (totalDemand > 0f) ? (coveredDemand / totalDemand) : 0f;
        float w   = (overCount   > 0  ) ? (sumOver / overCount)        : 0f;

        // 논문식 변형: cov = (2τ) / (1 + 0.5ω)
        currentCov = Mathf.Clamp01((2f * tau) / (1f + 0.5f * w));

        // --- ML-Agents 통계 로깅(초기화 되었을 때만) ---
        if (Academy.IsInitialized)
        {
            var sr = Academy.Instance.StatsRecorder;
            sr.Add("coverage/cov",   currentCov);
            sr.Add("coverage/tau",   tau);
            sr.Add("coverage/omega", w);

            // MultiAgentGroupManager에 public int AgentCount => _agents.Count; 추가 후 사용
            var gm = MultiAgentGroupManager.Instance;
            if (gm != null) sr.Add("group/num_agents", gm.AgentCount);
        }
    }

    public void SetCovFromTauOmega(float tau, float omega)
    {
        float cov = (2f * tau) / (1f + 0.5f * Mathf.Max(0f, omega));
        currentCov = Mathf.Clamp01(cov);
    }

    public void SetCovDirect(float cov01)
    {
        currentCov = Mathf.Clamp01(cov01);
    }
}
