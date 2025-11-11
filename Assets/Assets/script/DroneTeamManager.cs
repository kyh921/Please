using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;

public class DroneTeamManager : MonoBehaviour
{
    [Tooltip("씬에서 찾거나 수동 할당하세요.")]
    public List<DroneAgent> agents = new List<DroneAgent>();

    [Header("Group episode")]
    public int groupMaxSteps = 8000;     // 전체 팀 에피소드 길이
    public bool endWhenAllEliminated = true;

    // ===== λ 스케줄(개별→공통 혼합) =====
    [Header("Reward mix (λ schedule)")]
    [Range(0f, 1f)] public float lambdaStart = 0f;     // 시작 λ
    [Range(0f, 1f)] public float lambdaMid   = 0.55f;  // 목표 λ (0.5~0.6 권장)
    public int lambdaWarmupSteps = 600_000;            // 느린 워밍업

    public static float Lambda { get; private set; } = 0f;
    private int globalSteps = 0;

    private SimpleMultiAgentGroup group;
    private int groupStep;
    private int lastAliveCount = -1;

    void Awake()
    {
        if (agents == null || agents.Count == 0)
            agents = new List<DroneAgent>(FindObjectsOfType<DroneAgent>(true));
    }

    void Start()
    {
        group = new SimpleMultiAgentGroup();

        foreach (var a in agents)
        {
            if (a == null) continue;
            a.useGroupEpisodes = true;   // 개별 에피소드 종료 비활성화
            group.RegisterAgent(a);
        }

        groupStep = 0;
        Lambda = Mathf.Clamp01(lambdaStart);
    }

    void FixedUpdate()
    {
        // --- λ 스케줄 업데이트 (선형) ---
        if (lambdaWarmupSteps > 0 && Lambda < lambdaMid)
        {
            float t = Mathf.Clamp01((float)globalSteps / lambdaWarmupSteps);
            Lambda = Mathf.Lerp(lambdaStart, lambdaMid, t);
        }
        globalSteps++;

        // 1) 공통 보상: cov 계산
        float cov = DroneAgent.ComputeCoverageRewardForScene();

        // 2) 생존율 계산
        int N = 0, alive = 0;
        foreach (var a in agents)
        {
            if (a == null) continue;
            N++;
            if (!a.IsEliminated) alive++;
        }
        float surv = (N > 0) ? (float)alive / N : 0f;

        // --- 생존 기반 가드: 생존율이 떨어지면 λ를 즉시 낮은 값으로 캡 ---
        if (surv < 0.90f)
            Lambda = Mathf.Min(Lambda, 0.40f);

        // 3) 사망 이벤트 감지 → 초반/다수 사망일수록 더 큰 패널티
        if (lastAliveCount >= 0 && alive < lastAliveCount)
        {
            int deaths = lastAliveCount - alive;
            float tRemain = (groupMaxSteps > 0) ? (groupMaxSteps - groupStep) / (float)groupMaxSteps : 1f;
            float popFactor = (N > 0) ? (float)lastAliveCount / N : 1f;
            float deathPenalty = -1.0f * deaths * (0.5f + 0.5f * tRemain) * (0.5f + 0.5f * popFactor);
            group.AddGroupReward(deathPenalty);
        }
        lastAliveCount = alive;

        // 4) 그룹 보상 구성 (생존 가중 + 생존 유지 보상)
        float groupR = Lambda * ((cov * surv) + 0.1f * surv);
        group.AddGroupReward(groupR);

        groupStep++;

        // 5) 종료 조건
        if ((groupMaxSteps > 0 && groupStep >= groupMaxSteps) ||
            (endWhenAllEliminated && alive == 0))
        {
            group.EndGroupEpisode();
            groupStep = 0;
            lastAliveCount = -1;
        }

        // --- 1초마다 상태 로그 ---
        if (Time.frameCount % 60 == 0)
        {
            Debug.Log($"[Team] step={globalSteps} λ={Lambda:F2} cov={cov:F3} surv={surv:F2} groupR={groupR:F3} alive={alive}/{N}");
        }
    }
}
