using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;

public class DroneTeamManager : MonoBehaviour
{
    public static DroneTeamManager Instance { get; private set; }

    [Header("Elimination policy")]
    [Tooltip("사망 시 팀 전체에 소액 패널티를 줄지 여부")]
    public bool groupPenalizeOnElim = true;

    [Tooltip("사망 시 팀 전체에 부여할 소액 그룹 패널티 (살아있는 에이전트 모두에게 동일 적용)")]
    public float groupElimPenalty = -1f;   // 너무 크게 주지 말 것

    [Tooltip("한 명이라도 사망하면 즉시 전체 에피소드를 종료할지 여부 (자살 전략 방지를 위해 false 권장)")]
    public bool endOnAnyElimination = false; // ★ 변경: true -> false

    [Space(8)]
    [Tooltip("배터리 소진으로 사망한 에이전트에 부여할 추가 개별 패널티")]
    public float penaltyBattery = -100f; // ★ 변경: -10f -> -20f

    [Tooltip("충돌로 사망한 에이전트에 부여할 추가 개별 패널티")]
    public float penaltyCollision = -100f; // ★ 변경: -8f -> -20f

    [Tooltip("경계 위반으로 사망한 에이전트에 부여할 추가 개별 패널티")]
    public float penaltyBoundary  = -100f; // ★ 변경: -8f -> -20f

    [Tooltip("그 외 사유의 기본 개별 패널티")]
    public float penaltyDefault   = -100f; // (유지 또는 -20f로 통일)

    [Header("Agents")]
    [Tooltip("씬에서 찾거나 수동 할당하세요.")]
    public List<DroneAgent> agents = new List<DroneAgent>();

    [Header("Group episode")]
    [Tooltip("팀 에피소드 최대 스텝(0이면 무제한)")]
    public int groupMaxSteps = 20000;

    [Tooltip("모두 탈락하면 종료할지 여부")]
    public bool endWhenAllEliminated = true;

    private SimpleMultiAgentGroup group;
    private int groupStep;

    [Header("Debug")]
    public bool debugGroupReward = false;

    // ===== ★ 추가: 에이전트 관찰용 글로벌 상태 캐시 =====
    /// <summary>
    /// 현재 스텝의 팀 전체 커버리지 (0~1)
    /// </summary>
    public float CurrentGlobalCoverage { get; private set; } = 0f;
    /// <summary>
    /// 현재 스텝의 팀 생존율 (0~1)
    /// </summary>
    public float CurrentSurvivalRatio { get; private set; } = 0f;


    private void Awake()
    {
        // 싱글턴 및 에이전트 목록 준비
        Instance = this;

        if (agents == null || agents.Count == 0)
        {
            agents = new List<DroneAgent>(FindObjectsOfType<DroneAgent>(true));
        }
    }

    private void Start()
    {
        group = new SimpleMultiAgentGroup();

        // 그룹 에피소드 모드로 고정하고 그룹에 등록
        foreach (var a in agents)
        {
            if (a == null) continue;
            a.useGroupEpisodes = true;   // 개별 에피소드 종료 비활성화
            group.RegisterAgent(a);
        }

        groupStep = 0;

        // ===== agentIndex 할당 =====
        for (int i = 0; i < agents.Count; i++)
        {
            if (agents[i] != null)
                agents[i].agentIndex = i;   // 0 ~ 4 매핑
        }
    }

    private void FixedUpdate()
    {
        // 1) ★ 글로벌 상태 계산 (에이전트 관찰용) - 1회만 수행
        int total = 0;
        int alive = 0;
        foreach (var a in agents)
        {
            if (a == null) continue;
            total++;
            if (!a.IsEliminated) alive++;
        }
        CurrentSurvivalRatio = (total > 0) ? ((float)alive / total) : 0f;
        // ★ 원본 cov 계산
        float rawCov = DroneAgent.ComputeCoverageRewardForScene();

        // ★★★ 해결책 1: 생존율의 "세제곱"을 곱하여 희생 전략을 강력하게 처벌 ★★★
        float survivalPenalty = CurrentSurvivalRatio * CurrentSurvivalRatio * CurrentSurvivalRatio;
        CurrentGlobalCoverage = rawCov * survivalPenalty;

        // 2) 그룹 보상: cov만 사용
        group.AddGroupReward(CurrentGlobalCoverage);

        groupStep++;

        // 모두 사망 종료
        if (endWhenAllEliminated && alive == 0)
        {
            if (group != null)
            {
                group.EndGroupEpisode();
            }
            groupStep = 0;
            return;
        }

        // 길이 초과 종료
        if (groupMaxSteps > 0 && groupStep >= groupMaxSteps)
        {
            if (group != null)
            {
                group.EndGroupEpisode();
            }
            groupStep = 0;
            return;
        }

        // (선택) 상태 로그
        if (debugGroupReward && Time.frameCount % 60 == 0)
        {
            Debug.Log($"[Team] step={groupStep} cov={CurrentGlobalCoverage:F3} alive={alive}/{total}");
        }
    }

    /// <summary>
    /// 다음 팀 에피소드를 위해 모든 에이전트의 탈락 상태/물리 상태/스폰 위치를 초기화한다.
    /// (이 함수는 ML-Agents가 EndGroupEpisode() 호출 시 자동으로 호출합니다. - v2.0+ 기준)
    /// </summary>
    private void ResetAgentsForNextEpisode()
    {
        // (참고: ML-Agents v2.0+ 에서는 에이전트의 OnEpisodeBegin()이
        // group.EndGroupEpisode() 시점에 자동으로 호출됩니다.
        // RecoverFromElimination()이 OnEpisodeBegin()에 있으므로
        // 이 함수에서 별도 리셋 코드가 필요 없을 수 있습니다.)
    }

    /// <summary>
    /// 에이전트가 사망했음을 통지.
    /// 여기서 (1) 개별 패널티, (2) 팀 패널티, (3) 필요 시 전체 종료를 처리한다.
    /// </summary>
    public void NotifyAgentEliminated(DroneAgent agent, string reason)
    {
        if (agent == null) return;

        // (1) 개별 패널티
        float indiv = penaltyDefault;
        switch (reason)
        {
            case "battery":
                indiv = penaltyBattery;
                break;
            case "collision":
                indiv = penaltyCollision;
                break;
            case "boundary":
                indiv = penaltyBoundary;
                break;
            default:
                indiv = penaltyDefault;
                break;
        }

        if (indiv != 0f)
        {
            agent.AddReward(indiv);
        }

        // (2) 팀(그룹) 패널티: 소액으로 모든 '살아있는' 에이전트에 동일 적용
        if (groupPenalizeOnElim && groupElimPenalty != 0f)
        {
            foreach (var a in agents)
            {
                if (a != null && !a.IsEliminated)
                {
                    a.AddReward(groupElimPenalty);
                }
            }
        }

        // (3) 전체 종료 옵션: 한 명이라도 사망하면 즉시 그룹 에피소드 종료
        // ★★★ 'false'로 변경했으므로 이 로직은 거의 호출되지 않음 ★★★
        if (endOnAnyElimination && group != null)
        {
            group.EndGroupEpisode();
            groupStep = 0;
        }
    }
}