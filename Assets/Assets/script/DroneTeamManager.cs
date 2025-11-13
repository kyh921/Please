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
    public float groupElimPenalty = -0.1f;   // 너무 크게 주지 말 것

    [Tooltip("한 명이라도 사망하면 즉시 전체 에피소드를 종료할지 여부")]
    public bool endOnAnyElimination = false;

    [Space(8)]
    [Tooltip("배터리 소진으로 사망한 에이전트에 부여할 추가 개별 패널티")]
    public float penaltyBattery = -1.5f;

    [Tooltip("충돌로 사망한 에이전트에 부여할 추가 개별 패널티 (충돌 시 Agent에서 이미 1회 감점하므로 기본 0)")]
    public float penaltyCollision = 0f;

    [Tooltip("경계 위반으로 사망한 에이전트에 부여할 추가 개별 패널티 (경계 위반 시 Agent에서 이미 1회 감점하므로 기본 0)")]
    public float penaltyBoundary  = 0f;

    [Tooltip("그 외 사유의 기본 개별 패널티")]
    public float penaltyDefault   = -1.0f;

    [Tooltip("씬에서 찾거나 수동 할당하세요.")]
    public List<DroneAgent> agents = new List<DroneAgent>();

    [Header("Group episode")]
    [Tooltip("팀 에피소드 최대 스텝(0이면 무제한)")]
    public int groupMaxSteps = 20000;

    [Tooltip("모두 탈락하면 종료할지 여부")]
    public bool endWhenAllEliminated = true;

    private SimpleMultiAgentGroup group;
    private int groupStep;

    void Awake()
    {
        // 싱글턴 및 에이전트 목록 준비
        Instance = this;

        if (agents == null || agents.Count == 0)
            agents = new List<DroneAgent>(FindObjectsOfType<DroneAgent>(true));
    }

    void Start()
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
    }

    void FixedUpdate()
    {
        // 1) 그룹 보상: cov만 사용
        float cov = DroneAgent.ComputeCoverageRewardForScene();
        group.AddGroupReward(cov);

        // 2) 일반 종료 조건
        int N = 0, alive = 0;
        foreach (var a in agents)
        {
            if (a == null) continue;
            N++;
            if (!a.IsEliminated) alive++;
        }

        groupStep++;

        // 모두 사망 종료
        if (endWhenAllEliminated && alive == 0)
        {
            group.EndGroupEpisode();
            groupStep = 0;
            return;
        }

        // 길이 초과 종료
        if (groupMaxSteps > 0 && groupStep >= groupMaxSteps)
        {
            group.EndGroupEpisode();
            groupStep = 0;
            return;
        }

        // (선택) 상태 로그
        if (Time.frameCount % 60 == 0)
            Debug.Log($"[Team] step={groupStep} cov={cov:F3} alive={alive}/{N}");
    }

    /// <summary>
    /// 에이전트가 사망했음을 통지.
    /// 여기서 (1) 개별 패널티, (2) 팀 패널티, (3) 필요 시 전체 종료를 처리한다.
    /// </summary>
    public void NotifyAgentEliminated(DroneAgent agent, string reason)
    {
        if (agent == null) return;

        // (1) 개별 패널티: 충돌/경계는 Agent에서 1회 감점했으므로 기본 0, 배터리는 음수 권장
        float indiv = penaltyDefault;
        switch (reason)
        {
            case "battery":   indiv = penaltyBattery;   break;
            case "collision": indiv = penaltyCollision; break; // 중복 차감 방지
            case "boundary":  indiv = penaltyBoundary;  break; // 중복 차감 방지
            default:          indiv = penaltyDefault;   break;
        }
        if (indiv != 0f) agent.AddReward(indiv);

        // (2) 팀(그룹) 패널티: 소액으로 모든 '살아있는' 에이전트에 동일 적용
        if (groupPenalizeOnElim && groupElimPenalty != 0f)
        {
            foreach (var a in agents)
            {
                if (a != null && !a.IsEliminated)
                    a.AddReward(groupElimPenalty);
            }
        }

        // (3) 전체 종료 옵션: 한 명이라도 사망하면 즉시 그룹 에피소드 종료
        if (endOnAnyElimination && group != null)
        {
            group.EndGroupEpisode();
            groupStep = 0;
        }
    }
}
