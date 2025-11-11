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

    private SimpleMultiAgentGroup group;
    private int groupStep;

    // ▼ 추가: 직전 생존 수 추적
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
    }

    void FixedUpdate()
    {
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

        // 3) 사망 이벤트 감지 → 즉시 큰 패널티
        if (lastAliveCount >= 0 && alive < lastAliveCount)
        {
            // 방해 자살형 정책 방지용 원샷 패널티
            group.AddGroupReward(-0.5f * (lastAliveCount - alive));
        }
        lastAliveCount = alive;

        // 4) 그룹 보상 구성
        //    - cov에 생존 가중 적용
        //    - 생존 유지 보상(소량) 추가
        float groupR = (cov * surv) + 0.1f * surv;
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

        // 선택: 한 명이라도 탈락하면 즉시 종료하고 싶다면 아래 라인 사용
        // if (alive < N) { group.EndGroupEpisode(); groupStep = 0; lastAliveCount = -1; }
    }

    bool AreAllEliminated()
    {
        bool any = false;
        foreach (var a in agents)
        {
            if (a == null) continue;
            any = true;
            if (!a.IsEliminated) return false;
        }
        return any; // 하나라도 있었다면 모두 탈락
    }
}
