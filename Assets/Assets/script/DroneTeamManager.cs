using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;

public class DroneTeamManager : MonoBehaviour
{
    [Tooltip("씬에서 찾거나 수동 할당하세요.")]
    public List<DroneAgent> agents = new List<DroneAgent>();

    [Header("Group episode")]
    public int groupMaxSteps = 20000;      // 팀 에피소드 길이
    public bool endWhenAllEliminated = true;

    // 한 명이라도 탈락 시 즉시 종료
    [Header("Early termination")]
    public bool endOnAnyElimination = true;

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
    }

    void FixedUpdate()
    {
        // 1) (선택) 팀 커버리지에 대한 '양의' 그룹 보상은 유지
        float covTeam = DroneAgent.ComputeCoverageRewardForScene();
        float groupR = covTeam / Mathf.Max(1, groupMaxSteps);
        group.AddGroupReward(groupR);

        // 2) 생존 상태 확인
        int alive = 0;
        foreach (var a in agents)
            if (a != null && !a.IsEliminated) alive++;

        // ★ 3) 한 명이라도 줄어들었고, endOnAnyElimination이면 "패널티 없이" 즉시 종료
        if (lastAliveCount >= 0 && alive < lastAliveCount && endOnAnyElimination)
        {
            // 팀 패널티/개별 패널티 절대 부여하지 않음
            group.EndGroupEpisode();
            groupStep = 0;
            lastAliveCount = -1;
            return;
        }
        lastAliveCount = alive;

        // 4) 일반 종료
        groupStep++;
        if ((groupMaxSteps > 0 && groupStep >= groupMaxSteps) || (endWhenAllEliminated && alive == 0))
        {
            group.EndGroupEpisode();
            groupStep = 0;
            lastAliveCount = -1;
        }
    }
}
