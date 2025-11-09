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
        // --- 공통 보상: cov 한 번만 더함 ---
        float cov = DroneAgent.ComputeCoverageRewardForScene();
        group.AddGroupReward(cov);

        groupStep++;

        if ((groupMaxSteps > 0 && groupStep >= groupMaxSteps) ||
            (endWhenAllEliminated && AreAllEliminated()))
        {
            group.EndGroupEpisode();   // 모든 에이전트 에피소드 동시 종료
            groupStep = 0;             // 다음 에피소드 카운터 초기화
        }
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
