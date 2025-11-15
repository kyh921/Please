using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;

public class DroneTeamManager : MonoBehaviour
{
    [Tooltip("씬에서 찾거나 수동 할당하세요.")]
    public List<DroneAgent> agents = new List<DroneAgent>();

    [Header("Group episode")]
    public int groupMaxSteps = 20000;
    public bool endWhenAllEliminated = true;

    [Header("Early termination")]
    public bool endOnAnyElimination = true;

    private SimpleMultiAgentGroup group;
    private int groupStep;
    private int lastAliveCount = -1;

    // ---- 그룹 보상 램프업 설정 ----
    [Header("Team Reward Ramp-Up")]
    public long startTeamRewardStep = 300000;  // 그룹보상 시작
    public long fullRewardStep = 600000;       // 그룹보상 1.0 되는 시점

    private long globalStep = 0;               // 전체 스텝 카운터

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
            a.useGroupEpisodes = true;
            group.RegisterAgent(a);
        }

        groupStep = 0;
    }

    void FixedUpdate()
    {
        globalStep++;

        // ----- 1) 그룹 보상 weight 계산 -----
        float weight = 0f;

        if (globalStep >= startTeamRewardStep)
        {
            if (globalStep >= fullRewardStep)
                weight = 1f;
            else
            {
                float t = (float)(globalStep - startTeamRewardStep) /
                          (float)(fullRewardStep - startTeamRewardStep);
                weight = Mathf.Clamp01(t);
            }
        }

        // ----- 2) 그룹 보상 계산 및 적용 -----
        float cov = DroneAgent.ComputeCoverageRewardForScene();
        float groupR = cov * weight;

        group.AddGroupReward(groupR);

        // ----- 3) 생존 체크 -----
        int N = 0, alive = 0;
        foreach (var a in agents)
        {
            if (a == null) continue;
            N++;
            if (!a.IsEliminated) alive++;
        }

        if (lastAliveCount >= 0 && alive < lastAliveCount && endOnAnyElimination)
        {
            group.EndGroupEpisode();
            groupStep = 0;
            lastAliveCount = -1;

            if (Time.frameCount % 60 == 0)
                Debug.Log($"[Team] early-terminated: death. cov={cov:F3}, weight={weight:F2}");
            return;
        }
        lastAliveCount = alive;

        // ----- 4) 일반 종료 -----
        groupStep++;
        if ((groupMaxSteps > 0 && groupStep >= groupMaxSteps) ||
            (endWhenAllEliminated && alive == 0))
        {
            group.EndGroupEpisode();
            groupStep = 0;
            lastAliveCount = -1;
        }

        // ----- 5) 디버그 -----
        if (Time.frameCount % 60 == 0)
        {
            Debug.Log($"[Team] step={groupStep} cov={cov:F3} weight={weight:F2} alive={alive}/{N}");
        }
    }
}