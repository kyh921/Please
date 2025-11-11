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
        // 1) 그룹 보상: cov만 사용
        float cov = DroneAgent.ComputeCoverageRewardForScene();
        float groupR = cov;
        group.AddGroupReward(groupR);

        // 2) 생존 상태 확인 및 조기 종료
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

            // (선택) 간단 로그
            if (Time.frameCount % 60 == 0)
                Debug.Log($"[Team] early-terminated: a death occurred. cov={cov:F3} alive={alive}/{N}");
            return; // 이번 스텝의 나머지 계산 생략
        }
        lastAliveCount = alive;

        // 3) 일반 종료 조건
        groupStep++;
        if ((groupMaxSteps > 0 && groupStep >= groupMaxSteps) ||
            (endWhenAllEliminated && alive == 0))
        {
            group.EndGroupEpisode();
            groupStep = 0;
            lastAliveCount = -1;
        }

        // (선택) 1초마다 상태 로그
        if (Time.frameCount % 60 == 0)
        {
            Debug.Log($"[Team] step={groupStep} cov={cov:F3} alive={alive}/{N}");
        }
    }
}
