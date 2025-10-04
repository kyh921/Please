using UnityEngine;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;


[DefaultExecutionOrder(+300)]
public class GroupRewardManager : MonoBehaviour
{
    public float updateInterval = 0.20f;
    [SerializeField] private bool debugGroupReward = true;

    private float timer = 0f;
    private List<DroneAgent> agents = new List<DroneAgent>();
    private SimpleMultiAgentGroup mGroup;  // ← 네임스페이스 수정

    void Awake()
    {
        agents.Clear();
        agents.AddRange(FindObjectsOfType<DroneAgent>());
        mGroup = new SimpleMultiAgentGroup();  // ← 생성자 동일
        foreach (var a in agents) mGroup.RegisterAgent(a);
    }

    void Update()
    {
        // Lazy-init 방어 (Awake 실패/비활성 복구)
        if (mGroup == null)
        {
            mGroup = new SimpleMultiAgentGroup();
            if (agents == null) agents = new List<DroneAgent>();
            if (agents.Count == 0) agents.AddRange(FindObjectsOfType<DroneAgent>());
            foreach (var a in agents) if (a != null) mGroup.RegisterAgent(a);
            if (debugGroupReward) Debug.Log("[GroupManager] Lazy-initialized SimpleMultiAgentGroup");
        }
        Debug.Log("[GroupManager] Update tick");
        timer += Time.deltaTime;
        if (timer < updateInterval) return;
        timer = 0f;
        Debug.Log("[GroupManager] Passed interval gate");
        // null 정리 + 필요 시 재캐시
        agents.RemoveAll(a => a == null);
        if (agents.Count == 0)
        {
            agents.AddRange(FindObjectsOfType<DroneAgent>());
            foreach (var a in agents) if (a != null) mGroup.RegisterAgent(a);
            if (debugGroupReward) Debug.Log($"[GroupManager] Re-cached agents: {agents.Count}");
        }

        float totalOverlap = 0f;
        int n = 0;
        foreach (var a in agents)
        {
            if (a == null) continue;
            int ov = a.GetOverlap();
            totalOverlap += Mathf.Max(0, ov);
            n++;
            if (debugGroupReward) Debug.Log($"[GroupManager Debug] {a.name} overlap={ov}");
        }

        float avgOverlap = (n > 0) ? totalOverlap / n : 0f;
        float groupCovReward = 1f / (1f + avgOverlap);
        mGroup.AddGroupReward(groupCovReward);

        if (debugGroupReward)
            Debug.Log($"[GroupManager] AvgOverlap={avgOverlap:F2}, GroupCovReward={groupCovReward:F4}, Agents={n}");
    }

    // 선택: 팀 에피소드 종료가 필요할 때 호출
    public void EndGroupEpisode()
    {
        mGroup.EndGroupEpisode();
    }

    // 선택: 드론 동적 추가/삭제 시 갱신
    public void RefreshAgents()
    {
        foreach (var a in agents) mGroup.UnregisterAgent(a);
        agents.Clear();
        agents.AddRange(FindObjectsOfType<DroneAgent>());
        foreach (var a in agents) mGroup.RegisterAgent(a);

        if (debugGroupReward)
            Debug.Log($"[GroupManager] Refreshed and registered {agents.Count} agents.");
    }
}
