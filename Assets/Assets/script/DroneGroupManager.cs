using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;

public class DroneGroupManager : MonoBehaviour
{
    public List<DroneAgent> agents = new List<DroneAgent>();
    private SimpleMultiAgentGroup group;

    void Awake()
    {
        group = new SimpleMultiAgentGroup();
        foreach (var a in agents)
        {
            if (a == null) continue;
            group.RegisterAgent(a);
            a.Manager = this;       // 에이전트에서 매니저 참조
        }
    }

    // === 그룹 보상/종료 API ===
    public void AddSharedReward(float r) => group.AddGroupReward(r);
    public void EndGroupEpisode()        => group.EndGroupEpisode();

    // (선택) 런타임 동적 등록/해제용
    public void Register(DroneAgent a)   { group.RegisterAgent(a); a.Manager = this; }
}
