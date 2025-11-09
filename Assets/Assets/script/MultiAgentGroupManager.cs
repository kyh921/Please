using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
using System.Collections.Generic;

public class MultiAgentGroupManager : MonoBehaviour
{
    public static MultiAgentGroupManager Instance { get; private set; }
    public SimpleMultiAgentGroup Group { get; private set; }

    private readonly List<DroneAgent> _agents = new();

    void Awake(){
        if (Instance != null && Instance != this){ Destroy(gameObject); return; }
        Instance = this;
        Group = new SimpleMultiAgentGroup();
    }

    public void Register(DroneAgent a){
        if (!_agents.Contains(a)){ _agents.Add(a); Group.RegisterAgent(a); }
    }
    public void Unregister(DroneAgent a){
        if (_agents.Remove(a)){ Group.UnregisterAgent(a); }
    }

    public void EndEpisodeAllSuccess(){ Group.EndGroupEpisode(); }
    public void EndEpisodeAllInterrupted(){ Group.GroupEpisodeInterrupted(); }
}
