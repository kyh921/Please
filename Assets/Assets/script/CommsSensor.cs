using System.Collections.Generic;
using UnityEngine;

[DefaultExecutionOrder(+200)]
[DisallowMultipleComponent]
[RequireComponent(typeof(DroneAgent))]
public class CommsSensor : MonoBehaviour
{
    [Header("Scan")]
    [Tooltip("Read UE aggregates every N seconds")]
    public float scanInterval = 0.20f;

    [Header("Read-only (debug)")]
    public int linkableCount;        // count of UEs linked to THIS drone in this step
    public int maxOverlapThisFrame;  // max overconnect among UEs (change to sum if desired)
    public float sumQoEThisFrame;      // sum of per-UE aggregated metric*weight for THIS drone

    DroneAgent _agent;
    int _droneId;
    float _t;

    // UE cache (refresh once per frame)
    static readonly List<RadioReceiver> _ues = new List<RadioReceiver>();
    static int _cacheFrame = -1;

    public int linkableCountCached => linkableCount;

    void Awake()
    {
        _agent = GetComponent<DroneAgent>();
        _droneId = gameObject.GetInstanceID(); // RadioLinkModel must use the same ID
    }
   
    public bool debugLog = false;

    void Update()
    {
        _t += Time.deltaTime;
        if (_t < scanInterval) return;
        _t = 0f;

        CacheUEs();

        // reset per-step accumulators on the agent
        _agent.BeginStepAggregation();

        linkableCount = 0;
        maxOverlapThisFrame = 0;
        sumQoEThisFrame = 0f;

        // pop THIS drone's bucket from each UE and aggregate
        for (int i = 0; i < _ues.Count; i++)
        {
            var rx = _ues[i];
            if (!rx) continue;

            rx.PopQoeAndOverlapFor(_droneId, out float wq, out int ov);

            if (wq > 0f)
            {
                sumQoEThisFrame += wq;
                linkableCount++;
            }
            if (ov > maxOverlapThisFrame) maxOverlapThisFrame = ov;
        }

        // report the two numbers to the agent
        //_agent.ReportQoEAndOverlap(sumQoEThisFrame, maxOverlapThisFrame);
        // Note: ResetAggregation is optional because Pop removes the per-drone key.
        // If you prefer to clear all UE state at frame end, call ResetAggregation() from a manager.

        if (debugLog) Debug.Log($"[Pull] src={_droneId}  ¥Ò(weightedQoE)={sumQoEThisFrame:F3},  maxOverlap={maxOverlapThisFrame}");
        _agent.ReportQoEAndOverlap(sumQoEThisFrame, maxOverlapThisFrame);
    }

    static void CacheUEs()
    {
        if (_cacheFrame == Time.frameCount) return;
        _cacheFrame = Time.frameCount;

        _ues.Clear();
        _ues.AddRange(Object.FindObjectsOfType<RadioReceiver>(true));
    }
}
