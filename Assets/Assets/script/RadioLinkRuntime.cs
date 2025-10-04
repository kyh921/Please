using System.Collections.Generic;
using UnityEngine;
[DefaultExecutionOrder(-200)]

[DisallowMultipleComponent]
public class RadioLinkRuntime : MonoBehaviour
{
    [Tooltip("링크 계산 주기(초)")]
    public float updateInterval = 0.10f;

    [Tooltip("계산 코어(기존 RadioLinkModel)를 씬에 하나 둔 뒤 여기로 할당)")]
    public RadioLinkModel model;

    float _t;

    static readonly List<DroneAgent> _txList = new List<DroneAgent>(16);
    static readonly List<RadioReceiver> _rxList = new List<RadioReceiver>(256);

    void Reset()
    {
        if (!model) model = GetComponent<RadioLinkModel>();
    }

    void Update()
    {
        _t += Time.deltaTime;
        if (_t < updateInterval) return;
        _t = 0f;

        if (!model) return;

        // 1) 씬에서 드론/UE 수집
        CollectSceneObjects(_txList, _rxList);
        int txCount = _txList.Count;
        int rxCount = _rxList.Count;
        if (txCount == 0 || rxCount == 0) return;

        // 2) 모델 입력 채우기
        EnsureCapacity(model, txCount, rxCount);

        model.txPositions.Clear(); model.txHeights.Clear();
        for (int i = 0; i < txCount; i++)
        {
            var tx = _txList[i].transform.position;
            model.txPositions.Add(tx);
            model.txHeights.Add(Mathf.Max(1f, tx.y));
        }

        model.rxPositions.Clear(); model.rxHeights.Clear();
        for (int j = 0; j < rxCount; j++)
        {
            var p = _rxList[j].GetAntennaPosition();
            model.rxPositions.Add(p);
            model.rxHeights.Add(Mathf.Max(1f, p.y));
        }

        // 3) 거리/각도 → 경로손실 → 수신전력 → SINR
        model.GetAllDistancesAndAngles(out var distances, out var angles);
        var lossDb = model.GetAllHataLosses();                // 그대로 사용
        var rxPwrDbm = model.GetAllRxPowers(angles, lossDb);
        var rxPwrMw = RadioLinkModel.ConvertRxPowersToMw(rxPwrDbm);
        var (_, sinrDb) = model.GetAllSINR(rxPwrMw);

        // 4) SINR을 UE에 주입
        for (int i = 0; i < txCount; i++)
        {
            int droneId = _txList[i].gameObject.GetInstanceID();
            for (int j = 0; j < rxCount; j++)
            {
                _rxList[j].AcceptSinrFromModel(droneId, (float)sinrDb[i, j]);
            }
        }
    }

    static void CollectSceneObjects(List<DroneAgent> txOut, List<RadioReceiver> rxOut)
    {
        txOut.Clear(); rxOut.Clear();
        txOut.AddRange(Object.FindObjectsOfType<DroneAgent>(true));

        if (RadioReceiver.All != null)
            rxOut.AddRange(RadioReceiver.All as IEnumerable<RadioReceiver>);
        if (rxOut.Count == 0)
            rxOut.AddRange(Object.FindObjectsOfType<RadioReceiver>(true));
    }

    static void EnsureCapacity(RadioLinkModel m, int txCount, int rxCount)
    {
        if (m.txPositions == null) m.txPositions = new List<Vector3>(txCount);
        if (m.txHeights == null) m.txHeights = new List<float>(txCount);
        if (m.rxPositions == null) m.rxPositions = new List<Vector3>(rxCount);
        if (m.rxHeights == null) m.rxHeights = new List<float>(rxCount);
    }
}
