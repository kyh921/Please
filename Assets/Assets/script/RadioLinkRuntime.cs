using System.Collections.Generic;
using UnityEngine;
[DefaultExecutionOrder(-200)]

[DisallowMultipleComponent]
public class RadioLinkRuntime : MonoBehaviour
{
    [Tooltip("��ũ ��� �ֱ�(��)")]
    public float updateInterval = 0.10f;

    [Tooltip("��� �ھ�(���� RadioLinkModel)�� ���� �ϳ� �� �� ����� �Ҵ�)")]
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

        // 1) ������ ���/UE ����
        CollectSceneObjects(_txList, _rxList);
        int txCount = _txList.Count;
        int rxCount = _rxList.Count;
        if (txCount == 0 || rxCount == 0) return;

        // 2) �� �Է� ä���
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

        // 3) �Ÿ�/���� �� ��μս� �� �������� �� SINR
        model.GetAllDistancesAndAngles(out var distances, out var angles);
        var lossDb = model.GetAllHataLosses();                // �״�� ���
        var rxPwrDbm = model.GetAllRxPowers(angles, lossDb);
        var rxPwrMw = RadioLinkModel.ConvertRxPowersToMw(rxPwrDbm);
        var (_, sinrDb) = model.GetAllSINR(rxPwrMw);

        // 4) SINR�� UE�� ����
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
