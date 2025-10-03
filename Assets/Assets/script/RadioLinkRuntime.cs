using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// ���(�۽ű�) �� UE(���ű�) �� �ֿ� ����
/// �Ÿ�/����/��μս�/��������/SINR�� ����ϰ�
/// UE�� sinr(dB)�� �����ϴ� ��Ÿ�� ����̹�.
/// </summary>
[DisallowMultipleComponent]
public class RadioLinkRuntime : MonoBehaviour
{
    [Tooltip("��ũ ��� �ֱ�(��)")]
    public float updateInterval = 0.10f;

    [Tooltip("��� �ھ�(���� RadioLinkModel)�� ���� �ϳ� �� �� ����� �Ҵ�)")]
    public RadioLinkModel model;

    float _t;
    // ĳ��
    static readonly List<DroneAgent> _txList = new List<DroneAgent>(16);
    static readonly List<RadioReceiver> _rxList = new List<RadioReceiver>(256);

    void Reset()
    {
        // ���� ������Ʈ�� ������ �ڵ� ���� �õ�
        if (!model) model = GetComponent<RadioLinkModel>();
    }

    public bool debugLog = false;
    public int debugTopPairs = 5; // �����Ӵ� �� �ָ� ������

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

        // 2) �� �Է� ä��� (TX/RX ��ġ������)
        EnsureCapacity(model, txCount, rxCount);

        model.txPositions.Clear(); model.txHeights.Clear();
        for (int i = 0; i < txCount; i++)
        {
            var tx = _txList[i].transform.position;
            model.txPositions.Add(tx);
            model.txHeights.Add(Mathf.Max(1f, tx.y)); // ���� ���
        }

        model.rxPositions.Clear(); model.rxHeights.Clear();
        for (int j = 0; j < rxCount; j++)
        {
            var p = _rxList[j].GetAntennaPosition(); // ReceiverNode ������ �� ��ġ ���
            model.rxPositions.Add(p);
            model.rxHeights.Add(Mathf.Max(1f, p.y));
        }

        // 3) �Ÿ�/���� �� ��μս� �� �������� �� SINR
        model.GetAllDistancesAndAngles(out var distances, out var angles);
        var (dist, ang) = (distances, angles);
        var lossDb = model.GetAllHataLosses();
        var rxPwrDbm = model.GetAllRxPowers(angles, lossDb);
        var rxPwrMw = RadioLinkModel.ConvertRxPowersToMw(rxPwrDbm);
        var (_, sinrDb) = model.GetAllSINR(rxPwrMw);

        // 4) �� �� (���i, UEj)�� ���� UE�� sinr(dB) ����
        //    droneId: ��� ������Ʈ�� GetInstanceID (CommsSensor�� ���� ID�� Pop)
        for (int i = 0; i < txCount; i++)
        {
            int droneId = _txList[i].gameObject.GetInstanceID();

            for (int j = 0; j < rxCount; j++)
            {
                float sinrDb_ij = (float)sinrDb[i, j];
                _rxList[j].AcceptSinrFromModel(droneId, sinrDb_ij);
            }
        }

        if (debugLog)
        {
            int cnt = 0;
            for (int i = 0; i < txCount && cnt < debugTopPairs; i++)
                for (int j = 0; j < rxCount && cnt < debugTopPairs; j++, cnt++)
                {
                    Debug.Log($"[RT] TX{i}->UE{j}  d={dist[i, j]:F1}m, ��={ang[i, j] * Mathf.Rad2Deg:F1}��, " +
                              $"PL={lossDb[i, j]:F1} dB, Rx={rxPwrDbm[i, j]:F1} dBm, SINR={sinrDb[i, j]:F1} dB");
                }
        }
    }

    static void CollectSceneObjects(List<DroneAgent> txOut, List<RadioReceiver> rxOut)
    {
        txOut.Clear();
        rxOut.Clear();
        txOut.AddRange(Object.FindObjectsOfType<DroneAgent>(true));
        // RadioReceiver�� ���ο� ���� ������Ʈ�� All�� ���� ������ �װ� �켱 ���
        // (���ٸ� FindObjectsOfType�� ��ü)
        if (RadioReceiver.All != null)
            rxOut.AddRange(RadioReceiver.All as IEnumerable<RadioReceiver>);
        if (rxOut.Count == 0) // ������Ʈ���� ��������� ����
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
