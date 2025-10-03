using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 드론(송신기) ↔ UE(수신기) 전 쌍에 대해
/// 거리/각도/경로손실/수신전력/SINR을 계산하고
/// UE로 sinr(dB)을 주입하는 런타임 드라이버.
/// </summary>
[DisallowMultipleComponent]
public class RadioLinkRuntime : MonoBehaviour
{
    [Tooltip("링크 계산 주기(초)")]
    public float updateInterval = 0.10f;

    [Tooltip("계산 코어(기존 RadioLinkModel)를 씬에 하나 둔 뒤 여기로 할당)")]
    public RadioLinkModel model;

    float _t;
    // 캐시
    static readonly List<DroneAgent> _txList = new List<DroneAgent>(16);
    static readonly List<RadioReceiver> _rxList = new List<RadioReceiver>(256);

    void Reset()
    {
        // 같은 오브젝트에 있으면 자동 연결 시도
        if (!model) model = GetComponent<RadioLinkModel>();
    }

    public bool debugLog = false;
    public int debugTopPairs = 5; // 프레임당 몇 쌍만 찍을지

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

        // 2) 모델 입력 채우기 (TX/RX 위치·높이)
        EnsureCapacity(model, txCount, rxCount);

        model.txPositions.Clear(); model.txHeights.Clear();
        for (int i = 0; i < txCount; i++)
        {
            var tx = _txList[i].transform.position;
            model.txPositions.Add(tx);
            model.txHeights.Add(Mathf.Max(1f, tx.y)); // 높이 방어
        }

        model.rxPositions.Clear(); model.rxHeights.Clear();
        for (int j = 0; j < rxCount; j++)
        {
            var p = _rxList[j].GetAntennaPosition(); // ReceiverNode 있으면 그 위치 사용
            model.rxPositions.Add(p);
            model.rxHeights.Add(Mathf.Max(1f, p.y));
        }

        // 3) 거리/각도 → 경로손실 → 수신전력 → SINR
        model.GetAllDistancesAndAngles(out var distances, out var angles);
        var (dist, ang) = (distances, angles);
        var lossDb = model.GetAllHataLosses();
        var rxPwrDbm = model.GetAllRxPowers(angles, lossDb);
        var rxPwrMw = RadioLinkModel.ConvertRxPowersToMw(rxPwrDbm);
        var (_, sinrDb) = model.GetAllSINR(rxPwrMw);

        // 4) 각 쌍 (드론i, UEj)에 대해 UE로 sinr(dB) 주입
        //    droneId: 드론 오브젝트의 GetInstanceID (CommsSensor가 같은 ID로 Pop)
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
                    Debug.Log($"[RT] TX{i}->UE{j}  d={dist[i, j]:F1}m, θ={ang[i, j] * Mathf.Rad2Deg:F1}°, " +
                              $"PL={lossDb[i, j]:F1} dB, Rx={rxPwrDbm[i, j]:F1} dBm, SINR={sinrDb[i, j]:F1} dB");
                }
        }
    }

    static void CollectSceneObjects(List<DroneAgent> txOut, List<RadioReceiver> rxOut)
    {
        txOut.Clear();
        rxOut.Clear();
        txOut.AddRange(Object.FindObjectsOfType<DroneAgent>(true));
        // RadioReceiver는 내부에 정적 레지스트리 All을 갖고 있으니 그걸 우선 사용
        // (없다면 FindObjectsOfType로 대체)
        if (RadioReceiver.All != null)
            rxOut.AddRange(RadioReceiver.All as IEnumerable<RadioReceiver>);
        if (rxOut.Count == 0) // 레지스트리가 비어있으면 폴백
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
