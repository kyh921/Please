using System.Collections.Generic;
using UnityEngine;

// 빈 GameObject에 붙이세요. SRC_ID_LIST에 여러 드론 ID를 넣으면 각자 값 출력.
public class TestQoePull : MonoBehaviour
{
    [Tooltip("드론(에이전트)들이 사용할 srcId 목록")]
    public List<int> SRC_ID_LIST = new List<int>{ 4242 }; // 예: {4242, 4243, 4244, 4245, 4246}

    [Tooltip("매 프레임 자동으로 뽑기 (체크 해제시 L키로 수동)")]
    public bool autoPullEveryFrame = false;

    void Update()
    {
        if (autoPullEveryFrame || Input.GetKeyDown(KeyCode.L))
        {
            foreach (int src in SRC_ID_LIST)
            {
                float sumWeightedQoe = 0f;
                int   maxOverconnect = 0;

                var rrs = FindObjectsOfType<RadioReceiver>();
                for (int i = 0; i < rrs.Length; i++)
                {
                    rrs[i].PopQoeAndOverlapFor(src, out float wq, out int ov);
                    sumWeightedQoe += wq;
                    if (ov > maxOverconnect) maxOverconnect = ov;
                }

                Debug.Log($"[Pull] src={src}  Σ(weighted QoE)={sumWeightedQoe:F3},  overconnect={maxOverconnect}");
            }
        }
    }
}
