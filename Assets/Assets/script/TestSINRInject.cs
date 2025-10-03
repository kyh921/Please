using UnityEngine;

// 빈 GameObject에 붙여서 쓰세요. 여러 개 두고 SRC_ID만 다르게 설정하면 multi-agent 흉내 가능.
public class TestSINRInject : MonoBehaviour
{
    [Tooltip("이 주입기가 대표하는 드론 ID (RadioReceiver.AcceptSinrFromModel의 srcId)")]
    public int SRC_ID = 4242;

    [Header("주입할 SINR(dB)")]
    public float sinrGood = 8.6f;   // CQI~9 근처(표 기준 8.456dB 조금 위)
    public float sinrBad  = -3f;    // 연결 안됨

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.P)) // 좋은 SINR 주입
        {
            foreach (var rr in FindObjectsOfType<RadioReceiver>())
                rr.AcceptSinrFromModel(SRC_ID, sinrGood);
            Debug.Log($"[Inject] src={SRC_ID}, SINR={sinrGood} dB to all receivers.");
        }

        if (Input.GetKeyDown(KeyCode.O)) // 나쁜 SINR 주입
        {
            foreach (var rr in FindObjectsOfType<RadioReceiver>())
                rr.AcceptSinrFromModel(SRC_ID, sinrBad);
            Debug.Log($"[Inject] src={SRC_ID}, SINR={sinrBad} dB to all receivers.");
        }
    }
}
