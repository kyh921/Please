using UnityEngine;

public class TestGreen : MonoBehaviour
{
    public float goodSinrDb = 5f;   // 수신 성공용 SINR
    public float badSinrDb  = -5f;  // 실패용 SINR(반응 없음)

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.P))
        {
            foreach (var rr in FindObjectsOfType<RadioReceiver>())
                rr.AcceptSinrFromModel(srcId: 999, sinrDb: goodSinrDb);
            Debug.Log("[TestGreen] Injected SINR +5 dB to all UE receivers.");
        }

        if (Input.GetKeyDown(KeyCode.O))
        {
            foreach (var rr in FindObjectsOfType<RadioReceiver>())
                rr.AcceptSinrFromModel(srcId: 999, sinrDb: badSinrDb);
            Debug.Log("[TestGreen] Injected SINR -5 dB to all UE receivers.");
        }

        if (Input.GetKeyDown(KeyCode.R))
        {
            foreach (var da in FindObjectsOfType<DemandArea>())
                da.SetCovered(false);
            Debug.Log("[TestGreen] Reset all DemandArea.covered = false.");
        }
    }
}