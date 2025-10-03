
using System.Collections.Generic;
using UnityEngine;



// </summary>
// 통신 모델 스크립트
public class RadioLinkModel : MonoBehaviour
{
    [Header("RF Parameters")]

    // 사용 주파수 700MHz 
    const float frequencyMHz = 700f;
    // 배경 노이즈는 상수값이므로 고정 식(8)
    const float noisePowerDbm = -99f; // dBm 단위
    const float noisePowerMw = Mathf.Pow(10f, noisePowerDbm / 10f); // mW 단위로 변환
    // 송신 안테나 게인(Gmax,dBi)
    public static float Gpeak_dBi = 5.0f;
    // 송신 전력 (P l,tx,dBm)
    public static float Ptx_dBm = 39.0f;
    // 수신 안테나 게인 (G l,rx,dBi)
    public static float Grx_dBi = 3.0f;

    public List<Vector3> txPositions; // 송신기 위치 리스트
    public List<float> txHeights;     // 송신기 안테나 높이 (m)
    public List<Vector3> rxPositions; // 수신기 위치 리스트
    public List<float> rxHeights;     // 수신기 안테나 높이 (m)


    

    /// </summary>
    // 모든 송수신기 쌍별로 거리와 각도(고도각) 반환
    public void GetAllDistancesAndAngles(out float[,] distances, out float[,] angles)
    {
        int txCount = txPositions.Count;
        int rxCount = rxPositions.Count;
        distances = new float[txCount, rxCount];
        angles = new float[txCount, rxCount];

        for (int i = 0; i < txCount; i++)
        {
            for (int j = 0; j < rxCount; j++)
            {
                Vector3 tx = txPositions[i];
                Vector3 rx = rxPositions[j];

                // 거리 계산
                float distance = Vector3.Distance(tx, rx);
                distances[i, j] = distance;

                // 고도각 계산 (수평면 기준)
                Vector3 dir = (rx - tx).normalized;
                Vector3 dirProj = Vector3.ProjectOnPlane(dir, Vector3.up).normalized;
                float thetaRad = Mathf.Acos(Mathf.Clamp(Vector3.Dot(dir, dirProj), -1f, 1f));
                angles[i, j] = thetaRad;
            }
        }
    }
    //경로 손실 모델 (hata 모델)
    // 모든 송수신기 쌍에 대해 Hata 모델 경로 손실 계산
    public float[,] GetAllHataLosses()
    {
        int txCount = txPositions.Count;
        int rxCount = rxPositions.Count;
        float[,] distances, angles;
        GetAllDistancesAndAngles(out distances, out angles);

        float[,] lossMatrix = new float[txCount, rxCount];

        for (int i = 0; i < txCount; i++)
        {
            for (int j = 0; j < rxCount; j++)
            {
                float hB = txHeights[i];
                float hM = rxHeights[j];
                float d_km = distances[i, j] / 1000.0f;

                float cH = 3.2f * Mathf.Pow(Mathf.Log10(11.75f * hM), 2f) - 4.97f;
                float lossDb = 69.55f
                    + 26.16f * Mathf.Log10(frequencyMHz)
                    - 13.82f * Mathf.Log10(hB)
                    - cH
                    + (44.9f - 6.55f * Mathf.Log10(hB)) * Mathf.Log10(d_km);

                lossMatrix[i, j] = lossDb;
            }
        }
        return lossMatrix;
    }



    // 안테니 이득 계산 모델 (안테나 방사 패턴 모델링 식)
    public static float GetAntennaGainDbi(float thetaRad)
    {
        float sinTheta = Mathf.Sin(thetaRad);
        float sin2 = Mathf.Pow(sinTheta, 2f);
        if (sin2 <= 0f) sin2 = 1e-8f;
        // Gpeak_dBi는 한 번만 결정되고 모든 쌍에 동일하게 적용
        float gainDbi = Gpeak_dBi + 10f * Mathf.Log10(sin2);
        return gainDbi;
    }


    // 모든 송신기-수신기 쌍에 대해 안테나 이득 계산
    public float[,] GetAllTxAntennaGains(float[,] angles)
    {
        int txCount = txPositions.Count;
        int rxCount = rxPositions.Count;
        float[,] gains = new float[txCount, rxCount];
        for (int i = 0; i < txCount; i++)
        {
            for (int j = 0; j < rxCount; j++)
            {
                gains[i, j] = GetAntennaGainDbi(angles[i, j]);
            }
        }
        return gains;
    }

    //모든 송신기-수신기 쌍에 대해 수신 전력 계산(dB 단위)
    public float[,] GetAllRxPowers(float[,] angles, float[,] pathLosses)
    {
        int txCount = angles.GetLength(0);
        int rxCount = angles.GetLength(1);
        float[,] rxPowers = new float[txCount, rxCount];

        for (int i = 0; i < txCount; i++)
        {
            for (int j = 0; j < rxCount; j++)
            {
                // 송신기-수신기 각도별 송신 안테나 게인 계산
                float Gtx_dBi = GetAntennaGainDbi(angles[i, j]);

                // 수신 전력 계산 공식
                rxPowers[i, j] = Gtx_dBi + Ptx_dBm - pathLosses[i, j] + Grx_dBi;
            }
        }
        return rxPowers;
    }

    //수신 전력 쌍을 mW 단위로 변환(dBm -> mW)
    public static float[,] ConvertRxPowersToMw(float[,] rxPowers_dBm)
    {
        int txCount = rxPowers_dBm.GetLength(0);
        int rxCount = rxPowers_dBm.GetLength(1);
        float[,] rxPowers_mW = new float[txCount, rxCount];

        for (int i = 0; i < txCount; i++)
            for (int j = 0; j < rxCount; j++)
                rxPowers_mW[i, j] = Mathf.Pow(10f, rxPowers_dBm[i, j] / 10f);

        return rxPowers_mW;
    }

    // </summary>
    //SINR값 계산
    public float[,] GetAllSINR(float[,] rxPowers_mW, float noisePower_mW)
    {
        int txCount = rxPowers_mW.GetLength(0);
        int rxCount = rxPowers_mW.GetLength(1);
        float[,] sinrMatrix = new float[txCount, rxCount];

        for (int i = 0; i < txCount; i++)
        {
            for (int l = 0; l < rxCount; l++)
            {
                float signal = rxPowers_mW[i, l];
                float interference = 0f;
                for (int k = 0; k < txCount; k++)
                {
                    if (k != i)
                        interference += rxPowers_mW[k, l];
                }
                sinrMatrix[i, l] = signal / (noisePower_mW + interference);
            }
        }
        return sinrMatrix;
    }
}






