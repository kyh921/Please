using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 통신 모델 스크립트
/// </summary>
public class RadioLinkModel : MonoBehaviour
{
    [Header("RF Parameters")]

    // 사용 주파수 700 MHz 
    const double frequencyMHz = 700.0;
    // 배경 노이즈 (상수값)
    const double noisePowerMw = 1.2589254117941673e-10; // mW 단위로 변환
    // 송신 안테나 게인(Gmax, dBi)
    public static double Gpeak_dBi = 5.0;
    // 송신 전력 (dBm)
    public static double Ptx_dBm = 39.0;
    // 수신 안테나 게인 (dBi)
    public static double Grx_dBi = 3.0;

    public List<Vector3> txPositions; // 송신기 위치 리스트
    public List<float> txHeights;     // 송신기 안테나 높이 (m)
    public List<Vector3> rxPositions; // 수신기 위치 리스트
    public List<float> rxHeights;     // 수신기 안테나 높이 (m)


    /// <summary>
    /// 모든 송수신기 쌍별 거리와 각도(고도각) 반환
    /// </summary>
    public void GetAllDistancesAndAngles(out double[,] distances, out double[,] angles)
    {
        int txCount = txPositions.Count;
        int rxCount = rxPositions.Count;
        distances = new double[txCount, rxCount];
        angles = new double[txCount, rxCount];

        for (int i = 0; i < txCount; i++)
        {
            for (int j = 0; j < rxCount; j++)
            {
                Vector3 tx = txPositions[i];
                Vector3 rx = rxPositions[j];

                //  실제 거리 (스케일링 금지)
                double d = Vector3.Distance(tx, rx);
                if (d < 1e-3) d = 1e-3;
                distances[i, j] = d;

                //  도넛패턴용 각도: 수직축 기준 θ = atan2(horizontal, |Δy|)
                Vector3 diff = rx - tx;
                double dy = Math.Abs(diff.y);
                double h = Math.Sqrt(diff.x * diff.x + diff.z * diff.z);
                angles[i, j] = Math.Atan2(h, Math.Max(1e-6, dy)); // rad
            }

            int receiverIndex = 0;  // 리시버 1 (0-based)

            for (int k = 0; k < txCount; k++) // i 대신 k 등 다른 이름 사용
            {
                if (receiverIndex < rxCount)
                {
                    double angleDeg = angles[k, receiverIndex] * (180 / Math.PI);
                    Debug.Log($"tx[{k}]-rx[{receiverIndex}]: Angle = {angleDeg:F2} deg, Distance = {distances[k, receiverIndex]:F2} m");
                }
            }
        }
    }
    /// <summary>
    /// 모든 송수신기 쌍에 대해 Hata 모델 경로 손실 계산 (도시, 200~1500 MHz)
    /// </summary>
    public double[,] GetAllHataLosses()
    {
        // 네 환경에 맞춘 스케일. (원래 30, 50, 70 쓰던 그 값)
        const double distanceScale = 40.0;  // ← 네가 원하던 스케일 값

        int txCount = txPositions.Count;
        int rxCount = rxPositions.Count;
        GetAllDistancesAndAngles(out var distances, out _);

        double[,] lossMatrix = new double[txCount, rxCount];

        for (int i = 0; i < txCount; i++)
        {
            for (int j = 0; j < rxCount; j++)
            {
                double hB = txHeights[i];
                double hM = rxHeights[j];

                // 경로손실에만 거리 스케일 적용
                double d_km = (distances[i, j] * distanceScale) / 1000.0;
                if (d_km < 1e-4) d_km = 1e-4;

                double cH = 3.2 * Math.Pow(Math.Log10(11.75 * hM), 2.0) - 4.97;
                double lossDb = 69.55
                    + 26.16 * Math.Log10(700.0)
                    - 13.82 * Math.Log10(hB)
                    - cH
                    + (44.9 - 6.55 * Math.Log10(hB)) * Math.Log10(d_km);

                lossMatrix[i, j] = lossDb;
            }
        }
        return lossMatrix;
    }

    /// <summary>
    /// 안테나 이득 계산 (sin^2 패턴)
    /// </summary>
    public static double GetAntennaGainDbi(double thetaRad)
    {
        double sinTheta = Math.Sin(thetaRad);
        double sin2 = sinTheta * sinTheta;
        if (sin2 <= 0.0) sin2 = 1e-8; // 방어
        double gainDbi = Gpeak_dBi + 10.0 * Math.Log10(sin2);
        return gainDbi;
    }

    /// <summary>
    /// 모든 송신기-수신기 쌍에 대해 안테나 이득 계산
    /// </summary>
    /// 
  
    public double[,] GetAllTxAntennaGains(double[,] angles)
    {
        int txCount = angles.GetLength(0);
        int rxCount = angles.GetLength(1);
        double[,] gains = new double[txCount, rxCount];
        for (int i = 0; i < txCount; i++)
        {
            for (int j = 0; j < rxCount; j++)
            {
                gains[i, j] = GetAntennaGainDbi(angles[i, j]);
            }
        }
        return gains;
    }

    /// <summary>
    /// 모든 송신기-수신기 쌍에 대해 수신 전력 계산(dBm 단위)
    /// </summary>
    public double[,] GetAllRxPowers(double[,] angles, double[,] pathLosses)
    {
        int txCount = angles.GetLength(0);
        int rxCount = angles.GetLength(1);
        double[,] rxPowers = new double[txCount, rxCount];

        for (int i = 0; i < txCount; i++)
        {
            for (int j = 0; j < rxCount; j++)
            {
                double Gtx_dBi = GetAntennaGainDbi(angles[i, j]);
                rxPowers[i, j] = Gtx_dBi + Ptx_dBm - pathLosses[i, j] + Grx_dBi;
            }
        }
        return rxPowers;
    }

    /// <summary>
    /// 수신 전력(dBm) → mW 단위 변환
    /// </summary>
    public static double[,] ConvertRxPowersToMw(double[,] rxPowers_dBm)
    {
        int txCount = rxPowers_dBm.GetLength(0);
        int rxCount = rxPowers_dBm.GetLength(1);
        double[,] rxPowers_mW = new double[txCount, rxCount];

        for (int i = 0; i < txCount; i++)
            for (int j = 0; j < rxCount; j++)
                rxPowers_mW[i, j] = Math.Pow(10.0, rxPowers_dBm[i, j] / 10.0);

        return rxPowers_mW;
    }

    /// <summary>
    /// SINR 계산 (선형, dB 동시 리턴)
    /// </summary>
    public (double[,] linear, double[,] dB) GetAllSINR(double[,] rxPowers_mW)
    {
        int txCount = rxPowers_mW.GetLength(0);
        int rxCount = rxPowers_mW.GetLength(1);
        double[,] sinrLinear = new double[txCount, rxCount];
        double[,] sinrDb = new double[txCount, rxCount];

        for (int i = 0; i < txCount; i++)
        {
            for (int l = 0; l < rxCount; l++)
            {
                double signal = rxPowers_mW[i, l];
                double interference = 0.0;
                for (int k = 0; k < txCount; k++)
                {
                    if (k != i)
                        interference += rxPowers_mW[k, l];
                }
                double sinr = signal / (noisePowerMw + interference);
                sinrLinear[i, l] = sinr;
                sinrDb[i, l] = 10.0 * Math.Log10(sinr);
            }
        }
        return (sinrLinear, sinrDb);
    }
}

