using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 통신 모델 (Hata 기반 + 균일 초과 손실 적용)
/// </summary>
public class RadioLinkModel : MonoBehaviour
{
    [Header("RF Parameters")]

    // 주파수 (MHz)
    const double frequencyMHz = 700.0;
    // 배경 노이즈 (상수, -99 dBm 수준)
    const double noisePowerMw = 1.2589254117941673e-10; // mW

    // 송신 안테나 게인(Gmax, dBi)
    public static double Gpeak_dBi = 2.0;
    // 송신 전력 (dBm)
    public static double Ptx_dBm = 23.0;
    // 수신 안테나 게인 (dBi)
    public static double Grx_dBi = 0.0;

    [Header("Environment Loss (uniform)")]
    [Tooltip("모든 링크에 균일 적용되는 초과 경로손실(클러터/침투/환경 보정, dB)")]
    public double extraLossDb = 20.0;   // 10~30 dB 권장

    public List<Vector3> txPositions; // 송신기 위치 리스트
    public List<float> txHeights;     // 송신 안테나 높이
    public List<Vector3> rxPositions; // 수신기 위치 리스트
    public List<float> rxHeights;     // 수신 안테나 높이

    /// <summary>
    /// 송수신기 쌍별 거리 및 고도각 계산
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
                double d = Vector3.Distance(tx, rx);
                if (d < 1e-3) d = 1e-3;
                distances[i, j] = d;

                Vector3 diff = rx - tx;
                double dy = Math.Abs(diff.y);
                double h = Math.Sqrt(diff.x * diff.x + diff.z * diff.z);
                angles[i, j] = Math.Atan2(h, Math.Max(1e-6, dy)); // rad
            }
        }
    }

    /// <summary>
    /// Hata 모델 기반 경로 손실 계산 (Uniform 추가 손실 포함)
    /// </summary>
    public double[,] GetAllHataLosses()
    {
        int txCount = txPositions.Count;
        int rxCount = rxPositions.Count;
        GetAllDistancesAndAngles(out var distances, out _);

        double[,] lossMatrix = new double[txCount, rxCount];

        for (int i = 0; i < txCount; i++)
        {
            for (int j = 0; j < rxCount; j++)
            {
                double hB = Math.Max(1.0, txHeights[i]);
                double hM = Math.Max(1.0, rxHeights[j]);
                double d_km = Math.Max(1e-4, distances[i, j] / 1000.0);

                // Urban Hata model (150~1500 MHz)
                double cH = 3.2 * Math.Pow(Math.Log10(11.75 * hM), 2.0) - 4.97;
                double lossDb = 69.55
                    + 26.16 * Math.Log10(frequencyMHz)
                    - 13.82 * Math.Log10(hB)
                    - cH
                    + (44.9 - 6.55 * Math.Log10(hB)) * Math.Log10(d_km);

                // 균일 초과 손실 (ITU-R P.2108 / P.2109, 3GPP TR 38.901 O2I)
                lossMatrix[i, j] = lossDb + extraLossDb;
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
        if (sin2 <= 0.0) sin2 = 1e-8;
        return Gpeak_dBi + 10.0 * Math.Log10(sin2);
    }

    /// <summary>
    /// 송신기-수신기별 안테나 이득 행렬
    /// </summary>
    public double[,] GetAllTxAntennaGains(double[,] angles)
    {
        int txCount = angles.GetLength(0);
        int rxCount = angles.GetLength(1);
        double[,] gains = new double[txCount, rxCount];

        for (int i = 0; i < txCount; i++)
            for (int j = 0; j < rxCount; j++)
                gains[i, j] = GetAntennaGainDbi(angles[i, j]);

        return gains;
    }

    /// <summary>
    /// 수신 전력 계산 (dBm)
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
    /// dBm → mW 변환
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
    /// SINR 계산 (linear, dB)
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
                    if (k != i)
                        interference += rxPowers_mW[k, l];

                // 현재는 간섭 항 0배 (학습 안정용)
                double sinr = signal / (noisePowerMw + (1.0f * interference));
                sinrLinear[i, l] = sinr;
                sinrDb[i, l] = 10.0 * Math.Log10(sinr);
            }
        }
        return (sinrLinear, sinrDb);
    }
}
