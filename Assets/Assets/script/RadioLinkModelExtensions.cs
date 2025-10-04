using UnityEngine;

// 임시 호환용 확장 메서드: RadioTransceiver의 기존 코드가 컴파일되게만 해줍니다.
// TODO: 팀원이 LinkModel 정리하면 이 파일 삭제하거나 실제 계산으로 교체하세요.
public static class RadioLinkModelExtensions
{
    /// <summary>
    /// Legacy shim for old RadioTransceiver. Returns a fixed RSSI so code compiles.
    /// Replace with real model or remove when Transceiver path is retired.
    /// </summary>
    public static float EstimateRssiDbm(this RadioLinkModel _, Vector3 txPos, Vector3 rxPos)
    {
        // 간단한 더미: 거리 기반 감쇠를 흉내 내고 싶다면 아래처럼 바꾸세요.
        // float d = Mathf.Max(1f, Vector3.Distance(txPos, rxPos));
        // return -40f - 20f * Mathf.Log10(d);
        return -60f; // 고정값 (임시)
    }
}
