// UEDebugReceiver.cs (교체)
using UnityEngine;

public class UEDebugReceiver : MonoBehaviour
{
    RadioReceiver rr;
    Renderer ren;
    Color orig;
    bool hasBaseColor, hasColor;

    void Awake()
    {
        rr = GetComponent<RadioReceiver>();
        if (!rr) rr = GetComponentInParent<RadioReceiver>();
        if (!rr) rr = GetComponentInChildren<RadioReceiver>();

        ren = GetComponent<Renderer>();
        if (!ren) ren = GetComponentInChildren<Renderer>();

        if (ren && ren.sharedMaterial)
        {
            hasBaseColor = ren.sharedMaterial.HasProperty("_BaseColor");
            hasColor     = ren.sharedMaterial.HasProperty("_Color");

            var mpb = new MaterialPropertyBlock();
            ren.GetPropertyBlock(mpb);
            if (hasBaseColor) orig = mpb.GetColor("_BaseColor");
            else if (hasColor) orig = mpb.GetColor("_Color");
            else orig = Color.white;
        }

        if (rr != null) rr.OnReceive += OnRx;

        gameObject.tag = "UEViz"; // DemandArea 색 변경 제외용
    }

    void OnDestroy()
    {
        if (rr != null) rr.OnReceive -= OnRx;
    }

    void OnRx(int srcId, byte[] payload, float sinrDb)
    {
        if (!ren) return;
        StopAllCoroutines();
        StartCoroutine(Blink());
    }

    System.Collections.IEnumerator Blink()
    {
        var mpb = new MaterialPropertyBlock();
        ren.GetPropertyBlock(mpb);

        if (hasBaseColor) mpb.SetColor("_BaseColor", Color.cyan);
        else if (hasColor) mpb.SetColor("_Color", Color.cyan);
        ren.SetPropertyBlock(mpb);

        yield return new WaitForSeconds(0.2f);

        if (hasBaseColor) mpb.SetColor("_BaseColor", orig);
        else if (hasColor) mpb.SetColor("_Color", orig);
        ren.SetPropertyBlock(mpb);
    }
}
