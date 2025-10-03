using UnityEngine;

[DisallowMultipleComponent]
public class ReceiverNode : MonoBehaviour
{
    [Tooltip("          ")]
    public Transform rxAnchor;

    [Header("Auto Anchor (fallback)")]
    public float upOffset = 0;   
    public bool drawGizmo = true;

    public Vector3 GetRxPosition()
    {
        if (rxAnchor) return rxAnchor.position;

        // Collider        MeshRenderer   bounds    
        var col = GetComponentInParent<Collider>();
        if (col)
            return col.bounds.center + Vector3.up * upOffset;

        var r = GetComponentInParent<MeshRenderer>();
        if (r)
            return r.bounds.center + Vector3.up * upOffset;

        return transform.position + Vector3.up * upOffset;
    }

#if UNITY_EDITOR
    void OnDrawGizmos()
    {
        if (!drawGizmo) return;
        Gizmos.color = Color.yellow;
        Vector3 p = GetRxPosition();
        Gizmos.DrawSphere(p, 0.4f);
    }
#endif
}
