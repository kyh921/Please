using System.Linq;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

public class DemandCleanup : MonoBehaviour
{
    public Transform[] roots; // Buildings ���� ����

    // (1) ���� ���: �ڽ��� DemandArea/ReceiverNode�� ����(��Ʈ ����)
    [ContextMenu("Cleanup Demand/Receiver on children (keep on group roots)")]
    public void Cleanup()
    {
        foreach (var root in roots)
        {
            if (!root) continue;

            var areas = root.GetComponentsInChildren<DemandArea>(true);
            foreach (var da in areas)
            {
                if (da.transform != root && da.transform.parent != null)
                {
#if UNITY_EDITOR
                    Undo.DestroyObjectImmediate(da);
#else
                    DestroyImmediate(da);
#endif
                }
            }

            var rxs = root.GetComponentsInChildren<ReceiverNode>(true);
            foreach (var rn in rxs)
            {
                if (rn.transform != root && rn.transform.parent != null)
                {
#if UNITY_EDITOR
                    Undo.DestroyObjectImmediate(rn);
#else
                    DestroyImmediate(rn);
#endif
                }
            }
        }
        Debug.Log("[Cleanup] Done.");
    }

    // (2) �߰� ���: ��Ʈ�� ������ 'RxAnchor' (�Ǵ� ...Anchor) ������Ʈ�� ����
    [ContextMenu("Remove 'Anchor' nodes (RxAnchor) under roots")]
    public void RemoveAnchors()
    {
        int removed = 0;

        foreach (var root in roots)
        {
            if (!root) continue;

            var targets = root.GetComponentsInChildren<Transform>(true)
                              .Where(t => t != root &&
                                     (t.name.Equals("RxAnchor", System.StringComparison.OrdinalIgnoreCase) ||
                                      t.name.EndsWith("Anchor", System.StringComparison.OrdinalIgnoreCase)))
                              .Select(t => t.gameObject)
                              .ToArray();

            foreach (var go in targets)
            {
#if UNITY_EDITOR
                Undo.DestroyObjectImmediate(go);  // �ǵ����� ����
#else
                DestroyImmediate(go);
#endif
                removed++;
            }
        }
        Debug.Log($"[Cleanup] Removed Anchor nodes: {removed}");
    }
}