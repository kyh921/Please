using UnityEngine;

public class DemandCleanup : MonoBehaviour
{
    public Transform[] roots; // Buildings ���� ����

    [ContextMenu("Cleanup Demand/Receiver on children (keep on group roots)")]
    public void Cleanup()
    {
        foreach (var root in roots)
        {
            if (!root) continue;
            var areas = root.GetComponentsInChildren<DemandArea>(true);
            foreach (var da in areas)
            {
                // �׷� ��Ʈ���� �����, �� ���ڽġ��� �޸� �� ����
                if (da.transform != root && da.transform.parent != null)
                {
                    // �ڽ��ε� �׷� ��Ʈ�� �ƴ� �� ����
                    DestroyImmediate(da);
                }
            }

            var rxs = root.GetComponentsInChildren<ReceiverNode>(true);
            foreach (var rn in rxs)
            {
                if (rn.transform != root && rn.transform.parent != null)
                {
                    DestroyImmediate(rn);
                }
            }
        }
        Debug.Log("[Cleanup] Done.");
    }
}
