#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;
using System.Linq;
using System.Collections.Generic;

public static class SetupDemandPerBlock
{
    // ���õ� Buildings ��Ʈ���� ����
    [MenuItem("Tools/Demand/Setup Per-Building (Blocks �� Buildings)")]
    public static void SetupFromSelection()
    {
        var buildingsRoot = Selection.activeTransform;
        if (!buildingsRoot)
        {
            EditorUtility.DisplayDialog("Setup", "Hierarchy���� 'Buildings' ��Ʈ�� ������ �ּ���.", "OK");
            return;
        }
        Run(buildingsRoot);
    }

    // ������ �̸��� 'Buildings' �� ��Ʈ�� �ڵ� Ž���� ����
    [MenuItem("Tools/Demand/Auto-find 'Buildings' and run")]
    public static void SetupAuto()
    {
        var roots = Object.FindObjectsOfType<Transform>(true)
                          .Where(t => t.name.Equals("Buildings", System.StringComparison.OrdinalIgnoreCase))
                          .ToArray();
        if (roots.Length == 0)
        {
            EditorUtility.DisplayDialog("Setup", "'Buildings' ��Ʈ�� ã�� ���߽��ϴ�.", "OK");
            return;
        }
        foreach (var r in roots) Run(r);
    }

    private static void Run(Transform buildingsRoot)
    {
        int removedOnBuildingsRoot = 0;
        int removedOnBlocks = 0;
        int addedDemandAreas = 0;
        int untaggedMeshes = 0;
        int markerTagged = 0;

        // 0) Buildings ��Ʈ�� ���� DemandArea�� ���� (���� ���� ����ġ�� �Ǵ� ����)
        foreach (var da in buildingsRoot.GetComponents<DemandArea>()) { Undo.DestroyObjectImmediate(da); removedOnBuildingsRoot++; }

        // 1) 1�ܰ� �Ʒ��� Block* ���� �ִٰ� �����ϰ� �� ��� ó��
        var blocks = buildingsRoot.Cast<Transform>()
                                  .Where(t => t != null && t.name.ToLower().Contains("block"))
                                  .ToArray();

        // Block�� �ƴ϶� �ٷ� building* ���� �ڽ����� �ִ� ��쵵 ����� ����Ʈ ����
        var directBuildings = buildingsRoot.Cast<Transform>()
                                           .Where(t => t.name.ToLower().Contains("building"))
                                           .ToList();

        // ��� �� building* ����
        List<Transform> buildingRoots = new List<Transform>(directBuildings);
        foreach (var block in blocks)
        {
            // ��� ��ü�� ���� DemandArea�� ������ ����
            foreach (var da in block.GetComponents<DemandArea>()) { Undo.DestroyObjectImmediate(da); removedOnBlocks++; }

            buildingRoots.AddRange(
                block.Cast<Transform>()
                     .Where(t => t.name.ToLower().Contains("building"))
            );
        }

        // 2) �� building*_Var* ��Ʈ�� DemandArea 1���� Ȯ��
        foreach (var b in buildingRoots.Distinct())
        {
            if (!b) continue;

            var da = b.GetComponent<DemandArea>();
            if (!da)
            {
                da = Undo.AddComponent<DemandArea>(b.gameObject);
                da.kind = AreaKind.Building; // �� DemandArea.cs�� Start()���� �� �÷��̸��� ���� �ο�
                addedDemandAreas++;
            }

            // 3) �ǹ� �޽ÿ��� UEViz �±� ����, UE_Marker�� UEViz ����
            foreach (var r in b.GetComponentsInChildren<Renderer>(true))
            {
                if (!r) continue;

                // UE_Marker�� UEViz ����
                if (r.gameObject.name.Equals("UE_Marker", System.StringComparison.OrdinalIgnoreCase))
                {
                    if (r.gameObject.tag != "UEViz")
                    {
                        Undo.RecordObject(r.gameObject, "Tag UE_Marker as UEViz");
                        r.gameObject.tag = "UEViz";
                        markerTagged++;
                    }
                    continue;
                }

                // �Ϲ� �ǹ� �޽ÿ��� UEViz ���� (��ĥ ���� ����)
                if (r.CompareTag("UEViz"))
                {
                    Undo.RecordObject(r.gameObject, "Untag Building Mesh");
                    r.gameObject.tag = "Untagged";
                    untaggedMeshes++;
                }
            }
        }

        EditorUtility.DisplayDialog("Per-Building Setup (Blocks �� Buildings)",
            $"Buildings ��Ʈ���� ���ŵ� DemandArea: {removedOnBuildingsRoot}\n" +
            $"Block�鿡�� ���ŵ� DemandArea: {removedOnBlocks}\n" +
            $"�ǹ��� �߰��� DemandArea: {addedDemandAreas}\n" +
            $"�ǹ� �޽� UEViz ����: {untaggedMeshes}\n" +
            $"UE_Marker UEViz ����: {markerTagged}",
            "OK");
    }
}
#endif