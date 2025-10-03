#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;
using System.Linq;
using System.Collections.Generic;

public static class SetupDemandPerBlock
{
    // 선택된 Buildings 루트에서 실행
    [MenuItem("Tools/Demand/Setup Per-Building (Blocks → Buildings)")]
    public static void SetupFromSelection()
    {
        var buildingsRoot = Selection.activeTransform;
        if (!buildingsRoot)
        {
            EditorUtility.DisplayDialog("Setup", "Hierarchy에서 'Buildings' 루트를 선택해 주세요.", "OK");
            return;
        }
        Run(buildingsRoot);
    }

    // 씬에서 이름이 'Buildings' 인 루트를 자동 탐색해 실행
    [MenuItem("Tools/Demand/Auto-find 'Buildings' and run")]
    public static void SetupAuto()
    {
        var roots = Object.FindObjectsOfType<Transform>(true)
                          .Where(t => t.name.Equals("Buildings", System.StringComparison.OrdinalIgnoreCase))
                          .ToArray();
        if (roots.Length == 0)
        {
            EditorUtility.DisplayDialog("Setup", "'Buildings' 루트를 찾지 못했습니다.", "OK");
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

        // 0) Buildings 루트에 붙은 DemandArea는 제거 (전부 같은 가중치가 되던 원인)
        foreach (var da in buildingsRoot.GetComponents<DemandArea>()) { Undo.DestroyObjectImmediate(da); removedOnBuildingsRoot++; }

        // 1) 1단계 아래에 Block* 들이 있다고 가정하고 각 블록 처리
        var blocks = buildingsRoot.Cast<Transform>()
                                  .Where(t => t != null && t.name.ToLower().Contains("block"))
                                  .ToArray();

        // Block이 아니라 바로 building* 들이 자식으로 있는 경우도 대비해 리스트 구성
        var directBuildings = buildingsRoot.Cast<Transform>()
                                           .Where(t => t.name.ToLower().Contains("building"))
                                           .ToList();

        // 블록 밑 building* 수집
        List<Transform> buildingRoots = new List<Transform>(directBuildings);
        foreach (var block in blocks)
        {
            // 블록 자체에 붙은 DemandArea가 있으면 제거
            foreach (var da in block.GetComponents<DemandArea>()) { Undo.DestroyObjectImmediate(da); removedOnBlocks++; }

            buildingRoots.AddRange(
                block.Cast<Transform>()
                     .Where(t => t.name.ToLower().Contains("building"))
            );
        }

        // 2) 각 building*_Var* 루트에 DemandArea 1개씩 확보
        foreach (var b in buildingRoots.Distinct())
        {
            if (!b) continue;

            var da = b.GetComponent<DemandArea>();
            if (!da)
            {
                da = Undo.AddComponent<DemandArea>(b.gameObject);
                da.kind = AreaKind.Building; // 네 DemandArea.cs의 Start()에서 매 플레이마다 랜덤 부여
                addedDemandAreas++;
            }

            // 3) 건물 메시에는 UEViz 태그 제거, UE_Marker만 UEViz 유지
            foreach (var r in b.GetComponentsInChildren<Renderer>(true))
            {
                if (!r) continue;

                // UE_Marker는 UEViz 유지
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

                // 일반 건물 메시에서 UEViz 제거 (색칠 제외 방지)
                if (r.CompareTag("UEViz"))
                {
                    Undo.RecordObject(r.gameObject, "Untag Building Mesh");
                    r.gameObject.tag = "Untagged";
                    untaggedMeshes++;
                }
            }
        }

        EditorUtility.DisplayDialog("Per-Building Setup (Blocks → Buildings)",
            $"Buildings 루트에서 제거된 DemandArea: {removedOnBuildingsRoot}\n" +
            $"Block들에서 제거된 DemandArea: {removedOnBlocks}\n" +
            $"건물에 추가된 DemandArea: {addedDemandAreas}\n" +
            $"건물 메시 UEViz 해제: {untaggedMeshes}\n" +
            $"UE_Marker UEViz 설정: {markerTagged}",
            "OK");
    }
}
#endif