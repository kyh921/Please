#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;
using System.Linq;

public static class UEMarkerAutoPlacer
{
    [MenuItem("Tools/UE/Place Markers For All UE Buildings")]
    public static void PlaceMarkersForAll()
    {
        int made = 0, updated = 0;

        foreach (var rt in Object.FindObjectsOfType<RadioTransceiver>())
        {
            if (rt.role != NodeRole.UE) continue;

            Transform parent = rt.transform; // 건물 루트(UE 스크립트가 붙은 오브젝트)

            // ── 건물의 3D 경계 박스 중심(가로·세로·높이 모두 중앙)
            Vector3 center = GetBuildingBoundsCenter(parent);

            // UE_Marker 탐색/생성
            Transform marker = parent.Find("UE_Marker");
            if (!marker)
            {
                var go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                go.name = "UE_Marker";
                go.tag = "UEViz"; // DemandArea에서 색칠 제외
                Object.DestroyImmediate(go.GetComponent<Collider>());
                go.transform.SetParent(parent, true);
                go.transform.localScale = Vector3.one * 0.8f;
                go.AddComponent<UEDebugReceiver>();
                marker = go.transform;
                made++;
            }
            else updated++;

            // ▶︎ 위치/회전 갱신: “건물 중심”으로
            marker.position = center;
            marker.rotation = Quaternion.identity;
        }

        Debug.Log($"[UE Marker] Created={made}, Updated={updated}");
    }

    // 건물의 Renderer/Collider들을 모아 AABB 중심 반환
    private static Vector3 GetBuildingBoundsCenter(Transform root)
    {
        // 1) 콜라이더 우선
        var cols = root.GetComponentsInChildren<Collider>(true);
        if (cols.Length > 0)
        {
            Bounds b = new Bounds(cols[0].bounds.center, Vector3.zero);
            for (int i = 1; i < cols.Length; i++) b.Encapsulate(cols[i].bounds);
            return b.center;              // ← x,y,z 모두 ‘중앙’
        }

        // 2) 없으면 렌더러로 계산
        var rends = root.GetComponentsInChildren<Renderer>(true);
        if (rends.Length > 0)
        {
            Bounds b = new Bounds(rends[0].bounds.center, Vector3.zero);
            for (int i = 1; i < rends.Length; i++) b.Encapsulate(rends[i].bounds);
            return b.center;              // ← 지붕/바닥 아닌 ‘부피 중심’
        }

        // 3) 그래도 없으면 트랜스폼 위치
        return root.position;
    }
}
#endif
