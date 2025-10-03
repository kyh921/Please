// UEMarkerAutoPlacer.cs (UE_Marker를 "건물 중심"에 생성/업데이트)
#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;

public static class UEMarkerAutoPlacer
{
    [MenuItem("Tools/UE/Place Markers For All UE (Receiver-based)")]
    public static void PlaceMarkersForAll()
    {
        int made = 0, updated = 0;

        foreach (var rr in Object.FindObjectsOfType<RadioReceiver>(true))
        {
            Transform parent = rr.transform; // 보통 건물 루트

            Vector3 center = GetBoundsCenter(parent);

            Transform marker = parent.Find("UE_Marker");
            if (!marker)
            {
                var go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                go.name = "UE_Marker";
                go.tag = "UEViz";
                Object.DestroyImmediate(go.GetComponent<Collider>());
                go.transform.SetParent(parent, true);
                go.transform.localScale = Vector3.one * 0.8f;
                go.AddComponent<UEDebugReceiver>();
                marker = go.transform;
                made++;
            }
            else updated++;

            marker.position = center;
            marker.rotation = Quaternion.identity;
        }
        Debug.Log($"[UE Marker] Created={made}, Updated={updated}");
    }

    static Vector3 GetBoundsCenter(Transform root)
    {
        var cols = root.GetComponentsInChildren<Collider>(true);
        if (cols.Length > 0)
        {
            Bounds b = new Bounds(cols[0].bounds.center, Vector3.zero);
            for (int i = 1; i < cols.Length; i++) b.Encapsulate(cols[i].bounds);
            return b.center;
        }
        var rends = root.GetComponentsInChildren<Renderer>(true);
        if (rends.Length > 0)
        {
            Bounds b = new Bounds(rends[0].bounds.center, Vector3.zero);
            for (int i = 1; i < rends.Length; i++) b.Encapsulate(rends[i].bounds);
            return b.center;
        }
        return root.position;
    }
}
#endif