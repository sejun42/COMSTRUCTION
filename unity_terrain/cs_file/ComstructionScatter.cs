// Assets/Editor/ComstructionScatter.cs
using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using System.Collections.Generic;
using System.Linq;

public class ComstructionScatter : EditorWindow
{
    // =============== [ �ڵ� ������ ] ===============
    // �������� ������ �ڵ� ���� (�� �����鸸 �����ϴ�)
    // ��: BIRP ���� �������� �ִ� ����
    static readonly string[] PREFAB_FOLDERS = {
        "Assets/Prototype Collection/BIRP",
        // "Assets/MyProps",  // �ʿ� �� �߰�
    };

    // ����/������/����/���� �� ��ü �⺻��
    const int PRESET_COUNT = 30;
    const float PRESET_MIN_SCALE = 3.0f;
    const float PRESET_MAX_SCALE = 3.5f;
    const float PRESET_MAX_SLOPE = 60f;     // ��� �ִ� ��� (deg)
    const float PRESET_AVOID_RADIUS = 0.4f;   // ���� ����
    const float PRESET_Y_OFFSET = 0.00f;   // ��¦ ���� ������ +��
    const bool PRESET_ALIGN_NORMAL = true;    // ���� ��� ����
    static readonly Vector2 PRESET_Y_ROT = new Vector2(0, 360);

    // Ư�� �����տ� ����ġ�� �ְ� �ʹٸ�(������ �յ�):
    // Ű���忡 ��Ī�Ǵ� ������ �̸��� ����ġ �ο�
    static readonly (string keyword, float weight)[] PREFAB_WEIGHTS = {
        ("Cone",         3f),  // "Cone"�� �̸��� ���� 3�� Ȯ��
        ("Barrier",      1f),
        ("Barrel",       1f),
        // ("Tree",      0.5f),
    };
    // ===============================================

    // â ���� (���� ��ư�� ����)
    [MenuItem("COMSTRUCTION/Scatter Objects (Manual)")]
    static void ShowWindow() => GetWindow<ComstructionScatter>("Scatter Objects");

    // ��Ŭ��: �ڵ� �����¸����� ����
    [MenuItem("COMSTRUCTION/Scatter One-Click (Preset)")]
    static void OneClickMenu() => RunOneClickScatter();

    // �ܺ�(�ٸ� ��������)���� ȣ���� �� �ְ� static �޼��� ����
    public static void RunOneClickScatter()
    {
        // 1) Terrain �ڵ� ã��
#if UNITY_2022_2_OR_NEWER
        var terrain = Object.FindFirstObjectByType<Terrain>();
#else
        var terrain = Object.FindObjectOfType<Terrain>();
#endif
        if (!terrain)
        {
            EditorUtility.DisplayDialog("Scatter", "���� Terrain�� �����ϴ�.", "OK");
            return;
        }

        // 2) ������ ������ (���� �������� t:Prefab �˻�, FBX ����)
        var prefabs = LoadPrefabsFromFolders(PREFAB_FOLDERS)
                      .Where(p => p) // null ����
                      .ToList();

        if (prefabs.Count == 0)
        {
            EditorUtility.DisplayDialog("Scatter", "���� �������� �������� ã�� ���߽��ϴ�.", "OK");
            return;
        }

        // 3) ����ġ ���̺� ����
        var table = BuildWeightedTable(prefabs, PREFAB_WEIGHTS);

        // 4) ��ġ ����
        var placed = ScatterInternal(
            terrain, table,
            count: PRESET_COUNT,
            yRotRange: PRESET_Y_ROT,
            scaleRange: new Vector2(PRESET_MIN_SCALE, PRESET_MAX_SCALE),
            maxSlopeDeg: PRESET_MAX_SLOPE,
            avoidOverlap: true,
            avoidRadius: PRESET_AVOID_RADIUS,
            yOffset: PRESET_Y_OFFSET,
            alignToNormal: PRESET_ALIGN_NORMAL
        );

        EditorUtility.DisplayDialog("Scatter", $"��Ŭ�� �Ϸ�: {placed}/{PRESET_COUNT}", "OK");
    }

    // ������������������ ���� UI(���� â)�� ���� ������������������
    public Terrain targetTerrain;
    public List<GameObject> manualPrefabs = new List<GameObject>();
    public int count = PRESET_COUNT;
    public int randomSeed = 0;
    public bool useFullTerrain = true;
    public Vector2 areaMin = Vector2.zero;
    public Vector2 areaMax = new Vector2(50, 50);
    public bool alignToNormal = PRESET_ALIGN_NORMAL;
    [Range(0, 90)] public float maxSlopeDeg = PRESET_MAX_SLOPE;
    public float yOffset = PRESET_Y_OFFSET;
    public Vector2 yRotationRange = PRESET_Y_ROT;
    public Vector2 uniformScaleRange = new Vector2(PRESET_MIN_SCALE, PRESET_MAX_SCALE);
    public bool avoidOverlap = true;
    public float avoidRadius = PRESET_AVOID_RADIUS;
    public Transform parentContainer;

    void OnGUI()
    {
        EditorGUILayout.HelpBox("������ ��Ŭ���� �޴�: COMSTRUCTION/Scatter One-Click (Preset)", MessageType.Info);

        targetTerrain = (Terrain)EditorGUILayout.ObjectField("Target Terrain", targetTerrain, typeof(Terrain), true);

        // ���� ������ ����Ʈ
        EditorGUILayout.LabelField("Prefabs (manual)", EditorStyles.boldLabel);
        int remove = -1;
        for (int i = 0; i < manualPrefabs.Count; i++)
        {
            EditorGUILayout.BeginHorizontal();
            manualPrefabs[i] = (GameObject)EditorGUILayout.ObjectField(manualPrefabs[i], typeof(GameObject), false);
            if (GUILayout.Button("X", GUILayout.Width(22))) remove = i;
            EditorGUILayout.EndHorizontal();
        }
        if (remove >= 0) manualPrefabs.RemoveAt(remove);
        if (GUILayout.Button("+ Add Prefab Slot")) manualPrefabs.Add(null);

        EditorGUILayout.Space(6);
        count = EditorGUILayout.IntField("Count", count);
        randomSeed = EditorGUILayout.IntField("Random Seed (0=auto)", randomSeed);

        useFullTerrain = EditorGUILayout.Toggle("Use Full Terrain Bounds", useFullTerrain);
        using (new EditorGUI.DisabledScope(useFullTerrain))
        {
            areaMin = EditorGUILayout.Vector2Field("Area Min (x,z)", areaMin);
            areaMax = EditorGUILayout.Vector2Field("Area Max (x,z)", areaMax);
        }

        alignToNormal = EditorGUILayout.Toggle("Align To Ground Normal", alignToNormal);
        maxSlopeDeg = EditorGUILayout.Slider("Max Slope (deg)", maxSlopeDeg, 0, 90);
        yOffset = EditorGUILayout.FloatField("Y Offset", yOffset);

        yRotationRange = EditorGUILayout.Vector2Field("Y Rotation Range", yRotationRange);
        uniformScaleRange = EditorGUILayout.Vector2Field("Uniform Scale Range", uniformScaleRange);

        parentContainer = (Transform)EditorGUILayout.ObjectField("Parent Container", parentContainer, typeof(Transform), true);

        EditorGUILayout.Space(6);
        using (new EditorGUI.DisabledScope(!targetTerrain || manualPrefabs.All(p => !p)))
        {
            if (GUILayout.Button("SCATTER (Manual List)"))
            {
                var table = BuildWeightedTable(manualPrefabs.Where(p => p).ToList(), null); // �յ�
                ScatterInternal(targetTerrain, table, count, yRotationRange, uniformScaleRange, maxSlopeDeg,
                                avoidOverlap, avoidRadius, yOffset, alignToNormal,
                                useFullTerrain, areaMin, areaMax, parentContainer, randomSeed);
            }
        }

        if (GUILayout.Button("Run One-Click (Preset)"))
        {
            RunOneClickScatter();
        }
    }

    // ������������������ ������ ������������������

    class WeightedList<T>
    {
        public readonly List<T> items = new();
        public readonly List<float> cum = new();
        public float total;
        public T Next(System.Random rng)
        {
            if (items.Count == 0) return default;
            if (items.Count == 1) return items[0];
            double r = rng.NextDouble() * total;
            int lo = 0, hi = cum.Count - 1;
            while (lo < hi)
            {
                int mid = (lo + hi) >> 1;
                if (r <= cum[mid]) hi = mid; else lo = mid + 1;
            }
            return items[lo];
        }
    }

    static WeightedList<GameObject> BuildWeightedTable(List<GameObject> prefabs,
                                                       (string key, float w)[] rules)
    {
        var wlist = new WeightedList<GameObject>();
        foreach (var p in prefabs)
        {
            float w = 1f;
            if (rules != null)
            {
                string name = p.name;
                foreach (var r in rules)
                    if (!string.IsNullOrEmpty(r.key) && name.IndexOf(r.key, System.StringComparison.OrdinalIgnoreCase) >= 0)
                        w *= Mathf.Max(0.0001f, r.w);
            }
            wlist.items.Add(p);
            wlist.total += w;
            wlist.cum.Add(wlist.total);
        }
        return wlist;
    }

    static List<GameObject> LoadPrefabsFromFolders(string[] folders)
    {
        var list = new List<GameObject>();
        var guids = AssetDatabase.FindAssets("t:Prefab", folders);
        foreach (var g in guids)
        {
            var path = AssetDatabase.GUIDToAssetPath(g);
            var go = AssetDatabase.LoadAssetAtPath<GameObject>(path);
            if (go != null) list.Add(go);
        }
        return list;
    }

    static int ScatterInternal(
        Terrain terrain,
        WeightedList<GameObject> prefabTable,
        int count,
        Vector2 yRotRange,
        Vector2 scaleRange,
        float maxSlopeDeg,
        bool avoidOverlap,
        float avoidRadius,
        float yOffset,
        bool alignToNormal,
        bool useFullBounds = true,
        Vector2 areaMin = default,
        Vector2 areaMax = default,
        Transform parent = null,
        int seed = 0
    )
    {
        var td = terrain.terrainData;
        var tPos = terrain.transform.position;
        var size = td.size;

        Vector2 min = useFullBounds ? Vector2.zero :
                      new Vector2(Mathf.Min(areaMin.x, areaMax.x), Mathf.Min(areaMin.y, areaMax.y));
        Vector2 max = useFullBounds ? new Vector2(size.x, size.z) :
                      new Vector2(Mathf.Max(areaMin.x, areaMax.x), Mathf.Max(areaMin.y, areaMax.y));

        min = Vector2.Max(min, Vector2.zero);
        max = Vector2.Min(max, new Vector2(size.x, size.z));

        if (!parent)
        {
            var container = GameObject.Find("__ScatterContainer") ?? new GameObject("__ScatterContainer");
            parent = container.transform;
        }

        var rng = (seed == 0) ? new System.Random() : new System.Random(seed);
        var placedPositions = new List<Vector3>();
        int placed = 0, tries = 0, maxTries = count * 20;

        Undo.IncrementCurrentGroup();
        int undo = Undo.GetCurrentGroup();

        while (placed < count && tries < maxTries)
        {
            tries++;

            float rx = Mathf.Lerp(min.x, max.x, (float)rng.NextDouble());
            float rz = Mathf.Lerp(min.y, max.y, (float)rng.NextDouble());

            float nx = rx / size.x;
            float nz = rz / size.z;
            float h = td.GetInterpolatedHeight(nx, nz);
            Vector3 n = td.GetInterpolatedNormal(nx, nz);
            float slope = Vector3.Angle(n, Vector3.up);
            if (slope > maxSlopeDeg) continue;

            Vector3 wpos = tPos + new Vector3(rx, h, rz);

            if (avoidOverlap)
            {
                float r2 = avoidRadius * avoidRadius;
                bool tooClose = false;
                for (int i = 0; i < placedPositions.Count; i++)
                    if ((wpos - placedPositions[i]).sqrMagnitude < r2) { tooClose = true; break; }
                if (tooClose) continue;
            }

            var prefab = prefabTable.Next(rng);
            if (!prefab) continue;

            var go = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
            Undo.RegisterCreatedObjectUndo(go, "Scatter Object");
            go.transform.SetParent(parent, true);

            float yRot = Mathf.Lerp(yRotRange.x, yRotRange.y, (float)rng.NextDouble());
            Quaternion rotY = Quaternion.Euler(0, yRot, 0);
            go.transform.rotation = alignToNormal ? Quaternion.FromToRotation(Vector3.up, n) * rotY : rotY;

            float s = Mathf.Lerp(scaleRange.x, scaleRange.y, (float)rng.NextDouble());
            go.transform.localScale = Vector3.one * s;

            go.transform.position = wpos + n * yOffset;

            placedPositions.Add(wpos);
            placed++;
        }

        Undo.CollapseUndoOperations(undo);
        EditorSceneManager.MarkSceneDirty(EditorSceneManager.GetActiveScene());
        Debug.Log($"Scatter: placed {placed}/{count} (tries={tries})");
        return placed;
    }
}
