using UnityEditor;
using UnityEngine;
using System.IO;
using System.Linq;
using System.Diagnostics;
using System.Threading.Tasks;
using System.Threading; // Thread.Sleep
using System;

public class ComstructionOneClick : EditorWindow
{
    // ---- ���/�ɼ�(������Ʈ�� �°� �⺻���� �ٲٸ� ��) ----
    public string pythonExe = @"C:\Users\sejun\anaconda3\envs\comstruction\python.exe";
    public string capturePy = @"D:\COMSTRUCTION\depth_capture\capture_depth.py";
    public string convertPy = @"D:\COMSTRUCTION\scripts\depth_to_raw.py"; // ����Ž��/--center ���� ����
    public string savedDepthDir = @"D:\COMSTRUCTION\depth_capture\saved_depth_maps";
    public string heightmapOutDir = @"D:\COMSTRUCTION\terrain\heightmaps";
    public string projectRoot = @"D:\COMSTRUCTION";


    public int terrainHeightmapResolution = 513;  // depth_to_raw.py --size�� ��ġ
    public bool flipVertically = false;           // �ʿ�� ������
    public float terrainWidth = 100f;              // ���� ����(m)
    public float terrainLength = 100f;             // ���� ����(m)
    public float terrainHeight = 50f;             // ���� �ִ� ����(m) (ũ�� �����)

    private Terrain targetTerrain;

    [MenuItem("COMSTRUCTION/One-Click Terrain")]
    public static void ShowWindow()
    {
        GetWindow<ComstructionOneClick>("COMSTRUCTION");
    }

    void OnGUI()
    {
        GUILayout.Label("Paths", EditorStyles.boldLabel);
        pythonExe = EditorGUILayout.TextField("Python.exe", pythonExe);
        capturePy = EditorGUILayout.TextField("capture_depth.py", capturePy);
        convertPy = EditorGUILayout.TextField("depth_to_raw.py", convertPy);
        savedDepthDir = EditorGUILayout.TextField("Saved Depth Dir", savedDepthDir);
        heightmapOutDir = EditorGUILayout.TextField("Heightmap Out Dir", heightmapOutDir);

        GUILayout.Space(6);
        GUILayout.Label("Terrain", EditorStyles.boldLabel);
        targetTerrain = (Terrain)EditorGUILayout.ObjectField("Target Terrain", targetTerrain, typeof(Terrain), true);
        terrainHeightmapResolution = EditorGUILayout.IntPopup("Heightmap Resolution", terrainHeightmapResolution,
            new[] { "513", "1025", "2049" }, new[] { 513, 1025, 2049 });
        flipVertically = EditorGUILayout.Toggle("Flip Vertically", flipVertically);
        terrainWidth = EditorGUILayout.FloatField("Width (m)", terrainWidth);
        terrainLength = EditorGUILayout.FloatField("Length (m)", terrainLength);
        terrainHeight = EditorGUILayout.FloatField("Max Height (m)", terrainHeight);

        GUILayout.Space(8);
        if (GUILayout.Button("Run Capture Tool (ROI ���� �� SPACE�� ����)"))
            RunCaptureTool();

        if (GUILayout.Button("Build From Latest (.npy �� .r16 �� Apply)"))
            _ = BuildFromLatestAsync();

        GUILayout.Space(4);
        if (GUILayout.Button("ONE-CLICK (Capture �� Convert �� Apply)"))
            _ = OneClickAsync();
    }

    // ��ȯ��: Process
    Process RunCaptureTool()
    {
        if (!File.Exists(pythonExe) || !File.Exists(capturePy))
        {
            EditorUtility.DisplayDialog("Error", "Python �Ǵ� capture_depth.py ��ΰ� �ùٸ��� �ʽ��ϴ�.", "OK");
            return null;
        }

        var p = new Process();
        p.StartInfo.FileName = pythonExe;
        p.StartInfo.Arguments = $"\"{capturePy}\"";
        p.StartInfo.WorkingDirectory = projectRoot;   // �� CWD ���� �߿�
        p.StartInfo.UseShellExecute = false;
        p.StartInfo.CreateNoWindow = false;           // ROI â ������
        p.Start();

        ShowTimedNotification("ĸó ����: ROI �巡�� �� SPACE ���� �� â �ݱ�", 3f);
        return p;
    }



    async Task OneClickAsync()
    {
        ShowTimedNotification("��Ŭ�� ����", 1.5f);

        var proc = RunCaptureTool();
        if (proc == null) return;

        await Task.Run(() => proc.WaitForExit());
        await Task.Delay(300); // ���� ��� ����

        await BuildFromLatestAsync();

        ComstructionScatter.RunOneClickScatter();

        ShowTimedNotification("��Ŭ�� �Ϸ�", 1.5f);
    }




    async Task BuildFromLatestAsync()
    {
        if (!File.Exists(pythonExe) || !File.Exists(convertPy))
        {
            EditorUtility.DisplayDialog("Error", "Python �Ǵ� depth_to_raw.py ��ΰ� �ùٸ��� �ʽ��ϴ�.", "OK");
            return;
        }

        // �� ��ȯ '��'�� ���� ���� ���
        string before = GetLatestFile(heightmapOutDir, "*.r16");
        DateTime beforeTime = GetLastWriteTimeUtcOrMin(before);

        // ��ȯ ����
        var args = $"\"{convertPy}\" --size={terrainHeightmapResolution}";
        var proc = new Process();
        proc.StartInfo.FileName = pythonExe;
        proc.StartInfo.Arguments = args;
        proc.StartInfo.WorkingDirectory = projectRoot;      // �� �߰� (����� ����)
        proc.StartInfo.UseShellExecute = false;
        proc.StartInfo.CreateNoWindow = true;
        proc.StartInfo.RedirectStandardOutput = true;
        proc.StartInfo.RedirectStandardError = true;
        proc.Start();

        string stdout = await proc.StandardOutput.ReadToEndAsync();
        string stderr = await proc.StandardError.ReadToEndAsync();
        proc.WaitForExit();

        // ��ȯ ��: 'before'�� �ٸ� �� r16�� ���� ������ + ���� ����ȭ ���
        string r16 = await WaitForNewStableFileAsync(heightmapOutDir, "*.r16", before, beforeTime, 60);

        if (string.IsNullOrEmpty(r16))
        {
            UnityEngine.Debug.LogError($"Convert failed or no new .r16.\nOUT:\n{stdout}\nERR:\n{stderr}");
            EditorUtility.DisplayDialog("Error", "��ȯ�� .r16�� ã�� ���߽��ϴ�.", "OK");
            return;
        }

        ApplyR16ToTerrain(r16);
        AssetDatabase.Refresh();
        ShowTimedNotification($"���� �Ϸ�: {Path.GetFileName(r16)}", 3f);
        ComstructionScatter.RunOneClickScatter();
    }



    string GetLatestNpyOrNull()
    {
        if (!Directory.Exists(savedDepthDir)) return null;
        var files = Directory.GetFiles(savedDepthDir, "*.npy");
        if (files.Length == 0) return null;
        return files.OrderBy(File.GetLastWriteTime).Last();
    }

    // ������ "�����ϰ� ũ�Ⱑ N�� ���� ����"�ϸ� ����ȭ �� ������ ����
    bool IsFileStable(string path, int stableSamples = 3, int intervalMs = 200)
    {
        if (string.IsNullOrEmpty(path) || !File.Exists(path)) return false;
        long last = -1;
        int ok = 0;
        for (int i = 0; i < stableSamples * 5; i++) // �ִ� 5����� ��õ�
        {
            if (!File.Exists(path)) { ok = 0; last = -1; Thread.Sleep(intervalMs); continue; }
            long cur = new FileInfo(path).Length;
            if (cur > 0 && cur == last) ok++;
            else { ok = 0; last = cur; }
            if (ok >= stableSamples) return true;
            Thread.Sleep(intervalMs);
        }
        return false;
    }

    string GetLatestFile(string dir, string pattern)
    {
        if (!Directory.Exists(dir)) return null;
        var files = Directory.GetFiles(dir, pattern);
        if (files.Length == 0) return null;
        return files.OrderBy(File.GetLastWriteTimeUtc).Last();
    }

    DateTime GetLastWriteTimeUtcOrMin(string path)
        => string.IsNullOrEmpty(path) || !File.Exists(path) ? DateTime.MinValue : File.GetLastWriteTimeUtc(path);


    // dir �ȿ��� "���� ����" ������ ��ٸ��� + ����ȭ Ȯ��
    async Task<string> WaitForNewStableFileAsync(string dir, string pattern,
                                                 string prevPath, DateTime prevWriteUtc,
                                                 int timeoutSec)
    {
        var begin = DateTime.UtcNow;

        while ((DateTime.UtcNow - begin).TotalSeconds < timeoutSec)
        {
            await Task.Delay(400);

            var cur = GetLatestFile(dir, pattern);
            if (!string.IsNullOrEmpty(cur))
            {
                var curWrite = GetLastWriteTimeUtcOrMin(cur);

                bool isNewPath = !string.Equals(cur, prevPath, StringComparison.OrdinalIgnoreCase);
                bool isNewerWrite = curWrite > prevWriteUtc.AddMilliseconds(50);

                if (isNewPath || isNewerWrite)
                {
                    if (IsFileStable(cur, stableSamples: 3, intervalMs: 200))
                        return cur;
                }
            }
        }
        return null;
    }

    void ApplyR16ToTerrain(string r16Path)
    {
        if (targetTerrain == null)
        {
            targetTerrain = FindObjectOfType<Terrain>();
            if (targetTerrain == null)
            {
                // Terrain�� ������ ���� ����
                var go = Terrain.CreateTerrainGameObject(new TerrainData());
                targetTerrain = go.GetComponent<Terrain>();
            }
        }

        var td = targetTerrain.terrainData;
        // ũ��/�ػ� ����
        td.heightmapResolution = terrainHeightmapResolution;
        td.size = new Vector3(terrainWidth, terrainHeight, terrainLength);

        int size = terrainHeightmapResolution;
        // r16 �б� (little-endian ushort)
        byte[] bytes = File.ReadAllBytes(r16Path);
        int expected = size * size * 2;
        if (bytes.Length < expected)
        {
            UnityEngine.Debug.LogWarning($"R16 size mismatch: {bytes.Length} < {expected}");
        }

        ushort[] u16 = new ushort[size * size];
        System.Buffer.BlockCopy(bytes, 0, u16, 0, Mathf.Min(bytes.Length, expected));

        // Unity�� (y,x) ������ float[ , ] (0..1)
        float[,] heights = new float[size, size];
        // ������ ���� ��->�Ʒ�, ��->��. flip �ɼ� �ݿ�
        for (int y = 0; y < size; y++)
            for (int x = 0; x < size; x++)
            {
                int yy = flipVertically ? (size - 1 - y) : y;
                int idx = yy * size + x;
                float h = u16[idx] / 65535f;
                heights[y, x] = h;
            }

        td.SetHeights(0, 0, heights);
    }

    void ShowTimedNotification(string message, float seconds = 3f)
    {
        // ������ ��ܿ� ��Ƽ(������)
        ShowNotification(new GUIContent(message));
        double until = EditorApplication.timeSinceStartup + seconds;

        // seconds �� �ڵ� ����
        EditorApplication.update += HideLater;
        void HideLater()
        {
            if (EditorApplication.timeSinceStartup >= until)
            {
                RemoveNotification();
                EditorApplication.update -= HideLater;
            }
        }
    }

}


