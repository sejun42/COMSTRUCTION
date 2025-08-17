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
    // ---- 경로/옵션(프로젝트에 맞게 기본값만 바꾸면 됨) ----
    public string pythonExe = @"C:\Users\sejun\anaconda3\envs\comstruction\python.exe";
    public string capturePy = @"D:\COMSTRUCTION\depth_capture\capture_depth.py";
    public string convertPy = @"D:\COMSTRUCTION\scripts\depth_to_raw.py"; // 오토탐지/--center 지원 버전
    public string savedDepthDir = @"D:\COMSTRUCTION\depth_capture\saved_depth_maps";
    public string heightmapOutDir = @"D:\COMSTRUCTION\terrain\heightmaps";
    public string projectRoot = @"D:\COMSTRUCTION";


    public int terrainHeightmapResolution = 513;  // depth_to_raw.py --size와 일치
    public bool flipVertically = false;           // 필요시 뒤집기
    public float terrainWidth = 100f;              // 지형 가로(m)
    public float terrainLength = 100f;             // 지형 세로(m)
    public float terrainHeight = 50f;             // 지형 최대 높이(m) (크면 과장됨)

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
        if (GUILayout.Button("Run Capture Tool (ROI 지정 후 SPACE로 저장)"))
            RunCaptureTool();

        if (GUILayout.Button("Build From Latest (.npy → .r16 → Apply)"))
            _ = BuildFromLatestAsync();

        GUILayout.Space(4);
        if (GUILayout.Button("ONE-CLICK (Capture → Convert → Apply)"))
            _ = OneClickAsync();
    }

    // 반환형: Process
    Process RunCaptureTool()
    {
        if (!File.Exists(pythonExe) || !File.Exists(capturePy))
        {
            EditorUtility.DisplayDialog("Error", "Python 또는 capture_depth.py 경로가 올바르지 않습니다.", "OK");
            return null;
        }

        var p = new Process();
        p.StartInfo.FileName = pythonExe;
        p.StartInfo.Arguments = $"\"{capturePy}\"";
        p.StartInfo.WorkingDirectory = projectRoot;   // ← CWD 고정 중요
        p.StartInfo.UseShellExecute = false;
        p.StartInfo.CreateNoWindow = false;           // ROI 창 떠야함
        p.Start();

        ShowTimedNotification("캡처 실행: ROI 드래그 → SPACE 저장 → 창 닫기", 3f);
        return p;
    }



    async Task OneClickAsync()
    {
        ShowTimedNotification("원클릭 시작", 1.5f);

        var proc = RunCaptureTool();
        if (proc == null) return;

        await Task.Run(() => proc.WaitForExit());
        await Task.Delay(300); // 파일 기록 여유

        await BuildFromLatestAsync();

        ComstructionScatter.RunOneClickScatter();

        ShowTimedNotification("원클릭 완료", 1.5f);
    }




    async Task BuildFromLatestAsync()
    {
        if (!File.Exists(pythonExe) || !File.Exists(convertPy))
        {
            EditorUtility.DisplayDialog("Error", "Python 또는 depth_to_raw.py 경로가 올바르지 않습니다.", "OK");
            return;
        }

        // ★ 변환 '전'에 이전 파일 기억
        string before = GetLatestFile(heightmapOutDir, "*.r16");
        DateTime beforeTime = GetLastWriteTimeUtcOrMin(before);

        // 변환 실행
        var args = $"\"{convertPy}\" --size={terrainHeightmapResolution}";
        var proc = new Process();
        proc.StartInfo.FileName = pythonExe;
        proc.StartInfo.Arguments = args;
        proc.StartInfo.WorkingDirectory = projectRoot;      // ★ 추가 (상대경로 안전)
        proc.StartInfo.UseShellExecute = false;
        proc.StartInfo.CreateNoWindow = true;
        proc.StartInfo.RedirectStandardOutput = true;
        proc.StartInfo.RedirectStandardError = true;
        proc.Start();

        string stdout = await proc.StandardOutput.ReadToEndAsync();
        string stderr = await proc.StandardError.ReadToEndAsync();
        proc.WaitForExit();

        // 변환 후: 'before'와 다른 새 r16이 생길 때까지 + 파일 안정화 대기
        string r16 = await WaitForNewStableFileAsync(heightmapOutDir, "*.r16", before, beforeTime, 60);

        if (string.IsNullOrEmpty(r16))
        {
            UnityEngine.Debug.LogError($"Convert failed or no new .r16.\nOUT:\n{stdout}\nERR:\n{stderr}");
            EditorUtility.DisplayDialog("Error", "변환된 .r16을 찾지 못했습니다.", "OK");
            return;
        }

        ApplyR16ToTerrain(r16);
        AssetDatabase.Refresh();
        ShowTimedNotification($"적용 완료: {Path.GetFileName(r16)}", 3f);
        ComstructionScatter.RunOneClickScatter();
    }



    string GetLatestNpyOrNull()
    {
        if (!Directory.Exists(savedDepthDir)) return null;
        var files = Directory.GetFiles(savedDepthDir, "*.npy");
        if (files.Length == 0) return null;
        return files.OrderBy(File.GetLastWriteTime).Last();
    }

    // 파일이 "존재하고 크기가 N번 연속 동일"하면 안정화 된 것으로 간주
    bool IsFileStable(string path, int stableSamples = 3, int intervalMs = 200)
    {
        if (string.IsNullOrEmpty(path) || !File.Exists(path)) return false;
        long last = -1;
        int ok = 0;
        for (int i = 0; i < stableSamples * 5; i++) // 최대 5배까지 재시도
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


    // dir 안에서 "새로 생긴" 파일을 기다리고 + 안정화 확인
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
                // Terrain이 없으면 새로 생성
                var go = Terrain.CreateTerrainGameObject(new TerrainData());
                targetTerrain = go.GetComponent<Terrain>();
            }
        }

        var td = targetTerrain.terrainData;
        // 크기/해상도 설정
        td.heightmapResolution = terrainHeightmapResolution;
        td.size = new Vector3(terrainWidth, terrainHeight, terrainLength);

        int size = terrainHeightmapResolution;
        // r16 읽기 (little-endian ushort)
        byte[] bytes = File.ReadAllBytes(r16Path);
        int expected = size * size * 2;
        if (bytes.Length < expected)
        {
            UnityEngine.Debug.LogWarning($"R16 size mismatch: {bytes.Length} < {expected}");
        }

        ushort[] u16 = new ushort[size * size];
        System.Buffer.BlockCopy(bytes, 0, u16, 0, Mathf.Min(bytes.Length, expected));

        // Unity는 (y,x) 형태의 float[ , ] (0..1)
        float[,] heights = new float[size, size];
        // 파일은 보통 위->아래, 좌->우. flip 옵션 반영
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
        // 에디터 상단에 노티(비차단)
        ShowNotification(new GUIContent(message));
        double until = EditorApplication.timeSinceStartup + seconds;

        // seconds 후 자동 숨김
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


