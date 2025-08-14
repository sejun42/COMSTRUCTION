using System;
using System.Collections.Concurrent;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

public class UdpPositionReceiver : MonoBehaviour
{
    [Header("Target (움직일 대상)")]
    public Transform target;
    [Tooltip("cm → m 변환 (cm를 미터로 쓰려면 0.01)")]
    public float unitScale = 0.01f;
    [Tooltip("(x,y)을 (x,z)로 맵핑")]
    public bool mapXY_to_XZ = true;
    public float fixedY = 0f;

    [Header("UDP 수신")]
    public int listenPort = 5005;
    public bool autoStartUDP = true;

    [Header("위치 적용")]
    public bool applyPosition = true;
    public bool smoothPosition = true;
    [Range(0f, 20f)] public float basePosLerp = 6f;
    public float speedInfluence = 0.05f;
    public Vector2 posLerpClamp = new Vector2(3f, 20f);

    [Header("회전 적용")]
    public bool applyRotation = true;
    [Range(0f, 20f)] public float rotLerpSpeed = 8f;

    [Header("포텐셔미터 → 조인트(최대 3개)")]
    public Transform[] joints = new Transform[3];  // Cube1, Cube2, Cube3
    [Tooltip("각 조인트가 참조할 pot 인덱스(0/1/2)")]
    public int[] potIndex = new int[3] { 0, 1, 2 };
    public int potMin = 0;
    public int potMax = 1023;
    public bool invertPot0 = false, invertPot1 = false, invertPot2 = false;
    public float angleMin = 0f;
    public float angleMax = 120f;
    public enum Axis { X, Y, Z }
    public Axis jointAxis = Axis.Z;

    [Header("디버그 로그")]
    public bool logOpenClose = true;
    public bool logPackets = false;
    public bool logJointApply = false;

    // ---------- 내부 상태 ----------
    [Serializable]
    private class Payload
    {
        public float x, y, speed;  // cm
        public float yaw, pitch, roll;
        public int pot1, pot2, pot3;
    }

    private UdpClient _udp;
    private Thread _udpThread;
    private volatile bool _running;
    private readonly ConcurrentQueue<string> _queue = new ConcurrentQueue<string>();

    private Vector3 _targetPos;
    private Quaternion _targetRot = Quaternion.identity;
    private float _lastSpeed;

    private int _pot1, _pot2, _pot3; // 0~1023

    private Quaternion[] _initialLocalRot = new Quaternion[3];
    private float[] _curAngle = new float[3];

    void Reset()
    {
        if (!target) target = transform;
    }

    void Awake()
    {
        if (!target) target = transform;
        Application.runInBackground = true;

        // joints 초기 로컬 회전 저장
        for (int i = 0; i < joints.Length && i < _initialLocalRot.Length; i++)
        {
            if (joints[i]) _initialLocalRot[i] = joints[i].localRotation;
            _curAngle[i] = 0f;
        }
    }

    void OnEnable()
    {
        if (autoStartUDP) StartUDP();
    }

    void OnDisable()
    {
        StopUDP();
    }

    public void StartUDP()
    {
        try
        {
            StopUDP();
            _udp = new UdpClient(listenPort);
            _running = true;
            _udpThread = new Thread(UdpLoop) { IsBackground = true };
            _udpThread.Start();
            if (logOpenClose) Debug.Log($"[UDP] Listening on {listenPort}");
        }
        catch (Exception ex)
        {
            Debug.LogError($"[UDP] Start error: {ex.Message}");
        }
    }

    public void StopUDP()
    {
        _running = false;
        try { _udp?.Close(); } catch { }
        try { _udpThread?.Join(100); } catch { }
        _udp = null; _udpThread = null;
        if (logOpenClose) Debug.Log("[UDP] Stopped");
    }

    private void UdpLoop()
    {
        IPEndPoint ep = new IPEndPoint(IPAddress.Any, listenPort);
        while (_running)
        {
            try
            {
                var data = _udp.Receive(ref ep);
                var s = Encoding.UTF8.GetString(data).Trim();
                if (!string.IsNullOrEmpty(s)) _queue.Enqueue(s);
            }
            catch { /* 소켓 닫힘 등 무시 */ }
        }
    }

    void Update()
    {
        // 1) 수신처리
        while (_queue.TryDequeue(out var line))
        {
            if (line.Length < 2 || line[0] != '{' || line[line.Length - 1] != '}')
                continue;

            try
            {
                var d = JsonUtility.FromJson<Payload>(line);

                // 위치
                if (applyPosition)
                {
                    float X = d.x * unitScale;
                    float Z = d.y * unitScale;
                    _targetPos = mapXY_to_XZ ? new Vector3(X, fixedY, Z) : new Vector3(X, fixedY, 0f);
                    _lastSpeed = d.speed;
                }

                // 회전 (Unity: yaw=Y, pitch=X, roll=Z)
                if (applyRotation)
                    _targetRot = Quaternion.Euler(d.pitch, d.yaw, d.roll);

                // POT
                _pot1 = d.pot1;
                _pot2 = d.pot2;
                _pot3 = d.pot3;

                if (logPackets)
                    Debug.Log($"[UDP] pos=({d.x:F1},{d.y:F1})cm v={d.speed:F1} | ypr=({d.yaw:F1},{d.pitch:F1},{d.roll:F1}) | pot=({_pot1},{_pot2},{_pot3})");
            }
            catch (Exception e)
            {
                if (logPackets) Debug.LogWarning($"[UDP] JSON parse fail: {e.Message} | line={line}");
            }
        }

        // 2) 위치 보간 적용
        if (applyPosition)
        {
            if (smoothPosition)
            {
                float lerp = Mathf.Clamp(basePosLerp + (_lastSpeed * speedInfluence),
                                         posLerpClamp.x, posLerpClamp.y);
                target.position = Vector3.Lerp(target.position, _targetPos, Time.deltaTime * lerp);
            }
            else
            {
                target.position = _targetPos;
            }
        }

        // 3) 회전 보간 적용
        if (applyRotation)
            target.rotation = Quaternion.Slerp(target.rotation, _targetRot, Time.deltaTime * rotLerpSpeed);

        // 4) POT → Joints
        ApplyPotsToJoints();
    }

    private void ApplyPotsToJoints()
    {
        int[] pots = { _pot1, _pot2, _pot3 };
        bool[] inv = { invertPot0, invertPot1, invertPot2 };

        for (int i = 0; i < joints.Length; i++)
        {
            var j = joints[i];
            if (!j) continue;

            int pIdx = 0;
            if (i < potIndex.Length) pIdx = Mathf.Clamp(potIndex[i], 0, 2);

            int raw = pots[pIdx];
            float t = Mathf.InverseLerp(potMin, potMax, raw);
            t = Mathf.Clamp01(t);
            if (inv[pIdx]) t = 1f - t;

            float targetAngle = Mathf.Lerp(angleMin, angleMax, t);

            // 간단 스무딩 (원하면 별도 per-joint 매개변수로 확장 가능)
            float k = 1f - Mathf.Exp(-Time.deltaTime * 10f);
            _curAngle[i] = Mathf.Lerp(_curAngle[i], targetAngle, k);

            Vector3 e = jointAxis == Axis.X ? new Vector3(_curAngle[i], 0, 0)
                      : jointAxis == Axis.Y ? new Vector3(0, _curAngle[i], 0)
                      : new Vector3(0, 0, _curAngle[i]);

            j.localRotation = _initialLocalRot[i] * Quaternion.Euler(e);

            if (logJointApply)
                Debug.Log($"[JOINT] idx={i} use pot#{pIdx} raw={raw} t={t:0.00} angle={_curAngle[i]:0.0}° axis={jointAxis}");
        }
    }
}
