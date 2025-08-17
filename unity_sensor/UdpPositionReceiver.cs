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
    [Tooltip("cm -> m 변환 (cm를 미터로 쓰려면 0.01)")]
    public float unitScale = 0.01f;
    [Tooltip("(x,y)을 (x,z)로 매핑")]
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
    [Range(0f, 200f)] public float rotLerpSpeed = 0f; // 0 = 즉시 적용

    [Header("디버그")]
    public bool logOpenClose = true;
    public bool logPackets = false;

    // -------------------- Animator (최소->중간->최소 왕복) --------------------
    [Header("Animator (Motion Time 제어)")]
    public Animator animator;
    public string scrubStateName = "Take 001";
    public int scrubLayer = 0;
    public string motionTimeParam = "pot1Value";

    [Header("pot1 정규화 범위(실측값)")]
    public int pot1Min = 450;
    public int pot1Max = 701;
    public bool invertInput = false;

    [Header("입력(원시 pot1) 안정화")]
    public int inputDeadband = 3;
    [Range(0f, 60f)] public float inputLowpass = 12f;

    [Header("진행도(t) 스무딩/속도 제한")]
    [Range(0.01f, 1f)] public float tSmoothTime = 0.20f;
    public float maxTPerSecond = 0.25f;

    // ===== 시작점 + 상대이동 =====
    [Header("앵커(시작점) + 상대이동")]
    public bool useRelativeFromFirstPacket = true;
    public Vector3 startWorldPos = new Vector3(9.7f, 22.06f, 33.68f);
    public bool setStartOnEnable = true;

    // ===== 이동 강증폭(원하면 사용) =====
    [Header("이동 증폭(강)")]
    [Tooltip("단위 변환 이후 전체 배수. 1=그대로, 100=100배")]
    public float positionGain = 100f;
    [Tooltip("축별 배수(X,Z)")]
    public float axisGainX = 1f;
    public float axisGainZ = 1f;
    [Tooltip("비선형 지수. 1=선형, 2~3=작은 변화도 크게")]
    public float nonlinearPower = 1.5f;
    [Tooltip("스케일 전 데드존(m)")]
    public float deadzoneMeters = 0.001f;
    [Tooltip("스케일 후 최소 이동 보장(m)")]
    public float minVisibleStep = 0.05f;
    [Tooltip("수신 속도 기반 추가 게인. finalGain *= (1 + speed*speedGain)")]
    public float speedGain = 0.0f;

    // ===== 자이로 회전 (Y축: 받은 값 그대로 + 방향 스위치) =====
    [Header("Gyro 회전(Y축: 받은 값 그대로)")]
    [Tooltip("쿼터니언(qw,qx,qy,qz)로 오면 켬, yaw/pitch/roll로 오면 끔")]
    public bool useQuaternionRotation = false;
    [Tooltip("yaw/pitch/roll이 라디안이면 켜기 (라디안→도 변환)")]
    public bool gyroInRadians = false;

    [Header("Y축 방향/보정")]
    [Tooltip("회전 방향. -1=반대로(기본), 1=원래 방향")]
    public float yDirection = -1f;    // ★ 기본: 반대 방향
    [Tooltip("추가 오프셋(도). 필요 없으면 0")]
    public float yOffsetDeg = 0f;
    [Tooltip("로컬 Y축으로 회전(부모 기준) / 월드 Y축(기본)")]
    public bool useLocalY = false;

    // -------------------- 내부 상태 --------------------
    [Serializable]
    private class Payload
    {
        public float x, y, speed;         // 위치 cm
        public float yaw, pitch, roll;    // 도 또는 라디안
        public int pot1, pot2, pot3;
        // 선택: 쿼터니언 (보내면 사용)
        public float qw, qx, qy, qz;      // 단위 쿼터니언 (w,x,y,z)
    }

    private UdpClient _udp;
    private Thread _udpThread;
    private volatile bool _running;
    private readonly ConcurrentQueue<string> _queue = new ConcurrentQueue<string>();

    private Vector3 _targetPos;
    private Quaternion _targetRot = Quaternion.identity;
    private float _lastSpeed;

    // pot
    private int _pot1;

    // 애니 제어 준비/필터
    bool _scrubInited = false;
    int _scrubHash;
    bool _animInit = false;
    int _lastRawPot1;
    float _filtPot1;

    // 진행도(t)
    float _t = 0f;
    float _tVel = 0f;

    // 상대 이동 기준
    bool _haveBaseXY = false;
    Vector2 _baseXY;

    void Reset() { if (!target) target = transform; }

    void Awake()
    {
        if (!target) target = transform;
        Application.runInBackground = true;
        if (animator == null) animator = GetComponentInChildren<Animator>();
        _scrubHash = Animator.StringToHash(scrubStateName);
    }

    void OnEnable()
    {
        if (setStartOnEnable && target) target.position = startWorldPos;
        _haveBaseXY = false;
        if (autoStartUDP) StartUDP();
    }

    void OnDisable() { StopUDP(); }

    [ContextMenu("CalibrateOriginToCurrentSample")]
    public void CalibrateOriginToCurrentSample()
    {
        _haveBaseXY = false; // 다음 패킷을 XY 원점으로
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
        catch (Exception ex) { Debug.LogError($"[UDP] Start error: {ex.Message}"); }
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
            catch { /* ignore */ }
        }
    }

    void Update()
    {
        // 1) 수신 처리
        while (_queue.TryDequeue(out var line))
        {
            if (line.Length < 2 || line[0] != '{' || line[line.Length - 1] != '}')
                continue;

            try
            {
                var d = JsonUtility.FromJson<Payload>(line);

                // 위치 계산/적용
                if (applyPosition)
                {
                    if (useRelativeFromFirstPacket && !_haveBaseXY)
                    {
                        _baseXY = new Vector2(d.x, d.y);
                        _haveBaseXY = true;
                    }

                    if (useRelativeFromFirstPacket && _haveBaseXY)
                    {
                        float dx_m = (d.x - _baseXY.x) * unitScale;
                        float dz_m = (d.y - _baseXY.y) * unitScale;

                        float dx_s = ScaleDelta(dx_m, d.speed) * axisGainX;
                        float dz_s = ScaleDelta(dz_m, d.speed) * axisGainZ;

                        if (mapXY_to_XZ)
                            _targetPos = new Vector3(startWorldPos.x + dx_s, startWorldPos.y, startWorldPos.z + dz_s);
                        else
                            _targetPos = new Vector3(startWorldPos.x + dx_s, startWorldPos.y, startWorldPos.z);
                    }
                    else
                    {
                        float X_m = d.x * unitScale;
                        float Z_m = d.y * unitScale;

                        float X_s = ScaleDelta(X_m, d.speed) * axisGainX;
                        float Z_s = ScaleDelta(Z_m, d.speed) * axisGainZ;

                        _targetPos = mapXY_to_XZ ? new Vector3(X_s, fixedY, Z_s) : new Vector3(X_s, fixedY, 0f);
                    }

                    _lastSpeed = d.speed;
                }

                // ======= Y축 회전: "받은 각도 그대로" + 방향 스위치 =======
                if (applyRotation)
                {
                    float yDeg;
                    bool hasQuat = Mathf.Abs(d.qw) + Mathf.Abs(d.qx) + Mathf.Abs(d.qy) + Mathf.Abs(d.qz) > 0.0001f;

                    if (useQuaternionRotation && hasQuat)
                    {
                        // (qw,qx,qy,qz) → Y(Euler)만 사용
                        var q = new Quaternion(d.qx, d.qy, d.qz, d.qw);
                        yDeg = q.eulerAngles.y; // 0~360
                    }
                    else
                    {
                        // yaw 값 그대로(라디안이면 변환)
                        yDeg = gyroInRadians ? d.yaw * Mathf.Rad2Deg : d.yaw;
                    }

                    // 방향 전환 & 오프셋
                    yDeg *= (yDirection >= 0f ? 1f : -1f);  // ★ 여기서 반대로
                    yDeg += yOffsetDeg;

                    // 최종 목표 회전(Y만)
                    if (useLocalY)
                    {
                        var e = target.localEulerAngles;
                        e.y = yDeg;
                        _targetRot = Quaternion.Euler(e);
                    }
                    else
                    {
                        _targetRot = Quaternion.AngleAxis(yDeg, Vector3.up);
                    }
                }

                _pot1 = d.pot1;

                if (logPackets)
                    Debug.Log($"[UDP] y={(gyroInRadians ? d.yaw * Mathf.Rad2Deg : d.yaw):F1} deg  dir={(yDirection >= 0f ? "+" : "-")} | pos=({d.x:F1},{d.y:F1})");
            }
            catch (Exception e)
            {
                if (logPackets) Debug.LogWarning($"[UDP] JSON parse fail: {e.Message} | line={line}");
            }
        }

        // 2) 위치/회전 적용
        if (applyPosition && target)
        {
            if (smoothPosition)
            {
                float lerp = Mathf.Clamp(basePosLerp + (_lastSpeed * speedInfluence), posLerpClamp.x, posLerpClamp.y);
                target.position = Vector3.Lerp(target.position, _targetPos, Time.deltaTime * lerp);
            }
            else
            {
                target.position = _targetPos;
            }
        }

        if (applyRotation && target)
        {
            if (rotLerpSpeed <= 0f)
            {
                // 즉시 적용
                if (useLocalY) target.localRotation = _targetRot;
                else target.rotation = _targetRot;
            }
            else
            {
                // 부드럽게 따라가기
                if (useLocalY)
                    target.localRotation = Quaternion.Slerp(target.localRotation, _targetRot, Time.deltaTime * rotLerpSpeed);
                else
                    target.rotation = Quaternion.Slerp(target.rotation, _targetRot, Time.deltaTime * rotLerpSpeed);
            }
        }

        // 3) 애니메이션 진행도 갱신(필요 시)
        if (animator) UpdateStartMiddleStart();
    }

    // 이동 스케일링: meters in -> meters out (강증폭)
    float ScaleDelta(float deltaMeters, float speed)
    {
        float v = deltaMeters;
        float sign = Mathf.Sign(v);
        float mag = Mathf.Abs(v);

        if (mag <= deadzoneMeters) return 0f;
        mag -= deadzoneMeters;

        if (Mathf.Abs(nonlinearPower - 1f) > 0.0001f)
            mag = Mathf.Pow(mag, Mathf.Max(0.0001f, nonlinearPower));

        float g = Mathf.Max(0f, positionGain);
        if (speedGain != 0f) g *= (1f + Mathf.Max(0f, speed) * speedGain);

        v = sign * mag * g;

        if (v != 0f && Mathf.Abs(v) < minVisibleStep)
            v = sign * minVisibleStep;

        return v;
    }

    // === 애니메이션 로직 ===
    void UpdateStartMiddleStart()
    {
        if (!_scrubInited)
        {
            animator.cullingMode = AnimatorCullingMode.AlwaysAnimate;
            animator.speed = 0f;
            animator.Play(_scrubHash, scrubLayer, 0f);
            animator.Update(0f);
            _scrubInited = true;
        }

        if (!_animInit) { _lastRawPot1 = _pot1; _filtPot1 = _pot1; _animInit = true; }

        int raw = _pot1;
        if (Mathf.Abs(raw - _lastRawPot1) < inputDeadband) raw = _lastRawPot1;
        else _lastRawPot1 = raw;

        float filt = raw;
        if (inputLowpass > 0f)
        {
            float k = 1f - Mathf.Exp(-Time.deltaTime * inputLowpass);
            _filtPot1 = Mathf.Lerp(_filtPot1, filt, k);
        }
        else _filtPot1 = filt;

        float s = Mathf.InverseLerp(pot1Min, pot1Max, _filtPot1);
        s = Mathf.Clamp01(s);
        if (invertInput) s = 1f - s;

        // 0 -> 0.5 -> 0 로 사용
        float tTarget = s * 0.5f;

        float prev = _t;
        _t = Mathf.SmoothDamp(_t, tTarget, ref _tVel, tSmoothTime);
        float maxDelta = Mathf.Max(0f, maxTPerSecond) * Time.deltaTime;
        float delta = Mathf.Clamp(_t - prev, -maxDelta, maxDelta);
        _t = prev + delta;

        animator.SetFloat(motionTimeParam, _t);
    }
}
