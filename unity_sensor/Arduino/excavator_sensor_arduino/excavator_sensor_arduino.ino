#include <Wire.h>
#include "MPU6050.h"

MPU6050 imu;

int prevPot1=-1, prevPot2=-1, prevPot3=-1;
const int threshold=5;            // 변경 감지 민감도 (ADC 값 차이)

// === 포텐셔미터 보정 ===
const int POT1_MIN = 0;           // 필요시 교정 (측정해서 넣기)
const int POT1_MAX = 1023;
const int POT2_MIN = 0;
const int POT2_MAX = 1023;
const int POT3_MIN = 0;
const int POT3_MAX = 1023;

const float ANGLE_MIN = 0.0;      // 각도 매핑 범위(도)
const float ANGLE_MAX = 120.0;

bool INV_POT1 = false;            // 방향 반대면 true
bool INV_POT2 = false;
bool INV_POT3 = false;

// 디버그 출력 주기 제한
unsigned long lastLogMs = 0;
const unsigned long LOG_INTERVAL_MS = 250;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  imu.initialize();
  delay(500);
}

static float roll=0, pitch=0, yaw=0;

void loop() {
  // --- IMU ---
  static unsigned long lastUs = micros();
  unsigned long nowUs = micros();
  float dt = (nowUs - lastUs)/1e6; 
  lastUs = nowUs;

  int16_t ax, ay, az, gx, gy, gz;
  imu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);

  const float g_lsb=16384.0, dps_lsb=131.0;
  float axg=ax/g_lsb, ayg=ay/g_lsb, azg=az/g_lsb;
  float gx_dps=gx/dps_lsb, gy_dps=gy/dps_lsb, gz_dps=gz/dps_lsb;

  float roll_acc  = atan2(ayg, azg) * 180.0/PI;
  float pitch_acc = atan2(-axg, sqrt(ayg*ayg + azg*azg)) * 180.0/PI;

  const float alpha=0.98;
  roll  = alpha*(roll + gx_dps*dt) + (1-alpha)*roll_acc;
  pitch = alpha*(pitch + gy_dps*dt) + (1-alpha)*pitch_acc;
  yaw  += gz_dps*dt;
  if(yaw>180) yaw-=360;
  if(yaw<-180) yaw+=360;

  // --- 포텐셔미터 원값 ---
  int pot1=analogRead(A0);
  int pot2=analogRead(A1);
  int pot3=analogRead(A2);

  // 변화 감지(노이즈 억제)
  bool ch1 = (abs(pot1-prevPot1)>=threshold);
  bool ch2 = (abs(pot2-prevPot2)>=threshold);
  bool ch3 = (abs(pot3-prevPot3)>=threshold);
  if(ch1) prevPot1=pot1;
  if(ch2) prevPot2=pot2;
  if(ch3) prevPot3=pot3;

  // --- 사람이 보기 쉬운 디버그 출력 ---
  unsigned long nowMs = millis();
  if (ch1 || ch2 || ch3 || (nowMs - lastLogMs) > LOG_INTERVAL_MS) {
    // 정규화 0~1
    float t1 = constrain((float)(prevPot1 - POT1_MIN) / max(1, POT1_MAX - POT1_MIN), 0.0f, 1.0f);
    float t2 = constrain((float)(prevPot2 - POT2_MIN) / max(1, POT2_MAX - POT2_MIN), 0.0f, 1.0f);
    float t3 = constrain((float)(prevPot3 - POT3_MIN) / max(1, POT3_MAX - POT3_MIN), 0.0f, 1.0f);
    if (INV_POT1) t1 = 1.0f - t1;
    if (INV_POT2) t2 = 1.0f - t2;
    if (INV_POT3) t3 = 1.0f - t3;

    // 각도 매핑
    float a1 = ANGLE_MIN + (ANGLE_MAX - ANGLE_MIN) * t1;
    float a2 = ANGLE_MIN + (ANGLE_MAX - ANGLE_MIN) * t2;
    float a3 = ANGLE_MIN + (ANGLE_MAX - ANGLE_MIN) * t3;

    // 사람용 라인 (Unity/파이썬 파서에 영향 없도록 '#' 프리픽스)
    Serial.print("# POT1 raw="); Serial.print(prevPot1);
    Serial.print(" t="); Serial.print(t1,2);
    Serial.print(" -> "); Serial.print(a1,1); Serial.print(" deg | ");

    Serial.print("POT2 raw="); Serial.print(prevPot2);
    Serial.print(" t="); Serial.print(t2,2);
    Serial.print(" -> "); Serial.print(a2,1); Serial.print(" deg | ");

    Serial.print("POT3 raw="); Serial.print(prevPot3);
    Serial.print(" t="); Serial.print(t3,2);
    Serial.print(" -> "); Serial.print(a3,1); Serial.println(" deg");

    lastLogMs = nowMs;
  }

  // --- JSON (기계용) ---
  Serial.print("{");
  Serial.print("\"roll\":");  Serial.print(roll,2);
  Serial.print(",\"pitch\":");Serial.print(pitch,2);
  Serial.print(",\"yaw\":");  Serial.print(yaw,2);
  Serial.print(",\"pot1\":"); Serial.print(prevPot1);
  Serial.print(",\"pot2\":"); Serial.print(prevPot2);
  Serial.print(",\"pot3\":"); Serial.print(prevPot3);
  Serial.println("}");

  delay(10); // ≈100Hz
}
