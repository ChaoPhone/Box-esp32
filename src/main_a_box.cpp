#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// =============================================================
//  A 板箱子端 (main_a_box) — MPU6050 IMU + ESP-NOW 无线发送
// -------------------------------------------------------------
//  职责：IMU 采样 → Mahony 融合 → 推动/倾斜/抬举检测
//        ESP-NOW 发送 EVT/HEARTBEAT/IMUQ 到 B 板（桥接板）
//        接收 B 板命令（HAPTIC→电机, STATE→LED）
//
//  A 板 MAC: E8:3D:C1:F2:C7:B8    B 板 MAC: E8:3D:C1:FA:7A:0C
//
//  构建：pio run -e a_box -t upload
// =============================================================

// ---------------- 协议常量 ----------------
static const char *FIRMWARE_VERSION = "0.9.1-haptic-tilt";
static const int PROTO_VERSION = 1;
static const uint32_t ACK_TIMEOUT_MS = 300;
static const int MAX_RETRY = 5;
static const uint32_t HEARTBEAT_INTERVAL_MS = 1000;

// ---------------- 硬件引脚 ----------------
static const int BUTTON_PIN = 0;
static const int RGB_LED_PIN = 48;
static const int I2C_SDA_PIN = 8;
static const int I2C_SCL_PIN = 9;
static const int MPU_INT_PIN = 10;
static const int MOTOR_PIN = 1;
static const int MOTOR_PWM_CH = 0;
static const int MOTOR_PWM_FREQ = 1000;
static const int MOTOR_PWM_RES = 8;
static const uint8_t LED_BRIGHTNESS = 40;

// ---------------- MPU6050 ----------------
static const uint8_t MPU_ADDR = 0x68;
static const uint8_t MPU_REG_SMPLRT_DIV = 0x19;
static const uint8_t MPU_REG_CONFIG = 0x1A;
static const uint8_t MPU_REG_GYRO_CONFIG = 0x1B;
static const uint8_t MPU_REG_ACCEL_CONFIG = 0x1C;
static const uint8_t MPU_REG_INT_ENABLE = 0x38;
static const uint8_t MPU_REG_ACCEL_XOUT_H = 0x3B;
static const uint8_t MPU_REG_PWR_MGMT_1 = 0x6B;
static const uint8_t MPU_REG_WHO_AM_I = 0x75;
static const float ACCEL_LSB_PER_G = 16384.0f;
static const float GYRO_LSB_PER_DPS = 131.0f;
static const uint32_t IMU_SAMPLE_US = 5000;
static const uint32_t IMU_STREAM_NORMAL_MS = 50;
static const uint32_t IMU_STREAM_DEBUG_MS = 20;
static const uint32_t PUSH_WARMUP_MS = 1500;

// ---------------- ESP-NOW ----------------
static const uint8_t WIFI_CHANNEL = 1;
static const uint8_t MAC_A[6] = {0xE8, 0x3D, 0xC1, 0xF2, 0xC7, 0xB8};
static const uint8_t MAC_B[6] = {0xE8, 0x3D, 0xC1, 0xFA, 0x7A, 0x0C};
static uint8_t PEER_ADDR[6] = {0};
static bool gPeerKnown = false;
static volatile uint32_t foreignFrames = 0;
static char gBoard = '?';

struct RxMsg { uint8_t len; uint8_t mac[6]; char data[240]; };
static QueueHandle_t rxQueue;

// ---------------- 运行状态 ----------------
static uint32_t sequenceCounter = 0;
static bool debugMode = false;
static uint32_t lastHeartbeat = 0;
static uint32_t motorEndMs = 0;

struct PendingEvent {
  bool active = false;
  uint32_t sequence = 0;
  String action;
  float value = 0.0f;
  uint32_t timestamp_ms = 0;
  uint32_t last_send_ms = 0;
  int retry = 0;
};
static PendingEvent pending;

// ---------------- IMU 状态 ----------------
static bool imuPresent = false;
static uint32_t lastImuStream = 0;
static uint32_t lastImuSampleUs = 0;
static float gyroBias[3] = {0, 0, 0};
static float imuAccel[3] = {0, 0, 0};
static float imuGyro[3] = {0, 0, 0};
static float mahonyKp = 1.0f, mahonyKi = 0.0f;
static float mahonyQ[4] = {1, 0, 0, 0};
static float mahonyIntFB[3] = {0, 0, 0};

// ---------------- 推动检测 ----------------
static const float PUSH_THRESH = 0.20f;
static const uint32_t PUSH_PEAK_WIN_MS = 120;
static const uint32_t PUSH_DEBOUNCE_MS = 600;
static const uint32_t PUSH_RETURN_SUPPRESS_MS = 900;
static const uint32_t PUSH_SAME_DIR_SUPPRESS_MS = 100;
static const float PUSH_REARM_THRESH = 0.15f;
static const uint32_t PUSH_REARM_QUIET_MS = 200;
static float pushAccMA[3] = {0, 0, 0};
static bool pushBaselineReady = false;
static uint32_t pushDebounceUntil = 0;
static uint32_t pushArmedAt = 0;
static uint32_t pushCaptureUntil = 0;
static float pushPeakX = 0, pushPeakY = 0, pushPeakMag = 0;
static char pushLastDir = 0;
static uint32_t pushLastEmitMs = 0;
static bool pushNeedRearm = false;
static uint32_t pushQuietSince = 0;

// ---------------- 倾斜检测 ----------------
static const float TILT_ANGLE_THRESH_DEG = 30.0f;
static const uint32_t TILT_DEBOUNCE_MS = 800;
static const uint32_t TILT_SAME_DIR_SUPPRESS_MS = 3000;
static const float TILT_REARM_ANGLE_DEG = 20.0f;
static const uint32_t TILT_REARM_QUIET_MS = 300;
static char tiltLastDir = 0;
static uint32_t tiltLastEmitMs = 0;
static uint32_t tiltDebounceUntil = 0;
static bool tiltNeedRearm = false;
static uint32_t tiltQuietSince = 0;

// ---------------- 抬举检测 ----------------
static const float TILT_LIFT_THRESH_G = 0.35f;
static float liftWzMA = 0.0f;
static bool liftWzBaselineReady = false;
static uint32_t liftDebounceUntil = 0;
static uint32_t liftLastEmitMs = 0;

// ---- 翻面检测：翻转 > 80° → UNDO ----
static const float FLIP_THRESH_DEG = 80.0f;       // 翻面触发角度
static const uint32_t FLIP_DEBOUNCE_MS = 2000;     // 触发后冷却
static const uint32_t FLIP_HOLD_MS = 300;           // 需持续翻转时长（防误触）
static bool flipArmed = true;
static uint32_t flipLastMs = 0;
static uint32_t flipStartMs = 0;
static bool flipConfirmed = false;

// ---- 弹珠模式：保持倾斜 > 45° 持续 2s → 进入, 放平自动退出 ----
static bool marbleActive = false;
static uint32_t marbleTiltStart = 0;
static const float MARBLE_ENTER_DEG = 45.0f;
static const float MARBLE_EXIT_DEG = 15.0f;
static const uint32_t MARBLE_HOLD_MS = 2000;

// ---------------- 方向校准 ----------------
static bool pushSwapXY = false;
static int8_t pushSignX = 1, pushSignY = 1;

// ==================== 工具函数 ====================
static void setLed(uint8_t r, uint8_t g, uint8_t b) { neopixelWrite(RGB_LED_PIN, r, g, b); }

// ---------- 电机模式队列（多次脉冲，如胜利三震） ----------
struct MotorStep { uint8_t on; uint32_t ms; };
static const int MAX_MOTOR_STEPS = 8;
static MotorStep motorSteps[MAX_MOTOR_STEPS];
static int motorPatternCount = 0;
static int motorPatternIdx = 0;
static uint32_t motorPatternStepEnd = 0;

static void motorOn(uint8_t strength, uint32_t ms) {
  ledcWrite(MOTOR_PWM_CH, strength);
  motorEndMs = millis() + ms;
  motorPatternCount = 0;  // 取消模式播放
}

static void motorQueuePattern(const MotorStep *steps, int n) {
  if (n > MAX_MOTOR_STEPS) n = MAX_MOTOR_STEPS;
  memcpy(motorSteps, steps, sizeof(MotorStep) * n);
  motorPatternCount = n;
  motorPatternIdx = 0;
  motorPatternStepEnd = 0;
  motorEndMs = 0;
}

static void motorTick() {
  uint32_t now = millis();
  if (motorPatternCount > 0) {
    if (motorPatternStepEnd == 0 || now >= motorPatternStepEnd) {
      if (motorPatternIdx >= motorPatternCount) {
        ledcWrite(MOTOR_PWM_CH, 0);
        motorPatternCount = 0;
        return;
      }
      ledcWrite(MOTOR_PWM_CH, motorSteps[motorPatternIdx].on);
      motorPatternStepEnd = now + motorSteps[motorPatternIdx].ms;
      motorPatternIdx++;
    }
  } else if (motorEndMs && now >= motorEndMs) {
    ledcWrite(MOTOR_PWM_CH, 0);
    motorEndMs = 0;
  }
}

static void sendLog(const char *text) {
  char buf[128]; snprintf(buf, sizeof(buf), "LOG,%s", text);
  esp_now_send(PEER_ADDR, (const uint8_t *)buf, strlen(buf));
}

static void sendEventFrame() {
  char buf[96];
  snprintf(buf, sizeof(buf), "EVT,%lu,%s,%.1f,%lu",
           (unsigned long)pending.sequence, pending.action.c_str(),
           pending.value, (unsigned long)pending.timestamp_ms);
  esp_now_send(PEER_ADDR, (const uint8_t *)buf, strlen(buf));
}

static void espnowSendStr(const char *s) {
  esp_now_send(PEER_ADDR, (const uint8_t *)s, strlen(s));
}

// ==================== 板身份 ====================
static char boardLetter() {
  String mac = WiFi.macAddress(); mac.toUpperCase();
  if (mac == "E8:3D:C1:F2:C7:B8") return 'A';
  if (mac == "E8:3D:C1:FA:7A:0C") return 'B';
  return '?';
}

// ==================== ESP-NOW ====================
static void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (gPeerKnown && memcmp(mac, PEER_ADDR, 6) != 0) { foreignFrames++; return; }
  RxMsg m;
  m.len = (len > 239) ? 239 : (uint8_t)len;
  memcpy(m.data, data, m.len);
  m.data[m.len] = '\0';
  memcpy(m.mac, mac, 6);
  xQueueSend(rxQueue, &m, 0);
}

static void espnowSetup() {
  rxQueue = xQueueCreate(8, sizeof(RxMsg));
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  memcpy(PEER_ADDR, MAC_B, 6);  // A 的对端是 B
  gPeerKnown = true;
  if (esp_now_init() != ESP_OK) { Serial.println("LOG,espnow fail"); return; }
  esp_now_register_recv_cb(onEspNowRecv);
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, PEER_ADDR, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
}

// ==================== EVT 管理 ====================
static void emitEvent(const String &action, float value) {
  if (pending.active) return;
  sequenceCounter++;
  pending.active = true;
  pending.sequence = sequenceCounter;
  pending.action = action;
  pending.value = value;
  pending.timestamp_ms = millis();
  pending.retry = 0;
  pending.last_send_ms = millis();
  sendEventFrame();
}

static void handleAck(uint32_t seq) {
  if (pending.active && pending.sequence == seq) pending.active = false;
}

// ==================== MPU6050 ====================
static bool mpuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool mpuReadRegs(uint8_t reg, uint8_t *out, uint8_t count) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t got = Wire.requestFrom((int)MPU_ADDR, (int)count);
  if (got != count) return false;
  for (uint8_t i = 0; i < count; i++) out[i] = Wire.read();
  return true;
}

static bool imuInit() {
  uint8_t who = 0;
  if (!mpuReadRegs(MPU_REG_WHO_AM_I, &who, 1)) return false;
  if (who != 0x68) return false;
  mpuWriteReg(MPU_REG_PWR_MGMT_1, 0x00); delay(10);
  mpuWriteReg(MPU_REG_SMPLRT_DIV, 0x07);
  mpuWriteReg(MPU_REG_CONFIG, 0x03);
  mpuWriteReg(MPU_REG_GYRO_CONFIG, 0x00);
  mpuWriteReg(MPU_REG_ACCEL_CONFIG, 0x00);
  mpuWriteReg(MPU_REG_INT_ENABLE, 0x01);
  return true;
}

static bool imuRead() {
  uint8_t raw[14];
  if (!mpuReadRegs(MPU_REG_ACCEL_XOUT_H, raw, 14)) return false;
  int16_t ax = (int16_t)((raw[0] << 8) | raw[1]);
  int16_t ay = (int16_t)((raw[2] << 8) | raw[3]);
  int16_t az = (int16_t)((raw[4] << 8) | raw[5]);
  int16_t gx = (int16_t)((raw[8] << 8) | raw[9]);
  int16_t gy = (int16_t)((raw[10] << 8) | raw[11]);
  int16_t gz = (int16_t)((raw[12] << 8) | raw[13]);
  imuAccel[0] = ax / ACCEL_LSB_PER_G;
  imuAccel[1] = ay / ACCEL_LSB_PER_G;
  imuAccel[2] = az / ACCEL_LSB_PER_G;
  imuGyro[0] = gx / GYRO_LSB_PER_DPS - gyroBias[0];
  imuGyro[1] = gy / GYRO_LSB_PER_DPS - gyroBias[1];
  imuGyro[2] = gz / GYRO_LSB_PER_DPS - gyroBias[2];
  return true;
}

static bool imuCalibrateGyro(int samples) {
  float sum[3] = {0, 0, 0}; int ok = 0;
  gyroBias[0] = gyroBias[1] = gyroBias[2] = 0;
  for (int i = 0; i < samples; i++) {
    if (imuRead()) { sum[0] += imuGyro[0]; sum[1] += imuGyro[1]; sum[2] += imuGyro[2]; ok++; }
    delay(3);
  }
  if (ok == 0) return false;
  gyroBias[0] = sum[0] / ok; gyroBias[1] = sum[1] / ok; gyroBias[2] = sum[2] / ok;
  return true;
}

static bool mahonyDetectStatic(float gx, float gy, float gz, float ax, float ay, float az) {
  float gn = sqrt(gx*gx + gy*gy + gz*gz);
  float an = sqrt(ax*ax + ay*ay + az*az);
  return (gn < 0.05f && fabs(an - 1.0f) < 0.1f);
}

static void mahonyReset() {
  mahonyQ[0] = 1; mahonyQ[1] = mahonyQ[2] = mahonyQ[3] = 0;
  mahonyIntFB[0] = mahonyIntFB[1] = mahonyIntFB[2] = 0;
}

// ==================== Mahony AHRS ====================
static void mahonyUpdate(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  float accMag = sqrt(ax*ax + ay*ay + az*az);
  float accTrust = 1.0f - fabs(accMag - 1.0f) * 4.0f;
  if (accTrust < 0) accTrust = 0;
  float gyroMag = sqrt(gx*gx + gy*gy + gz*gz);
  float gyroTrust = 1.0f - gyroMag * 0.002f;
  if (gyroTrust < 0.3f) gyroTrust = 0.3f;
  float adaptiveKp = mahonyKp * accTrust * gyroTrust;
  const float D2R = 0.0174533f;
  float wx = gx * D2R, wy = gy * D2R, wz = gz * D2R;
  float accNorm = sqrt(ax*ax + ay*ay + az*az);
  if (accNorm > 1e-6f) {
    float nax = ax / accNorm, nay = ay / accNorm, naz = az / accNorm;
    float halfvx = mahonyQ[1]*mahonyQ[3] - mahonyQ[0]*mahonyQ[2];
    float halfvy = mahonyQ[0]*mahonyQ[1] + mahonyQ[2]*mahonyQ[3];
    float halfvz = mahonyQ[0]*mahonyQ[0] - 0.5f + mahonyQ[3]*mahonyQ[3];
    float halfex = nay*halfvz - naz*halfvy;
    float halfey = naz*halfvx - nax*halfvz;
    float halfez = nax*halfvy - nay*halfvx;
    wx += adaptiveKp * halfex; wy += adaptiveKp * halfey; wz += adaptiveKp * halfez;
  }
  float halfDt = 0.5f * dt;
  float q0 = mahonyQ[0], q1 = mahonyQ[1], q2 = mahonyQ[2], q3 = mahonyQ[3];
  mahonyQ[0] += (-q1*wx - q2*wy - q3*wz) * halfDt;
  mahonyQ[1] += ( q0*wx + q2*wz - q3*wy) * halfDt;
  mahonyQ[2] += ( q0*wy - q1*wz + q3*wx) * halfDt;
  mahonyQ[3] += ( q0*wz + q1*wy - q2*wx) * halfDt;
  float norm = 1.0f / sqrt(mahonyQ[0]*mahonyQ[0] + mahonyQ[1]*mahonyQ[1] + mahonyQ[2]*mahonyQ[2] + mahonyQ[3]*mahonyQ[3]);
  mahonyQ[0] *= norm; mahonyQ[1] *= norm; mahonyQ[2] *= norm; mahonyQ[3] *= norm;
}

static void quatRotateWorld(const float q[4], float vx, float vy, float vz, float &wx, float &wy, float &wz) {
  float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
  wx = (1 - 2*(qy*qy + qz*qz))*vx + 2*(qx*qy - qw*qz)*vy + 2*(qx*qz + qw*qy)*vz;
  wy = 2*(qx*qy + qw*qz)*vx + (1 - 2*(qx*qx + qz*qz))*vy + 2*(qy*qz - qw*qx)*vz;
  wz = 2*(qx*qz - qw*qy)*vx + 2*(qy*qz + qw*qx)*vy + (1 - 2*(qx*qx + qy*qy))*vz;
}

static void sendImuQuat() {
  char buf[96];
  snprintf(buf, sizeof(buf), "IMUQ,%.4f,%.4f,%.4f,%.4f,%lu",
           mahonyQ[0], mahonyQ[1], mahonyQ[2], mahonyQ[3], (unsigned long)millis());
  espnowSendStr(buf);
}

// 倾斜持续状态上报（与 IMUQ 同频）：TILTS,<pitchDeg>,<rollDeg>,<ms>
static void sendTiltState() {
  float qw = mahonyQ[0], qx = mahonyQ[1], qy = mahonyQ[2], qz = mahonyQ[3];
  float roll  = atan2f(2.0f*(qw*qx+qy*qz), 1.0f-2.0f*(qx*qx+qy*qy));
  float sinp  = 2.0f*(qw*qy-qz*qx);
  if (sinp >  1.0f) sinp =  1.0f;
  if (sinp < -1.0f) sinp = -1.0f;
  float pitch = asinf(sinp);
  float rollDeg  = roll  * 57.29578f;
  float pitchDeg = pitch * 57.29578f;
  char buf[96];
  snprintf(buf, sizeof(buf), "TILTS,%.2f,%.2f,%lu",
           pitchDeg, rollDeg, (unsigned long)millis());
  espnowSendStr(buf);
}

// ==================== 推动检测 ====================
static void pushDetectReset() {
  pushBaselineReady = false;
  pushAccMA[0] = pushAccMA[1] = pushAccMA[2] = 0;
  pushCaptureUntil = 0; pushLastDir = 0;
  pushNeedRearm = false; pushQuietSince = 0;
  pushArmedAt = millis() + PUSH_WARMUP_MS;
  tiltLastDir = 0; tiltDebounceUntil = 0;
  tiltNeedRearm = false; tiltQuietSince = 0;
  liftWzBaselineReady = false; liftWzMA = 0;
  liftDebounceUntil = 0;
  flipArmed = true;
  flipStartMs = 0;
  flipConfirmed = false;
  marbleActive = false;
  marbleTiltStart = 0;
}

static const char* pushDetect(float bx, float by) {
  if (!pushBaselineReady) { pushAccMA[0] = bx; pushAccMA[1] = by; pushBaselineReady = true; return nullptr; }
  const float alpha = 0.02f;
  pushAccMA[0] = pushAccMA[0] * (1-alpha) + bx * alpha;
  pushAccMA[1] = pushAccMA[1] * (1-alpha) + by * alpha;
  float dynX = bx - pushAccMA[0], dynY = by - pushAccMA[1];
  uint32_t now = millis();
  if (now < pushArmedAt) return nullptr;
  float mag = sqrt(dynX*dynX + dynY*dynY);
  if (pushCaptureUntil != 0) {
    if (mag > pushPeakMag) { pushPeakMag = mag; pushPeakX = dynX; pushPeakY = dynY; }
    if (now < pushCaptureUntil) return nullptr;
    pushCaptureUntil = 0; pushNeedRearm = true; pushQuietSince = 0;
    float px = pushPeakX, py = pushPeakY;
    if (pushSwapXY) { float t = px; px = py; py = t; }
    px *= pushSignX; py *= pushSignY;
    char dir;
    if (fabs(px) > fabs(py)) dir = (px > 0) ? 'R' : 'L';
    else                     dir = (py > 0) ? 'F' : 'B';
    bool opp = (dir=='F'&&pushLastDir=='B') || (dir=='B'&&pushLastDir=='F') || (dir=='L'&&pushLastDir=='R') || (dir=='R'&&pushLastDir=='L');
    if (opp && now - pushLastEmitMs < PUSH_RETURN_SUPPRESS_MS) { pushLastDir = 0; return nullptr; }
    if (dir == pushLastDir && now - pushLastEmitMs < PUSH_SAME_DIR_SUPPRESS_MS) return nullptr;
    pushLastDir = dir; pushLastEmitMs = now;
    switch (dir) { case 'F': return "MOVE_FORWARD"; case 'B': return "MOVE_BACKWARD"; case 'L': return "MOVE_LEFT"; default: return "MOVE_RIGHT"; }
  }
  if (pushNeedRearm) {
    if (mag < PUSH_REARM_THRESH) { if (pushQuietSince == 0) pushQuietSince = now; else if (now - pushQuietSince >= PUSH_REARM_QUIET_MS) { pushNeedRearm = false; pushQuietSince = 0; } }
    else pushQuietSince = 0;
    return nullptr;
  }
  if (now < pushDebounceUntil) return nullptr;
  if (mag < PUSH_THRESH) return nullptr;
  pushCaptureUntil = now + PUSH_PEAK_WIN_MS; pushDebounceUntil = now + PUSH_DEBOUNCE_MS;
  pushPeakMag = mag; pushPeakX = dynX; pushPeakY = dynY;
  return nullptr;
}

// ==================== 倾斜检测 ====================
static const char* tiltDetect(float &outValue) {
  float qw = mahonyQ[0], qx = mahonyQ[1], qy = mahonyQ[2], qz = mahonyQ[3];
  float roll = atan2f(2.0f*(qw*qx+qy*qz), 1.0f-2.0f*(qx*qx+qy*qy));
  float sinp = 2.0f*(qw*qy-qz*qx);
  if (sinp > 1.0f) sinp = 1.0f; if (sinp < -1.0f) sinp = -1.0f;
  float pitch = asinf(sinp);
  float rollDeg = roll * 57.29578f, pitchDeg = pitch * 57.29578f;
  uint32_t now = millis();
  if (now < pushArmedAt) return nullptr;
  if (tiltNeedRearm) {
    float maxAngle = (fabsf(rollDeg) > fabsf(pitchDeg)) ? fabsf(rollDeg) : fabsf(pitchDeg);
    if (maxAngle < TILT_REARM_ANGLE_DEG) {
      if (tiltQuietSince == 0) tiltQuietSince = now;
      else if (now - tiltQuietSince >= TILT_REARM_QUIET_MS) { tiltNeedRearm = false; tiltQuietSince = 0; }
    } else tiltQuietSince = 0;
    return nullptr;
  }
  if (now < tiltDebounceUntil) return nullptr;
  float pitchRatio = fabsf(pitchDeg) / TILT_ANGLE_THRESH_DEG;
  float rollRatio  = fabsf(rollDeg)  / TILT_ANGLE_THRESH_DEG;
  char bestDir = 0; float bestRatio = 0, bestValue = 0;
  if (pitchRatio >= 1.0f && pitchRatio > bestRatio) { bestRatio = pitchRatio; bestDir = (pitchDeg > 0.0f) ? 'R' : 'L'; bestValue = fabsf(pitchDeg); }
  if (rollRatio >= 1.0f && rollRatio > bestRatio)   { bestRatio = rollRatio;   bestDir = (rollDeg > 0.0f) ? 'B' : 'F'; bestValue = fabsf(rollDeg); }
  if (bestDir == 0) return nullptr;
  if (bestDir == tiltLastDir && now - tiltLastEmitMs < TILT_SAME_DIR_SUPPRESS_MS) return nullptr;
  tiltLastDir = bestDir; tiltLastEmitMs = now;
  tiltDebounceUntil = now + TILT_DEBOUNCE_MS;
  tiltNeedRearm = true; tiltQuietSince = 0; outValue = bestValue;
  switch (bestDir) { case 'F': return "TILT_FORWARD"; case 'B': return "TILT_BACKWARD"; case 'L': return "TILT_LEFT"; case 'R': return "TILT_RIGHT"; }
  return nullptr;
}

// ==================== 抬举检测 ====================
static const char* liftDetect(float wzWorld) {
  uint32_t now = millis();
  if (!liftWzBaselineReady) { liftWzMA = wzWorld; liftWzBaselineReady = true; return nullptr; }
  const float alpha = 0.02f;
  liftWzMA = liftWzMA * (1.0f - alpha) + wzWorld * alpha;
  float wzDyn = wzWorld - liftWzMA;
  if (now < pushArmedAt) return nullptr;
  if (now < liftDebounceUntil) return nullptr;
  if (wzDyn < TILT_LIFT_THRESH_G) return nullptr;
  if (now - liftLastEmitMs < TILT_SAME_DIR_SUPPRESS_MS) return nullptr;
  liftDebounceUntil = now + TILT_DEBOUNCE_MS; liftLastEmitMs = now;
  return "LIFT_UP";
}

// ==================== 下行命令处理 ====================

// ==================== 下行命令处理 ====================
static void handleCommand(const char *raw) {
  String line(raw); line.trim();
  if (line.length() == 0) return;

  if (line.startsWith("ACK,")) {
    handleAck((uint32_t)line.substring(4).toInt());
    return;
  }
  if (line.startsWith("STATE,")) {
    String s = line.substring(6); s.trim(); s.toUpperCase();
    uint8_t r = 0, g = 0, b = 0;
    if (s == "IDLE" || s == "READY")      { g = LED_BRIGHTNESS; }
    else if (s == "MOVE" || s == "STEP")  { b = LED_BRIGHTNESS; }
    else if (s == "WALL" || s == "BLOCK") { r = LED_BRIGHTNESS; g = LED_BRIGHTNESS/3; }
    else if (s == "WIN" || s == "SUCCESS"){ r = LED_BRIGHTNESS; g = LED_BRIGHTNESS; }
    else if (s == "LOSE" || s == "FAIL")  { r = LED_BRIGHTNESS; }
    else if (s == "OFF") { /* 灭 */ }
    setLed(r, g, b);
    return;
  }
  if (line.startsWith("HAPTIC,")) {
    String args = line.substring(7);
    int c = args.indexOf(',');
    String pattern = (c >= 0) ? args.substring(0, c) : args;
    pattern.trim();
    uint32_t duration = 150;
    if (c >= 0) { String msStr = args.substring(c+1); msStr.trim(); if (msStr.length()>0) duration = msStr.toInt(); }
    uint8_t strength = 255;
    if (pattern == "short") { motorOn(255, 50); }
    else if (pattern == "long") { motorOn(255, 300); }
    else if (pattern.equalsIgnoreCase("win")) {
      // 胜利长震三次：300ms 开，100ms 关，重复三次
      MotorStep winSteps[] = {
        {255, 300}, {0, 100},
        {255, 300}, {0, 100},
        {255, 300}
      };
      motorQueuePattern(winSteps, 5);
    }
    else { int s = pattern.toInt(); if (s>0 && s<=255) strength = (uint8_t)s; motorOn(strength, duration); }
    setLed(LED_BRIGHTNESS, 0, LED_BRIGHTNESS);
    return;
  }
  if (line.startsWith("CALIBRATE")) {
    if (imuPresent) {
      if (imuCalibrateGyro(200)) { mahonyReset(); pushDetectReset(); }
    }
    return;
  }
  if (line.startsWith("ORIENT,")) {
    int c1 = line.indexOf(','), c2 = line.indexOf(',', c1+1), c3 = line.indexOf(',', c2+1);
    pushSwapXY = line.substring(c1+1, c2).toInt() != 0;
    pushSignX = (int8_t)(line.substring(c2+1, c3).toInt() < 0 ? -1 : 1);
    pushSignY = (int8_t)(line.substring(c3+1).toInt() < 0 ? -1 : 1);
    return;
  }
  if (line.startsWith("DEBUG,")) { debugMode = line.substring(6).toInt() != 0; return; }
}

// ==================== 生命周期 ====================
void setup() {
  Serial.begin(115200);  // 本地调试用
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(MPU_INT_PIN, INPUT);

  // 电机 PWM
  ledcSetup(MOTOR_PWM_CH, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcAttachPin(MOTOR_PIN, MOTOR_PWM_CH);
  ledcWrite(MOTOR_PWM_CH, 0);

  delay(200);

  // IMU
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);
  imuPresent = imuInit();

  if (imuPresent) {
    setLed(LED_BRIGHTNESS, LED_BRIGHTNESS, 0); // 黄灯校准中
    int stillFrames = 0; uint32_t waitStart = millis();
    while (stillFrames < 30 && millis() - waitStart < 3000) {
      if (imuRead() && mahonyDetectStatic(imuGyro[0], imuGyro[1], imuGyro[2], imuAccel[0], imuAccel[1], imuAccel[2]))
        stillFrames++;
      else stillFrames = 0;
      delay(5);
    }
    imuCalibrateGyro(200);
    mahonyReset();
    pushDetectReset();
  }

  // ESP-NOW
  espnowSetup();
  gBoard = boardLetter();
  Serial.printf("LOG,[%c] A-board ready, MAC %s, IMU=%d\n", gBoard, WiFi.macAddress().c_str(), imuPresent);
  setLed(0, LED_BRIGHTNESS, 0); // 绿灯就绪

  char boot[48];
  snprintf(boot, sizeof(boot), "BOOT,%s,proto=%d", FIRMWARE_VERSION, PROTO_VERSION);
  espnowSendStr(boot);
  lastHeartbeat = millis();
}

void loop() {
  // 下行命令
  RxMsg m;
  while (xQueueReceive(rxQueue, &m, 0)) handleCommand(m.data);

  // 重传
  if (pending.active) {
    uint32_t now = millis();
    if (now - pending.last_send_ms >= ACK_TIMEOUT_MS) {
      if (pending.retry >= MAX_RETRY) { pending.active = false; }
      else { pending.retry++; pending.last_send_ms = now; sendEventFrame(); }
    }
  }

  // 按钮
  static int lastBtn = HIGH, stableBtn = HIGH; static uint32_t btnChange = 0;
  int reading = digitalRead(BUTTON_PIN); uint32_t now = millis();
  if (reading != lastBtn) { lastBtn = reading; btnChange = now; }
  if ((now - btnChange) > 40 && stableBtn != reading) {
    stableBtn = reading;
    if (stableBtn == LOW) emitEvent("MOVE_RIGHT", 200.0f);
  }

  // IMU 采样
  if (imuPresent) {
    uint32_t nowUs = micros();
    if (nowUs - lastImuSampleUs >= IMU_SAMPLE_US) {
      float dt = (lastImuSampleUs > 0) ? (nowUs - lastImuSampleUs) * 1e-6f : 0.005f;
      if (dt < 0.001f) dt = 0.001f; if (dt > 0.05f) dt = 0.05f;
      lastImuSampleUs = nowUs;

      if (imuRead()) {
        mahonyUpdate(imuGyro[0], imuGyro[1], imuGyro[2], imuAccel[0], imuAccel[1], imuAccel[2], dt);

        // 推动
        float wx, wy, wz;
        quatRotateWorld(mahonyQ, imuAccel[0], imuAccel[1], imuAccel[2], wx, wy, wz);
        float yawSin = 2.0f * (mahonyQ[0]*mahonyQ[3] + mahonyQ[1]*mahonyQ[2]);
        float yawCos = 1.0f - 2.0f * (mahonyQ[2]*mahonyQ[2] + mahonyQ[3]*mahonyQ[3]);
        float yaw = atan2f(yawSin, yawCos);
        float cy = cosf(yaw), sy = sinf(yaw);
        float bx = cy*wx + sy*wy, by = -sy*wx + cy*wy;
        uint32_t nowMs = millis();

        // ---- 翻面检测：翻转 > 80° 保持 300ms → UNDO ----
        {
          float qwf = mahonyQ[0], qxf = mahonyQ[1], qyf = mahonyQ[2], qzf = mahonyQ[3];
          float rf  = atan2f(2.0f*(qwf*qxf+qyf*qzf), 1.0f-2.0f*(qxf*qxf+qyf*qyf));
          float sp  = 2.0f*(qwf*qyf-qzf*qxf);
          if (sp >  1.0f) sp =  1.0f;
          if (sp < -1.0f) sp = -1.0f;
          float maxA = fmaxf(fabsf(rf * 57.29578f), fabsf(asinf(sp) * 57.29578f));

          if (maxA > FLIP_THRESH_DEG && flipArmed && !flipConfirmed) {
            if (flipStartMs == 0) flipStartMs = nowMs;
            else if (nowMs - flipStartMs > FLIP_HOLD_MS && nowMs - flipLastMs > FLIP_DEBOUNCE_MS) {
              emitEvent("UNDO", 0);
              flipArmed = false; flipConfirmed = true; flipLastMs = nowMs;
              motorOn(200, 100); // 短震确认
            }
          } else if (maxA < FLIP_THRESH_DEG - 10.0f) {
            flipStartMs = 0;
            flipConfirmed = false;
          }

          // 弹珠模式：保持倾斜 > 45° 持续 2s → 进入, 放平 < 15° → 退出
          if (!marbleActive && maxA > MARBLE_ENTER_DEG) {
            if (marbleTiltStart == 0) marbleTiltStart = nowMs;
            else if (nowMs - marbleTiltStart > MARBLE_HOLD_MS) {
              emitEvent("MARBLE_ENTER", 0);
              marbleActive = true; marbleTiltStart = 0;
              motorOn(255, 80); // 短震
            }
          } else if (marbleActive && maxA < MARBLE_EXIT_DEG) {
            emitEvent("MARBLE_EXIT", 0);
            marbleActive = false; marbleTiltStart = 0;
          } else if (maxA < MARBLE_ENTER_DEG - 5.0f) {
            marbleTiltStart = 0; // 放平重置计时
          }

          if (maxA < 30.0f) flipArmed = true; // 翻面回正重新武装
        }

        // 倾斜 → 已改为持续状态 TILTS 消息上报（不再发 EVT）
        float tiltVal = 0;
        tiltDetect(tiltVal); // 维持 re-arm 状态机，但不发射 EVT

        // 推动（中等优先级）
        const char* pushDir = pushDetect(bx, by);
        if (pushDir) emitEvent(pushDir, 200.0f);

        // 抬举（最低优先级 + 震动）
        const char* liftDir = liftDetect(wz);
        if (liftDir) {
          emitEvent(liftDir, 200.0f);
          motorOn(255, 150);
        }

        // 上报 IMUQ
        uint32_t interval = debugMode ? IMU_STREAM_DEBUG_MS : IMU_STREAM_NORMAL_MS;
        if (nowMs - lastImuStream >= interval) {
          lastImuStream = nowMs;
          sendImuQuat();
          sendTiltState();
        }
      }
    }
  }

  // 心跳
  if (now - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeat = now;
    char buf[48];
    snprintf(buf, sizeof(buf), "HEARTBEAT,%lu,STABLE", (unsigned long)now);
    espnowSendStr(buf);
  }

  // 电机超时/模式播放
  motorTick();
}
