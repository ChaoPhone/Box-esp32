#include <Arduino.h>
#include <Wire.h>

// =============================================================
//  B-Box ESP32 固件 —— 有线串口联调（含 MPU6050 IMU）
//  构建环境： pio run -e wired_sim -t upload
// -------------------------------------------------------------
//  实现 B-Box Serial Protocol v1（完整规范见 docs/PROTOCOL.md）。
//  传输：USB CDC 串口 115200 8N1，每条消息一行 ASCII，以 '\n' 结尾，
//        字段用逗号分隔，首字段为大写 TYPE。
//
//  IMU：MPU6050 挂 I2C（SDA=GPIO8/SCL=GPIO9，地址 0x68），接线见
//        docs/MPU6050_WIRING.md。上电自动检测：接好则上报 IMU 遥测，
//        未接/接错则优雅降级为纯模拟，协议其余功能不受影响。
//
//  上行（板 -> Bridge/Unity）：
//    BOOT,<fw>,proto=<n>                 启动一次
//    HEARTBEAT,<ms>,<state>              约 1Hz 心跳
//    EVT,<seq>,<action>,<value>,<ms>     高层动作事件（需 ACK），action 可为：
//                                        MOVE_FORWARD/BACKWARD/LEFT/RIGHT（水平推动）
//                                        TILT_FORWARD/BACKWARD/LEFT/RIGHT（倾斜 >45°）
//                                        LIFT_UP（向上抬举加速度脉冲）
//    OK,<TYPE>[,<detail...>]             下行命令已执行的确认
//    NAK,<TYPE>,<reason>                 下行命令被拒绝/未识别
//    PONG[,<token>]                      对 PING 的应答
//    IMUQ,<qw>,<qx>,<qy>,<qz>,<ms>       Mahony 四元数姿态（常态 + DEBUG 都上报）
//    IMU,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<ms> 原始IMU遥测（仅 DEBUG 模式，用于前端调试）
//    LOG,<text>                          人类可读日志
//
//  下行（Bridge/Unity -> 板）：
//    STATE,<name>        游戏状态可视化（IDLE/MOVE/WALL/WIN/LOSE/OFF...）
//    LED,<r>,<g>,<b>     直接设色（0-255），调试/覆盖
//    HAPTIC,<pattern>[,<ms>]  触觉反馈（马达未到货，先用灯+屏蔽窗口模拟）
//    ACK,<seq>           确认某条上行 EVT
//    PING[,<token>]      存活探测 -> 板回 PONG
//    CALIBRATE           传感器标定
//    DEBUG,<0|1>         详细日志开关
//    ORIENT,<swapXY>,<signX>,<signY>  推动方向校准（轴交换+符号）
//    SIM,<action>        仅开发用：模拟触发一次上行动作
//
//  扩展规则（保证向前兼容）：未知 TYPE -> 回 NAK 且不崩；命令末尾多余字段忽略。
//
//  手动测试（串口监视器直接输入）：
//    w/a/s/d = MOVE_FORWARD/LEFT/BACKWARD/RIGHT   q/e = ROTATE_CCW / ROTATE_CW
//    板载 BOOT 按钮按一下 = 触发一次 MOVE_RIGHT
// =============================================================

static const char *FIRMWARE_VERSION = "0.8.1-rearm";
static const int PROTO_VERSION = 1;   // B-Box Serial Protocol 版本
static const uint32_t BAUD = 115200;

// 板载 BOOT 按钮通常接在 GPIO0，可当作手动触发按钮
static const int BUTTON_PIN = 0;
// 板载 WS2812 全彩灯（GPIO48），用作“游戏状态/反馈”的可视出口
static const int RGB_LED_PIN = 48;

// ---- MPU6050 IMU 接线（详见 docs/MPU6050_WIRING.md）----
static const int I2C_SDA_PIN = 8;    // MPU6050 SDA
static const int I2C_SCL_PIN = 9;    // MPU6050 SCL
static const int MPU_INT_PIN = 10;   // MPU6050 INT（数据就绪中断，当前以轮询为主）
static const uint8_t MPU_ADDR = 0x68;      // AD0 接 GND → 0x68
static const uint8_t MPU_REG_SMPLRT_DIV = 0x19;
static const uint8_t MPU_REG_CONFIG = 0x1A;
static const uint8_t MPU_REG_GYRO_CONFIG = 0x1B;
static const uint8_t MPU_REG_ACCEL_CONFIG = 0x1C;
static const uint8_t MPU_REG_INT_ENABLE = 0x38;
static const uint8_t MPU_REG_ACCEL_XOUT_H = 0x3B;
static const uint8_t MPU_REG_PWR_MGMT_1 = 0x6B;
static const uint8_t MPU_REG_WHO_AM_I = 0x75;
static const float ACCEL_LSB_PER_G = 16384.0f;    // ±2g 量程
static const float GYRO_LSB_PER_DPS = 131.0f;     // ±250°/s 量程

static const uint32_t HEARTBEAT_INTERVAL_MS = 1000;  // 心跳间隔
static const uint32_t ACK_TIMEOUT_MS = 300;          // 等 ACK 超时后重传
static const int MAX_RETRY = 5;                      // 最大重传次数
static const uint32_t HAPTIC_GUARD_MS = 150;         // 震动屏蔽窗口（模拟）
static const uint32_t BUTTON_DEBOUNCE_MS = 40;       // 按钮消抖
static const uint32_t IMU_SAMPLE_US = 5000;          // IMU 采样固定 200Hz（micros 定时，与上报解耦）
static const uint32_t IMU_STREAM_NORMAL_MS = 50;     // 常态 IMU 上报间隔（20Hz）
static const uint32_t IMU_STREAM_DEBUG_MS = 20;      // DEBUG 开启后 50Hz 密采上报
static const uint32_t PUSH_WARMUP_MS = 1500;         // 开机/标定后静默，等高通基线与姿态收敛

// ---------------- 运行状态 ----------------
static uint32_t sequenceCounter = 0;
static bool debugMode = false;
static uint32_t lastHeartbeat = 0;
static uint32_t hapticGuardUntil = 0;

// ---------------- IMU 运行状态 ----------------
static bool imuPresent = false;        // 上电检测结果：MPU6050 是否在线
static uint32_t lastImuStream = 0;      // 上次上报 IMU 遥测的时刻（millis，上报限流用）
static uint32_t lastImuSampleUs = 0;    // 上次采样时刻（micros，与上报解耦的 200Hz 节拍）
static float gyroBias[3] = {0, 0, 0};   // 陀螺零偏（CALIBRATE 后填充，单位 °/s）
// 最近一次读数：加速度(g) ax/ay/az，角速度(°/s) gx/gy/gz
static float imuAccel[3] = {0, 0, 0};
static float imuGyro[3] = {0, 0, 0};

// ---- 推动方向校准（运行时由 ORIENT 命令配置，默认 = 原硬编码行为）----
static bool     pushSwapXY = false;     // 水平面内是否交换 X↔Y
static int8_t   pushSignX  = 1;         // 水平面 X 符号
static int8_t   pushSignY  = 1;         // 水平面 Y 符号

// 单条待确认事件（阶段 2 简化为单条 pending，足够联调）
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

// ---------------- 发送辅助 ----------------
static void sendEventLine() {
  char buf[96];
  snprintf(buf, sizeof(buf), "EVT,%lu,%s,%.1f,%lu",
           (unsigned long)pending.sequence,
           pending.action.c_str(),
           pending.value,
           (unsigned long)pending.timestamp_ms);
  Serial.println(buf);
}

static void sendLog(const char *text) {
  Serial.print("LOG,");
  Serial.println(text);
}

// 下行命令确认：OK,<TYPE>[,<detail>] / NAK,<TYPE>,<reason>
static void sendOk(const char *type, const char *detail) {
  Serial.print("OK,");
  Serial.print(type);
  if (detail && detail[0]) {
    Serial.print(',');
    Serial.print(detail);
  }
  Serial.println();
}

static void sendNak(const char *type, const char *reason) {
  Serial.print("NAK,");
  Serial.print(type);
  Serial.print(',');
  Serial.println(reason);
}

// ---------------- MPU6050 IMU ----------------
// 写单个寄存器；返回 I2C 事务是否成功
static bool mpuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

// 从 reg 起连续读 count 字节到 out；返回是否读满
static bool mpuReadRegs(uint8_t reg, uint8_t *out, uint8_t count) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;  // repeated start
  uint8_t got = Wire.requestFrom((int)MPU_ADDR, (int)count);
  if (got != count) return false;
  for (uint8_t i = 0; i < count; i++) out[i] = Wire.read();
  return true;
}

// 初始化并检测 MPU6050：成功返回 true 并写好量程/滤波
static bool imuInit() {
  uint8_t who = 0;
  if (!mpuReadRegs(MPU_REG_WHO_AM_I, &who, 1)) return false;
  if (who != 0x68) {                 // WHO_AM_I 不符 → 认为未接/接错
    char buf[48];
    snprintf(buf, sizeof(buf), "IMU not found (whoami=0x%02X)", who);
    sendLog(buf);
    return false;
  }
  mpuWriteReg(MPU_REG_PWR_MGMT_1, 0x00);     // 解除睡眠
  delay(10);
  mpuWriteReg(MPU_REG_SMPLRT_DIV, 0x07);     // 采样分频
  mpuWriteReg(MPU_REG_CONFIG, 0x03);         // DLPF ~44Hz
  mpuWriteReg(MPU_REG_GYRO_CONFIG, 0x00);    // 陀螺 ±250°/s
  mpuWriteReg(MPU_REG_ACCEL_CONFIG, 0x00);   // 加速度 ±2g
  mpuWriteReg(MPU_REG_INT_ENABLE, 0x01);     // 数据就绪中断（INT 引脚备用）
  sendLog("IMU ok whoami=0x68");
  return true;
}

// 读取一帧加速度/陀螺，转换为 g / °/s，写入全局最近值；返回是否成功
static bool imuRead() {
  uint8_t raw[14];
  if (!mpuReadRegs(MPU_REG_ACCEL_XOUT_H, raw, 14)) return false;
  int16_t ax = (int16_t)((raw[0] << 8) | raw[1]);
  int16_t ay = (int16_t)((raw[2] << 8) | raw[3]);
  int16_t az = (int16_t)((raw[4] << 8) | raw[5]);
  // raw[6..7] 为温度，本项目暂不使用
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

// 静态标定陀螺零偏：板子静止时采样求平均，作为后续读数的偏置
static bool imuCalibrateGyro(int samples) {
  float sum[3] = {0, 0, 0};
  int ok = 0;
  gyroBias[0] = gyroBias[1] = gyroBias[2] = 0.0f;  // 先清零再采原始值
  for (int i = 0; i < samples; i++) {
    if (imuRead()) {
      sum[0] += imuGyro[0];
      sum[1] += imuGyro[1];
      sum[2] += imuGyro[2];
      ok++;
    }
    delay(3);
  }
  if (ok == 0) return false;
  gyroBias[0] = sum[0] / ok;
  gyroBias[1] = sum[1] / ok;
  gyroBias[2] = sum[2] / ok;
  return true;
}

// 上报一帧 IMU 遥测：IMU,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<ms>（保留兼容，DEBUG 模式用）
static void sendImuTelemetry() {
  char buf[96];
  snprintf(buf, sizeof(buf), "IMU,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%lu",
           imuAccel[0], imuAccel[1], imuAccel[2],
           imuGyro[0], imuGyro[1], imuGyro[2],
           (unsigned long)millis());
  Serial.println(buf);
}

// ---- Mahony AHRS 互补滤波（移植自 2025HeroGimbal FusionAHRS） ----
// 陀螺仪积分 + 加速度计重力修正 → 四元数姿态输出

static float mahonyKp = 1.0f;       // 比例增益（加速度计信任度）
static float mahonyKi = 0.0f;       // 积分增益
static float mahonyQ[4] = {1,0,0,0}; // 四元数 (qw,qx,qy,qz)
static float mahonyIntFB[3] = {0,0,0};

static void mahonyReset() {
  mahonyQ[0] = 1; mahonyQ[1] = 0; mahonyQ[2] = 0; mahonyQ[3] = 0;
  mahonyIntFB[0] = 0; mahonyIntFB[1] = 0; mahonyIntFB[2] = 0;
}

// 检测静止：陀螺+加速度都接近零/1g
static bool mahonyDetectStatic(float gx, float gy, float gz,
                                float ax, float ay, float az) {
  float gyroNorm = sqrt(gx*gx + gy*gy + gz*gz);
  float accNorm  = sqrt(ax*ax + ay*ay + az*az);
  return (gyroNorm < 0.05f && fabs(accNorm - 1.0f) < 0.1f);
}

// Mahony 互补滤波更新 — 加速度单位 g，陀螺单位 °/s
// dt 为真实时间差（秒），由调用方用 micros() 计算并钳制
static void mahonyUpdate(float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float dt) {
  // 自适应 Kp：加速度偏离 1g 时降低信任
  float accMag = sqrt(ax*ax + ay*ay + az*az);
  float accTrust = 1.0f - fabs(accMag - 1.0f) * 4.0f;
  if (accTrust < 0) accTrust = 0;
  float gyroMag = sqrt(gx*gx + gy*gy + gz*gz);
  float gyroTrust = 1.0f - gyroMag * 0.002f;
  if (gyroTrust < 0.3f) gyroTrust = 0.3f;
  float adaptiveKp = mahonyKp * accTrust * gyroTrust;

  // 度/秒 → 弧度/秒
  const float D2R = 0.0174533f; // PI/180
  float wx = gx * D2R;
  float wy = gy * D2R;
  float wz = gz * D2R;

  // 加速度归一化
  float accNorm = sqrt(ax*ax + ay*ay + az*az);
  if (accNorm > 1e-6f) {
    float nax = ax / accNorm;
    float nay = ay / accNorm;
    float naz = az / accNorm;

    // 从四元数估计重力方向
    float halfvx = mahonyQ[1]*mahonyQ[3] - mahonyQ[0]*mahonyQ[2];
    float halfvy = mahonyQ[0]*mahonyQ[1] + mahonyQ[2]*mahonyQ[3];
    float halfvz = mahonyQ[0]*mahonyQ[0] - 0.5f + mahonyQ[3]*mahonyQ[3];

    // 叉积 = 姿态误差
    float halfex = nay*halfvz - naz*halfvy;
    float halfey = naz*halfvx - nax*halfvz;
    float halfez = nax*halfvy - nay*halfvx;

    // PI 修正
    if (mahonyKi > 1e-6f) {
      mahonyIntFB[0] += mahonyKi * halfex * dt;
      mahonyIntFB[1] += mahonyKi * halfey * dt;
      mahonyIntFB[2] += mahonyKi * halfez * dt;
      wx += adaptiveKp * halfex + mahonyIntFB[0];
      wy += adaptiveKp * halfey + mahonyIntFB[1];
      wz += adaptiveKp * halfez + mahonyIntFB[2];
    } else {
      wx += adaptiveKp * halfex;
      wy += adaptiveKp * halfey;
      wz += adaptiveKp * halfez;
    }
  }

  // 四元数一阶积分: dq/dt = 0.5 * q ⊗ ω（真实 dt）
  float halfDt = 0.5f * dt;
  float q0 = mahonyQ[0], q1 = mahonyQ[1], q2 = mahonyQ[2], q3 = mahonyQ[3];
  mahonyQ[0] += (-q1*wx - q2*wy - q3*wz) * halfDt;
  mahonyQ[1] += ( q0*wx + q2*wz - q3*wy) * halfDt;
  mahonyQ[2] += ( q0*wy - q1*wz + q3*wx) * halfDt;
  mahonyQ[3] += ( q0*wz + q1*wy - q2*wx) * halfDt;

  // 归一化
  float norm = 1.0f / sqrt(mahonyQ[0]*mahonyQ[0] + mahonyQ[1]*mahonyQ[1] +
                           mahonyQ[2]*mahonyQ[2] + mahonyQ[3]*mahonyQ[3]);
  mahonyQ[0] *= norm; mahonyQ[1] *= norm; mahonyQ[2] *= norm; mahonyQ[3] *= norm;
}

// 上报四元数：IMUQ,<qw>,<qx>,<qy>,<qz>,<ms>
static void sendImuQuat() {
  char buf[96];
  snprintf(buf, sizeof(buf), "IMUQ,%.4f,%.4f,%.4f,%.4f,%lu",
           mahonyQ[0], mahonyQ[1], mahonyQ[2], mahonyQ[3],
           (unsigned long)millis());
  Serial.println(buf);
}

// 用四元数把传感器帧向量旋转到世界帧：v_world = q ⊗ v ⊗ q*
// 输出世界帧加速度，Z 轴为重力方向，X/Y 为水平面（姿态无关）
static void quatRotateWorld(const float q[4], float vx, float vy, float vz,
                            float &wx, float &wy, float &wz) {
  float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
  wx = (1 - 2*(qy*qy + qz*qz)) * vx + 2*(qx*qy - qw*qz) * vy + 2*(qx*qz + qw*qy) * vz;
  wy = 2*(qx*qy + qw*qz) * vx + (1 - 2*(qx*qx + qz*qz)) * vy + 2*(qy*qz - qw*qx) * vz;
  wz = 2*(qx*qz - qw*qy) * vx + 2*(qy*qz + qw*qx) * vy + (1 - 2*(qx*qx + qy*qy)) * vz;
}

// ---- 推动脉冲检测 ----
// 机体航向系水平加速度（倾斜补偿、去 yaw）→ 高通去DC → 阈值触发
// → 峰值捕获窗判向 → 方向校准映射 → 回摆抑制 + 锁定 → EVT
// 参数依据 docs/bbox_push_capture_2026-07-25 实测：
//   静置噪声底 p95≈0.09g，持稳噪声 p95≈0.22g，推动峰值 0.74~1.57g

static float pushAccMA[3] = {0,0,0};  // 加速度移动平均（用于高通）
static bool pushBaselineReady = false; // 基线是否已用真实读数初始化
static uint32_t pushDebounceUntil = 0; // 触发锁定截止时间
static uint32_t pushArmedAt = 0;       // 静默期截止：此前只更新基线不判定
static uint32_t pushCaptureUntil = 0;  // 峰值捕获窗截止（0=未在捕获）
static float pushPeakX = 0, pushPeakY = 0, pushPeakMag = 0; // 窗内峰值
static char pushLastDir = 0;           // 上次触发方向（'F'/'B'/'L'/'R'，回摆抑制用）
static uint32_t pushLastEmitMs = 0;    // 上次触发时刻
static bool pushNeedRearm = false;     // 触发后待复位：信号安静前不允许再触发
static uint32_t pushQuietSince = 0;    // 信号低于复位阈值的起始时刻（0=未安静）

// ---- 倾斜 / 抬举检测状态 ----
static char     tiltLastDir = 0;            // 上次触发方向（F/B/L/R）
static uint32_t tiltLastEmitMs = 0;         // 上次触发时刻
static uint32_t tiltDebounceUntil = 0;      // 触发锁定截止时间
static bool     tiltNeedRearm = false;      // 触发后待复位：姿态回正前不允许再触发
static uint32_t tiltQuietSince = 0;         // 低于复位阈值的起始时刻（0=未安静）

// ---- 向上抬举检测状态（独立于倾斜）----
static float    liftWzMA = 0.0f;             // 世界帧Z加速度移动平均
static bool     liftWzBaselineReady = false;  // Z轴基线是否已初始化
static uint32_t liftDebounceUntil = 0;        // 触发锁定截止时间
static uint32_t liftLastEmitMs = 0;           // 上次触发时刻

static const float PUSH_THRESH = 0.25f;          // 推动阈值 (g)，噪声底 3x 以上
static const uint32_t PUSH_PEAK_WIN_MS = 120;    // 触发后峰值捕获窗（取最强样本判向）
static const uint32_t PUSH_DEBOUNCE_MS = 600;    // 触发后锁定窗口
static const uint32_t PUSH_RETURN_SUPPRESS_MS = 900; // 回摆抑制：紧随的反向脉冲丢弃
static const uint32_t PUSH_SAME_DIR_SUPPRESS_MS = 100; // 同向抑制：100ms 内相同方向不重复上报
static const float PUSH_REARM_THRESH = 0.15f;    // 复位阈值：低于此值才算"安静"
static const uint32_t PUSH_REARM_QUIET_MS = 200; // 需持续安静时长，满足后才能再触发

// ---- 倾斜 / 抬举检测（基于四元数姿态） ----
static const float TILT_ANGLE_THRESH_DEG = 30.0f;      // 倾斜触发角度阈值（度）
static const float TILT_LIFT_THRESH_G = 0.35f;          // 向上抬举加速度阈值 (g)
static const uint32_t TILT_DEBOUNCE_MS = 800;           // 触发后锁定窗口
static const uint32_t TILT_SAME_DIR_SUPPRESS_MS = 3000; // 同向抑制 3s
static const float TILT_REARM_ANGLE_DEG = 20.0f;        // 回落到此角度以下才算安静
static const uint32_t TILT_REARM_QUIET_MS = 300;        // 需持续安静的时长
static const float TILT_LIFT_REARM_G = 0.15f;           // 抬举复位阈值 (g)

// 标定/重置后调用：基线作废、重新进入静默期
static void pushDetectReset() {
  pushBaselineReady = false;
  pushAccMA[0] = pushAccMA[1] = pushAccMA[2] = 0.0f;
  pushCaptureUntil = 0;
  pushLastDir = 0;
  pushNeedRearm = false;
  pushQuietSince = 0;
  pushArmedAt = millis() + PUSH_WARMUP_MS;
  // 同步重置倾斜检测
  tiltLastDir = 0;
  tiltDebounceUntil = 0;
  tiltNeedRearm = false;
  tiltQuietSince = 0;
  // 同步重置抬举检测
  liftWzBaselineReady = false;
  liftWzMA = 0.0f;
  liftDebounceUntil = 0;
}

// 入参为机体航向系水平加速度 bx/by（g）：世界系旋转后再旋回 -yaw，
// 前后左右锚定盒体自身轴（硬件标记方向），不受 yaw 漂移影响
static const char* pushDetect(float bx, float by) {
  // 首帧用真实读数初始化基线，避免从 0 起把重力残差当猛推
  if (!pushBaselineReady) {
    pushAccMA[0] = bx;
    pushAccMA[1] = by;
    pushBaselineReady = true;
    return nullptr;
  }

  // 高通滤波：移动平均估计 DC → 减去得动态分量
  const float alpha = 0.02f; // 高通截止 ~0.6Hz @200Hz
  pushAccMA[0] = pushAccMA[0] * (1-alpha) + bx * alpha;
  pushAccMA[1] = pushAccMA[1] * (1-alpha) + by * alpha;
  float dynX = bx - pushAccMA[0];
  float dynY = by - pushAccMA[1];

  // 静默期内只更新基线不判定（等姿态与高通基线收敛）
  uint32_t now = millis();
  if (now < pushArmedAt) return nullptr;

  float mag = sqrt(dynX*dynX + dynY*dynY);

  // ---- 峰值捕获窗：触发后 120ms 内持续追踪最强样本，窗口结束才判向 ----
  if (pushCaptureUntil != 0) {
    if (mag > pushPeakMag) { pushPeakMag = mag; pushPeakX = dynX; pushPeakY = dynY; }
    if (now < pushCaptureUntil) return nullptr;
    pushCaptureUntil = 0;
    // 一次手势只报一次：窗口收尾即进入待复位，安静前不再触发
    pushNeedRearm = true;
    pushQuietSince = 0;

    // 方向校准映射（ORIENT 命令配置，默认 = 原硬编码行为）
    float px = pushPeakX, py = pushPeakY;
    if (pushSwapXY) { float t = px; px = py; py = t; }
    px *= pushSignX;
    py *= pushSignY;

    char dir;
    if (fabs(px) > fabs(py)) dir = (px > 0) ? 'R' : 'L';
    else                     dir = (py > 0) ? 'F' : 'B';

    // 回摆抑制：推完回位的反向脉冲不重复触发
    bool opposite = (dir == 'F' && pushLastDir == 'B') || (dir == 'B' && pushLastDir == 'F') ||
                    (dir == 'L' && pushLastDir == 'R') || (dir == 'R' && pushLastDir == 'L');
    if (opposite && now - pushLastEmitMs < PUSH_RETURN_SUPPRESS_MS) {
      pushLastDir = 0; // 回摆消费掉，避免连锁抑制真实反向推动
      return nullptr;
    }

    // 同向抑制：距上次上报不足 1s 的相同方向丢弃（不刷新计时，1s 后自然放行）
    if (dir == pushLastDir && now - pushLastEmitMs < PUSH_SAME_DIR_SUPPRESS_MS) {
      return nullptr;
    }
    pushLastDir = dir;
    pushLastEmitMs = now;
    switch (dir) {
      case 'F': return "MOVE_FORWARD";
      case 'B': return "MOVE_BACKWARD";
      case 'L': return "MOVE_LEFT";
      default:  return "MOVE_RIGHT";
    }
  }

  // ---- 再武装迟滞：触发过后须回落安静一段时间才允许下一次触发 ----
  // 根治一次推动（推+回摆+余振可拖 1s+）产生多条 EVT 的问题
  if (pushNeedRearm) {
    if (mag < PUSH_REARM_THRESH) {
      if (pushQuietSince == 0) pushQuietSince = now;
      else if (now - pushQuietSince >= PUSH_REARM_QUIET_MS) {
        pushNeedRearm = false;
        pushQuietSince = 0;
      }
    } else {
      pushQuietSince = 0; // 仍在晃，安静计时清零
    }
    return nullptr;
  }

  // ---- 触发判定：锁定期外且过阈值 → 开峰值捕获窗 ----
  if (now < pushDebounceUntil) return nullptr;
  if (mag < PUSH_THRESH) return nullptr;
  pushCaptureUntil = now + PUSH_PEAK_WIN_MS;
  pushDebounceUntil = now + PUSH_DEBOUNCE_MS;
  pushPeakMag = mag; pushPeakX = dynX; pushPeakY = dynY;
  return nullptr;
}

// ---- 姿态倾斜检测（基于四元数 pitch/roll，不含抬举）----
// 多方向同时超阈值时只选比率最大的方向。
// 出参 outValue: 触发时的角度（度）
static const char* tiltDetect(float &outValue) {
  // ---- 从四元数提取欧拉角 ----
  float qw = mahonyQ[0], qx = mahonyQ[1], qy = mahonyQ[2], qz = mahonyQ[3];
  float roll  = atan2f(2.0f * (qw*qx + qy*qz), 1.0f - 2.0f * (qx*qx + qy*qy));
  float sinp  = 2.0f * (qw*qy - qz*qx);
  if (sinp >  1.0f) sinp =  1.0f;
  if (sinp < -1.0f) sinp = -1.0f;
  float pitch = asinf(sinp);
  float rollDeg  = roll  * 57.29578f;  // 180/PI
  float pitchDeg = pitch * 57.29578f;

  uint32_t now = millis();

  // 静默期内不判定（复用 pushArmedAt，等姿态收敛）
  if (now < pushArmedAt) return nullptr;

  // ---- 再武装迟滞：触发后须回落安静才允许下一次触发 ----
  if (tiltNeedRearm) {
    float maxAngle = (fabsf(rollDeg) > fabsf(pitchDeg)) ? fabsf(rollDeg) : fabsf(pitchDeg);
    if (maxAngle < TILT_REARM_ANGLE_DEG) {
      if (tiltQuietSince == 0) tiltQuietSince = now;
      else if (now - tiltQuietSince >= TILT_REARM_QUIET_MS) {
        tiltNeedRearm = false;
        tiltQuietSince = 0;
      }
    } else {
      tiltQuietSince = 0;
    }
    return nullptr;
  }

  // debounce 检查
  if (now < tiltDebounceUntil) return nullptr;

  // ---- 计算各方向比率，选最大者 ----
  float pitchRatio = fabsf(pitchDeg) / TILT_ANGLE_THRESH_DEG;
  float rollRatio  = fabsf(rollDeg)  / TILT_ANGLE_THRESH_DEG;

  char  bestDir   = 0;
  float bestRatio = 0.0f;
  float bestValue = 0.0f;

  // 左右倾斜（pitch，绕X轴 → 加速度X方向）
  if (pitchRatio >= 1.0f && pitchRatio > bestRatio) {
    bestRatio = pitchRatio;
    bestDir   = (pitchDeg > 0.0f) ? 'R' : 'L';
    bestValue = fabsf(pitchDeg);
  }
  // 前后倾斜（roll，绕Y轴 → 加速度Y方向，符号翻转以对齐加速度方向）
  if (rollRatio >= 1.0f && rollRatio > bestRatio) {
    bestRatio = rollRatio;
    bestDir   = (rollDeg > 0.0f) ? 'B' : 'F';
    bestValue = fabsf(rollDeg);
  }

  if (bestDir == 0) return nullptr;

  // ---- 同向抑制：相同方向 3s 内不重复上报 ----
  if (bestDir == tiltLastDir && now - tiltLastEmitMs < TILT_SAME_DIR_SUPPRESS_MS) {
    return nullptr;
  }

  // ---- 触发 ----
  tiltLastDir       = bestDir;
  tiltLastEmitMs    = now;
  tiltDebounceUntil = now + TILT_DEBOUNCE_MS;
  tiltNeedRearm     = true;
  tiltQuietSince    = 0;
  outValue          = bestValue;

  switch (bestDir) {
    case 'F': return "TILT_FORWARD";
    case 'B': return "TILT_BACKWARD";
    case 'L': return "TILT_LEFT";
    case 'R': return "TILT_RIGHT";
  }
  return nullptr;
}

// ---- 向上抬举检测（世界帧 Z 轴加速度脉冲，优先级最低）----
static const char* liftDetect(float wzWorld) {
  uint32_t now = millis();

  if (!liftWzBaselineReady) {
    liftWzMA = wzWorld;
    liftWzBaselineReady = true;
    return nullptr;
  }

  const float alpha = 0.02f;
  liftWzMA = liftWzMA * (1.0f - alpha) + wzWorld * alpha;
  float wzDyn = wzWorld - liftWzMA;  // 正值 = 向上

  if (now < pushArmedAt) return nullptr;
  if (now < liftDebounceUntil) return nullptr;
  if (wzDyn < TILT_LIFT_THRESH_G) return nullptr;
  if (now - liftLastEmitMs < TILT_SAME_DIR_SUPPRESS_MS) return nullptr;

  liftDebounceUntil = now + TILT_DEBOUNCE_MS;
  liftLastEmitMs    = now;
  return "LIFT_UP";
}


// ---------------- 游戏状态可视化（下行/发送方向） ----------------
// 板子没有屏幕/马达时，用板载 WS2812 全彩灯作为“游戏状态”的可视出口。
static uint8_t LED_BRIGHTNESS = 40;  // 限亮度，避免刺眼

static void setLed(uint8_t r, uint8_t g, uint8_t b) {
  neopixelWrite(RGB_LED_PIN, r, g, b);
}

// 把 Unity 发来的游戏状态映射为一种颜色，并回一条 LOG 便于终端验收
static void applyGameState(const String &rawState) {
  String s = rawState;
  s.trim();
  s.toUpperCase();
  uint8_t r = 0, g = 0, b = 0;
  if (s == "IDLE" || s == "READY") { r = 0;  g = LED_BRIGHTNESS; b = 0; }          // 绿：待命
  else if (s == "MOVE" || s == "STEP") { r = 0; g = 0; b = LED_BRIGHTNESS; }        // 蓝：移动一格
  else if (s == "WALL" || s == "BLOCK") { r = LED_BRIGHTNESS; g = LED_BRIGHTNESS/3; b = 0; } // 橙：撞墙
  else if (s == "WIN" || s == "SUCCESS") { r = LED_BRIGHTNESS; g = LED_BRIGHTNESS; b = 0; }   // 黄：通关
  else if (s == "LOSE" || s == "FAIL") { r = LED_BRIGHTNESS; g = 0; b = 0; }        // 红：失败
  else if (s == "OFF") { r = 0; g = 0; b = 0; }                                     // 熄灭
  else {
    sendNak("STATE", "unknown");
    return;
  }
  setLed(r, g, b);
  char buf[48];
  snprintf(buf, sizeof(buf), "%s,%u,%u,%u", s.c_str(), r, g, b);
  sendOk("STATE", buf);
}

// 触发一次高层事件，进入待 ACK 状态
// fromImu=true 时跳过触觉屏蔽窗口（推动事件不被 HAPTIC 反馈吞掉）
static void emitEvent(const String &action, float value, bool fromImu = false) {
  if (!fromImu && millis() < hapticGuardUntil) {
    if (debugMode) sendLog("haptic guard active, event ignored");
    return;
  }
  if (pending.active) {
    // 上一条尚未确认，阶段 2 简单丢弃（正式固件会排队）
    if (debugMode) sendLog("previous event pending, new trigger ignored");
    return;
  }
  sequenceCounter++;
  pending.active = true;
  pending.sequence = sequenceCounter;
  pending.action = action;
  pending.value = value;
  pending.timestamp_ms = millis();
  pending.retry = 0;
  pending.last_send_ms = millis();
  sendEventLine();
}

static void handleAck(uint32_t seq) {
  if (pending.active && pending.sequence == seq) {
    pending.active = false;
    if (debugMode) {
      char buf[32];
      snprintf(buf, sizeof(buf), "ack ok seq=%lu", (unsigned long)seq);
      sendLog(buf);
    }
  }
}

// ---------------- 命令解析（来自 PC / Bridge） ----------------
static void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line.startsWith("ACK,")) {
    handleAck((uint32_t)line.substring(4).toInt());
    return;
  }
  if (line.startsWith("STATE,")) {
    // 下行/发送方向：Unity 把游戏状态发给板子，用板载灯可视化
    applyGameState(line.substring(6));
    return;
  }
  if (line.startsWith("LED,")) {
    // 直接设色：LED,<r>,<g>,<b>（0-255），调试/覆盖用
    int c1 = line.indexOf(',');
    int c2 = line.indexOf(',', c1 + 1);
    int c3 = line.indexOf(',', c2 + 1);
    if (c2 < 0 || c3 < 0) { sendNak("LED", "badargs"); return; }
    int r = line.substring(c1 + 1, c2).toInt();
    int g = line.substring(c2 + 1, c3).toInt();
    int b = line.substring(c3 + 1).toInt();
    r = constrain(r, 0, 255); g = constrain(g, 0, 255); b = constrain(b, 0, 255);
    setLed((uint8_t)r, (uint8_t)g, (uint8_t)b);
    char buf[24];
    snprintf(buf, sizeof(buf), "%d,%d,%d", r, g, b);
    sendOk("LED", buf);
    return;
  }
  if (line.startsWith("PING")) {
    // 存活探测：PING 或 PING,<token> -> 回 PONG[,<token>]
    int c1 = line.indexOf(',');
    if (c1 >= 0) {
      Serial.print("PONG,");
      Serial.println(line.substring(c1 + 1));
    } else {
      Serial.println("PONG");
    }
    return;
  }
  if (line.startsWith("HAPTIC,")) {
    // 马达未到货：记录 + 模拟屏蔽窗口，并用灯闪一下表示收到反馈
    hapticGuardUntil = millis() + HAPTIC_GUARD_MS;
    setLed(LED_BRIGHTNESS, 0, LED_BRIGHTNESS);  // 紫：收到一次触觉反馈
    sendOk("HAPTIC", line.substring(7).c_str());
    return;
  }
  if (line.startsWith("CALIBRATE")) {
    if (!imuPresent) {
      sendOk("CALIBRATE", "no-sensor");
      return;
    }
    if (imuCalibrateGyro(200)) {
      mahonyReset(); // 标定后重置姿态
      pushDetectReset(); // 基线作废，重新进入静默期
      char buf[64];
      snprintf(buf, sizeof(buf), "gx=%.2f,gy=%.2f,gz=%.2f",
               gyroBias[0], gyroBias[1], gyroBias[2]);
      sendOk("CALIBRATE", buf);
    } else {
      sendNak("CALIBRATE", "read-fail");
    }
    return;
  }
  if (line.startsWith("ORIENT,")) {
    // 推动方向校准：ORIENT,<swapXY>,<signX>,<signY>（signX/Y 取 1 或 -1）
    int c1 = line.indexOf(',');
    int c2 = line.indexOf(',', c1 + 1);
    int c3 = line.indexOf(',', c2 + 1);
    if (c2 < 0 || c3 < 0) { sendNak("ORIENT", "badargs"); return; }
    pushSwapXY = line.substring(c1 + 1, c2).toInt() != 0;
    pushSignX = (int8_t)(line.substring(c2 + 1, c3).toInt() < 0 ? -1 : 1);
    pushSignY = (int8_t)(line.substring(c3 + 1).toInt() < 0 ? -1 : 1);
    char buf[24];
    snprintf(buf, sizeof(buf), "%d,%d,%d", pushSwapXY ? 1 : 0, pushSignX, pushSignY);
    sendOk("ORIENT", buf);
    return;
  }
  if (line.startsWith("DEBUG,")) {
    debugMode = line.substring(6).toInt() != 0;
    sendOk("DEBUG", debugMode ? "1" : "0");
    return;
  }
  if (line.startsWith("SIM,")) {
    emitEvent(line.substring(4), 200.0f);
    return;
  }
  if (line.length() == 1) {
    switch (line[0]) {
      case 'w': emitEvent("MOVE_FORWARD", 200.0f); return;
      case 's': emitEvent("MOVE_BACKWARD", 200.0f); return;
      case 'a': emitEvent("MOVE_LEFT", 200.0f); return;
      case 'd': emitEvent("MOVE_RIGHT", 200.0f); return;
      case 'q': emitEvent("ROTATE_CCW", 45.0f); return;
      case 'e': emitEvent("ROTATE_CW", 45.0f); return;
    }
  }
  Serial.print("NAK,");
  int c = line.indexOf(',');
  Serial.print(c > 0 ? line.substring(0, c) : line);
  Serial.println(",unknown");
}

// ---------------- 各种轮询 ----------------
static void pollSerial() {
  static String buf;
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (buf.length() > 0) {
        handleCommand(buf);
        buf = "";
      }
    } else {
      buf += c;
      if (buf.length() > 128) buf = "";  // 防溢出
    }
  }
}

static void pollButton() {
  static int lastReading = HIGH;
  static int stableState = HIGH;
  static uint32_t lastChange = 0;

  int reading = digitalRead(BUTTON_PIN);
  uint32_t now = millis();
  if (reading != lastReading) {
    lastReading = reading;
    lastChange = now;
  }
  if ((now - lastChange) > BUTTON_DEBOUNCE_MS && stableState != reading) {
    stableState = reading;
    if (stableState == LOW) {          // 按下（BOOT 按钮按下为低）
      emitEvent("MOVE_RIGHT", 200.0f);  // 按一下 = 向右移动一格
    }
  }
}

static void pollRetransmit() {
  if (!pending.active) return;
  uint32_t now = millis();
  if (now - pending.last_send_ms < ACK_TIMEOUT_MS) return;

  if (pending.retry >= MAX_RETRY) {
    char buf[48];
    snprintf(buf, sizeof(buf), "event seq=%lu gave up after retries",
             (unsigned long)pending.sequence);
    sendLog(buf);
    pending.active = false;
    return;
  }
  pending.retry++;
  pending.last_send_ms = now;
  sendEventLine();  // 相同 sequence 重传，Bridge 端负责去重
}

// 采样/融合/检测 与 上报 解耦：
//   采样段固定 200Hz（micros 定时）：imuRead + Mahony 融合 + 推动检测
//   上报段独立限流（millis）：常态 20Hz / DEBUG 50Hz
static void pollImu() {
  if (!imuPresent) return;

  // ---- 采样段：200Hz ----
  uint32_t nowUs = micros();
  if (nowUs - lastImuSampleUs < IMU_SAMPLE_US) return;

  // 真实 dt（秒），钳制 [0.001, 0.05] 抵御调度抖动/首帧
  float dt = (lastImuSampleUs > 0) ? (nowUs - lastImuSampleUs) * 1e-6f : 0.005f;
  if (dt < 0.001f) dt = 0.001f;
  if (dt > 0.05f) dt = 0.05f;
  lastImuSampleUs = nowUs;

  if (!imuRead()) return;

  // Mahony 互补滤波（真实 dt）
  mahonyUpdate(imuGyro[0], imuGyro[1], imuGyro[2],
               imuAccel[0], imuAccel[1], imuAccel[2], dt);

  // 推动脉冲检测：旋到世界帧（倾斜补偿）后再旋回 -yaw，
  // 得到机体航向系水平分量——方向锚定盒体自身轴，yaw 漂移不影响判向
  float wx, wy, wz;
  quatRotateWorld(mahonyQ, imuAccel[0], imuAccel[1], imuAccel[2], wx, wy, wz);
  float yawSin = 2.0f * (mahonyQ[0]*mahonyQ[3] + mahonyQ[1]*mahonyQ[2]);
  float yawCos = 1.0f - 2.0f * (mahonyQ[2]*mahonyQ[2] + mahonyQ[3]*mahonyQ[3]);
  float yaw = atan2f(yawSin, yawCos);
  float cy = cosf(yaw), sy = sinf(yaw);
  float bx =  cy * wx + sy * wy;   // Rz(-yaw)·[wx, wy]
  float by = -sy * wx + cy * wy;
  uint32_t nowMs = millis();
  // 姿态倾斜检测（优先级：TILT > MOVE > LIFT_UP）
  float tiltValue = 0;
  const char* tiltDir = tiltDetect(tiltValue);
  if (tiltDir) emitEvent(tiltDir, tiltValue, true);

  // 推动脉冲检测（tiltDetect 触发后 pending.active 会自然拦截）
  const char* dir = pushDetect(bx, by);
  if (dir) emitEvent(dir, 200.0f, true);

  // 向上抬举检测（优先级最低）
  const char* liftDir = liftDetect(wz);
  if (liftDir) emitEvent(liftDir, 200.0f, true);

  // ---- 上报段：独立限流 ----
  uint32_t interval = debugMode ? IMU_STREAM_DEBUG_MS : IMU_STREAM_NORMAL_MS;
  if (nowMs - lastImuStream < interval) return;
  lastImuStream = nowMs;

  // 输出四元数（常态和 DEBUG 都输出 IMUQ）
  sendImuQuat();

  // DEBUG 模式下额外输出原始 IMU 数据用于前端调试
  if (debugMode) sendImuTelemetry();
}

// ---------------- 生命周期 ----------------
void setup() {
  Serial.begin(BAUD);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(MPU_INT_PIN, INPUT);
  delay(200);

  // 初始化 I2C 并检测 MPU6050
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);
  imuPresent = imuInit();

  // 上电自动零偏校准：先等板子静止再采零偏，避免拿在手里/晃动时标出坏零偏
  if (imuPresent) {
    setLed(LED_BRIGHTNESS, LED_BRIGHTNESS, 0); // 黄灯 = 校准中
    sendLog("waiting for still, then auto-calibrating...");
    int stillFrames = 0;
    uint32_t waitStart = millis();
    while (stillFrames < 30 && millis() - waitStart < 3000) {
      if (imuRead() && mahonyDetectStatic(imuGyro[0], imuGyro[1], imuGyro[2],
                                          imuAccel[0], imuAccel[1], imuAccel[2])) {
        stillFrames++;
      } else {
        stillFrames = 0;  // 一旦动了就重新计数
      }
      delay(5);
    }
    if (stillFrames < 30) sendLog("still-wait timeout, calibrating anyway");
    if (imuCalibrateGyro(200)) {
      char buf[64];
      snprintf(buf, sizeof(buf), "auto-calib ok gx=%.2f,gy=%.2f,gz=%.2f",
               gyroBias[0], gyroBias[1], gyroBias[2]);
      sendLog(buf);
    } else {
      sendLog("auto-calib failed, using zero bias");
    }
    mahonyReset();
    pushDetectReset(); // 静默期：等高通基线与姿态收敛后才开始判定推动
    setLed(0, LED_BRIGHTNESS, 0); // 绿灯 = 就绪
  }

  char boot[48];
  snprintf(boot, sizeof(boot), "BOOT,%s,proto=%d", FIRMWARE_VERSION, PROTO_VERSION);
  Serial.println(boot);
  lastHeartbeat = millis();
}

void loop() {
  pollSerial();
  pollButton();
  pollRetransmit();
  pollImu();

  uint32_t now = millis();
  if (now - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeat = now;
    char buf[48];
    snprintf(buf, sizeof(buf), "HEARTBEAT,%lu,STABLE", (unsigned long)now);
    Serial.println(buf);
  }
}
