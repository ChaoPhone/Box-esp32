#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// =============================================================
//  B 板 (main_B_esp) — USB 连 PC 的桥接板
// -------------------------------------------------------------
//  职责：ESP-NOW 接收 A 板（箱子端）的协议行 → 过滤 → 转发 USB 串口
//        PC 下行命令 → ESP-NOW 转发给 A 板
//        HAPTIC 命令 → 本地电机 GPIO1
//
//  A 板 MAC: E8:3D:C1:F2:C7:B8    B 板 MAC: E8:3D:C1:FA:7A:0C
//
//  构建：pio run -e a_win -t upload
// =============================================================

static const uint32_t BAUD = 115200;
static const uint8_t WIFI_CHANNEL = 1;

// 两板固定 MAC
static const uint8_t MAC_A[6] = {0xE8, 0x3D, 0xC1, 0xF2, 0xC7, 0xB8};
static const uint8_t MAC_B[6] = {0xE8, 0x3D, 0xC1, 0xFA, 0x7A, 0x0C};
static uint8_t PEER_ADDR[6] = {0};
static bool gPeerKnown = false;
static volatile uint32_t foreignFrames = 0;

// 震动电机（GPIO1，LEDC PWM）
static const int MOTOR_PIN = 1;
static const int MOTOR_PWM_CH = 0;
static const int MOTOR_PWM_FREQ = 1000;
static const int MOTOR_PWM_RES = 8;
static uint32_t motorEndMs = 0;

// 板载 LED (GPIO48 WS2812)
static const int RGB_LED_PIN = 48;
static uint8_t LED_BRIGHTNESS = 40;

// ESP-NOW 接收队列
struct RxMsg {
  uint8_t len;
  uint8_t mac[6];
  char data[240];
};
static QueueHandle_t rxQueue;
static volatile uint32_t rxFrameCount = 0;
static volatile uint32_t droppedFrames = 0;
static uint32_t lastLinkLog = 0;
static bool loggedAlive = false;
static String lastState;

// 本板身份
static char gBoard = '?';

static char boardLetter() {
  String mac = WiFi.macAddress();
  mac.toUpperCase();
  if (mac == "E8:3D:C1:F2:C7:B8") return 'A';
  if (mac == "E8:3D:C1:FA:7A:0C") return 'B';
  return '?';
}

// ---- LED / 电机 ----
static void setLed(uint8_t r, uint8_t g, uint8_t b) {
  neopixelWrite(RGB_LED_PIN, r, g, b);
}

static void motorOn(uint8_t strength, uint32_t ms) {
  ledcWrite(MOTOR_PWM_CH, strength);
  motorEndMs = millis() + ms;
}

// ---- ESP-NOW ----
static void espnowSend(const char *line) {
  if (!gPeerKnown) return;
  esp_now_send(PEER_ADDR, (const uint8_t *)line, strlen(line));
}

static void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (gPeerKnown && memcmp(mac, PEER_ADDR, 6) != 0) {
    foreignFrames++;
    return;
  }
  RxMsg m;
  m.len = (len > 239) ? 239 : (uint8_t)len;
  memcpy(m.data, data, m.len);
  m.data[m.len] = '\0';
  memcpy(m.mac, mac, 6);
  rxFrameCount++;
  xQueueSend(rxQueue, &m, 0);
}

static void espnowSetup() {
  rxQueue = xQueueCreate(16, sizeof(RxMsg));
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  // 自己的对端就是 A 板
  memcpy(PEER_ADDR, MAC_A, 6);
  gPeerKnown = true;

  if (esp_now_init() != ESP_OK) {
    Serial.println("LOG,esp_now_init failed");
    return;
  }
  esp_now_register_recv_cb(onEspNowRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, PEER_ADDR, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
}

// ---- 协议行过滤 ----
static bool isProtocolLine(const char *s) {
  static const char *PREFIX[] = {
    "EVT,", "ACK,", "HEARTBEAT,", "HAPTIC,", "STATE,",
    "BOOT,", "LOG,", "IMUQ,", "IMU,", "PONG,", "OK,", "NAK,",
    "CALIBRATE", "DEBUG,"
  };
  for (auto p : PREFIX) {
    if (strncmp(s, p, strlen(p)) == 0) return true;
  }
  return false;
}

// ---- 串口读行 ----
static bool readSerialLine(String &out) {
  static String buf;
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (buf.length() > 0) {
        out = buf;
        buf = "";
        return true;
      }
    } else {
      buf += c;
      if (buf.length() > 200) buf = "";
    }
  }
  return false;
}

// ---- 下行命令处理 ----
static void handlePcCommand(const String &line) {
  if (line.length() == 0) return;

  // HAPTIC → 本地电机 + 转发 B 板
  if (line.startsWith("HAPTIC,")) {
    String args = line.substring(7);
    int c = args.indexOf(',');
    String pattern = (c >= 0) ? args.substring(0, c) : args;
    pattern.trim();
    uint32_t duration = 150;
    if (c >= 0) {
      String msStr = args.substring(c + 1);
      msStr.trim();
      if (msStr.length() > 0) duration = msStr.toInt();
    }
    uint8_t strength = 200;
    if (pattern == "short")        duration = 50;
    else if (pattern == "long")    duration = 300;
    else if (pattern == "double")  duration = 50;
    else { int s = pattern.toInt(); if (s > 0 && s <= 255) strength = (uint8_t)s; }
    motorOn(strength, duration);
    setLed(LED_BRIGHTNESS, 0, LED_BRIGHTNESS); // 紫灯确认
    // 也转发给 A 板
    espnowSend(line.c_str());
    return;
  }

  // STATE → 记录 + 转发 A 板
  if (line.startsWith("STATE,")) {
    lastState = line;
    espnowSend(line.c_str());
    return;
  }

  // 其他命令直接转发
  espnowSend(line.c_str());
}

// ---- 上行转发 ----
static void forwardToPc(const char *data) {
  if (isProtocolLine(data)) {
    Serial.println(data);
  } else {
    droppedFrames++;
  }
}

// ---- 生命周期 ----
void setup() {
  Serial.begin(BAUD);
  delay(200);

  // 电机 PWM
  ledcSetup(MOTOR_PWM_CH, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcAttachPin(MOTOR_PIN, MOTOR_PWM_CH);
  ledcWrite(MOTOR_PWM_CH, 0);

  espnowSetup();
  gBoard = boardLetter();
  Serial.printf("LOG,[%c] B-board ready, MAC %s\n", gBoard, WiFi.macAddress().c_str());

  setLed(0, LED_BRIGHTNESS, 0); // 绿灯就绪
}

void loop() {
  // 上行：无线帧 → 转发 PC
  RxMsg m;
  while (xQueueReceive(rxQueue, &m, 0)) {
    if (!loggedAlive) {
      loggedAlive = true;
      Serial.printf("LOG,[%c] link alive, first frame from A\n", gBoard);
    }
    forwardToPc(m.data);
  }

  // 下行：PC 命令 → 转发 B 板 / 本地执行
  String line;
  if (readSerialLine(line)) handlePcCommand(line);

  // 电机超时停止
  uint32_t now = millis();
  if (motorEndMs && now >= motorEndMs) {
    ledcWrite(MOTOR_PWM_CH, 0);
    motorEndMs = 0;
  }

  // 每 5 秒链路统计
  if (now - lastLinkLog >= 5000) {
    lastLinkLog = now;
    Serial.printf("LOG,[%c] link frames=%lu dropped=%lu foreign=%lu\n",
                  gBoard, (unsigned long)rxFrameCount,
                  (unsigned long)droppedFrames, (unsigned long)foreignFrames);
  }
}
