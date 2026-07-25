#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// =============================================================
//  B-Box ESP32-S3 固件 —— 阶段 10 预研：ESP-NOW 无线链路
// -------------------------------------------------------------
//  用两块 ESP32-S3 验证无线传输，并直接作为最终无线桥接的传输层：
//
//    箱子端(ROLE_SENDER) --ESP-NOW--> 接收端(ROLE_RECEIVER) --USB串口--> PC(Bridge)
//
//  设计要点：ESP-NOW 只做“透明搬运 ASCII 协议行”，上层串口协议
//  (EVT/ACK/HEARTBEAT/HAPTIC) 与有线版本完全一致，因此 Python
//  Bridge / Unity 无需任何改动。
//
//  采用广播地址(FF:FF:FF:FF:FF:FF)，两块板子无需预先烧录对方 MAC。
//  角色由 platformio.ini 的 -D ROLE_SENDER / -D ROLE_RECEIVER 决定：
//    pio run -e espnow_sender   -t upload   # 烧到“箱子端”那块
//    pio run -e espnow_receiver -t upload   # 烧到插 PC 的那块
// =============================================================

#if !defined(ROLE_SENDER) && !defined(ROLE_RECEIVER)
#error "请通过 -D ROLE_SENDER 或 -D ROLE_RECEIVER 指定角色（见 platformio.ini）"
#endif

static const uint32_t BAUD = 115200;
static const uint8_t WIFI_CHANNEL = 1;   // 两块板锁定同一信道，避免漂移

// 两块板的固定 MAC（用于点对点单播 + 源 MAC 过滤）
static const uint8_t MAC_A[6] = {0xE8, 0x3D, 0xC1, 0xF2, 0xC7, 0xB8};
static const uint8_t MAC_B[6] = {0xE8, 0x3D, 0xC1, 0xFA, 0x7A, 0x0C};
static uint8_t PEER_ADDR[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // 运行时按自身身份填对方 MAC
static bool gPeerKnown = false;               // 是否已识别出合法对方
static volatile uint32_t foreignFrames = 0;   // 被源 MAC 过滤挡掉的外来帧计数

// ESP-NOW 接收回调运行在 WiFi 任务中，用 FreeRTOS 队列交给 loop() 处理
struct RxMsg {
  uint8_t len;
  uint8_t mac[6];
  char data[240];
};
static QueueHandle_t rxQueue;
static volatile uint32_t rxFrameCount = 0;

// 本板身份（按固定硬件特征 MAC 判定）：'A' / 'B' / '?'
static char gBoard = '?';

static char boardLetter() {
  String mac = WiFi.macAddress();
  mac.toUpperCase();
  if (mac == "E8:3D:C1:F2:C7:B8") return 'A';
  if (mac == "E8:3D:C1:FA:7A:0C") return 'B';
  return '?';
}

static void espnowSend(const char *line) {
  esp_now_send(PEER_ADDR, (const uint8_t *)line, strlen(line));
}

// core 2.0.x 的接收回调签名：(mac, data, len)
static void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (gPeerKnown && memcmp(mac, PEER_ADDR, 6) != 0) {
    foreignFrames++;   // 非对方发来的帧（外来广播/干扰）：直接拒收
    return;
  }
  RxMsg m;
  m.len = (len > 239) ? 239 : (uint8_t)len;
  memcpy(m.data, data, m.len);
  m.data[m.len] = '\0';
  memcpy(m.mac, mac, 6);
  rxFrameCount++;
  xQueueSend(rxQueue, &m, 0);  // 回调在任务上下文，非严格 ISR
}

static void espnowSetup() {
  rxQueue = xQueueCreate(16, sizeof(RxMsg));
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);  // 锁定信道

  // 预烧录对方 MAC：全程点对点单播，不使用广播
  //   A 的对端是 B，B 的对端是 A（两块板 MAC 固定，编译期已知）
  if (boardLetter() == 'B') {
    memcpy(PEER_ADDR, MAC_A, 6);
  } else {
    memcpy(PEER_ADDR, MAC_B, 6);
  }
  gPeerKnown = true;   // 源 MAC 过滤常开：只认对端，外来帧一律拒收

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

// 从本地 USB 串口读取一整行（供两种角色复用）
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

// =============================================================
#ifdef ROLE_SENDER
// ------------------- 箱子端（无线发送） -------------------
static const int BUTTON_PIN = 0;
static const uint32_t HEARTBEAT_INTERVAL_MS = 1000;
static const uint32_t ACK_TIMEOUT_MS = 300;
static const uint32_t HAPTIC_GUARD_MS = 150;
static const uint32_t BUTTON_DEBOUNCE_MS = 40;
static const int MAX_RETRY = 5;
static const char *FIRMWARE_VERSION = "0.1.0-espnow";

static uint32_t sequenceCounter = 0;
static bool debugMode = false;
static uint32_t lastHeartbeat = 0;
static uint32_t hapticGuardUntil = 0;

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

// ---- 工况压测（丢包率）：按频率连续发带序号的 TEST 帧 ----
static uint32_t loadHz = 0;
static uint32_t loadTotal = 0;
static uint32_t loadSeq = 0;
static bool loadActive = false;
static uint32_t lastLoadSend = 0;
static uint8_t doneBurst = 0;

static void localLog(const char *text) {
  Serial.print("LOG,");
  Serial.println(text);
}

static void sendEventFrame() {
  char buf[96];
  snprintf(buf, sizeof(buf), "EVT,%lu,%s,%.1f,%lu",
           (unsigned long)pending.sequence, pending.action.c_str(),
           pending.value, (unsigned long)pending.timestamp_ms);
  espnowSend(buf);
  Serial.printf("[%c->air] %s\n", gBoard, buf);  // 本地回显发送的事件帧
  if (debugMode) {
    Serial.print("LOG,tx ");
    Serial.println(buf);
  }
}

static void emitEvent(const String &action, float value) {
  if (millis() < hapticGuardUntil) {
    if (debugMode) localLog("haptic guard active, event ignored");
    return;
  }
  if (pending.active) {
    if (debugMode) localLog("previous event pending, new trigger ignored");
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
  sendEventFrame();
}

// 处理无线收到的命令（来自接收端转发的 PC 命令）
static void handleWireless(const char *raw) {
  String line(raw);
  line.trim();
  if (line.startsWith("ACK,")) {
    uint32_t seq = (uint32_t)line.substring(4).toInt();
    if (pending.active && pending.sequence == seq) {
      pending.active = false;
      if (debugMode) localLog("ack ok");
    }
    return;
  }
  if (line.startsWith("HAPTIC,")) {
    hapticGuardUntil = millis() + HAPTIC_GUARD_MS;  // 马达未到货：仅模拟屏蔽
    Serial.print("LOG,haptic recv: ");
    Serial.println(line);
    return;
  }
  if (line.startsWith("CALIBRATE")) {
    localLog("calibrate requested (no sensor yet)");
    return;
  }
  if (line.startsWith("DEBUG,")) {
    debugMode = line.substring(6).toInt() != 0;
    localLog(debugMode ? "debug=1" : "debug=0");
    return;
  }
}

static void pollLocalSerial() {
  String line;
  if (!readSerialLine(line)) return;
  line.trim();
  if (line.startsWith("LOAD,")) {
    // LOAD,<hz>,<count>  开始工况压测；LOAD,0 停止
    int c1 = line.indexOf(',');
    int c2 = line.indexOf(',', c1 + 1);
    uint32_t hz = (c2 > 0) ? (uint32_t)line.substring(c1 + 1, c2).toInt()
                           : (uint32_t)line.substring(c1 + 1).toInt();
    uint32_t cnt = (c2 > 0) ? (uint32_t)line.substring(c2 + 1).toInt() : 0;
    if (hz == 0 || cnt == 0) {
      loadActive = false;
      localLog("load test stopped");
    } else {
      loadHz = hz; loadTotal = cnt; loadSeq = 0; doneBurst = 0;
      lastLoadSend = 0; loadActive = true;
      char b[56];
      snprintf(b, sizeof(b), "load test start hz=%lu count=%lu",
               (unsigned long)hz, (unsigned long)cnt);
      localLog(b);
    }
    return;
  }
  if (line.startsWith("SIM,")) {
    emitEvent(line.substring(4), 200.0f);
    return;
  }
  if (line.length() == 1) {
    switch (line[0]) {
      case 'w': emitEvent("MOVE_UP", 200.0f); return;
      case 's': emitEvent("MOVE_DOWN", 200.0f); return;
      case 'a': emitEvent("MOVE_LEFT", 200.0f); return;
      case 'd': emitEvent("MOVE_RIGHT", 200.0f); return;
      case 'q': emitEvent("ROTATE_CCW_45", 45.0f); return;
      case 'e': emitEvent("ROTATE_CW_45", 45.0f); return;
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
    if (stableState == LOW) emitEvent("MOVE_RIGHT", 200.0f);
  }
}

static void pollRetransmit() {
  if (!pending.active) return;
  uint32_t now = millis();
  if (now - pending.last_send_ms < ACK_TIMEOUT_MS) return;
  if (pending.retry >= MAX_RETRY) {
    localLog("event gave up after retries");
    pending.active = false;
    return;
  }
  pending.retry++;
  pending.last_send_ms = now;
  sendEventFrame();
}

void setup() {
  Serial.begin(BAUD);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  delay(200);
  espnowSetup();
  gBoard = boardLetter();
  Serial.printf("LOG,[%c] sender ready, MAC %s\n", gBoard, WiFi.macAddress().c_str());
  espnowSend((String("BOOT,") + FIRMWARE_VERSION).c_str());
  lastHeartbeat = millis();
}

void loop() {
  RxMsg m;
  while (xQueueReceive(rxQueue, &m, 0)) handleWireless(m.data);
  pollLocalSerial();
  pollButton();
  pollRetransmit();

  uint32_t now = millis();
  if (now - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeat = now;
    char buf[48];
    snprintf(buf, sizeof(buf), "HEARTBEAT,%lu,STABLE", (unsigned long)now);
    espnowSend(buf);
    Serial.printf("[%c->air] %s\n", gBoard, buf);  // 本地回显：证明本板正在发
  }

  // 工况压测：按频率连续发带序号的 TEST 帧，发完补发若干次 TESTDONE
  if (loadActive) {
    uint32_t interval = 1000 / loadHz;
    if (interval == 0) interval = 1;
    uint32_t nowL = millis();
    if (nowL - lastLoadSend >= interval) {
      lastLoadSend = nowL;
      char b[48];
      if (loadSeq < loadTotal) {
        snprintf(b, sizeof(b), "TEST,%lu,%lu", (unsigned long)loadSeq, (unsigned long)nowL);
        espnowSend(b);
        loadSeq++;
      } else {
        snprintf(b, sizeof(b), "TESTDONE,%lu", (unsigned long)loadTotal);
        espnowSend(b);
        doneBurst++;
        if (doneBurst >= 5) {
          loadActive = false;
          localLog("load test tx done");
        }
      }
    }
  }
}
#endif  // ROLE_SENDER

// =============================================================
#ifdef ROLE_RECEIVER
// ------------------- 接收端（插 PC，透明桥接） -------------------
static uint32_t lastLinkLog = 0;
static uint32_t droppedFrames = 0;

// 记录最近一个被丢弃的杂帧，供每 5 秒采样打印一次（用于溯源）
static uint8_t lastJunkMac[6] = {0};
static uint8_t lastJunkLen = 0;
static char lastJunkText[24] = {0};
static bool haveJunk = false;

// ---- 工况压测统计：按 TEST 帧序号算丢包率 ----
static bool testRunning = false;
static bool testResultDone = false;
static long testLastSeq = -1;
static uint32_t testReceived = 0;
static uint32_t testGapLost = 0;
static uint32_t lastTestStat = 0;

// 只认这些前缀的合法协议行，其余（噪声/损坏帧）一律丢弃
static bool isProtocolLine(const char *s) {
  static const char *PREFIX[] = {"EVT,", "ACK,", "HEARTBEAT,", "HAPTIC,",
                                 "BOOT,", "LOG,", "CALIBRATE", "DEBUG,"};
  for (auto p : PREFIX) {
    if (strncmp(s, p, strlen(p)) == 0) return true;
  }
  return false;
}

void setup() {
  Serial.begin(BAUD);
  delay(200);
  espnowSetup();
  gBoard = boardLetter();
  Serial.printf("LOG,[%c] receiver ready, MAC %s\n", gBoard, WiFi.macAddress().c_str());
}

void loop() {
  // 无线收到的协议行 -> 原样转发到 PC 串口（Bridge 看到的与有线一致）
  RxMsg m;
  while (xQueueReceive(rxQueue, &m, 0)) {
    if (strncmp(m.data, "TESTDONE,", 9) == 0) {
      // 压测结束标记：按发送总数精确算丢包率（含尾部丢失）
      if (testRunning && !testResultDone) {
        uint32_t total = (uint32_t)atol(m.data + 9);
        uint32_t lost = (total > testReceived) ? (total - testReceived) : 0;
        double loss = total ? (100.0 * (double)lost / (double)total) : 0.0;
        char rb[112];
        snprintf(rb, sizeof(rb),
                 "LOG,[%c] TEST RESULT sent=%lu rx=%lu lost=%lu loss=%.2f%%",
                 gBoard, (unsigned long)total, (unsigned long)testReceived,
                 (unsigned long)lost, loss);
        Serial.println(rb);
        testResultDone = true;
        testRunning = false;
      }
      continue;
    }
    if (strncmp(m.data, "TEST,", 5) == 0) {
      long seq = atol(m.data + 5);
      if (!testRunning || seq == 0) {   // 新一轮压测开始
        testRunning = true; testResultDone = false;
        testReceived = 0; testGapLost = 0; testLastSeq = -1;
      }
      if (testLastSeq >= 0 && seq > testLastSeq + 1) {
        testGapLost += (uint32_t)(seq - testLastSeq - 1);
      }
      testReceived++;
      testLastSeq = seq;
      continue;   // 压测帧只统计，不转发
    }
    if (isProtocolLine(m.data)) {
      Serial.println(m.data);   // 合法协议行 -> 原样转发（Bridge 看到的与有线一致）
    } else {
      droppedFrames++;          // 噪声/损坏帧 -> 丢弃并计数
      memcpy(lastJunkMac, m.mac, 6);
      lastJunkLen = m.len;
      uint8_t n = m.len < 16 ? m.len : 16;
      for (uint8_t i = 0; i < n; i++) {
        char c = m.data[i];
        lastJunkText[i] = (c >= 32 && c < 127) ? c : '.';
      }
      lastJunkText[n] = '\0';
      haveJunk = true;
    }
  }
  // PC 串口来的命令(ACK/HAPTIC/CALIBRATE/DEBUG) -> 无线转发给箱子端
  String line;
  if (readSerialLine(line)) {
    espnowSend(line.c_str());
  }
  // 压测进行中：每秒打印一次进度（收到数 / 序号跳变丢失）
  if (testRunning) {
    uint32_t nowt = millis();
    if (nowt - lastTestStat >= 1000) {
      lastTestStat = nowt;
      char tb[96];
      snprintf(tb, sizeof(tb), "LOG,[%c] TEST progress rx=%lu gaplost=%lu last=%ld",
               gBoard, (unsigned long)testReceived, (unsigned long)testGapLost,
               testLastSeq);
      Serial.println(tb);
    }
  }
  // 每 5 秒打印一次链路计数，便于确认无线连通
  uint32_t now = millis();
  if (now - lastLinkLog >= 5000) {
    lastLinkLog = now;
    char buf[96];
    snprintf(buf, sizeof(buf), "LOG,[%c] link frames=%lu dropped=%lu foreign=%lu",
             gBoard, (unsigned long)rxFrameCount, (unsigned long)droppedFrames,
             (unsigned long)foreignFrames);
    Serial.println(buf);
    if (haveJunk) {
      char jb[96];
      snprintf(jb, sizeof(jb),
               "LOG,[%c] junk from %02X:%02X:%02X:%02X:%02X:%02X len=%u data=%s",
               gBoard, lastJunkMac[0], lastJunkMac[1], lastJunkMac[2],
               lastJunkMac[3], lastJunkMac[4], lastJunkMac[5],
               lastJunkLen, lastJunkText);
      Serial.println(jb);
      haveJunk = false;
    }
  }
}
#endif  // ROLE_RECEIVER
