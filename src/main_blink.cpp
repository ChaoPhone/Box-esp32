#include <Arduino.h>

// =============================================================
//  ESP32-S3 板载全彩小灯（WS2812 / NeoPixel）闪烁例程
//  构建 / 烧录 / 监视：
//    pio run -e blink -t upload --upload-port <下载态COM>
//    pio device monitor -e blink
// -------------------------------------------------------------
//  说明：
//  S3-N16R8D 核心板的“板载小灯”多为 WS2812 全彩灯，必须用
//  neopixelWrite() 按时序驱动，普通 digitalWrite 点不亮。
//
//  默认数据脚 GPIO48（这类核心板最常见）。如果烧进去不亮：
//    1) 把 RGB_LED_PIN 改成 38 再试（另一常见引脚）；
//    2) 若你的板载灯其实是普通单色 LED，则改用 digitalWrite 版本。
//  不确定亮不亮时，也可看串口打印确认程序在跑。
// =============================================================

static const int RGB_LED_PIN = 48;         // 板载 WS2812 数据脚（不亮就试 38）
static const uint8_t BRIGHTNESS = 40;      // 亮度 0~255，别太刺眼
static const uint32_t BLINK_INTERVAL_MS = 500;

static bool ledOn = false;
static uint8_t colorIndex = 0;
static uint32_t lastToggle = 0;

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.printf("[blink] onboard WS2812 on GPIO%d, interval=%lums\n",
                RGB_LED_PIN, (unsigned long)BLINK_INTERVAL_MS);
  neopixelWrite(RGB_LED_PIN, 0, 0, 0);  // 先熄灭
  lastToggle = millis();
}

void loop() {
  uint32_t now = millis();
  if (now - lastToggle < BLINK_INTERVAL_MS) return;
  lastToggle = now;
  ledOn = !ledOn;

  if (!ledOn) {
    neopixelWrite(RGB_LED_PIN, 0, 0, 0);
    Serial.printf("[blink] %lu ms  LED=OFF\n", (unsigned long)now);
    return;
  }

  // 点亮：每次换一种颜色，红 -> 绿 -> 蓝 循环，方便确认灯确实是全彩
  uint8_t r = 0, g = 0, b = 0;
  switch (colorIndex % 3) {
    case 0: r = BRIGHTNESS; break;
    case 1: g = BRIGHTNESS; break;
    case 2: b = BRIGHTNESS; break;
  }
  colorIndex++;
  neopixelWrite(RGB_LED_PIN, r, g, b);
  Serial.printf("[blink] %lu ms  LED=ON rgb(%u,%u,%u)\n",
                (unsigned long)now, r, g, b);
}
