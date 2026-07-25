# B-Box 硬件板卡登记

> 本文件登记项目使用的 ESP32-S3 板卡信息与烧录流程。
> 目前共两块板，本页先登记 **A 板**（B 板到位后按同样格式补充）。

---

## A 板

### 身份信息（esptool 实测）

| 项目 | 值 |
|---|---|
| 别名 | **A 板** |
| 型号 | 4D Systems ESP32-S3 核心开发板（N16R8D） |
| 芯片 | ESP32-S3 (QFN56) revision **v0.2** |
| 无线 | WiFi + BLE（ESP-NOW 基于 WiFi） |
| PSRAM | **8MB**（Embedded, AP_3v3）→ 对应 R8 |
| Flash | **16MB**（Manufacturer 0x68 / Device 0x4018）→ 对应 N16 |
| Flash 模式 | quad（4 数据线），3.3V |
| 晶振 | 40MHz |
| USB 模式 | 芯片内置 USB-Serial/JTAG（无外挂 CH340/CP2102） |
| **MAC** | **E8:3D:C1:F2:C7:B8**（WiFi/ESP-NOW 站点 MAC） |
| PlatformIO board | `4d_systems_esp32s3_gen4_r8n16`（分区表 default_16MB.csv） |

### COM 端口行为（重要）

这块板**运行态和下载态是两个不同的 USB 身份 / COM 口**：

| 状态 | VID:PID | 实测 COM 口 | 说明 |
|---|---|---|---|
| 运行态（跑固件） | 303A:**4001** | COM9 | 平时监视串口用这个 |
| 下载态（烧录） | 303A:**1001** | COM10 | 手动进下载模式后出现 |

> COM 号是本机当次枚举结果，换电脑/换 USB 口可能变化，以实际 `pio device list` 为准；但“运行态/下载态是两个口”这个规律不变。

### 已知注意事项

1. **首次烧录/换固件需手动进下载模式**：出厂 demo 会一直占用串口打印日志，esptool 自动复位进不了下载模式（症状：`Invalid head of packet (0x1B)` 或 `No serial data received`）。
2. 进下载模式后 COM 口会**重新枚举变号**，烧录前务必重新确认端口。

---

## B 板

### 身份信息（esptool 实测）

| 项目 | 值 |
|---|---|
| 别名 | **B 板** |
| 型号 | ESP32-S3 核心开发板（N16R8D），与 A 板同型号 |
| 芯片 | ESP32-S3 (QFN56) revision v0.2 |
| PSRAM / Flash | 8MB / 16MB（quad, 3.3V） |
| USB 模式 | 芯片内置 USB-Serial/JTAG |
| **MAC** | **E8:3D:C1:FA:7A:0C**（唯一固定标识） |
| PlatformIO board | `4d_systems_esp32s3_gen4_r8n16` |

---

## 双板区分规范（重要）

A 板与 B 板**芯片规格完全相同**，唯一可靠的固定区分特征是 **MAC 地址**：

| | A 板 | B 板 |
|---|---|---|
| **MAC** | `E8:3D:C1:F2:C7:B8` | `E8:3D:C1:FA:7A:0C` |

- **不要用 COM 号或 USB LOCATION 区分两板**：COM 号随运行态/下载态、插拔、换口而变，不是芯片固有特征。
- 下载态（VID:PID=303A:1001）时，USB 描述符的 `SER` 字段直接就是 MAC，可用
  `python -c "import serial.tools.list_ports as lp;[print(p.device,p.hwid) for p in lp.comports()]"`
  一眼确认哪个口是哪块板。
- 运行态（303A:4001）`SER` 为通用值（如 123456），此时需靠固件自报 MAC 或 esptool 读取。
- 后续 ESP-NOW 若要点对点绑定，用上面两个 MAC 作为 peer 地址。

---

## 烧录流程（照此操作）

### 步骤 1：进入下载模式
> **按住 BOOT 键 → 点按一下 RST/EN 键 → 松开 BOOT 键**

### 步骤 2：确认下载态 COM 口
在 `esp32_shot/` 目录执行：
```powershell
pio device list
```
找到 `VID:PID=303A:1001` 那个（本机是 COM10），记下端口号。

### 步骤 3：编译并烧录
选一个环境（见 platformio.ini）：
```powershell
pio run -e blink          -t upload --upload-port COM10   # 点灯例程
pio run -e wired_sim      -t upload --upload-port COM10   # 有线串口协议固件
pio run -e espnow_sender  -t upload --upload-port COM10   # 无线-箱子端
pio run -e espnow_receiver -t upload --upload-port COM10  # 无线-接收端
```
- 若卡在 `Connecting...`：说明没进下载模式，重做步骤 1 再烧。
- 烧完 esptool 会 `Hard resetting...`，板子自动复位回运行态。

### 步骤 4：打开串口监视器看输出
烧了我们的固件后（USB-CDC on boot），运行态串口即可看日志：
```powershell
pio device list                       # 重新确认运行态 COM 口
pio device monitor -p COM9 -b 115200  # 波特率 115200
```
- `blink`：应每 0.5s 打印 `[blink] ... LED=ON/OFF`
- `wired_sim`：应看到 `BOOT,...` 和每秒 `HEARTBEAT,...`；输入 `w/a/s/d` 可触发动作事件

### 常用命令速查
```powershell
pio run                     # 编译全部环境
pio run -e <env>            # 只编译某个环境
pio device list             # 列出串口
pio device monitor -p COMx -b 115200
```

---

## 验收 / 串口监视（标准方式）

日常看串口、验收无线/固件功能，统一用 `docs/monitor.py`——**一条命令同时监视两块板，按 MAC 自动标 `[A板]`/`[B板]`，颜色区分**：

```powershell
cd E:\project\2026\2026advx\esp32\esp32_shot
python docs\monitor.py               # 自动发现两块板，实时滚动（Ctrl+C 退出）
python docs\monitor.py --seconds 8   # 只抓 8 秒自动退出（快速取样）
python docs\monitor.py COM7 COM10    # 指定端口
```

怎么看懂输出（以 ESP-NOW 双板为例）：

| 行 | 含义 |
|---|---|
| `[B板/COM7] [B->air] HEARTBEAT,<ms>,STABLE` | B 板（发送端）**发出**的帧（本地回显） |
| `[A板/COM10] HEARTBEAT,<ms>,STABLE` | A 板（接收端）**无线收到**并转发到 USB 的帧 |
| 两行 `<ms>` 相同 | 说明同一帧「B 发→A 收」，链路连通 |
| `LOG,[A] link frames=N dropped=M foreign=K` | A 板累计收到对方 N 帧、丢弃 M 个非法帧、挡掉 K 个外来设备帧 |

- 单块板抓固定秒数也可用 `python docs\serial_read.py <COM> <秒数>`。
- 端口被 `monitor.py`/监视器占用时无法同时烧录，需先 Ctrl+C 退出。

---

## 工况压测 / 丢包率测量

无线链路已改为**点对点单播**（预烧录对方 MAC，两端锁定信道 1，源 MAC 过滤常开），不再使用广播，外来设备的 ESP-NOW 帧一律被拒收。

用 `docs/loadtest.py` 一键测丢包率：命令 B 板按指定频率连续发带序号的 `TEST` 帧，A 板按序号统计收到/丢失，结束打印精确丢包率。

```powershell
cd E:\project\2026\2026advx\esp32\esp32_shot
python docs\loadtest.py                    # 默认 100 帧/秒 × 1000 帧
python docs\loadtest.py --hz 200 --count 2000
```

结果行：`LOG,[A] TEST RESULT sent=<发送> rx=<收到> lost=<丢失> loss=<丢包率>%`

实测性能基线（两板相距约 3cm、单播 + 链路层 ACK 重传）：

| 工况 | 发送 | 收到 | 丢包率 | 说明 |
|---|---|---|---|---|
| 100 帧/秒 | 1000 | 1000 | **0.00%** | 远超 B-Box 实际需求，零丢包 |
| 200 帧/秒 | 2000 | 1939 | 3.05% | 接近单播吞吐上限 |
| 500 帧/秒 | 2000 | 1023 | 48.85% | 故意超压测出的天花板，非实际工况 |

> B-Box 体感数据实际为几十帧/秒级别，工作在 **100 帧/秒零丢包** 区间，余量充足。
> 运行 `loadtest.py` 前需先 Ctrl+C 关闭 `monitor.py` 等占用串口的程序。
