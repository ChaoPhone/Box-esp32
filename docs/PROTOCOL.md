# B-Box Serial Protocol v1

> 面向 **Unity / 桥接脚本作者** 的单一事实来源。板子固件（`src/main_wired.cpp`）已按本规范实现并烧录。
> 你只需按本文档收发文本行，即可与板子通信；无需读固件源码。

---

## 1. 传输层

| 项目 | 值 |
|---|---|
| 物理层 | USB CDC 虚拟串口（板子上电即枚举出一个 COM 口） |
| 波特率 | **115200**，8 数据位 / 无校验 / 1 停止位（8N1） |
| 编码 | 纯 ASCII |
| 帧格式 | **一条消息 = 一行**，以 `\n`（LF）结尾；板子同时容忍 `\r\n` |
| 字段分隔 | 英文逗号 `,` |
| 首字段 | 大写 **TYPE**（消息类型），决定后续字段含义 |

一条消息的通用形状：

```
TYPE,arg1,arg2,...,argN\n
```

---

## 2. 方向约定

- **上行 Uplink**：板子 → 桥接 → Unity（板子主动上报）。
- **下行 Downlink**：Unity → 桥接 → 板子（把游戏状态/命令发给板子，即“发送方向”）。

桥接脚本职责：把串口收到的**上行行**解析后转给 Unity；把 Unity 发来的**下行行**原样按行写入串口（末尾补 `\n`）。

---

## 3. 上行消息（板子 → 你）

| TYPE | 格式 | 触发时机 / 含义 |
|---|---|---|
| `BOOT` | `BOOT,<fw>,proto=<n>` | 板子上电/复位后发一次。`fw`=固件版本，`n`=协议版本（本文件=1）。收到即可认为板子就绪，建议清空本地会话状态。 |
| `HEARTBEAT` | `HEARTBEAT,<ms>,<state>` | 约每 1 秒一条。`ms`=板子开机毫秒数，`state` 目前恒为 `STABLE`。**超过 3 秒收不到 = 链路异常**。 |
| `EVT` | `EVT,<seq>,<action>,<value>,<ms>` | 板子上报一次高层动作事件。`seq` 单调递增，需回 `ACK,<seq>`。`value`：推动=200.0，倾斜=角度(°)，抬举=加速度(g)。action 名清单见下。 |
| `OK` | `OK,<TYPE>[,<detail>]` | 对某条下行命令执行成功的确认。 |
| `NAK` | `NAK,<TYPE>,<reason>` | 下行命令被拒绝或未识别。 |
| `PONG` | `PONG` 或 `PONG,<token>` | 对下行 `PING` 的应答。 |
| `IMUQ` | `IMUQ,<qw>,<qx>,<qy>,<qz>,<ms>` | Mahony 互补滤波后的四元数姿态（4 位小数），常态+DEBUG 均上报。**推荐 Unity 用此消息获取姿态。** |
| `TILTS` | `TILTS,<pitch>,<roll>,<ms>` | 倾斜持续状态（pitch/roll 角度°，20Hz），供弹珠模式持续消费。 |
| `IMU` | `IMU,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<ms>` | 原始遥测（加速度 g / 角速度 °/s），**仅 DEBUG 模式**。 |
| `LOG` | `LOG,<text>` | 人类可读日志，无需机器解析。 |

### 3.1 EVT action 名清单（v0.9.0-flip）

以下为 B-Box 全部 action 名。✅ = 固件已实现，📋 = 预留待实现。

#### 推动（加速度脉冲检测，✅ 已实现）

| action | 含义 |
|--------|------|
| `MOVE_FORWARD` | 推向盒体"前"方（外壳箭头方向） |
| `MOVE_BACKWARD` | 推向盒体"后"方 |
| `MOVE_RIGHT` | 推向盒体"右"方 |
| `MOVE_LEFT` | 推向盒体"左"方 |
| `LIFT_UP` | 向上提起（Z轴加速度脉冲） |

#### 翻面 / 倾斜手势（姿态角检测，✅ 已实现）

| action | 含义 | 触发条件 |
|--------|------|----------|
| `UNDO` | 撤销一步 | 0.5s内快速翻转：90°→110°→回落（<110°） |
| `RESET` | 重置游戏 | 翻面 >110° 保持 2s |
| `MARBLE_ENTER` | 进入弹珠模式 | 倾斜角在 [45°,90°] 内持续 3s |
| `MARBLE_EXIT` | 退出弹珠模式 | 弹珠模式中，倾斜角在 [45°,90°] 内持续 3s |

> **TILT_FORWARD/BACKWARD/LEFT/RIGHT 已废弃**，倾斜改为持续状态消息 `TILTS,<pitch>,<roll>,<ms>`（与 IMUQ 同频 20Hz 上报）。宿主侧从 TILTS 提取 pitch/roll 计算 tilt 向量。

#### 旋转（陀螺仪 Z 轴角速度检测，📋 预留）

| action | 含义 | 预期触发条件 |
|--------|------|-------------|
| `ROTATE_CW` | 顺时针旋转 | 陀螺 Z 轴持续 > 阈值 |
| `ROTATE_CCW` | 逆时针旋转 | 陀螺 Z 轴持续 < -阈值 |
| `LIFT_UP` | 向上提起 | 世界帧 Z 轴动态加速度 > 0.40g，且比率大于各倾斜轴 |

**方向基准**：与推动方向统一，以加速度方向为准——"向前推"和"向前倾"语义一致。
多方向同时超阈值时只取**相对阈值比率最大**者。

**判定链**：四元数 → pitch/roll → 过 45° 阈值 → 比率优先选向 →
同向抑制（3s）+ debounce（800ms）→ re-arm 迟滞（回落 < 20° 持续 300ms）。

当前参数（固件常量，调参需重烧）：

| 参数 | 值 | 含义 |
|---|---|---|
| `TILT_ANGLE_THRESH_DEG` | 45° | 倾斜触发阈值 |
| `TILT_LIFT_THRESH_G` | 0.35 g | 抬举加速度阈值 |
| `TILT_DEBOUNCE_MS` | 800 ms | 触发后锁定 |
| `TILT_SAME_DIR_SUPPRESS_MS` | 3000 ms | 同向抑制窗 |
| `TILT_REARM_ANGLE_DEG` / `QUIET_MS` | 20° / 300 ms | 复位迟滞：回落至 20° 以下持续 300ms |
| `TILT_LIFT_REARM_G` | 0.15 g | 抬举复位阈值 |

#### 拿起/放置（加速度幅值 + 姿态检测，📋 预留。向上提起已实现为 `LIFT_UP`，见倾斜/抬举节）

| action | 含义 | 预期触发条件 |
|--------|------|-------------|
| `LIFT` | 拿起 | 加速度幅值变化 + 姿态偏离水平 |
| `PLACE` | 放下 | 加速度恢复 ~1g + 姿态回归水平 |

#### 摇晃/敲击（加速度高频检测，📋 预留）

| action | 含义 | 预期触发条件 |
|--------|------|-------------|
| `SHAKE` | 摇晃 | 加速度高频振动幅度 > 阈值 |
| `TAP` | 敲击 | 单次加速度尖峰 |

#### 控制（📋 预留，戒指输入）

| action | 含义 |
|--------|------|
| `CONFIRM` | 确认 |
| `UNDO` | 撤销 |
| `RESET` | 重置 |

### EVT 的 ACK 约定（可靠上报）
板子发出 `EVT,<seq>,...` 后会等待 `ACK,<seq>`。若 `ACK_TIMEOUT`（300ms）内没收到，会用**相同 seq** 重传，最多 5 次。
因此桥接侧要**按 seq 去重**：收到 EVT 先回 `ACK,<seq>`，若该 seq 已处理过则只回 ACK、不重复转发给 Unity。

---

## 4. 下行消息（你 → 板子）

> 所有下行命令，板子都会回 `OK,...` 或 `NAK,...`（`PING` 除外，它回 `PONG`）。你可用它确认命令已送达生效。

| TYPE | 格式 | 作用 | 板子回复 |
|---|---|---|---|
| `STATE` | `STATE,<name>` | **核心：发送游戏状态**，板子据此做可视反馈（当前=板载 WS2812 灯上色，后续换屏幕/马达不改协议）。 | `OK,STATE,<NAME>,<r>,<g>,<b>` / `NAK,STATE,unknown` |
| `LED` | `LED,<r>,<g>,<b>` | 直接指定 RGB（各 0–255），用于调试或自定义效果，绕过状态映射。 | `OK,LED,<r>,<g>,<b>` / `NAK,LED,badargs` |
| `HAPTIC` | `HAPTIC,<pattern>[,<ms>]` | 震动电机反馈（GPIO1，PWM 驱动）。pattern：`short`(50ms)/`long`(300ms)/`double`(双震)/`0-255`(PWM 强度)。ms 覆盖默认时长。 | `OK,HAPTIC,motor=<强度>,<时长>ms` |
| `ACK` | `ACK,<seq>` | 确认收到板子的 `EVT,<seq>`（见 §3）。 | 无回复（内部清除待确认） |
| `PING` | `PING` 或 `PING,<token>` | 存活/往返探测。 | `PONG` / `PONG,<token>` |
| `CALIBRATE` | `CALIBRATE` | **完整复位链**：陀螺零偏标定（约 1s，200 样本）→ 姿态四元数重置 → 推动检测基线/状态清零 → 进入 1.5s 静默期。**标定期间必须保持盒子静止**。 | 有 IMU：`OK,CALIBRATE,gx=..,gy=..,gz=..`；无 IMU：`OK,CALIBRATE,no-sensor`；读失败：`NAK,CALIBRATE,read-fail` |
| `ORIENT` | `ORIENT,<swapXY>,<signX>,<signY>` | 推动方向软件校正（组装方向与约定不符时用，掉电不保存）。`swapXY`=0/1 交换前后↔左右；`signX`/`signY`=1/-1 翻转左右/前后。例：前后颠倒→`ORIENT,0,1,-1`。 | `OK,ORIENT,<swapXY>,<signX>,<signY>` / `NAK,ORIENT,badargs` |
| `DEBUG` | `DEBUG,<0|1>` | 打开/关闭板子的详细日志。 | `OK,DEBUG,0` / `OK,DEBUG,1` |
| `SIM` | `SIM,<action>` | **仅开发用**：让板子模拟产生一次上行 `EVT`（用来自测上行链路）。 | 触发一条 `EVT,...` |

### 4.1 STATE 的标准状态名与灯色映射

| `<name>`（大小写不敏感） | 别名 | 灯色 | 语义 |
|---|---|---|---|
| `IDLE` | `READY` | 绿 | 待命 |
| `MOVE` | `STEP` | 蓝 | 移动一格 |
| `WALL` | `BLOCK` | 橙 | 撞墙/受阻 |
| `WIN` | `SUCCESS` | 黄 | 通关 |
| `LOSE` | `FAIL` | 红 | 失败 |
| `OFF` | — | 灭 | 关闭反馈 |

未列出的名字 → 板子回 `NAK,STATE,unknown`（不会崩）。**需要新增状态时**：在固件 `applyGameState()` 里加一条映射即可，协议本身不变。

---

## 5. 扩展性规则（向前兼容保证）

设计这些规则，是为了让**协议先落地、以后随时加东西而不破坏已有实现**：

1. **未知 TYPE 不致命**：板子对不认识的命令回 `NAK,<TYPE>,unknown` 并继续正常运行。
2. **多余尾部字段被忽略**：给某条命令追加新字段（如未来 `STATE,WIN,score=100`），旧固件会忽略多出来的部分，不报错。
3. **每条下行命令有确认**：靠 `OK`/`NAK` 判断送达，不要假设“发了就一定生效”。
4. **协议版本可协商**：`BOOT` 里带 `proto=<n>`。桥接可据此判断板子协议版本，做兼容处理。
5. **新增消息类型**：直接定义新的大写 TYPE 即可，不影响既有类型解析。

---

## 6. 最小桥接实现指引（给 Unity 侧）

桥接脚本（Python/pyserial 或 Unity 直连串口）最少要做四件事：

1. **打开串口** `115200 8N1`，按行读（`readline`）。
2. **上行分发**：按行首 TYPE 分流——
   - `EVT,` → 回写 `ACK,<seq>`，按 seq 去重后转发给游戏逻辑；
   - `HEARTBEAT,` → 刷新“最后存活时间”，>3s 无心跳则告警；
   - `BOOT,` → 重置本地会话（清空已见 seq）；
   - `OK,`/`NAK,`/`PONG,` → 与你最近发出的下行命令对账；
   - `LOG,` → 打印即可。
3. **下行写入**：把游戏状态按 §4 拼成一行、末尾加 `\n` 写串口。核心就是 `STATE,<name>`。
4. **健壮性**：丢弃空行/非法行不要崩；一条命令没等到 `OK` 可重发。

### 端到端示例（一次通关反馈）
```
你 →板：  STATE,WIN
板 →你：  OK,STATE,WIN,40,40,0        # 已上色（黄），确认生效
```
```
你 →板：  PING,42
板 →你：  PONG,42                     # 链路可用
```
```
板 →你：  EVT,7,MOVE_RIGHT,200.0,15230
你 →板：  ACK,7                       # 确认，板子停止重传
```

### 6.1 推荐的复位（校准）流程

固件的方向判定依赖姿态收敛，**何时复位**由宿主控制。推荐时机：

1. **会话开始（必做）**：收到 `BOOT` 或串口刚连上 → 提示玩家"盒子放平不要动" → 发 `CALIBRATE` → 等 `OK,CALIBRATE,...`（约 1s）→ 再等 **1.5s 静默期** → 进入可玩状态。总耗时约 2.5s，期间 UI 显示"校准中"。
2. **关卡开始/重开**：如果盒子刚被搬动或翻转过，重复上面流程；只是正常放着则不必。
3. **异常恢复**：出现方向持续判错（玩家反馈"推前出后"）→ 引导静置 → `CALIBRATE`。若是安装方向错位（前后颠倒/前后左右互换），用 `ORIENT` 一次性修正即可，不用反复校准。
4. **不要在游戏进行中随意 CALIBRATE**：静默期内所有推动都会被吞掉。

```
宿主状态机：  DISCONNECTED → (BOOT) → CALIBRATING → (OK+1.5s) → READY ⇄ PLAYING
```

### 6.2 推荐的 EVT 消费逻辑

固件已保证**一次物理推动 = 恰好一条 EVT**（峰值窗判向 + 回摆抑制 + 迟滞复位），所以宿主侧应当：

1. **直接消费，不要再做时间去抖**——固件已经做完了，宿主再叠一层只会丢输入。
2. **按 seq 去重**（必做）：EVT 有 300ms×5 次的同 seq 重传机制。收到 `EVT,<seq>,...` 先立刻回 `ACK,<seq>`；`seq` 已处理过则**只回 ACK 不重复执行**。维护"最近已处理 seq"即可（seq 单调递增，记一个水位线就够）。
3. **一条 EVT = 一步游戏动作**：`MOVE_FORWARD` → 角色前移一格。移动失败（撞墙）不需要特殊处理输入，只需回发 `STATE,WALL` 给玩家灯光反馈。
4. **忽略"过期"事件**：EVT 自带板子时间戳 `<ms>`。若宿主刚做完场景切换/暂停恢复，可丢弃 `<ms>` 早于恢复时刻的事件（结合心跳的 ms 换算），避免暂停期间积压的推动突然执行。
5. **闭环反馈**：每消费一条 EVT，回发对应 `STATE,MOVE` / `STATE,WALL` / `STATE,WIN`，玩家从灯色立即知道这次推动被识别了——这是实体交互手感的关键。

伪代码：

```
onLine(line):
  if line.startswith("EVT,"):
    seq, action, value, ms = parse(line)
    send("ACK," + seq)              # 先 ACK，止住重传
    if seq <= lastSeq: return       # 重传去重
    lastSeq = seq
    if state != PLAYING: return     # 校准中/菜单中不消费
    ok = game.tryMove(action)       # 一条 EVT 一步
    send("STATE," + (ok ? "MOVE" : "WALL"))
```

---

## 7. 零成本自测（不接 Unity 也能验）

参考仓库里的 `docs/esp32_bridge.py`（现有桥接样例）：它监听本地 UDP `47801`，会把收到的文本**原样透传**给板子。所以可以这样单机验证下行：

```powershell
cd e:\project\2026\2026advx\esp32\esp32_shot
pio run -e wired_sim -t upload --upload-port <COM>
python docs\esp32_bridge.py --port <COM>
# 另开一个终端：
python -c "import socket; socket.socket(2,2).sendto(b'STATE,WIN',('127.0.0.1',47801))"
```
桥接终端应打印 `OK,STATE,WIN,40,40,0`，同时板载灯变黄。

> 板子靠固定 MAC 区分 A/B，不依赖 COM 号；具体见 `docs/HARDWARE_BOARDS.md`。
