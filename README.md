# Pico EEPROM HID 烧录器

用树莓派 Pico 当作 **USB → I²C 桥**，读写 24C01 ~ 24C256 系列 EEPROM。上位机是免驱的自定义 USB HID 设备（VID `2E8A` / PID `0005`），**不用装 CH341 驱动、不用选串口**。

- 在线模式：PC 上打开上位机，直接读取 / 烧录芯片。
- 离线模式：把镜像存进 Pico 板载 Flash，**拔掉电脑**后每次上电自动烧一片，适合批量复制。

---

## 1. 文件清单

| 文件 | 作用 |
|---|---|
| `RPI_PICO-20260406-v1.28.0.uf2` | MicroPython v1.28.0 固件，全新 Pico 必须先刷 |
| `boot.py` | 保持启动阶段为纯 CDC，不碰 USBDevice |
| `hid_usb.py` | 64B vendor HID 接口（EP3 IN/OUT） |
| `hidproto.py` | 帧协议、CRC8、命令字定义 |
| `eeprom24.py` | 24Cxx I²C 驱动（GP4/GP5） |
| `eeprom_image.py` | 板载 Flash 镜像存取（`eeprom.img` / `burn.stat`） |
| `main.py` | 在线命令分发 + 上电离线自动烧录 + 提示灯 |
| `PythonEEPROM_v2.py` | PC 上位机（十六进制编辑、读写、离线镜像、灯配置） |
| `deploy_new_pico.py` | 一键部署脚本（刷 UF2 + 传固件 + CRC 校验） |

---

## 2. 硬件接线（重点）

### 2.1 需要的材料

- 树莓派 Pico（RP2040）一块
- 24C01 ~ 24C256 EEPROM 一片
- **4.7 kΩ 电阻 × 2**（SDA、SCL 上拉到 3V3；若用现成 EEPROM 模块板且已带上拉，可省）
- 提示灯：默认用 Pico 板载 GP25 LED，不用外接；想外扩可接 GPIO + 330 Ω 限流电阻

> ⚠️ 所有接线必须在**断电状态**下完成。带电插拔 I²C 线容易烧 IO。

### 2.2 Pico ↔ EEPROM（裸芯片 DIP-8）

EEPROM 芯片**缺口朝上**时引脚如下：

```
        24Cxx (DIP-8)
       ┌────────────┐
  A0 ──┤1          8├── VCC ── 3V3(OUT)
  A1 ──┤2          7├── WP  ── GND（允许写入）
  A2 ──┤3          6├── SCL ── Pico GP5
 GND ──┤4          5├── SDA ── Pico GP4
       └────────────┘
```

**Pico 端只用 5 根线**（脚号是 Pico 物理引脚编号，USB 口朝上、左列从 1 数起）：

| Pico 物理脚 | GPIO | 接到 EEPROM | 说明 |
|---|---|---|---|
| 6 | **GP4** | SDA（5 脚） | I²C 数据 |
| 7 | **GP5** | SCL（6 脚） | I²C 时钟 |
| 36 | **3V3(OUT)** | VCC（8 脚） | 3.3 V 供电，**不要接 5V/VBUS** |
| 38（或 3/8/13…） | GND | GND（4 脚） | 共地 |
| — | — | A0/A1/A2（1/2/3 脚）、WP（7 脚） | 全部接 GND → 从机地址 `0x50`、允许写入 |

上拉电阻接法：

```
3V3(OUT) ──┬── 4.7k ── SDA ── GP4
           └── 4.7k ── SCL ── GP5
```

### 2.3 可选：离线烧录跳线与外接提示灯

| Pico 物理脚 | GPIO | 用途 |
|---|---|---|
| 9 | **GP6** | 上电时接 GND = 跳过本次自动烧录（方便插电脑调试）。悬空则正常自动烧 |
| — | **GP25** | 默认提示灯 = 板载 LED。若在上位机改成其它 GPIO，需自行外接 LED（串 330 Ω 到 GND），**避开 GP4/GP5/GP6** |

### 2.4 用现成 EEPROM 模块板

模块板通常已经焊好 4.7 kΩ 上拉，只需接 `VCC / GND / SDA / SCL` 四根线；确认模块是 **3.3 V 逻辑电平**，且 `A0~A2 / WP` 已被板子拉到 GND（否则地址不是 `0x50`，扫描不到）。

---

## 3. 快速开始

### 3.1 新 Pico 首次部署

环境要求：macOS 上装了 `pyserial` 的 Python（本机是 `/Users/mac/.local/bin/python3.12`），以及 `mpremote` 可用。

1. **只接一块 Pico**（拔掉其它板子，否则按 VID/PID 会认错）。
2. 先本地自检，不碰硬件：
   ```
   /Users/mac/.local/bin/python3.12 deploy_new_pico.py --check
   ```
3. 按住 Pico 上的 **BOOTSEL** 键插入 USB，出现 `RPI-RP2` 盘后运行：
   ```
   /Users/mac/.local/bin/python3.12 deploy_new_pico.py
   ```
   脚本会自动刷 UF2 → 等 CDC 串口 → 上传 6 个 `.py`（256B 分块、`os.sync()` + 逐文件 CRC 校验）→ 软复位。
4. 如果板上已有固件或镜像，脚本默认会停下防误刷；确定要重装就加 `--force`。

> 脚本**不会**调用 `hid_usb.start()`——那会把 CDC 一起摘掉导致 USB 挂死。HID 由上位机打开后自动激活。

### 3.2 打开上位机

```
/Users/mac/.local/bin/python3.12 PythonEEPROM_v2.py
```

首次打开时，后台经 CDC REPL 自动下发 `hid_usb.start()` + `main.main()` 激活 HID。状态栏灯变绿（已识别 EEPROM）或黄（已连接但没找到 EEPROM）就说明链路通了。

### 3.3 在线读写

1. 顶部「选择芯片」下拉框选型号（24C01 ~ 24C256）。
2. 点「读取」把芯片内容读进编辑区，或直接「打开文件」载入 `.bin`。
3. 在十六进制编辑区修改字节，点「烧录」写入芯片。
4. 「保存」导出 `.bin`；「清空FF / 清空00」填充缓冲区。

---

## 4. 离线烧录（脱离电脑批量复制）

```
① 上位机「存入设备」→ 镜像经 HID 分块写进 Pico Flash（eeprom.img，带长度 + CRC32 校验）
② 关上位机、断开 USB，把 EEPROM 接到已接好线的目标板
③ 给 Pico 上电 → 自动读镜像 → 校验 CRC → 写 EEPROM → 回读比对
④ 断电再上电 = 再烧一片；GP6 接 GND 上电可跳过本次
```

**板上提示灯（默认 GP25）灯语**：

| 灯语 | 含义 |
|---|---|
| 慢闪 3 次 | 烧录成功且回读一致 |
| 快闪 10 次 | 失败（无芯片 / 无镜像 / I²C 失败 / 校验不一致） |
| 在线读写命令亮约 220 ms | 链路活动指示；PING 不亮 |

菜单「离线烧录(O)」里还有：从设备 Flash 读取、在线触发一次烧录、查看设备镜像、清除设备镜像。镜像最大 32 KB（24C256），写在 MicroPython 的 LFS 里；**刷 UF2 不会清掉它，`flash_nuke.uf2` 才会**。

---

## 5. 支持的芯片

| 型号 | 容量 | 页大小 | 字地址 |
|---|---|---|---|
| 24C01 | 128 B | 8 B | 1 字节 |
| 24C02 | 256 B | 8 B | 1 字节 |
| 24C04 | 512 B | 16 B | 1 字节 |
| 24C08 | 1 KB | 16 B | 1 字节 |
| 24C16 | 2 KB | 16 B | 1 字节 |
| 24C32 | 4 KB | 32 B | 2 字节 |
| 24C64 | 8 KB | 32 B | 2 字节 |
| 24C128 | 16 KB | 64 B | 2 字节 |
| 24C256 | 32 KB | 64 B | 2 字节 |

24C16 及以下的高位地址编码在 I²C 从机地址 bit3:1，驱动已处理，传输不跨 256B 边界；页写后使用 ACK polling 等待写周期完成。

---

## 6. HID 协议（二次开发参考）

64 字节定长中断报告，无 Report ID，Usage Page `0xFF00` / Usage `0x01`，IN `0x83` / OUT `0x03`。hidapi 写入格式（首字节补 `0x00` 的 `id0` 或直接 64B 的 `raw`）由 PING 握手自动探测。

```
字节 0      : 0xA5 帧头
字节 1      : CMD（响应 = CMD | 0x80）
字节 2      : SEQ 主机自增，响应原样带回
字节 3      : LEN 负载长度 0..59
字节 4..    : PAYLOAD
字节 4+LEN  : CRC8（多项式 0x07，初值 0，覆盖 [0, 4+LEN)）
其余        : 补 0
```

| CMD | 名称 | 说明 |
|---|---|---|
| `0x01` | PING | 握手，返回固件标识 `Pico EEPROM HID v1.2` |
| `0x03` | GET_INFO | 固件信息 |
| `0x50` | EEPROM_PROBE | 扫描 I²C 总线 |
| `0x51` | EEPROM_READ | `[chip_id, addr_hi, addr_lo, len]` → `[status, data]` |
| `0x52` | EEPROM_WRITE | `[chip_id, addr_hi, addr_lo, len, data]` → `[status, n]` |
| `0x60`~`0x67` | FLASH_* | INFO / BEGIN / DATA / COMMIT / READ / ERASE / BURN / STAT |
| `0x68` / `0x69` | LED_GET / LED_SET | 提示灯 GPIO 查询 / 设置（存 `led.cfg`，掉电保留） |
| `0x40`~`0x43` | ERR_* | 未知命令 / 非法帧 / I²C 失败 / Flash 失败 |

读分块 56 B，写分块 16 B。镜像格式：16B 头 `PICOIMG1 + chip_id + size(LE) + CRC32(LE) + flags` + 数据区。

---

## 7. 常见问题

| 现象 | 排查 |
|---|---|
| 状态栏黄灯「未检测到 EEPROM」 | 断电检查 SDA/SCL 是否接反、A0~A2/WP 是否接地、4.7k 上拉是否接上、是否接到 3V3(OUT) |
| 上位机一直转圈 | 只开一个上位机进程（hidapi 独占设备）；Thonny 若占着 CDC 会阻止 HID 激活 |
| 拔插后连不上 | 关掉旧进程重开；脚本不会重复调 `hid_usb.start()`，重插后上位机自动重新激活 |
| 下拉框选不了 / 界面卡住 | 关掉旧窗口重开；旧版代码有状态残留问题 |
| 提示灯不亮 | 菜单「设备(D) → 提示灯 GPIO」查看当前脚；默认 GP25，改脚后板上会闪两下确认 |
| 一次接多块 Pico | 不行，脚本和上位机按 VID/PID 只认第一块，请只接一块 |
| macOS 上提示 `No module named hid` | 必须用装了依赖的解释器运行：`<解释器绝对路径> -m pip install hidapi pyserial`，例如 `/Users/mac/.local/bin/python3.12 -m pip install hidapi pyserial` |

---

## 8. 实现说明

- 冷启动保持纯 CDC；HID 由 PC 经 CDC REPL 一次性激活，因此**不需要手动选串口**。要彻底去掉 CDC、上电即 HID，需要迁到 C SDK + TinyUSB（当前 MicroPython 方案的边界）。
- USB 回调运行在中断上下文，只置标志，不做 `print()` / `submit_xfer()`；所有处理在主循环完成。
- 主循环对普通异常 `except Exception: pass`（保留 `KeyboardInterrupt`），否则一次 USB 重枚举就会让 HID 链路永久死掉。
- 上位机工作线程通过 `queue.Queue` 把结果交回主线程，不在工作线程调 `root.after(0, ...)`（Tk 9.0 在 macOS 会静默丢事件）。
