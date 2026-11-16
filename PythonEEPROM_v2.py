"""
24Cxx EEPROM 烧录工具 v3 — 树莓派 Pico / 自定义 USB HID 版
==========================================================
相对 v2(CH341 DLL 版) 的改动:
1. 通信层整体换成「Pico + 自定义 USB HID」: 免驱、不用选端口、不用 CH341 DLL。
   Pico 作为 USB<->I2C 桥, 板上跑 MicroPython 读写 24C01~24C256。
2. 协议: 64 字节定长中断报告, 无 Report ID, Usage Page 0xFF00 / Usage 0x01,
   VID=2E8A / PID=0005。
   帧 = A5 | CMD | SEQ | LEN | PAYLOAD(<=59) | CRC8(ATM 0x07) | 补零;
   响应 CMD = 请求 CMD|0x80, SEQ 原样带回; hidapi 写入格式(id0/raw)自动探测。
3. 找不到 HID 时, 后台线程经 CDC REPL 下发 hid_usb.start() + main.main()
   自动激活板子(15 秒冷却), 不阻塞界面, 可用 AUTO_ACTIVATE=False 彻底关闭。
4. 读/写按 HID 帧长分块(读 56B / 写 16B), 带进度显示。
5. 十六进制编辑器、文件拖放、状态栏、按钮逻辑全部沿用 v2, 未改动。
6. 离线烧录: 「存入设备」把编辑区镜像经 HID 分块写进 Pico 板载 Flash
   (BEGIN/DATA/COMMIT, 固件核对长度+CRC32 后才生效)。之后 Pico 每次上电
   自动把镜像烧进 EEPROM 并用板载 LED 指示结果, 完全不需要电脑。
   GP6 接 GND 上电可跳过本次自动烧录; 断电一次再上电 = 再烧一片。
7. 工具栏两行分组卡片: 文件 / 在线 EEPROM / 芯片型号, 离线镜像 / 编辑;
   芯片下拉框点击整块即可弹出, 并同步显示容量。
8. 状态栏 LED 提示灯: 绿=已连接且识别到 EEPROM, 黄=已连接但无 EEPROM,
   红=未连接, 蓝闪=读写/传输中, 灰=检测中; 右侧小圆点表示 Flash 里已存离线镜像。
9. 物理提示灯 GPIO 可自定义(菜单 设备 → 提示灯 GPIO): 命令 0x68 GET / 0x69 SET,
   固件存 led.cfg 掉电保留; 默认 GP25(板载 LED), 在线数据命令点亮约 220ms,
   设置时新 IO 闪两下确认。需固件 v1.2+。

依赖: <用来运行本脚本的 python> -m pip install hidapi pyserial   (import 名为 hid)
运行: 不同 Python 解释器不共享已装的包 —— 给哪个解释器装, 就必须用哪个解释器运行,
      直接 `pip3 install` 很容易装到另一个解释器里去(本机 pip3 指向 WorkBuddy 内置 3.13)。
配套固件: /Users/mac/Downloads/pico_eeprom_hid/ 下 5 个 .py 上传到 Pico。
"""

import os
import sys
import time
import base64
import tempfile
import threading
import queue

import tkinter as tk
from tkinter import ttk, filedialog

try:
    import hid                      # pip3 install hidapi
    HAS_HID = True
except ImportError:
    hid = None
    HAS_HID = False

try:
    import serial                   # pip3 install pyserial
    from serial.tools import list_ports
    HAS_SERIAL = True
except ImportError:
    serial = None
    list_ports = None
    HAS_SERIAL = False

try:
    from tkinterdnd2 import DND_FILES, TkinterDnD
    HAS_DND = True
except ImportError:
    HAS_DND = False

# ---------------------------------------------------------------- 常量
# 芯片型号 -> (传给 Pico 的型号ID, 容量字节数)。ID 必须与固件 eeprom24.CHIPS 下标一致。
EEPROM_CHIPS = {
    '24C01': (0, 128),
    '24C02': (1, 256),
    '24C04': (2, 512),
    '24C08': (3, 1024),
    '24C16': (4, 2048),
    '24C32': (5, 4096),
    '24C64': (6, 8192),
    '24C128': (7, 16384),
    '24C256': (8, 32768),
}
DEFAULT_CHIP = '24C02'

# --- Pico 自定义 HID 协议 ---
HID_VID, HID_PID = 0x2E8A, 0x0005
HID_USAGE_PAGE, HID_USAGE = 0xFF00, 0x01
REPORT_LEN = 64
MAX_PAYLOAD = 59
FRAME_MAGIC = 0xA5
CMD_PING = 0x01
CMD_GET_INFO = 0x03
CMD_EEPROM_PROBE = 0x50
CMD_EEPROM_READ = 0x51
CMD_EEPROM_WRITE = 0x52

# 板载 Flash 镜像 / 离线烧录 (与固件 hidproto.py 一致)
CMD_FLASH_INFO = 0x60
CMD_FLASH_BEGIN = 0x61
CMD_FLASH_DATA = 0x62
CMD_FLASH_COMMIT = 0x63
CMD_FLASH_READ = 0x64
CMD_FLASH_ERASE = 0x65
CMD_FLASH_BURN = 0x66
CMD_FLASH_STAT = 0x67
CMD_LED_GET = 0x68
CMD_LED_SET = 0x69
LED_PIN_DEFAULT = 25
LED_PIN_MAX = 28

ERR_BAD_CMD = 0x40
ERR_BAD_FRAME = 0x41
ERR_I2C = 0x42
ERR_FLASH = 0x43
FW_TAG = b'Pico EEPROM HID'        # 防止误连到跑着别的固件的 Pico

READ_CHUNK = 56                   # 响应 = status(1) + data, payload 上限 59
WRITE_CHUNK = 16                  # 对齐 8B 页, 兼容 16/32/64B 页
FLASH_CHUNK = 56                  # Flash 分块: 3B 偏移头 + 56B 数据 <= 59
BYTES_PER_ROW = 16
POLL_INTERVAL_MS = 800            # 设备轮询间隔
ACTIVATE_COOLDOWN = 15.0          # 自动激活失败后的冷却秒数
AUTO_ACTIVATE = True              # 设 False = 上位机完全不碰 CDC 串口

# macOS 没有 Consolas, Tk 会静默回退到系统比例字体(.AppleSystemUIFont):
# 不同字符宽度不同(FF=17px, 00=19px), bold 又比 normal 宽 —— hex 列必然漂移。
# 用系统自带的真等宽 Menlo, 保证列头/字节/ASCII 三列严格对齐。
FONT_MONO = ('Menlo', 11) if sys.platform == 'darwin' else ('Consolas', 11)
FONT_UI = ('PingFang SC', 10) if sys.platform == 'darwin' else ('Microsoft YaHei UI', 10)

COLOR_FF = "#1A8D10"              # 空字节(FF)
COLOR_DATA = "#051BE7"            # 非FF数据
COLOR_MODIFIED = "#E80A0A"        # 被修改过的字节
COLOR_ADDR = "#020202"            # 地址列
COLOR_ASCII = '#E9390D'           # ASCII列
COLOR_CURSOR_BG = '#1565C0'       # 当前字节: 深蓝底 + 白字(反白, 最跳)
COLOR_CURSOR_FG = '#FFFFFF'
COLOR_ROW_BG = '#E8F1FF'          # 当前行整行浅底
COLOR_COL_BG = '#FFD54F'          # 列头/ASCII 对应位置(十字定位)
COLOR_COL_FG = '#1F2933'
COLOR_ADDR_CHIP = '#1565C0'       # 状态栏"当前地址"徽标底色
COLOR_OK = '#2E7D32'
COLOR_ERR = '#C62828'
COLOR_INFO = '#1565C0'

# 状态提示灯: (灯芯, 光晕, 高光)
LAMP_COLORS = {
    'idle':  ('#9AA0A6', '#DDE1E6', '#FFFFFF'),   # 检测中
    'ok':    ('#2E7D32', '#A5D6A7', '#E8F5E9'),   # 已连接 + EEPROM 已识别
    'warn':  ('#F9A825', '#FFE082', '#FFF8E1'),   # 已连接但没检测到 EEPROM
    'error': ('#C62828', '#EF9A9A', '#FFEBEE'),   # 未连接 / 缺依赖
    'busy':  ('#1565C0', '#90CAF9', '#E3F2FD'),   # 读写中(呼吸)
}
LAMP_HELP = ('设备提示灯\n'
             '绿   已连接, EEPROM 已识别\n'
             '黄   已连接, 但未检测到 EEPROM (查接线/上拉)\n'
             '红   未连接 / hidapi 缺失\n'
             '蓝闪 正在读写或传输镜像\n'
             '灰   检测中\n'
             '右侧小圆点: 蓝 = 设备 Flash 已存离线镜像')

icon_data = b"AAABAAEAEBAAAAEAGABoAwAAFgAAACgAAAAQAAAAIAAAAAEAGAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAD////9/f28vLyioqKioqKioqKioqKioqKioqKioqKioqKioqKioqK4uLjU1NTv7+/19fX////7+/p2cWk9OC09Nyw9Nyw9Nyw9Nyw9Nyw9Nyw9Nyw9NyxEPzSKh4D8/Pz+/v7////JyMU8NitAOjA8NitBOi88NitBOi9BOi88NitBOi88NitBOjA8Nive3tz///////+/vro8NitwbGQ8NitwbGRIQjhlX1dtaGA/OS5wbGQ8NitxbGQ8NivU1NH///////+/vro8NiugnJh+eXKnpJ+Lh4GalpGkoJyCfXenpKB+enOYlZA8NivU1NH///////+/vro8NityfXAtm2Ytm2Ytm2Ytm2Ytm2Ytm2Ytm2Yum2Zycmo8NivU1NH///////+/vro8NitxgXIquHcquHcquHcquHcquHcquHcquHcruHdyc2o8NivU1NH///////+/vro8NitxgXIquHcnq28loWgpsnMlomknrHApsnMst3dyc2o8NivU1NH///////+/vro8NitxgXMquHcgj10ZcEkYaUQbdkwZbUYknmYrt3dyc2o8NivU1NH///////+/vro8NitxgXIquHcquHcquHcquHcquHcquHcruHdyc2o8NivU1NH///////+/vro8NitxgXIquHcquHcquHcquHcquHcquHcruHdyc2o8NivU1NH///////+/vro8NiuFioBclXhdlXddlXdclXddlXdclXddlXddlXeAf3g8NivU1NH///////+/vro8NiuZlY9VT0aempZsaGCHgnyYlI5cVk2empZVT0eVkY08NivU1NH///////+/vro8NitUT0Y8NitVT0Y+OC1TTkRUT0U8NitVUEY8NitVT0Y8NivV1NH////////d3No9Nyw8Nis8Nis8Nis8Nis8Nis8Nis8Nis8Nis8NitBOzHu7ez///////////+9u7eKh4CKhoCKhoCKhoCKhoCKhoCKhoCKhoCKhoCLh4HJx8T///////8AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"


# ---------------------------------------------------------------- Pico 自定义 HID 通信层
class PicoHIDEEPROM:
    """
    Pico 作 USB<->I2C 桥, 通过自定义 HID 读写 24Cxx。
    对外接口与旧 Ch341I2C 完全一致(scan / read_eeprom / write_eeprom / close / connected),
    UI 层不用改。

    线程模型: self.lock(RLock) 只保护「一次 HID 事务」或「一次连接状态变更」,
    整次读/写不长期持锁, 所以 800ms 的设备轮询不会把界面卡住。
    MainApp 用 _busy 保证忙碌期间不调 scan(), 避免 PING 与数据帧插队。
    """

    WRITE_MODES = (('id0', True), ('raw', False))   # 首字节补 0x00 / 直接发 64B

    def __init__(self, progress=None):
        self.lock = threading.RLock()
        self.h = None                # hidapi 句柄
        self.mode = None             # 探测成功的写入格式
        self.info = None             # hid.enumerate() 返回的设备字典
        self.seq = 0
        self.fw_info = ''
        self.eeprom_addr = 0
        self.last_error = ''
        self.progress = progress
        self._activating = False
        self._last_activate = -1e9
        self._last_progress = None
        self._cdc_seen = False       # 上轮是否看得到 CDC, 用于识别「重新插入」事件
        self._io_pause = False       # 长任务期间 True: scan() 不碰 HID, 防串帧
        self._probe_fails = 0        # HID 在但 PING 连续失败次数
        self._revive_at = 0.0        # 上次补发主循环的时间
        self._reviving = False

    # ---------------- 状态
    @property
    def hid_ok(self):
        return HAS_HID

    @property
    def connected(self):
        return self.h is not None and self.mode is not None

    def _notify(self, done, total, phase):
        """按百分比节流: 24C256 有 2000+ 帧, 不能每帧都往 Tk 扔一个 after。"""
        if not self.progress:
            return
        key = (phase, done * 100 // max(1, total))
        if key == self._last_progress:
            return
        self._last_progress = key
        try:
            self.progress(done, total, phase)
        except Exception:
            pass

    # ---------------- 帧格式(与固件 hidproto.py 逐字节一致)
    @staticmethod
    def crc8(data):
        crc = 0
        for b in data:
            crc ^= b
            for _ in range(8):
                crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
        return crc

    @classmethod
    def pack(cls, cmd, seq, payload=b''):
        payload = bytes(payload)
        n = len(payload)
        if n > MAX_PAYLOAD:
            raise ValueError('payload 超长: %d > %d' % (n, MAX_PAYLOAD))
        f = bytearray(REPORT_LEN)
        f[0] = FRAME_MAGIC
        f[1] = cmd & 0xFF
        f[2] = seq & 0xFF
        f[3] = n
        f[4:4 + n] = payload
        f[4 + n] = cls.crc8(f[:4 + n])
        return bytes(f)

    @classmethod
    def unpack(cls, frame):
        if len(frame) != REPORT_LEN or frame[0] != FRAME_MAGIC:
            raise ValueError('帧头错')
        n = frame[3]
        if n > MAX_PAYLOAD or cls.crc8(frame[:4 + n]) != frame[4 + n]:
            raise ValueError('长度/CRC 错')
        return frame[1], frame[2], bytes(frame[4:4 + n])

    # ---------------- 设备发现
    @staticmethod
    def _find_device():
        """严格匹配 VID/PID + Usage。Apple 键盘也报 0xFF00, 必须同时限定 usage=0x01。"""
        try:
            items = hid.enumerate(HID_VID, HID_PID)
        except TypeError:
            items = hid.enumerate()
        hit = [d for d in items
               if d.get('usage_page') == HID_USAGE_PAGE and d.get('usage') == HID_USAGE]
        if hit:
            return hit[0]
        # 兜底: 有些平台复合设备的 VID/PID 报成 0000/0000
        try:
            all_items = hid.enumerate()
        except Exception:
            return None
        for d in all_items:
            if (d.get('usage_page') == HID_USAGE_PAGE and d.get('usage') == HID_USAGE
                    and d.get('vendor_id') != 0x05AC):
                return d
        return None

    # ---------------- 底层句柄
    def _close(self):
        if self.h is not None:
            try:
                self.h.close()
            except Exception:
                pass
        self.h = None
        self.mode = None

    def _open(self):
        path = self.info['path']
        self._close()
        h = hid.device()
        h.open_path(path)
        h.set_nonblocking(0)
        self.h = h

    def _write_report(self, frame):
        buf = (b'\x00' + frame) if self.mode == 'id0' else bytes(frame)
        n = self.h.write(buf)
        if n <= 0:
            raise RuntimeError('hid_write 失败 (%s)' % self.mode)

    def _read_frame(self, timeout_ms):
        try:
            raw = self.h.read(128, timeout_ms)
        except TypeError:            # 老版本 hidapi 不支持 timeout_ms
            raw = self.h.read(128)
        if not raw:
            return None
        b = bytes(raw)
        # 有的平台返回 [0x00]+64B, 有的直接 64B; 统一按帧头 A5 定位
        pos = b.find(FRAME_MAGIC)
        if pos < 0:
            return None
        if pos:
            b = b[pos:]
        if len(b) < REPORT_LEN:
            b += b'\x00' * (REPORT_LEN - len(b))
        b = b[:REPORT_LEN]
        try:
            return self.unpack(b)
        except ValueError:
            return None

    def _transact(self, cmd, payload=b'', timeout=1.5):
        """发一帧并等匹配响应。RLock 可重入, scan() 持锁时也能调。"""
        with self.lock:
            if self.h is None:
                raise RuntimeError('设备未连接')
            self.seq = (self.seq + 1) & 0xFF
            seq = self.seq
            self._write_report(self.pack(cmd, seq, payload))
            want = (cmd | 0x80) & 0xFF
            deadline = time.monotonic() + timeout
            while True:
                remain = deadline - time.monotonic()
                if remain <= 0:
                    raise TimeoutError('CMD %02X 响应超时' % cmd)
                f = self._read_frame(max(1, int(remain * 1000)))
                if f is None:
                    continue
                c, s, d = f
                if c in (ERR_BAD_CMD, ERR_BAD_FRAME, ERR_I2C, ERR_FLASH):
                    raise RuntimeError('设备返回错误 0x%02X: %r' % (c, bytes(d[:24])))
                if c == want and s == seq:
                    return d
                # 残留帧 / 乱序帧丢弃

    def _probe_mode(self):
        """握手时试探 hidapi 写入格式, 成功那次的 PING 响应直接当固件标识用。"""
        errors = []
        for name, _ in self.WRITE_MODES:
            try:
                self._open()
                self.mode = name
                return self._transact(CMD_PING, timeout=0.6)
            except Exception as e:
                errors.append('%s: %s' % (name, e))
                self._close()
                if 'hid_write' in str(e):
                    break        # OUT 端点没人收, 换写入格式也是白等 5 秒
        raise RuntimeError('HID 输出报告写入失败 (已试 id0/raw): ' + '; '.join(errors))

    # ---------------- CDC 自动激活(后台线程, 不阻塞界面)
    @staticmethod
    def _cdc_ports():
        try:
            out = []
            for p in list_ports.comports():
                # macOS 上 pyserial 有时拿不到 vid, 用 /dev/cu.usbmodem* 兜底
                if p.vid == HID_VID or (p.vid is None
                                        and 'usbmodem' in (p.device or '').lower()):
                    out.append(p.device)
            return out
        except Exception:
            return []

    def _repl_send(self, port, code):
        """进 raw REPL 下发一条语句, 不等返回就断开(start() 会让 USB 重新枚举)。"""
        s = serial.Serial(port, 115200, timeout=0.1)
        try:
            s.write(b'\r\x03\x03')                    # 打断可能正在跑的 main 循环
            time.sleep(0.15)
            try:
                s.reset_input_buffer()
            except Exception:
                pass
            s.write(b'\r\x01')                        # Ctrl-A 进 raw REPL
            time.sleep(0.15)
            s.write(code.encode('ascii') + b'\x04')    # Ctrl-D 执行
            time.sleep(0.3)
        finally:
            try:
                s.close()
            except Exception:
                pass

    def _retry_soon(self, secs=3.0):
        """失败后短冷却: 让下一轮 scan() 几秒后就重试, 不白等整个冷却周期。"""
        self._last_activate = time.monotonic() - ACTIVATE_COOLDOWN + secs

    def request_activation(self):
        """非阻塞。找不到 HID 时由 scan() 调用, 冷却期内最多触发一次。"""
        if not (AUTO_ACTIVATE and HAS_SERIAL and HAS_HID):
            return
        # 拔插检测: CDC 从「无」变「有」= 刚插上, 立刻允许激活, 跳过冷却
        has_cdc = bool(self._cdc_ports())
        if has_cdc and not self._cdc_seen:
            self._last_activate = -1e9
        self._cdc_seen = has_cdc
        if self._activating or time.monotonic() - self._last_activate < ACTIVATE_COOLDOWN:
            return
        self._last_activate = time.monotonic()
        self._activating = True
        threading.Thread(target=self._activate_worker, daemon=True).start()

    def _activate_worker(self):
        try:
            # 拔插瞬间 CDC 可能还没枚举完, 轮询等最多 8 秒; 期间 HID 若已出现就收工
            t0 = time.monotonic()
            ports = []
            while time.monotonic() - t0 < 8:
                if self._find_device() is not None:
                    self.last_error = ''
                    return
                ports = self._cdc_ports()
                if ports:
                    break
                self.last_error = '等待 Pico CDC 枚举...'
                time.sleep(0.4)
            if not ports:
                self.last_error = '未找到 Pico (HID/CDC 均未枚举), 请插入或重插 USB'
                self._retry_soon()
                return
            self.last_error = '正在经 CDC 自动激活 HID...'
            try:
                self._repl_send(ports[0], 'import hid_usb; hid_usb.start()')
            except Exception as e:
                self.last_error = '下发 start() 时串口重枚举: %s' % e
            t1 = time.monotonic()
            while time.monotonic() - t1 < 12:
                if self._find_device() is not None:
                    break
                time.sleep(0.3)
            else:
                self.last_error = 'HID 未枚举: 确认 boot.py 纯 CDC、hid_usb.py 已上传'
                self._retry_soon()
                return
            cur = self._cdc_ports() or ports
            main_err = ''
            try:
                self._repl_send(cur[0], 'import main; main.main()')
            except Exception as e:
                main_err = '下发 main.main() 异常: %s' % e
            time.sleep(0.8)
            if main_err:
                # 不要无条件覆盖: 主循环没起来时 hid_write 会一直失败, 必须暴露出来
                self.last_error = main_err
                self._retry_soon()
            else:
                self.last_error = 'HID 已激活, 等待连接'
        finally:
            self._activating = False

    def _maybe_revive_main(self):
        # HID 接口在但 PING 不通 = 板上主循环没跑。只补发 main.main(),
        # 绝不重复 hid_usb.start(): 那会把内置 CDC 一起摘掉, USB 全挂。
        if not (AUTO_ACTIVATE and HAS_SERIAL):
            return
        if self._find_device() is None:
            return
        self._probe_fails += 1
        now = time.monotonic()
        if self._probe_fails < 2 or self._reviving or now - self._revive_at < 10:
            return
        ports = self._cdc_ports()
        if not ports:
            return
        self._probe_fails = 0
        self._revive_at = now
        self._reviving = True

        def work():
            try:
                self.last_error = 'HID 接口已在但无响应, 正在补发主循环...'
                self._repl_send(ports[0], 'import main; main.main()')
                time.sleep(0.8)
                self.last_error = '主循环已补发, 等待连接'
            except Exception as e:
                self.last_error = '补发主循环失败: %s' % e
            finally:
                self._reviving = False
        threading.Thread(target=work, daemon=True).start()

    # ---------------- 对外接口(与旧 Ch341I2C 同名同签名)
    def scan(self):
        """800ms 调一次。长任务期间直接返回缓存状态, 不碰 HID。"""
        if self._io_pause:
            return self.connected
        with self.lock:
            if self._io_pause:
                return self.connected
            if not HAS_HID:
                return False
            if self.connected:
                try:
                    r = self._transact(CMD_PING, timeout=0.4)
                    if FW_TAG in r:
                        return True
                    self.last_error = 'Pico 上跑的不是 EEPROM 固件'
                except Exception as e:
                    self.last_error = '连接已断开: %s' % e
                self._close()

            info = self._find_device()
            if info is None:
                self.info = None
                self.request_activation()
                return False
            try:
                self.info = info
                r = self._probe_mode()
                if FW_TAG not in r:
                    raise RuntimeError('固件标识不匹配: %r (请上传 pico_eeprom_hid 固件)'
                                       % bytes(r[:24]))
                self.fw_info = bytes(r).decode('utf-8', 'replace').rstrip('\x00')
                try:
                    p = self._transact(CMD_EEPROM_PROBE, timeout=1.0)
                    self.eeprom_addr = p[1] if len(p) >= 2 and p[0] == 0 else 0
                except Exception:
                    self.eeprom_addr = 0
                self.last_error = ''
                self._probe_fails = 0
                return True
            except Exception as e:
                self.last_error = str(e)
                self._close()
                self._maybe_revive_main()
                return False

    def read_eeprom(self, chip_id, addr, length):
        if not self.connected:
            raise RuntimeError('设备未连接')
        self._last_progress = None
        out = bytearray()
        done = 0
        while done < length:
            a = addr + done
            # 不跨 256B 边界: 24C04/08/16 的高位地址编码在 I2C 从机地址里
            n = min(READ_CHUNK, length - done, 256 - (a % 256))
            p = bytes((chip_id & 0xFF, (a >> 8) & 0xFF, a & 0xFF, n))
            r = self._transact(CMD_EEPROM_READ, p, timeout=2.0)
            if not r or r[0] != 0 or len(r) != 1 + n:
                raise RuntimeError('读取 0x%04X 失败 (status=%r, len=%d)'
                                   % (a, bytes(r[:1]), len(r)))
            out += r[1:]
            done += n
            self._notify(done, length, '读取')
        return bytes(out)

    def write_eeprom(self, chip_id, addr, data):
        if not self.connected:
            raise RuntimeError('设备未连接')
        self._last_progress = None
        data = bytes(data)
        total = len(data)
        off = 0
        while off < total:
            a = addr + off
            n = min(WRITE_CHUNK, total - off)
            p = bytes((chip_id & 0xFF, (a >> 8) & 0xFF, a & 0xFF, n)) + data[off:off + n]
            r = self._transact(CMD_EEPROM_WRITE, p, timeout=2.0)
            if not r or r[0] != 0:
                raise RuntimeError('写入 0x%04X 失败 (status=%r)' % (a, bytes(r[:2])))
            off += n
            self._notify(off, total, '烧录')

    # ---------------- 板载 Flash 镜像 / 离线烧录
    def flash_info(self):
        """返回 dict(present, chip_id, size, crc, flags)。"""
        r = self._transact(CMD_FLASH_INFO, timeout=1.5)
        if len(r) < 9:
            raise RuntimeError('FLASH_INFO 响应异常 (Pico 固件版本过旧?)')
        return dict(
            present=bool(r[0]),
            chip_id=r[1],
            size=r[2] | (r[3] << 8),
            flags=r[4],
            crc=int.from_bytes(bytes(r[5:9]), 'little'),
        )

    def flash_write(self, chip_id, data):
        """三段式: BEGIN -> DATA*n -> COMMIT(固件核对长度+CRC 后才转正)。"""
        import binascii
        if not self.connected:
            raise RuntimeError('设备未连接')
        data = bytes(data)
        if not data or len(data) > 32768:
            raise ValueError('镜像长度必须在 1..32768 之间')
        crc = binascii.crc32(data) & 0xFFFFFFFF
        p = (bytes((chip_id & 0xFF, len(data) & 0xFF, len(data) >> 8))
             + crc.to_bytes(4, 'little'))
        r = self._transact(CMD_FLASH_BEGIN, p, timeout=2.0)
        if not r or r[0] != 0:
            raise RuntimeError('设备拒绝开始写入镜像')
        self._last_progress = None
        off = 0
        while off < len(data):
            n = min(FLASH_CHUNK, len(data) - off)
            p = bytes((off & 0xFF, off >> 8, n)) + data[off:off + n]
            r = self._transact(CMD_FLASH_DATA, p, timeout=2.0)
            if not r or r[0] != 0:
                raise RuntimeError('写入设备 Flash 偏移 0x%04X 失败' % off)
            off += n
            self._notify(off, len(data), '存入设备')
        r = self._transact(CMD_FLASH_COMMIT, timeout=8.0)
        if not r or r[0] != 0:
            raise RuntimeError('镜像校验失败: %s'
                               % bytes(r[1:]).decode('ascii', 'replace').strip())

    def flash_read(self):
        """返回 (info, data), 用于把设备里的镜像读回编辑区。"""
        import binascii
        if not self.connected:
            raise RuntimeError('设备未连接')
        info = self.flash_info()
        if not info['present']:
            raise RuntimeError('设备 Flash 里没有镜像 (请先用「存入设备」保存)')
        size = info['size']
        self._last_progress = None
        out = bytearray()
        off = 0
        while off < size:
            n = min(FLASH_CHUNK, size - off)
            p = bytes((off & 0xFF, off >> 8, n))
            r = self._transact(CMD_FLASH_READ, p, timeout=2.0)
            if not r or r[0] != 0 or len(r) < 1 + n:
                raise RuntimeError('读取镜像 0x%04X 失败 (status=%r, len=%d)'
                                   % (off, bytes(r[:1]), len(r)))
            out += r[1:1 + n]
            off += n
            self._notify(off, size, '读取镜像')
        data = bytes(out)
        if (binascii.crc32(data) & 0xFFFFFFFF) != info['crc']:
            raise RuntimeError('镜像 CRC 校验失败, 读到的数据不完整')
        return info, data

    def flash_erase(self):
        r = self._transact(CMD_FLASH_ERASE, timeout=2.0)
        if not r or r[0] != 0:
            raise RuntimeError('清除镜像失败')

    def flash_burn(self):
        """在线触发一次离线烧录(等价于断电重启那一次), 24C256 可能耗时数秒。"""
        r = self._transact(CMD_FLASH_BURN, timeout=90.0)
        if not r:
            raise RuntimeError('烧录无响应')
        msg = bytes(r[1:]).decode('ascii', 'replace').strip('\x00 ')
        if r[0] != 0:
            raise RuntimeError('离线烧录失败: %s' % (msg or '未知错误'))
        return msg or '完成'

    def flash_stat(self):
        r = self._transact(CMD_FLASH_STAT, timeout=1.5)
        return bytes(r).decode('ascii', 'replace').strip('\x00 ')

    def led_get(self):
        """读取物理提示灯当前 GPIO(固件 v1.2+)。"""
        r = self._transact(CMD_LED_GET, timeout=1.5)
        if not r:
            raise RuntimeError('读取提示灯 IO 无响应 (需要 Pico 固件 v1.2+)')
        return r[0]

    def led_set(self, pin):
        """设置提示灯 GPIO, 写入板载 Flash 掉电保留; 板子会闪两下确认。"""
        if not isinstance(pin, int) or not 0 <= pin <= LED_PIN_MAX:
            raise ValueError('GPIO 必须是 0~%d 的整数' % LED_PIN_MAX)
        r = self._transact(CMD_LED_SET, bytes((pin,)), timeout=2.5)
        if len(r) < 2:
            raise RuntimeError('设置提示灯 IO 无响应 (需要 Pico 固件 v1.2+)')
        if r[0] != 0:
            raise RuntimeError('设备拒绝了该 GPIO')
        return r[1]

    def close(self):
        with self.lock:
            self._close()


# ---------------------------------------------------------------- 十六进制编辑器(单Text实现)
class HexEditor(tk.Frame):
    """
    单 Text 实现的十六进制编辑区:
      0000  FF FF FF ...  | ASCII
    data 是唯一数据源(bytearray), Text 只是视图。
    """

    ADDR_LEN = 6                    # "0000  "
    COL_STRIDE = 3                  # 每字节占3列: "XX "
    ASCII_START = ADDR_LEN + BYTES_PER_ROW * COL_STRIDE + 1  # ASCII列起始列
    MAX_VISIBLE_ROWS = 18           # 超出则滚动(大容量芯片)
    HIGHLIGHT_TAGS = ('rowhl', 'cur', 'addr_cur', 'hdr_cur', 'asc_cur')

    def __init__(self, master, size=256, on_change=None, on_cursor=None):
        super().__init__(master)
        self.size = size
        self.rows = size // BYTES_PER_ROW
        self.data = bytearray([0xFF] * size)
        self.on_change = on_change
        self.on_cursor = on_cursor

        self.text = tk.Text(
            self, font=FONT_MONO, wrap='none',
            width=self.ASCII_START + BYTES_PER_ROW + 1,
            height=min(self.rows, self.MAX_VISIBLE_ROWS) + 1,
            bg='white', relief='flat', bd=0,
            padx=10, pady=8, spacing1=2, spacing3=2,
            insertwidth=2, highlightthickness=1,
            highlightbackground='#D0D0D0', highlightcolor='#4A90D9',
        )
        vsb = ttk.Scrollbar(self, orient='vertical', command=self.text.yview)
        self.text.configure(yscrollcommand=vsb.set)
        self.text.pack(side='left', fill='both', expand=True)
        vsb.pack(side='right', fill='y')

        # 基础着色
        self.text.tag_configure('hdr', foreground='#020202')
        self.text.tag_configure('addr', foreground=COLOR_ADDR)
        self.text.tag_configure('asc', foreground=COLOR_ASCII)
        self.text.tag_configure('ff', foreground=COLOR_FF)
        self.text.tag_configure('nz', foreground=COLOR_DATA)
        self.text.tag_configure('mod', foreground=COLOR_MODIFIED)

        # 十字定位高亮: 只改配色, 绝不给 Text tag 设 font。
        # tag 一旦换字体, 该段字符宽度就变, 同一行后面的字节/ASCII 会整体右移,
        # 列头也跟着错位; 选中位置一变, 错位位置也变 —— 这是"不对齐"的根因。
        self.text.tag_configure('rowhl', background=COLOR_ROW_BG)
        self.text.tag_configure('addr_cur', background=COLOR_CURSOR_BG,
                                foreground=COLOR_CURSOR_FG)
        self.text.tag_configure('hdr_cur', background=COLOR_COL_BG,
                                foreground=COLOR_COL_FG)
        self.text.tag_configure('asc_cur', background=COLOR_COL_BG,
                                foreground=COLOR_COL_FG)
        self.text.tag_configure('cur', background=COLOR_CURSOR_BG,
                                foreground=COLOR_CURSOR_FG)
        # 明确优先级: 行底色最下, 其余高亮依次盖过基础着色
        self.text.tag_lower('rowhl')
        for t in ('hdr_cur', 'asc_cur', 'addr_cur', 'cur'):
            self.text.tag_raise(t)

        self.text.bind('<Key>', self._on_key)
        self.text.bind('<ButtonRelease-1>', self._on_click)
        self.text.bind('<KeyRelease>', lambda e: self._cursor_changed())

        self.render()

    # ---------------- 渲染
    def render(self):
        """从 data 全量重建视图(一次性insert, 很快)。第1行为 00~0F 列头。"""
        header = (' ' * self.ADDR_LEN
                  + ' '.join(f'{i:02X}' for i in range(BYTES_PER_ROW))
                  + '  ASCII')
        lines = [header]
        for row in range(self.rows):
            base = row * BYTES_PER_ROW
            chunk = self.data[base:base + BYTES_PER_ROW]
            hexpart = ' '.join(f'{b:02X}' for b in chunk)
            asciipart = ''.join(chr(b) if 32 <= b < 127 else '.' for b in chunk)
            lines.append(f'{base:04X}  {hexpart}  {asciipart}')
        self.text.configure(state='normal',
                            height=min(self.rows, self.MAX_VISIBLE_ROWS) + 1)
        self.text.delete('1.0', 'end')
        self.text.insert('1.0', '\n'.join(lines))
        # 列头着色
        self.text.tag_add('hdr', '1.0', '1.end')
        # 地址列 / ASCII列着色
        for row in range(self.rows):
            ln = row + 2
            self.text.tag_add('addr', f'{ln}.0', f'{ln}.4')
            self.text.tag_add('asc', f'{ln}.{self.ASCII_START}',
                              f'{ln}.{self.ASCII_START + BYTES_PER_ROW}')
        # 数据着色: FF灰 / 非FF青
        for idx, b in enumerate(self.data):
            self._tag_byte(idx, 'ff' if b == 0xFF else 'nz')
        self.text.mark_set('insert', '2.6')
        self._cursor_changed()

    def resize(self, size):
        """切换芯片容量: 保留重叠部分数据, 超出截断、不足补FF。"""
        if size == self.size:
            return
        new = bytearray([0xFF] * size)
        n = min(size, self.size)
        new[:n] = self.data[:n]
        self.size = size
        self.rows = size // BYTES_PER_ROW
        self.data = new
        self.render()

    def _byte_range(self, idx):
        row, col_in = divmod(idx, BYTES_PER_ROW)
        col = self.ADDR_LEN + col_in * self.COL_STRIDE
        return f'{row + 2}.{col}', f'{row + 2}.{col + 2}'

    def _tag_byte(self, idx, tag):
        p1, p2 = self._byte_range(idx)
        for t in ('ff', 'nz', 'mod'):
            self.text.tag_remove(t, p1, p2)
        self.text.tag_add(tag, p1, p2)

    def _ascii_pos(self, idx):
        row, col_in = divmod(idx, BYTES_PER_ROW)
        return f'{row + 2}.{self.ASCII_START + col_in}'

    # ---------------- 光标定位
    def _cursor_byte(self):
        """返回 (字节索引, 半字节0/1), 光标不在数据区时返回 (None, None)。"""
        line, col = map(int, self.text.index('insert').split('.'))
        row = line - 2          # 第1行是 00~0F 列头
        rel = col - self.ADDR_LEN
        if not (0 <= row < self.rows and 0 <= rel < BYTES_PER_ROW * self.COL_STRIDE - 1):
            return None, None
        byte_in_row, nib = divmod(rel, self.COL_STRIDE)
        if nib > 1:
            return None, None
        idx = row * BYTES_PER_ROW + byte_in_row
        if idx >= self.size:
            return None, None
        return idx, nib

    def _set_cursor(self, idx, nib=0):
        row, col_in = divmod(idx, BYTES_PER_ROW)
        col = self.ADDR_LEN + col_in * self.COL_STRIDE + nib
        self.text.mark_set('insert', f'{row + 2}.{col}')
        self.text.see('insert')
        self._cursor_changed()

    def _snap_insert(self):
        """光标落在地址列/字节间隙/ASCII 区时, 吸附到最近的半字节。
        返回吸附后的字节索引; 光标不在数据行时返回 None。"""
        line, col = map(int, self.text.index('insert').split('.'))
        row = line - 2
        if not (0 <= row < self.rows):
            return None
        rel = col - self.ADDR_LEN
        if rel < 0:
            byte_in_row, nib = 0, 0
        elif rel < BYTES_PER_ROW * self.COL_STRIDE - 1:
            byte_in_row, nib = divmod(rel, self.COL_STRIDE)
            if nib > 1:                       # 落在 "XX " 的空格上
                if byte_in_row < BYTES_PER_ROW - 1:
                    byte_in_row, nib = byte_in_row + 1, 0
                else:
                    nib = 1
        else:
            acol = col - self.ASCII_START
            if 0 <= acol < BYTES_PER_ROW:
                byte_in_row, nib = acol, 0
            else:
                byte_in_row, nib = BYTES_PER_ROW - 1, 1
        idx = min(row * BYTES_PER_ROW + byte_in_row, self.size - 1)
        r, c = divmod(idx, BYTES_PER_ROW)
        self.text.mark_set('insert',
                           f'{r + 2}.{self.ADDR_LEN + c * self.COL_STRIDE + nib}')
        return idx

    def _cursor_detail(self, idx):
        """选中字节的完整读数, 由编辑器自己算出, 构造期回调也拿得到。"""
        if idx is None:
            return None
        row, col = divmod(idx, BYTES_PER_ROW)
        return (idx, self.size, row, col, self.data[idx])

    def _cursor_changed(self):
        txt = self.text
        # 先清空全部高亮: 顺带修掉 text.delete/insert 继承旧 tag 的残留
        for t in self.HIGHLIGHT_TAGS:
            txt.tag_remove(t, '1.0', 'end')
        idx, _ = self._cursor_byte()
        if idx is None:
            idx = self._snap_insert()        # 键盘导航后地址不再丢成 "--"
        if idx is not None:
            row, col_in = divmod(idx, BYTES_PER_ROW)
            ln = row + 2
            txt.tag_add('rowhl', f'{ln}.0', f'{ln}.end')          # 整行
            txt.tag_add('addr_cur', f'{ln}.0', f'{ln}.4')         # 行首地址
            hc = self.ADDR_LEN + col_in * self.COL_STRIDE
            txt.tag_add('hdr_cur', f'1.{hc}', f'1.{hc + 2}')      # 列头
            p1, p2 = self._byte_range(idx)
            txt.tag_add('cur', p1, p2)                             # 当前字节
            ap = self._ascii_pos(idx)
            txt.tag_add('asc_cur', ap, f'{ap}+1c')                 # ASCII 同位
        if self.on_cursor:
            self.on_cursor(idx, self._cursor_detail(idx))

    def _on_click(self, event):
        """点击落在非数据区时, 把光标吸附到最近的数据位置。"""
        line, col = map(int, self.text.index('insert').split('.'))
        row = line - 2
        if not (0 <= row < self.rows):
            self._cursor_changed()
            return
        rel = col - self.ADDR_LEN
        if rel < 0:
            self._set_cursor(row * BYTES_PER_ROW, 0)
            return
        if rel >= BYTES_PER_ROW * self.COL_STRIDE - 1:
            # ASCII区点击 -> 定位到对应字节
            acol = col - self.ASCII_START
            if 0 <= acol < BYTES_PER_ROW:
                self._set_cursor(row * BYTES_PER_ROW + acol, 0)
            else:
                self._set_cursor(row * BYTES_PER_ROW + BYTES_PER_ROW - 1, 1)
            return
        _, nib = divmod(rel, self.COL_STRIDE)
        if nib > 1:  # 点在空格上 -> 吸附到下一个字节
            byte_in_row = rel // self.COL_STRIDE
            if byte_in_row < BYTES_PER_ROW - 1:
                self._set_cursor(row * BYTES_PER_ROW + byte_in_row + 1, 0)
            else:
                self._set_cursor(row * BYTES_PER_ROW + byte_in_row, 1)
            return
        self._cursor_changed()

    # ---------------- 编辑
    NAV_KEYS = {'Left', 'Right', 'Up', 'Down', 'Home', 'End', 'Prior', 'Next',
                'Shift_L', 'Shift_R', 'Control_L', 'Control_R', 'Alt_L', 'Alt_R',
                'Caps_Lock', 'Escape'}

    def _on_key(self, event):
        ks = event.keysym
        if ks in self.NAV_KEYS:
            return None
        # 允许 Ctrl+C 复制
        if event.state & 0x4 and ks.lower() == 'c':
            return None
        if ks == 'BackSpace':
            self._backspace()
            return 'break'
        ch = event.char.upper() if event.char else ''
        # 注意: '' in '0123...' 为 True, 必须判空, 否则无字符按键会触发 int('',16)
        if ch and ch in '0123456789ABCDEF':
            self._type_nibble(ch)
        return 'break'

    def _type_nibble(self, ch):
        idx, nib = self._cursor_byte()
        if idx is None:
            return
        old = self.data[idx]
        v = int(ch, 16)
        self.data[idx] = ((v << 4) | (old & 0x0F)) if nib == 0 else ((old & 0xF0) | v)

        # 替换视图中的单个字符
        line, col = map(int, self.text.index('insert').split('.'))
        self.text.delete(f'{line}.{col}', f'{line}.{col + 1}')
        self.text.insert(f'{line}.{col}', ch)

        # 同步 ASCII 列
        b = self.data[idx]
        pos = self._ascii_pos(idx)
        self.text.delete(pos, f'{pos}+1c')
        self.text.insert(pos, chr(b) if 32 <= b < 127 else '.')
        # 保持ASCII着色
        self.text.tag_add('asc', pos, f'{pos}+1c')

        self._tag_byte(idx, 'mod')

        # 光标前进一个半字节
        if nib == 0:
            self._set_cursor(idx, 1)
        elif idx + 1 < self.size:
            self._set_cursor(idx + 1, 0)
        else:
            self._set_cursor(idx, 1)

        if self.on_change:
            self.on_change()

    def _backspace(self):
        idx, nib = self._cursor_byte()
        if idx is None:
            return
        # 退到上一个半字节并置F
        if nib == 1:
            t_idx, t_nib = idx, 0
        elif idx > 0:
            t_idx, t_nib = idx - 1, 1
        else:
            return
        old = self.data[t_idx]
        self.data[t_idx] = (old & 0x0F) | 0xF0 if t_nib == 0 else (old & 0xF0) | 0x0F
        row, col_in = divmod(t_idx, BYTES_PER_ROW)
        col = self.ADDR_LEN + col_in * self.COL_STRIDE + t_nib
        self.text.delete(f'{row + 2}.{col}', f'{row + 2}.{col + 1}')
        self.text.insert(f'{row + 2}.{col}', 'F')
        pos = self._ascii_pos(t_idx)
        b = self.data[t_idx]
        self.text.delete(pos, f'{pos}+1c')
        self.text.insert(pos, chr(b) if 32 <= b < 127 else '.')
        self.text.tag_add('asc', pos, f'{pos}+1c')
        self._tag_byte(t_idx, 'mod')
        self._set_cursor(t_idx, t_nib)
        if self.on_change:
            self.on_change()

    # ---------------- 数据接口
    def load(self, data: bytes):
        buf = bytearray([0xFF] * self.size)
        n = min(len(data), self.size)
        buf[:n] = data[:n]
        self.data = buf
        self.render()

    def fill(self, value: int):
        self.data = bytearray([value] * self.size)
        self.render()

    def checksum(self):
        return sum(self.data) % 0xFFFF


# ---------------------------------------------------------------- 主程序
class MainApp:
    def __init__(self):
        self.i2c = PicoHIDEEPROM(progress=self._on_progress)
        self._busy = False
        self._scan_running = False
        self._ui_queue = queue.Queue()
        self._last_connected = None
        self.chip = DEFAULT_CHIP
        self.chip_id, self.chip_size = EEPROM_CHIPS[self.chip]

        self.root = TkinterDnD.Tk() if HAS_DND else tk.Tk()
        self.root.title('EEPROM Tool - Pico HID')
        self.root.geometry('980x640')
        self.root.minsize(900, 520)
        self.root.configure(bg='#F2F3F5')
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)

        self._set_icon()
        self._build_style()
        self._build_menu()
        self._build_toolbar()
        self._build_statusbar()   # 先建状态栏: 编辑器初始化时会回调光标/校验码
        self._build_editor()

        if HAS_DND:
            self.root.drop_target_register(DND_FILES)
            self.root.dnd_bind('<<Drop>>', self._on_drop)
            self.set_message('就绪 (支持拖入 .bin 文件)')

        # 启动设备轮询
        self.root.after(50, self._drain_ui_queue)
        self.root.after(200, self._poll_device)
        self.root.mainloop()

    # ---------------- 界面构建
    def _set_icon(self):
        try:
            path = os.path.join(tempfile.gettempdir(), 'eeprom_tool.ico')
            with open(path, 'wb') as f:
                f.write(base64.b64decode(icon_data))
            self.root.iconbitmap(path)
        except Exception:
            pass

    def _build_style(self):
        style = ttk.Style(self.root)
        try:
            style.theme_use('clam')
        except tk.TclError:
            pass

        def f(size, bold=False):
            return (FONT_UI[0], size, 'bold') if bold else (FONT_UI[0], size)

        style.configure('TFrame', background='#F2F3F5')
        style.configure('Toolbar.TFrame', background='#FFFFFF')
        style.configure('TLabel', background='#F2F3F5', font=f(10))
        style.configure('Toolbar.TLabel', background='#FFFFFF', font=f(10))
        style.configure('Status.TLabel', background='#E8EAED', font=f(9))

        # 次要按钮: 白底细描边, 等宽对齐, hover 有反馈
        style.configure('Tool.TButton', background='#FFFFFF', foreground='#2B3038',
                        bordercolor='#D6DAE0', lightcolor='#FFFFFF',
                        darkcolor='#ECEFF3', borderwidth=1, relief='solid',
                        font=f(10), padding=(12, 7), width=10)
        style.map('Tool.TButton',
                  background=[('pressed', '#E3EBF7'), ('active', '#F3F7FD'),
                              ('disabled', '#F4F5F7')],
                  foreground=[('disabled', '#AEB4BD')],
                  bordercolor=[('active', '#9CBEEA'), ('disabled', '#E3E6EA')])

        # 主操作(读) / 写入(烧录): 实心色块, 一眼分清主次
        for name, bg, hover, pressed in (
                ('Primary.TButton', '#1565C0', '#1976D2', '#0D47A1'),
                ('Warm.TButton', '#E65100', '#F57C00', '#BF360C')):
            style.configure(name, background=bg, foreground='#FFFFFF',
                            bordercolor=bg, lightcolor=bg, darkcolor=pressed,
                            borderwidth=0, relief='flat', font=f(10, True),
                            padding=(12, 7), width=10)
            style.map(name,
                      background=[('pressed', pressed), ('active', hover),
                                  ('disabled', '#CBD5E1')],
                      foreground=[('disabled', '#F1F5F9')],
                      bordercolor=[('disabled', '#CBD5E1')])

        # 轻量按钮(清空): 白底细描边, 与次要按钮同族但更窄
        style.configure('Ghost.TButton', background='#FFFFFF',
                        foreground='#5A6472', bordercolor='#DDE2E8',
                        lightcolor='#FFFFFF', darkcolor='#ECEFF3',
                        borderwidth=1, relief='solid', font=f(10),
                        padding=(10, 7), width=8)
        style.map('Ghost.TButton',
                  background=[('pressed', '#E3EBF7'), ('active', '#EDF2F9'),
                              ('disabled', '#F4F5F7')],
                  foreground=[('disabled', '#B4BAC3')],
                  bordercolor=[('active', '#9CBEEA'), ('disabled', '#E3E6EA')])

        # 芯片下拉框: 白底描边 + 蓝色箭头, 与按钮等高
        style.configure('Chip.TCombobox', fieldbackground='#FFFFFF',
                        background='#FFFFFF', foreground='#1F2933',
                        bordercolor='#C8D2DE', arrowcolor='#1565C0',
                        lightcolor='#FFFFFF', darkcolor='#FFFFFF',
                        borderwidth=1, relief='solid', padding=(8, 7),
                        font=f(10))
        style.map('Chip.TCombobox',
                  fieldbackground=[('disabled', '#F1F3F6')],
                  foreground=[('disabled', '#AEB4BD')],
                  bordercolor=[('focus', '#1565C0'), ('active', '#9CBEEA'),
                               ('disabled', '#E3E6EA')])

        # 兼容旧代码里可能引用的样式名
        style.configure('Accent.TButton', foreground='#1565C0')
        style.configure('Danger.TButton', foreground='#C62828')

    def _build_menu(self):
        menubar = tk.Menu(self.root)
        menu_file = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label='文件(F)', menu=menu_file)
        menu_file.add_command(label='打开...', command=self.open_file)
        menu_file.add_command(label='保存', command=self.save_file)
        menu_file.add_separator()
        menu_file.add_command(label='清空(0xFF)', command=lambda: self._fill(0xFF))
        menu_file.add_command(label='清空(0x00)', command=lambda: self._fill(0x00))
        menu_file.add_separator()
        menu_file.add_command(label='退出', command=self._on_close)

        # 离线烧录: 把镜像存进 Pico 板载 Flash, 之后脱离电脑也能烧
        menu_off = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label='离线烧录(O)', menu=menu_off)
        menu_off.add_command(label='存入设备 Flash...', command=self.flash_save)
        menu_off.add_command(label='从设备 Flash 读取', command=self.flash_load)
        menu_off.add_command(label='立即烧录(在线触发)', command=self.flash_burn)
        menu_off.add_separator()
        menu_off.add_command(label='查看设备镜像', command=self.flash_show_info)
        menu_off.add_command(label='清除设备镜像', command=self.flash_erase)

        # 设备设置: 物理提示灯接哪个 GPIO
        menu_dev = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label='设备(D)', menu=menu_dev)
        menu_dev.add_command(label='提示灯 GPIO...', command=self.led_config)
        self.root.config(menu=menubar)

    def _build_toolbar(self):
        bar = ttk.Frame(self.root, style='Toolbar.TFrame')
        bar.pack(fill='x', padx=0, pady=0)

        def group(parent, title, side):
            # 浅底分组卡片: 标题 + 按钮行, 靠留白分区而不是竖线挤压
            card = tk.Frame(parent, bg='#F8F9FB', bd=0,
                            highlightbackground='#E4E8EE', highlightthickness=1)
            card.pack(side=side, fill='y',
                      padx=(10, 0) if side == 'right' else (0, 10), pady=0)
            tk.Label(card, text=title, bg='#F8F9FB', fg='#8A9099',
                     font=(FONT_UI[0], 9), anchor='w').pack(
                fill='x', padx=10, pady=(5, 0))
            body = tk.Frame(card, bg='#F8F9FB')
            body.pack(fill='x', padx=8, pady=(3, 8))
            return body

        # 第一行: 文件 / 在线 EEPROM 读写 / 芯片型号
        row1 = ttk.Frame(bar, style='Toolbar.TFrame')
        row1.pack(fill='x', padx=14, pady=(10, 4))

        g = group(row1, '文件', 'left')
        self.btn_open = ttk.Button(g, text='打开文件', style='Tool.TButton',
                                   command=self.open_file)
        self.btn_open.pack(side='left', padx=3)
        self.btn_save = ttk.Button(g, text='保存', style='Tool.TButton',
                                   command=self.save_file)
        self.btn_save.pack(side='left', padx=3)

        g = group(row1, '在线 EEPROM', 'left')
        self.btn_read = ttk.Button(g, text='读取', style='Primary.TButton',
                                   command=self.read_all)
        self.btn_read.pack(side='left', padx=3)
        self.btn_write = ttk.Button(g, text='烧录', style='Warm.TButton',
                                    command=self.write_all)
        self.btn_write.pack(side='left', padx=3)

        g = group(row1, '芯片型号', 'right')
        self.chip_var = tk.StringVar(value=DEFAULT_CHIP)
        self.cbo_chip = ttk.Combobox(g, textvariable=self.chip_var, width=9,
                                     state='readonly', values=list(EEPROM_CHIPS.keys()),
                                     style='Chip.TCombobox')
        self.cbo_chip.pack(side='left', padx=3)
        self.cbo_chip.bind('<<ComboboxSelected>>', self._on_chip_change)
        self.cbo_chip.bind('<Button-1>', self._open_chip_menu)
        cap = self.chip_size
        self.lbl_chip_size = tk.Label(
            g, text=(f'{cap} B' if cap < 1024 else f'{cap // 1024} KB'),
            bg='#F8F9FB', fg='#1565C0', font=(FONT_UI[0], 10, 'bold'),
            width=7, anchor='w')
        self.lbl_chip_size.pack(side='left', padx=(8, 4))

        # 第二行: 离线镜像 / 编辑 / 操作提示
        row2 = ttk.Frame(bar, style='Toolbar.TFrame')
        row2.pack(fill='x', padx=14, pady=(4, 10))

        g = group(row2, '离线镜像 · Pico 板载 Flash', 'left')
        self.btn_flash_save = ttk.Button(g, text='存入设备', style='Tool.TButton',
                                         command=self.flash_save)
        self.btn_flash_save.pack(side='left', padx=3)
        self.btn_flash_load = ttk.Button(g, text='取出设备', style='Tool.TButton',
                                         command=self.flash_load)
        self.btn_flash_load.pack(side='left', padx=3)

        g = group(row2, '编辑', 'left')
        self.btn_fillff = ttk.Button(g, text='清空FF', style='Ghost.TButton',
                                     command=lambda: self._fill(0xFF))
        self.btn_fillff.pack(side='left', padx=3)
        self.btn_fill00 = ttk.Button(g, text='清空00', style='Ghost.TButton',
                                     command=lambda: self._fill(0x00))
        self.btn_fill00.pack(side='left', padx=3)

        tk.Label(row2, text='存入后断电重插即自动烧录',
                 bg='#FFFFFF', fg='#8A9099', font=(FONT_UI[0], 9)).pack(
            side='right', padx=(0, 4))

        # 工具栏与编辑区之间的一条细分隔线
        tk.Frame(bar, bg='#E3E6EA', height=1).pack(side='bottom', fill='x')

    def _open_chip_menu(self, event):
        # readonly 下拉框默认只有点右侧箭头才弹。这里做渐进增强:
        # 点左侧文本区也弹, 但箭头区一律交回 ttk 默认处理 —— 即使某个
        # Tk 版本没有 ttk::combobox::Post, 原生点击仍然 100% 可用。
        cb = self.cbo_chip
        if 'disabled' in cb.state():
            return 'break'
        x = getattr(event, 'x', 0)
        if x >= cb.winfo_width() - 22:      # 右侧箭头留给默认行为
            return None
        try:
            cb.tk.call('ttk::combobox::Post', str(cb))
        except tk.TclError:
            return None
        return 'break'

    def _build_editor(self):
        wrap = ttk.Frame(self.root)
        wrap.pack(fill='both', expand=True, padx=10, pady=(8, 4))
        self.editor = HexEditor(wrap, size=self.chip_size,
                                on_change=self._on_data_change,
                                on_cursor=self._on_cursor_move)
        self.editor.pack(fill='both', expand=True)

    def _build_statusbar(self):
        bar = tk.Frame(self.root, bg='#E8EAED', height=34)
        bar.pack(fill='x', side='bottom')
        bar.pack_propagate(False)

        self.lbl_msg = tk.Label(bar, text='就绪', bg='#E8EAED', fg='#333333',
                                font=(FONT_UI[0], 11, 'bold'), anchor='w')
        self.lbl_msg.pack(side='left', padx=12)

        # 设备提示灯: LED(光晕+灯芯+高光) + 状态文字 + 离线镜像副灯
        lamp_box = tk.Frame(bar, bg='#E8EAED')
        lamp_box.pack(side='right', padx=12)

        self.lamp = tk.Canvas(lamp_box, width=20, height=20, bg='#E8EAED',
                              highlightthickness=0)
        self.lamp.pack(side='left', padx=(0, 7))
        self.lamp_glow = self.lamp.create_oval(1, 1, 19, 19, fill='#DDE1E6', outline='')
        self.lamp_core = self.lamp.create_oval(3, 3, 17, 17, fill='#9AA0A6', outline='')
        self.lamp_spec = self.lamp.create_oval(5, 5, 10, 10, fill='#FFFFFF', outline='')

        # 定宽: 设备状态文字长短变化时不会把左侧的地址徽标推来推去
        self.lbl_device = tk.Label(lamp_box, text='检测中...', bg='#E8EAED',
                                   fg='#555555', font=(FONT_UI[0], 10, 'bold'),
                                   width=14, anchor='w')
        self.lbl_device.pack(side='left')

        self.lamp_img = tk.Canvas(lamp_box, width=12, height=12, bg='#E8EAED',
                                  highlightthickness=0)
        self.lamp_img.pack(side='left', padx=(10, 3))
        self.img_dot = self.lamp_img.create_oval(2, 2, 10, 10,
                                                 fill='#C4C9CF', outline='')
        tk.Label(lamp_box, text='镜像', bg='#E8EAED', fg='#8A9099',
                 font=(FONT_UI[0], 9)).pack(side='left')

        self._lamp_state = 'idle'
        self._lamp_after = None
        self._lamp_on = True
        self._tip_win = None
        for w in (self.lamp, self.lbl_device, self.lamp_img):
            w.bind('<Enter>', self._show_lamp_tip)
            w.bind('<Leave>', self._hide_lamp_tip)

        # 当前地址徽标: 深蓝底白字。整块固定像素宽度 + 读数定宽,
        # 切换地址/数值/容量时主地址不会左右漂移。
        addr_chip = tk.Frame(bar, bg=COLOR_ADDR_CHIP, width=312, height=24,
                             highlightbackground='#0D47A1', highlightthickness=1)
        addr_chip.pack(side='right', padx=(4, 12), pady=5)
        addr_chip.pack_propagate(False)
        tk.Label(addr_chip, text='地址', bg=COLOR_ADDR_CHIP, fg='#BBDEFB',
                 font=(FONT_UI[0], 9, 'bold')).pack(side='left', padx=(9, 5))
        self.lbl_addr = tk.Label(addr_chip, text='--', bg=COLOR_ADDR_CHIP,
                                 fg='#FFFFFF', font=(FONT_MONO[0], 13, 'bold'),
                                 width=7, anchor='w')
        self.lbl_addr.pack(side='left', padx=(0, 4))
        self.lbl_addr_sub = tk.Label(addr_chip, text='', bg=COLOR_ADDR_CHIP,
                                     fg='#BBDEFB', font=(FONT_MONO[0], 9),
                                     anchor='w')
        self.lbl_addr_sub.pack(side='left', fill='x', expand=True, padx=(0, 9))

        self.lbl_sum = tk.Label(bar, text='校验码: FF00', bg='#E8EAED', fg='#D84315',
                                font=(FONT_MONO[0], 13, 'bold'))
        self.lbl_sum.pack(side='right', padx=12)

    # ---------------- 状态提示灯
    def _set_lamp(self, state):
        # 所有调用都在主线程; headless 测试没建状态栏时静默跳过
        lamp = getattr(self, 'lamp', None)
        if lamp is None:
            return
        if getattr(self, '_lamp_after', None):
            try:
                self.root.after_cancel(self._lamp_after)
            except Exception:
                pass
            self._lamp_after = None
        self._lamp_state = state
        core, glow, spec = LAMP_COLORS[state]
        lamp.itemconfig(self.lamp_glow, fill=glow)
        lamp.itemconfig(self.lamp_core, fill=core)
        lamp.itemconfig(self.lamp_spec, fill=spec)
        self._lamp_on = True
        if state == 'busy':
            self._blink_lamp()

    def _blink_lamp(self):
        if getattr(self, '_lamp_state', None) != 'busy':
            return
        # 先按当前状态绘制(进入忙碌时先亮), 再翻转供下一拍
        c, g, sp = (LAMP_COLORS['busy'] if self._lamp_on
                    else ('#7891B4', '#D6E2F0', '#EDF3FA'))
        self.lamp.itemconfig(self.lamp_core, fill=c)
        self.lamp.itemconfig(self.lamp_glow, fill=g)
        self.lamp.itemconfig(self.lamp_spec, fill=sp)
        self._lamp_on = not self._lamp_on
        self._lamp_after = self.root.after(420, self._blink_lamp)

    def _device_label(self):
        # 短而定长, 配合 lbl_device 的 width=14: 完整信息仍显示在左侧消息区
        if not getattr(self, '_last_connected', False):
            return '设备未连接'
        if self.i2c.eeprom_addr:
            return 'Pico HID 0x%02X' % self.i2c.eeprom_addr
        return 'Pico HID 无芯片'

    def _refresh_lamp(self):
        if not getattr(self, '_last_connected', False):
            self._set_lamp('error')
        elif self.i2c.eeprom_addr:
            self._set_lamp('ok')
        else:
            self._set_lamp('warn')

    def _set_img_lamp(self, has):
        canvas = getattr(self, 'lamp_img', None)
        if canvas is None:
            return
        canvas.itemconfig(self.img_dot, fill='#1565C0' if has else '#C4C9CF')
        self._has_image = bool(has)

    def _show_lamp_tip(self, event):
        if getattr(self, '_tip_win', None) is not None:
            return
        try:
            w = tk.Toplevel(self.root)
            w.wm_overrideredirect(True)
            w.wm_geometry('+%d+%d' % (event.x_root + 12, event.y_root + 18))
            tip = LAMP_HELP + ('\n物理提示灯 GPIO: 默认 GP25, 可自定义\n'
                               '(菜单 设备 → 提示灯 GPIO 可查看/修改)')
            tk.Label(w, text=tip, bg='#2B3038', fg='#FFFFFF',
                     font=(FONT_UI[0], 9), padx=9, pady=6,
                     justify='left').pack()
            self._tip_win = w
        except Exception:
            self._tip_win = None

    def _hide_lamp_tip(self, event=None):
        w = getattr(self, '_tip_win', None)
        if w is not None:
            try:
                w.destroy()
            except Exception:
                pass
            self._tip_win = None

    # ---------------- 线程安全的主线程回调队列
    # Tcl/Tk 不是线程安全的: 从工作线程调 root.after() 在 macOS Tk 9.0 上
    # 会返回成功但回调永不执行(界面永远停在"忙碌")。统一改用队列,
    # 工作线程只 put, 主线程定时取出来执行。
    def _post(self, fn, *args):
        self._ui_queue.put((fn, args))

    def _drain_ui_queue(self):
        try:
            while True:
                fn, args = self._ui_queue.get_nowait()
                try:
                    fn(*args)
                except Exception as e:
                    try:
                        self.set_message('界面回调异常: %s' % e, COLOR_ERR)
                    except Exception:
                        pass
        except queue.Empty:
            pass
        self.root.after(50, self._drain_ui_queue)

    # ---------------- 状态反馈
    def set_message(self, text, color='#333333'):
        self.lbl_msg.config(text=text, fg=color)

    def _on_progress(self, done, total, phase):
        # 工作线程只 put 队列, 绝不碰 Tk 对象
        pct = done * 100 // max(1, total)
        text = f'{phase} {done}/{total} 字节 ({pct}%)'
        self._post(self.set_message, text, COLOR_INFO)

    def _on_data_change(self):
        self.lbl_sum.config(text=f'校验码: {self.editor.checksum():04X}')

    def _on_cursor_move(self, idx, info=None):
        lbl = getattr(self, 'lbl_addr', None)
        sub = getattr(self, 'lbl_addr_sub', None)
        if lbl is None:
            return
        if idx is None or info is None:
            lbl.config(text='--')
            if sub is not None:
                sub.config(text='')
            return
        n, size, row, col, val = info
        lbl.config(text=f'0x{n:04X}')
        if sub is not None:
            # 全部定宽: 地址从 0 变到 32767 时副信息长度不变, 主地址不漂移
            sub.config(text=(f'{n:05d}/{size:05d}  ·  行{row:04d} 列{col:02d}'
                             f'  ·  值 {val:02X}'))

    def _update_device_ui(self, connected):
        if connected:
            tag = (f' (EEPROM 0x{self.i2c.eeprom_addr:02X})'
                   if self.i2c.eeprom_addr else ' (未检测到 EEPROM)')
            self._set_lamp('ok' if self.i2c.eeprom_addr else 'warn')
            self.lbl_device.config(text=self._device_label(), foreground=COLOR_OK)
            self.btn_read.state(['!disabled'])
            self.btn_write.state(['!disabled'])
            self.btn_flash_save.state(['!disabled'])
            self.btn_flash_load.state(['!disabled'])
            self.set_message(self.i2c.fw_info or 'Pico HID 已连接', COLOR_OK)
        else:
            self._set_lamp('error')
            self.lbl_device.config(text='设备未连接', foreground=COLOR_ERR)
            self.btn_read.state(['disabled'])
            self.btn_write.state(['disabled'])
            self.btn_flash_save.state(['disabled'])
            self.btn_flash_load.state(['disabled'])
            if not self.i2c.hid_ok:
                self.lbl_device.config(text='hidapi 缺失', foreground=COLOR_ERR)
                self.set_message('hidapi 缺失 (%s)，执行: %s -m pip install hidapi'
                                 % (sys.executable, sys.executable), COLOR_ERR)
            else:
                self.set_message(self.i2c.last_error or '请插入 Pico (自动激活 HID 中...)',
                                 COLOR_ERR)

    def _poll_device(self):
        # hidapi 的 hid_write 在设备不收 OUT 报告时会硬卡 5 秒(macOS,
        # set_nonblocking 也无效), 所以设备扫描必须放到工作线程,
        # tkinter 主线程只消费结果, 永远不会被 USB 阻塞。
        if not self._busy and not self._scan_running:
            self._scan_running = True

            def work():
                try:
                    connected = self.i2c.scan()
                    err = self.i2c.last_error
                except Exception as e:
                    connected = False
                    err = f'扫描异常: {e}'
                    self.i2c.last_error = err
                self._post(self._scan_done, connected, err)

            threading.Thread(target=work, daemon=True).start()
        self.root.after(POLL_INTERVAL_MS, self._poll_device)

    def _scan_done(self, connected, err):
        self._scan_running = False
        if connected != self._last_connected:
            self._last_connected = connected
            self._update_device_ui(connected)
        elif not connected and err and self.i2c.hid_ok:
            self.set_message(err, COLOR_ERR)

    # ---------------- 忙碌状态
    def _set_busy(self, busy, text=''):
        self._busy = busy
        # 忙碌=蓝色呼吸灯; 结束按当前连接状态恢复
        if getattr(self, 'lamp', None) is not None:
            if busy:
                self._set_lamp('busy')
                self.lbl_device.config(text='工作中...', fg=COLOR_INFO)
            else:
                self._refresh_lamp()
                self.lbl_device.config(text=self._device_label(), fg='#555555')
        # 长任务期间冻结后台设备轮询, 避免 PING 与数据帧互抢导致误判断线
        self.i2c._io_pause = busy
        for btn in (self.btn_read, self.btn_write, self.btn_open,
                    self.btn_save, self.btn_fillff, self.btn_fill00,
                    self.btn_flash_save, self.btn_flash_load):
            btn.state(['disabled'] if busy else ['!disabled'])
        # ttk.Widget.state() 是增量修改: 只写 ['readonly'] 不会清掉上次
        # 加上的 'disabled', 忙碌一次后下拉框会永久锁死。必须显式取反。
        self.cbo_chip.state(['disabled'] if busy else ['!disabled', 'readonly'])
        if not busy:
            # 恢复读/写按钮的连接状态约束
            if not self._last_connected:
                self.btn_read.state(['disabled'])
                self.btn_write.state(['disabled'])
                self.btn_flash_save.state(['disabled'])
                self.btn_flash_load.state(['disabled'])
        if text:
            self.set_message(text, COLOR_INFO)

    # ---------------- 文件操作
    def _load_path(self, file_path):
        try:
            with open(file_path, 'rb') as f:
                data = f.read()
        except OSError as e:
            self.set_message(f'打开失败: {e}', COLOR_ERR)
            return
        self.editor.load(data)
        self._on_data_change()
        self.root.title(f'{os.path.basename(file_path)} - EEPROM Tool (Pico HID)')
        # msg = f'已加载 {os.path.basename(file_path)} ({len(data)} 字节)'
        # if len(data) > self.chip_size:
        #     msg += f', 超出 {self.chip_size} 部分已截断'
        # elif len(data) < self.chip_size:
        #     msg += ', 不足部分填充FF'
        # self.set_message(msg, COLOR_INFO)

    def _on_drop(self, event):
        file_path = event.data.strip('{}').replace('\\', '/')
        self._load_path(file_path)

    def open_file(self):
        path = filedialog.askopenfilename(
            title='打开bin文件',
            filetypes=[('bin files', '*.bin'), ('All files', '*.*')])
        if path:
            self._load_path(path)

    def save_file(self):
        path = filedialog.asksaveasfilename(
            title='保存bin文件',
            initialfile=f'{self.chip}_0X{self.editor.checksum():04X}',
            defaultextension='.bin',
            filetypes=[('bin files', '*.bin'), ('All files', '*.*')])
        if not path:
            return
        try:
            with open(path, 'wb') as f:
                f.write(bytes(self.editor.data))
            self.set_message(f'已保存: {os.path.basename(path)}', COLOR_OK)
        except OSError as e:
            self.set_message(f'保存失败: {e}', COLOR_ERR)

    def _fill(self, value):
        self.editor.fill(value)
        self._on_data_change()
        self.set_message(f'已清空为 {value:02X}', COLOR_INFO)

    def _on_chip_change(self, event=None):
        name = self.chip_var.get()
        if name not in EEPROM_CHIPS:
            return
        self.chip = name
        self.chip_id, self.chip_size = EEPROM_CHIPS[name]
        self.editor.resize(self.chip_size)
        self._on_data_change()
        cap = self.chip_size
        try:
            self.lbl_chip_size.config(
                text=(f'{cap} B' if cap < 1024 else f'{cap // 1024} KB'))
        except Exception:
            pass
        self.set_message(f'已切换 {name} ({self.chip_size} 字节)', COLOR_INFO)

    # ---------------- 读写(工作线程)
    def read_all(self):
        if not self.i2c.connected:
            self.set_message('设备未连接', COLOR_ERR)
            return
        self._set_busy(True, f'读取 {self.chip} 中...')

        def work():
            try:
                data = self.i2c.read_eeprom(self.chip_id, 0, self.chip_size)
                self._post(self._read_done, data)
            except Exception as e:
                self._post(self._op_done, f'读取失败: {e}', False)

        threading.Thread(target=work, daemon=True).start()

    def _read_done(self, data):
        self.editor.load(data)
        self._on_data_change()
        self._set_busy(False)
        self.set_message('读取完成', COLOR_OK)

    def write_all(self):
        if not self.i2c.connected:
            self.set_message('设备未连接', COLOR_ERR)
            return
        payload = bytes(self.editor.data)
        self._set_busy(True, f'烧录 {self.chip} 中...')

        def work():
            try:
                self.i2c.write_eeprom(self.chip_id, 0, payload)
                time.sleep(0.02)
                back = self.i2c.read_eeprom(self.chip_id, 0, self.chip_size)
                ok = (back == payload)
                self._post(self._write_done, ok, back)
            except Exception as e:
                self._post(self._op_done, f'烧录失败: {e}', False)

        threading.Thread(target=work, daemon=True).start()

    def _write_done(self, ok, back):
        self._set_busy(False)
        if ok:
            self.editor.load(back)
            self._on_data_change()
            self.set_message('烧录成功, 校验一致', COLOR_OK)
        else:
            diff = sum(1 for a, b in zip(back, bytes(self.editor.data)) if a != b)
            self.set_message(f'烧录校验失败, {diff} 个字节不一致', COLOR_ERR)

    # ---------------- 离线烧录(镜像存进 Pico 板载 Flash)
    def _need_dev(self):
        if not self.i2c.connected:
            self.set_message('设备未连接', COLOR_ERR)
            return False
        return True

    def _finish_bg(self, done, result):
        self._set_busy(False)
        done(result)

    def _run_bg(self, text, fn, done):
        # 工作线程跑 fn, 结果经队列回主线程; 无论成败都解除忙碌
        self._set_busy(True, text)

        def work():
            try:
                result = fn()
                self._post(self._finish_bg, done, result)
            except Exception as e:
                self._post(self._op_done, '操作失败: %s' % e, False)

        threading.Thread(target=work, daemon=True).start()

    def _chip_name(self, cid):
        return next((k for k, (v, _) in EEPROM_CHIPS.items() if v == cid), '?')

    def flash_save(self):
        # 把编辑区整片数据存进 Pico Flash, 之后上电即自动烧录
        if not self._need_dev():
            return
        payload = bytes(self.editor.data)
        chip_id, name = self.chip_id, self.chip
        n = len(payload)

        def done(_):
            self._set_img_lamp(True)
            self.set_message('已存入设备 Flash: %s / %dB - 断电重插即自动烧录'
                             % (name, n), COLOR_OK)
        self._run_bg('存入设备 Flash...',
                     lambda: self.i2c.flash_write(chip_id, payload), done)

    def flash_load(self):
        if not self._need_dev():
            return

        def done(result):
            info, data = result
            self._set_img_lamp(True)
            name = self._chip_name(info['chip_id'])
            if name in EEPROM_CHIPS and name != self.chip:
                self.chip_var.set(name)
                self._on_chip_change()
            self.editor.load(data)
            self._on_data_change()
            self.set_message('已从设备读取镜像: %s / %dB / CRC %08X'
                             % (name, len(data), info['crc']), COLOR_OK)
        self._run_bg('读取设备镜像...', lambda: self.i2c.flash_read(), done)

    def flash_erase(self):
        if not self._need_dev():
            return

        def done(_):
            self._set_img_lamp(False)
            self.set_message('设备镜像已清除 (上电不再自动烧录)', COLOR_OK)
        self._run_bg('清除设备镜像...', lambda: self.i2c.flash_erase(), done)

    def flash_burn(self):
        # 在线触发一次离线烧录, 不必断电重启
        if not self._need_dev():
            return

        def done(msg):
            self.set_message('离线烧录完成: %s' % msg, COLOR_OK)
        self._run_bg('正在离线烧录 EEPROM...', lambda: self.i2c.flash_burn(), done)

    def flash_show_info(self):
        if not self._need_dev():
            return

        def work():
            return self.i2c.flash_info(), self.i2c.flash_stat()

        def done(result):
            info, stat = result
            self._set_img_lamp(info['present'])
            if not info['present']:
                self.set_message('设备 Flash 里没有镜像', COLOR_INFO)
            else:
                self.set_message('设备镜像: %s / %dB / CRC %08X%s'
                                 % (self._chip_name(info['chip_id']),
                                    info['size'], info['crc'],
                                    ' | 上次烧录: ' + stat if stat else ''),
                                 COLOR_OK)
        self._run_bg('查询设备镜像...', work, done)

    # ---------------- 物理提示灯 GPIO 配置对话框
    def led_config(self):
        if not self._need_dev():
            return
        self._run_bg('读取提示灯 GPIO...',
                     lambda: self.i2c.led_get(), self._led_dialog)

    def _led_dialog(self, pin):
        self._led_pin = pin
        win = tk.Toplevel(self.root)
        win.title('提示灯 GPIO')
        win.resizable(False, False)
        win.transient(self.root)
        win.configure(bg='#F2F3F5')

        frm = ttk.Frame(win, padding=18, style='TFrame')
        frm.pack(fill='both', expand=True)
        tk.Label(frm, text='物理提示灯 GPIO', bg='#F2F3F5', fg='#2B3038',
                 font=(FONT_UI[0], 13, 'bold')).pack(anchor='w')
        tk.Label(frm, text='默认 GP25 = 板载 LED；改成其它脚可外接 LED'
                           '（串联 330Ω 到 GND）。在线读写会亮一下，'
                           '离线烧录结果仍用慢闪3次/快闪10次。',
                 bg='#F2F3F5', fg='#5A6472', font=(FONT_UI[0], 9),
                 wraplength=360, justify='left').pack(anchor='w', pady=(6, 10))

        row = ttk.Frame(frm, style='TFrame')
        row.pack(fill='x')
        tk.Label(row, text='GPIO 编号', bg='#F2F3F5',
                 font=FONT_UI).pack(side='left')
        var = tk.StringVar(value=str(pin))
        sp = tk.Spinbox(row, from_=0, to=LED_PIN_MAX, width=6,
                        textvariable=var, font=FONT_UI, bg='#FFFFFF',
                        relief='solid', bd=1)
        sp.pack(side='left', padx=10)

        cur = tk.Label(frm, text='当前: GP%d' % pin, bg='#F2F3F5',
                       fg=COLOR_INFO, font=(FONT_UI[0], 10, 'bold'))
        cur.pack(anchor='w', pady=(10, 2))
        tk.Label(frm, text='避开 GP4/GP5(I2C) 和 GP6(跳过烧录跳线)。'
                           '设置保存在 Pico 板载 Flash，掉电不丢。',
                 bg='#F2F3F5', fg='#8A9099', font=(FONT_UI[0], 9),
                 wraplength=360, justify='left').pack(anchor='w')

        def apply():
            try:
                new_pin = int(var.get())
            except ValueError:
                self.set_message('GPIO 必须是数字', COLOR_ERR)
                return
            if not 0 <= new_pin <= LED_PIN_MAX:
                self.set_message('GPIO 超出范围 0~%d' % LED_PIN_MAX, COLOR_ERR)
                return

            def done(actual):
                self._led_pin = actual
                cur.config(text='当前: GP%d' % actual)
                self.set_message('提示灯已设为 GP%d，板上应闪两下' % actual,
                                 COLOR_OK)
            self._run_bg('设置提示灯 GPIO...',
                         lambda: self.i2c.led_set(new_pin), done)

        btns = ttk.Frame(frm, style='TFrame')
        btns.pack(fill='x', pady=(14, 0))
        ttk.Button(btns, text='保存并测试', style='Primary.TButton',
                   command=apply).pack(side='right', padx=4)
        ttk.Button(btns, text='关闭', style='Tool.TButton',
                   command=win.destroy).pack(side='right', padx=4)
        try:
            win.grab_set()
        except tk.TclError:
            pass                   # headless / 窗口不可见时降级为普通弹窗

    def _op_done(self, text, ok):
        self._set_busy(False)
        self.set_message(text, COLOR_OK if ok else COLOR_ERR)

    def _on_close(self):
        self.i2c.close()
        self.root.destroy()


if __name__ == '__main__':
    MainApp()
