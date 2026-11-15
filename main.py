# main.py — Pico 端: 在线 HID 命令 + 上电离线自动烧录
#
# 两种模式共用这一份文件, 但互不干扰:
#  * 离线(上电默认): Flash 里存有镜像就自动烧一次 EEPROM, 提示灯指示结果,
#    全程不碰 machine.USBDevice -> USB 保持纯 CDC, 不会挂死, 也不需要电脑。
#    烧完直接退回 REPL, 不留死循环。
#  * 在线(PC 接管): 上位机经 CDC REPL 下发
#        import hid_usb; hid_usb.start()
#        import main; main.main()
#    进入 HID 主循环, 处理在线读写 / Flash 镜像存取。
#
# USB 回调运行在中断上下文: hid_usb 内部只入队/置标志, 所有处理都在本主循环完成。
# 主循环里不要 print(), 因为 CDC 在 start() 后可能已断开。

import time
import machine
import hid_usb
import hidproto as P
import eeprom24
import eeprom_image

INFO = b'Pico EEPROM HID v1.2'

# --- 离线烧录配置 ---
OFFLINE_AUTO = True          # 上电自动烧一次; 改 False 则只能由上位机点「离线烧录」触发
OFFLINE_SKIP_PIN = 6         # GP6 接 GND = 跳过本次自动烧录(方便插电脑调试)

# --- 物理提示灯: GPIO 可自定义, 配置存 led.cfg, 掉电不丢 ---
LED_PIN_FILE = 'led.cfg'
LED_PIN_DEFAULT = 25         # 标准 Pico 板载 LED
LED_PIN_MAX = 28             # RP2040 可用 GPIO 0~28
LED_ACTIVE_TICKS = 220       # 在线数据命令点亮约 220ms(主循环 1ms/轮)

_led = None
_led_pin = LED_PIN_DEFAULT
_led_left = 0


def _led_load():
    # 只读文件, 不碰 machine, 导入模块时即可安全执行
    global _led_pin
    try:
        v = int(open(LED_PIN_FILE).read().strip())
        if 0 <= v <= LED_PIN_MAX:
            _led_pin = v
    except Exception:
        _led_pin = LED_PIN_DEFAULT
    return _led_pin


def _led_setup():
    global _led
    try:
        _led = machine.Pin(_led_pin, machine.Pin.OUT)
        _led.value(0)
    except Exception:
        _led = None


def _led_save(pin):
    global _led_pin
    with open(LED_PIN_FILE, 'w') as f:
        f.write(str(pin))
    try:
        import os
        os.sync()          # 小文件也强制落盘, 防止复位后 LFS 尾部丢失
    except Exception:
        pass
    _led_pin = pin
    _led_setup()


def _mark_active():
    # 在线读写 / Flash 命令时点亮; PING 保活不算, 免得灯一直闪
    global _led_left
    _led_left = LED_ACTIVE_TICKS
    if _led is not None:
        try:
            _led.value(1)
        except Exception:
            pass


def _led_tick():
    global _led_left
    if _led_left:
        _led_left -= 1
        if _led_left == 0 and _led is not None:
            try:
                _led.value(0)
            except Exception:
                pass


_led_load()


def _blink(times, on_ms, off_ms):
    global _led
    if _led is None:
        return
    try:
        for _ in range(times):
            _led.value(1)
            time.sleep_ms(on_ms)
            _led.value(0)
            time.sleep_ms(off_ms)
    except Exception:
        pass


def _indicate(ok):
    """成功: 慢闪 3 次; 失败: 快闪 10 次。用 led.cfg 配置的 GPIO。"""
    _led_setup()
    _blink(3, 200, 120) if ok else _blink(10, 60, 60)
    if _led is not None:
        try:
            _led.value(0)
        except Exception:
            pass


def offline_burn(indicate=True):
    """从 Flash 读镜像 -> CRC 校验 -> 写 EEPROM -> 回读比对。返回 (ok, msg)。"""
    img = eeprom_image.load()
    if img is None:
        return False, 'no image'
    chip, data = img
    try:
        eeprom24.init()
        if not eeprom24.present():
            raise OSError('no eeprom')
        eeprom24.write(chip, 0, data)
        back = eeprom24.read(chip, 0, len(data))
        ok = (back == data)
        msg = ('ok %dB' % len(data)) if ok else 'verify fail'
    except Exception as e:
        ok, msg = False, str(e)[:24]
    eeprom_image.set_stat(ok, msg)
    if indicate:
        _indicate(ok)
    return ok, msg


def _reply(cmd, seq, payload=b''):
    hid_usb.send(P.pack((cmd | 0x80) & 0xFF, seq, payload))


def _err(seq, code, msg=b''):
    hid_usb.send(P.pack(code, seq, msg[:24]))


def handle(frame):
    try:
        cmd, seq, payload = P.unpack(frame)
    except ValueError:
        return                     # 坏帧直接丢弃, 不回包

    if cmd not in (P.CMD_PING, P.CMD_LED_SET):
        _mark_active()             # 在线数据命令: 提示灯亮一下

    if cmd == P.CMD_PING:
        _reply(cmd, seq, INFO)

    elif cmd == P.CMD_GET_INFO:
        txt = 'Pico EEPROM HID | fw=1.2 | led=GP%d | i2c=%s' % (
            _led_pin, eeprom24.I2C_BUS)
        _reply(cmd, seq, txt.encode())

    elif cmd == P.CMD_EEPROM_PROBE:
        a = eeprom24.present()
        _reply(cmd, seq, bytes((0 if a else 1, a)))

    elif cmd == P.CMD_EEPROM_READ:
        if len(payload) < 4:
            _err(seq, P.ERR_BAD_FRAME, b'need 4B')
            return
        chip, ah, al, n = payload[0], payload[1], payload[2], payload[3]
        if n > P.MAX_PAYLOAD - 1:
            _err(seq, P.ERR_BAD_FRAME, b'len too big')
            return
        try:
            data = eeprom24.read(chip, (ah << 8) | al, n)
            _reply(cmd, seq, b'\x00' + data)
        except Exception:
            _err(seq, P.ERR_I2C, b'i2c read error')

    elif cmd == P.CMD_EEPROM_WRITE:
        if len(payload) < 4:
            _err(seq, P.ERR_BAD_FRAME, b'need 4B')
            return
        chip, ah, al, n = payload[0], payload[1], payload[2], payload[3]
        data = payload[4:]
        if n == 0 or n != len(data):
            _err(seq, P.ERR_BAD_FRAME, b'len mismatch')
            return
        try:
            eeprom24.write(chip, (ah << 8) | al, data)
            _reply(cmd, seq, bytes((0, n)))
        except Exception:
            _err(seq, P.ERR_I2C, b'i2c write error')

    # ---------------- 板载 Flash 镜像 ----------------
    elif cmd == P.CMD_FLASH_INFO:
        present, chip, size, crc, flags = eeprom_image.info()
        _reply(cmd, seq,
               bytes((present, chip, size & 0xFF, size >> 8, flags))
               + crc.to_bytes(4, 'little'))

    elif cmd == P.CMD_FLASH_BEGIN:
        if len(payload) < 7:
            _err(seq, P.ERR_BAD_FRAME, b'need 7B')
            return
        chip = payload[0]
        size = payload[1] | (payload[2] << 8)
        crc = int.from_bytes(payload[3:7], 'little')
        try:
            eeprom_image.begin(chip, size, crc)
            _reply(cmd, seq, b'\x00')
        except Exception:
            _err(seq, P.ERR_FLASH, b'begin fail')

    elif cmd == P.CMD_FLASH_DATA:
        if len(payload) < 3:
            _err(seq, P.ERR_BAD_FRAME, b'need 3B')
            return
        off = payload[0] | (payload[1] << 8)
        n = payload[2]
        data = payload[3:]
        if n == 0 or n != len(data) or n > P.MAX_PAYLOAD - 3:
            _err(seq, P.ERR_BAD_FRAME, b'len mismatch')
            return
        try:
            eeprom_image.put(off, data)
            _reply(cmd, seq, bytes((0, off & 0xFF, off >> 8)))
        except Exception:
            _err(seq, P.ERR_FLASH, b'put fail')

    elif cmd == P.CMD_FLASH_COMMIT:
        try:
            ok, msg = eeprom_image.commit()
            _reply(cmd, seq, bytes((0 if ok else 1,)) + msg.encode()[:20])
        except Exception:
            _err(seq, P.ERR_FLASH, b'commit fail')

    elif cmd == P.CMD_FLASH_READ:
        if len(payload) < 3:
            _err(seq, P.ERR_BAD_FRAME, b'need 3B')
            return
        off = payload[0] | (payload[1] << 8)
        n = payload[2]
        if n > P.MAX_PAYLOAD - 1:
            _err(seq, P.ERR_BAD_FRAME, b'len too big')
            return
        try:
            _reply(cmd, seq, b'\x00' + eeprom_image.read(off, n))
        except Exception:
            _err(seq, P.ERR_FLASH, b'read fail')

    elif cmd == P.CMD_FLASH_ERASE:
        try:
            eeprom_image.erase()
            _reply(cmd, seq, b'\x00')
        except Exception:
            _err(seq, P.ERR_FLASH, b'erase fail')

    elif cmd == P.CMD_FLASH_BURN:
        ok, msg = offline_burn(indicate=False)
        _reply(cmd, seq, bytes((0 if ok else 1,)) + msg.encode()[:24])

    elif cmd == P.CMD_FLASH_STAT:
        _reply(cmd, seq, eeprom_image.stat().encode()[:48])

    # ---------------- 物理提示灯 GPIO ----------------
    elif cmd == P.CMD_LED_GET:
        _reply(cmd, seq, bytes((_led_pin,)))

    elif cmd == P.CMD_LED_SET:
        if len(payload) < 1:
            _err(seq, P.ERR_BAD_FRAME, b'need 1B')
            return
        pin = payload[0]
        if not 0 <= pin <= LED_PIN_MAX:
            _reply(cmd, seq, bytes((1, _led_pin)))
            return
        try:
            _led_save(pin)
            _blink(2, 80, 60)     # 新 IO 上立刻闪两下, 方便确认接线
            _reply(cmd, seq, bytes((0, _led_pin)))
        except Exception:
            _err(seq, P.ERR_FLASH, b'led save fail')

    else:
        _err(seq, P.ERR_BAD_CMD, bytes((cmd,)))


def main():
    eeprom24.init()
    _led_setup()
    while True:
        try:
            hid_usb.poll()         # 保持一笔 OUT 传输常驻
            frame = hid_usb.recv()
            if frame is not None:
                handle(frame)
            _led_tick()           # 活动灯到时熄灭
        except KeyboardInterrupt:
            raise                 # 保留 Ctrl-C, PC 随时能接管 REPL
        except Exception:
            # USB 重枚举 / I2C 偶发失败绝不能让主循环退出, 否则 HID 链路直接死掉
            pass
        time.sleep_ms(1)


if __name__ == '__main__':
    # 上电离线模式: 有镜像就烧一次, 然后直接退回 REPL 让 PC 随时接管。
    # 每断一次电再上电 = 再烧一片, 天然适合批量复制。
    try:
        skip = False
        if OFFLINE_AUTO:
            try:
                skip = machine.Pin(OFFLINE_SKIP_PIN, machine.Pin.IN,
                                   machine.Pin.PULL_UP).value() == 0
            except Exception:
                skip = False
        if OFFLINE_AUTO and not skip and eeprom_image.info()[0]:
            offline_burn()
    except KeyboardInterrupt:
        pass
