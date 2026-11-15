# eeprom24.py — 24C01~24C256 的 I2C 读写(型号ID 与上位机 EEPROM_CHIPS 一一对应)
#
# 关键点:
#  * 24C01/02/04/08/16 用 1 字节字地址, A8~A10 编码在 I2C 从机地址 bit3:1,
#    所以每次传输都不能跨 256 字节边界;
#  * 24C32 及以上用 2 字节字地址, 从机地址固定 0x50;
#  * 页写不能跨页, 写完用 ACK polling 等内部写周期(比固定 sleep 5ms 快且准)。

import machine
import time

I2C_BUS = 0
SDA_PIN = 4                 # GP4 / 物理脚 6
SCL_PIN = 5                 # GP5 / 物理脚 7
EEPROM_ADDR = 0x50          # A0=A1=A2=GND
I2C_FREQ = 100000           # 长线/弱上拉不改; 短线可提到 400000

# 型号ID -> (容量字节, 页大小, 字地址字节数, 高位地址是否编码进从机地址)
CHIPS = (
    (128,   8, 1, True),    # 0  24C01
    (256,   8, 1, True),    # 1  24C02
    (512,  16, 1, True),    # 2  24C04
    (1024, 16, 1, True),    # 3  24C08
    (2048, 16, 1, True),    # 4  24C16
    (4096, 32, 2, False),   # 5  24C32
    (8192, 32, 2, False),   # 6  24C64
    (16384, 64, 2, False),  # 7  24C128
    (32768, 64, 2, False),  # 8  24C256
)

_i2c = None


def init():
    global _i2c
    _i2c = machine.I2C(I2C_BUS,
                       sda=machine.Pin(SDA_PIN),
                       scl=machine.Pin(SCL_PIN),
                       freq=I2C_FREQ)
    return _i2c


def present():
    """扫描总线, 返回 EEPROM 从机地址; 未找到返回 0。"""
    global _i2c
    if _i2c is None:
        init()
    try:
        for a in _i2c.scan():
            if EEPROM_ADDR <= a <= EEPROM_ADDR + 7:
                return a
    except OSError:
        pass
    return 0


def _addr(chip_id, a):
    size, page, naddr, enc = CHIPS[chip_id]
    if enc:
        dev = EEPROM_ADDR | ((a >> 8) & 0x07)
        mem = a & 0xFF
    else:
        dev = EEPROM_ADDR
        mem = a
    return size, page, naddr, dev, mem


def _wait_ready(dev):
    """ACK polling: 内部写周期内器件不应答, 直到可再次寻址。"""
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < 20:
        try:
            _i2c.writeto(dev, b'')
            return True
        except OSError:
            time.sleep_ms(1)
    return False


def read(chip_id, addr, length):
    global _i2c
    if _i2c is None:
        init()
    size = CHIPS[chip_id][0]
    if addr < 0 or length < 0 or addr + length > size:
        raise ValueError('range')
    out = bytearray()
    off = 0
    while off < length:
        a = addr + off
        # 响应 payload = status(1) + data, 所以单帧最多 58B; 上位机按 56B 分块
        n = min(length - off, 58, 256 - (a % 256))
        _, _, naddr, dev, mem = _addr(chip_id, a)
        out += _i2c.readfrom_mem(dev, mem, n, addrsize=8 if naddr == 1 else 16)
        off += n
    return bytes(out)


def write(chip_id, addr, data):
    global _i2c
    if _i2c is None:
        init()
    size, page, _, _ = CHIPS[chip_id]
    data = bytes(data)
    if addr < 0 or addr + len(data) > size:
        raise ValueError('range')
    off = 0
    while off < len(data):
        a = addr + off
        n = min(page - (a % page), 256 - (a % 256), len(data) - off)
        _, _, naddr, dev, mem = _addr(chip_id, a)
        _i2c.writeto_mem(dev, mem, data[off:off + n],
                         addrsize=8 if naddr == 1 else 16)
        _wait_ready(dev)
        off += n
    return off
