# hidproto.py — 自定义 HID 帧协议(上位机 PicoHIDEEPROM 与此处逐字节一致)
#
# 每帧固定 64 字节(不足补 0):
#   0      : 0xA5 帧头
#   1      : CMD (响应时最高位置 1, 即 cmd | 0x80)
#   2      : SEQ 序号, 主机自增, 响应原样带回
#   3      : LEN 负载长度 0..59
#   4..    : PAYLOAD
#   4+LEN  : CRC8 (多项式 0x07, 初值 0x00, 覆盖 [0, 4+LEN))
#   其余   : 0
# 64 = 4(头) + 59(负载) + 1(CRC)

MAGIC = 0xA5
REPORT_LEN = 64
MAX_PAYLOAD = 59

# 命令字(沿用 pico_hid_demo 的 PING/INFO, EEPROM 专用命令从 0x50 起)
CMD_PING = 0x01
CMD_GET_INFO = 0x03
CMD_EEPROM_PROBE = 0x50
CMD_EEPROM_READ = 0x51
CMD_EEPROM_WRITE = 0x52

# 板载 Flash 镜像 / 离线烧录命令 (0x60 起)
CMD_FLASH_INFO = 0x60     # -> [present, chip, size_lo, size_hi, flags, crc32(4B LE)]
CMD_FLASH_BEGIN = 0x61    # <- [chip, size_lo, size_hi, crc32(4B LE)]
CMD_FLASH_DATA = 0x62     # <- [off_lo, off_hi, n, data...]
CMD_FLASH_COMMIT = 0x63   # -> [status, msg]
CMD_FLASH_READ = 0x64     # <- [off_lo, off_hi, n] -> [status, data...]
CMD_FLASH_ERASE = 0x65    # -> [status]
CMD_FLASH_BURN = 0x66     # -> [status, msg]  在线触发一次离线烧录
CMD_FLASH_STAT = 0x67     # -> 上次离线烧录结果文本

# 物理提示灯 GPIO 配置 (0x68 起)
CMD_LED_GET = 0x68        # -> [pin]
CMD_LED_SET = 0x69        # <- [pin] -> [status, pin]; 写入 led.cfg, 掉电保留

# 错误响应(不带 0x80 标志位, 上位机单独识别)
ERR_BAD_CMD = 0x40
ERR_BAD_FRAME = 0x41
ERR_I2C = 0x42
ERR_FLASH = 0x43


def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def pack(cmd, seq, payload=b''):
    payload = bytes(payload)
    n = len(payload)
    if n > MAX_PAYLOAD:
        raise ValueError('payload too long')
    f = bytearray(REPORT_LEN)
    f[0] = MAGIC
    f[1] = cmd & 0xFF
    f[2] = seq & 0xFF
    f[3] = n
    f[4:4 + n] = payload
    f[4 + n] = crc8(f[:4 + n])
    return bytes(f)


def unpack(frame):
    """返回 (cmd, seq, payload); 帧非法抛 ValueError。"""
    if len(frame) != REPORT_LEN or frame[0] != MAGIC:
        raise ValueError('bad magic')
    n = frame[3]
    if n > MAX_PAYLOAD or crc8(frame[:4 + n]) != frame[4 + n]:
        raise ValueError('bad crc')
    return frame[1], frame[2], bytes(frame[4:4 + n])
