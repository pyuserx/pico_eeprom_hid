# eeprom_image.py — 把「预烧录镜像」存进 Pico 板载 Flash(MicroPython LFS)
#
# 文件布局(/ 根目录下):
#   eeprom.img : 16B 头 + 原始镜像数据(最大 32KB, 24C256)
#   eeprom.tmp : 上位机分块写入时的临时文件, COMMIT 校验通过后才转正
#   burn.stat  : 上次离线烧录结果, 回电脑后可查询
#
# 头 16 字节:
#   0..7   b'PICOIMG1'
#   8      chip_id (与 eeprom24.CHIPS 下标一致)
#   9..10  size, little endian
#   11..14 CRC32(数据区), little endian
#   15     flags (保留)
#
# 为什么要 BEGIN/DATA/COMMIT 三段: 32KB 要拆成 ~600 帧, 中途拔线会留下
# 半截文件; 只有 COMMIT 时长度+CRC 全对才改名为正式镜像, 绝不拿坏数据去烧芯片。

import binascii
import os

MAGIC = b'PICOIMG1'
HDR_LEN = 16
IMG_PATH = 'eeprom.img'
TMP_PATH = 'eeprom.tmp'
STAT_PATH = 'burn.stat'
MAX_SIZE = 32768


def crc32(data):
    return binascii.crc32(data) & 0xFFFFFFFF


def _head(chip, size, crc, flags=0):
    return (MAGIC + bytes((chip & 0xFF, size & 0xFF, (size >> 8) & 0xFF))
            + crc.to_bytes(4, 'little') + bytes((flags,)))


def info():
    """返回 (present, chip_id, size, crc, flags); 无镜像/头坏返回 (0,0,0,0,0)。"""
    try:
        with open(IMG_PATH, 'rb') as f:
            h = f.read(HDR_LEN)
    except OSError:
        return (0, 0, 0, 0, 0)
    if len(h) != HDR_LEN or h[:8] != MAGIC:
        return (0, 0, 0, 0, 0)
    size = h[9] | (h[10] << 8)
    crc = int.from_bytes(h[11:15], 'little')
    return (1, h[8], size, crc, h[15])


def begin(chip, size, crc):
    """开一次写入: 只建临时文件并写头, 不动已有镜像。"""
    if size <= 0 or size > MAX_SIZE:
        raise ValueError('size')
    with open(TMP_PATH, 'wb') as f:
        f.write(_head(chip, size, crc))
    return size


def put(off, data):
    """按绝对偏移写一帧, 丢帧重传也能落回正确位置。"""
    with open(TMP_PATH, 'r+b') as f:
        f.seek(HDR_LEN + off)
        f.write(data)
    return off + len(data)


def commit():
    """校验长度+CRC, 通过才转正。返回 (ok, msg)。"""
    try:
        with open(TMP_PATH, 'rb') as f:
            h = f.read(HDR_LEN)
            body = f.read()
    except OSError:
        return False, 'no temp'
    if len(h) != HDR_LEN or h[:8] != MAGIC:
        return False, 'bad header'
    size = h[9] | (h[10] << 8)
    crc = int.from_bytes(h[11:15], 'little')
    if len(body) != size:
        return False, 'len %d/%d' % (len(body), size)
    if crc32(body) != crc:
        return False, 'crc mismatch'
    try:
        os.remove(IMG_PATH)
    except OSError:
        pass
    try:
        os.rename(TMP_PATH, IMG_PATH)
    except OSError:
        # 个别端口没有 rename: 退化为整片拷贝
        with open(TMP_PATH, 'rb') as src, open(IMG_PATH, 'wb') as dst:
            while True:
                b = src.read(256)
                if not b:
                    break
                dst.write(b)
        try:
            os.remove(TMP_PATH)
        except OSError:
            pass
    return True, 'ok'


def read(off, n):
    """读镜像数据区(不含头), 供上位机回读校验。"""
    if not info()[0]:
        return b''
    with open(IMG_PATH, 'rb') as f:
        f.seek(HDR_LEN + off)
        return f.read(n)


def load():
    """离线烧录用: 整片读进 RAM 并校验, 返回 (chip_id, data) 或 None。
    最大 32KB, Pico 有 264KB RAM, 放得下。"""
    present, chip, size, crc, _ = info()
    if not present or size <= 0 or size > MAX_SIZE:
        return None
    with open(IMG_PATH, 'rb') as f:
        f.seek(HDR_LEN)
        data = f.read(size)
    if len(data) != size or crc32(data) != crc:
        return None
    return chip, data


def erase():
    for p in (IMG_PATH, TMP_PATH, STAT_PATH):
        try:
            os.remove(p)
        except OSError:
            pass


def set_stat(ok, msg):
    try:
        with open(STAT_PATH, 'w') as f:
            f.write(('1 ' if ok else '0 ') + str(msg)[:60])
    except OSError:
        pass


def stat():
    try:
        with open(STAT_PATH, 'r') as f:
            return f.read()[:64]
    except OSError:
        return ''
