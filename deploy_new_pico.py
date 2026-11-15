#!/usr/bin/env python3
"""给一块全新(或想重装)的 Raspberry Pi Pico 装好 EEPROM HID 烧录器固件。

用法:
    /Users/mac/.local/bin/python3.12 /Users/mac/Downloads/pico_eeprom_hid/deploy_new_pico.py
    /Users/mac/.local/bin/python3.12 .../deploy_new_pico.py --check   # 只自检, 不碰硬件
    /Users/mac/.local/bin/python3.12 .../deploy_new_pico.py --force   # 重装已有固件的板子

流程:
  0) 运行前请只接一块 Pico(先拔掉已在使用的旧板)
  1) 发现 BOOTSEL 盘(/Volumes/RPI-RP2)就自动刷入 MicroPython UF2
  2) 等 CDC 串口枚举出来
  3) 经 raw REPL 上传 6 个 .py -> os.sync + 静置 -> 逐文件 CRC 校验
  4) 软复位, 让 boot.py / main.py 的上电入口生效(板内无镜像时不会烧芯片)

脚本不会调用 hid_usb.start() —— 那会把 CDC 一起摘掉。
HID 由上位机 PythonEEPROM_v2.py 打开后自动激活。
"""
import os
import sys
import glob
import time
import base64
import binascii
import shutil

FWDIR = os.path.dirname(os.path.abspath(__file__))
UF2 = os.path.join(FWDIR, 'RPI_PICO-20260406-v1.28.0.uf2')
FILES = ('boot.py', 'hid_usb.py', 'hidproto.py', 'eeprom24.py',
         'eeprom_image.py', 'main.py')
CDC_GLOB = '/dev/cu.usbmodem*'
BOOTSEL_GLOB = '/Volumes/RPI-RP2*'
HOST = '/Users/mac/Downloads/PythonEEPROM_v2.py'
GUI_PY = '/Users/mac/.local/bin/python3.12'


def check_local():
    """只检查本地文件/依赖, 完全不碰硬件。"""
    ok = True
    if not os.path.isfile(UF2):
        print('[缺失] MicroPython UF2:', UF2)
        ok = False
    else:
        print('[OK]   MicroPython UF2: %s (%d KB)'
              % (os.path.basename(UF2), os.path.getsize(UF2) // 1024))
    for n in FILES:
        p = os.path.join(FWDIR, n)
        if not os.path.isfile(p):
            print('[缺失] 固件文件:', n)
            ok = False
            continue
        d = open(p, 'rb').read()
        print('[OK]   %-16s %5d B  CRC %08X' % (n, len(d), binascii.crc32(d)))
    try:
        import serial  # noqa: F401
        print('[OK]   pyserial 可用')
    except ImportError:
        print('[缺失] pyserial —— 请改用 %s 运行本脚本' % GUI_PY)
        ok = False
    print('[OK]   上位机:', HOST if os.path.isfile(HOST) else '[缺失] ' + HOST)
    if not os.path.isfile(HOST):
        ok = False
    return ok


if '--check' in sys.argv:
    sys.exit(0 if check_local() else 1)

if not check_local():
    sys.exit('本地文件/依赖不全，先补齐再运行。')
import serial  # noqa: E402


def cdc_ports():
    return sorted(glob.glob(CDC_GLOB))


def bootsel():
    v = sorted(glob.glob(BOOTSEL_GLOB))
    return v[0] if v else None


def wait_for(fn, timeout, what):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        r = fn()
        if r:
            return r
        time.sleep(0.3)
    print('[超时] %s' % what)
    return None


def drain(s):
    try:
        s.read_all()
    except OSError:
        pass


def rex(s, code, timeout=12.0):
    """raw REPL 执行一条语句; 响应结尾有两个 \\x04, 读到第一个就 drain 余下。"""
    s.write(code.encode('ascii') + b'\x04')
    out = bytearray()
    dl = time.monotonic() + timeout
    while time.monotonic() < dl:
        b = s.read(1)
        if not b:
            continue
        if b == b'\x04':
            time.sleep(0.04)
            drain(s)
            return bytes(out).decode('utf-8', 'replace')
        out += b
    raise TimeoutError('raw REPL 无响应: %s' % code[:60])


print('\n>>> 请确认: 只接一块 Pico, 已拔掉正在使用的旧板, 且 Thonny 等串口程序已关闭\n')

vol = bootsel()
if vol:
    print('[1/4] 发现 BOOTSEL 盘 %s, 自动刷入 MicroPython...' % vol)
    shutil.copyfile(UF2, os.path.join(vol, os.path.basename(UF2)))
    wait_for(lambda: not bootsel(), 30, 'BOOTSEL 盘未自动弹出')
    print('      刷入完成, 板子正在重启')
elif not cdc_ports():
    print('[1/4] 未发现 Pico。现在请按住 BOOTSEL 键, 把新 Pico 插入 USB (等待 90 秒)...')
    vol = wait_for(bootsel, 90, '未看到 RPI-RP2 盘')
    if vol:
        print('      发现 BOOTSEL 盘, 自动刷入 MicroPython...')
        shutil.copyfile(UF2, os.path.join(vol, os.path.basename(UF2)))
        wait_for(lambda: not bootsel(), 30, 'BOOTSEL 盘未自动弹出')
    elif not cdc_ports():
        sys.exit('未发现 Pico: 检查 USB 线(必须是数据线)和 BOOTSEL 是否按住。')
else:
    print('[1/4] 已有 CDC 串口(板子已跑 MicroPython), 跳过刷 UF2, 直接覆盖固件文件')

ports = wait_for(cdc_ports, 60, 'CDC 串口未枚举出来')
if not ports:
    sys.exit('没找到 /dev/cu.usbmodem*, 无法继续。')
if len(ports) > 1:
    sys.exit('检测到 %d 个 CDC 串口 %s —— 请只接一块 Pico 再运行。'
             % (len(ports), ports))
port = ports[0]
print('[2/4] CDC 串口: %s' % port)

files = {}
for n in FILES:
    d = open(os.path.join(FWDIR, n), 'rb').read()
    files[n] = (d, binascii.crc32(d) & 0xFFFFFFFF)

s = serial.Serial(port, 115200, timeout=0.15)
try:
    s.write(b'\r\x03\x03')          # 打断可能正在跑的主循环
    time.sleep(0.3)
    drain(s)
    s.write(b'\r\x01')              # Ctrl-A 进 raw REPL
    time.sleep(0.3)
    drain(s)
    rex(s, 'import ubinascii, os')

    # 防呆: 当前插着的很可能就是已在使用的旧板, 默认绝不覆盖它
    before = rex(s, 'print(os.listdir())')
    already = all(n in before for n in FILES)
    has_img = 'eeprom.img' in before
    if (already or has_img) and '--force' not in sys.argv:
        print('\n[停止] 这块板已经装过固件 / 存有离线镜像, 看起来不是新板:')
        print('       %s' % before.strip())
        print('       请先拔掉旧 Pico, 只接新板再运行本脚本。')
        print('       若确实要重装这块板, 加 --force 重新运行。\n')
        sys.exit(0)
    if has_img:
        print('      [提醒] 板内已有离线镜像, 复位上电会自动烧一次;'
              ' 不想烧就把 GP6 接 GND 后再上电')

    print('[3/4] 上传 6 个固件文件...')
    for name, (data, want_crc) in files.items():
        rex(s, "_f=open('%s','wb')" % name)
        for i in range(0, len(data), 256):
            b64 = base64.b64encode(data[i:i + 256]).decode('ascii')
            rex(s, "_f.write(ubinascii.a2b_base64('%s'))" % b64)
        rex(s, '_f.close()')
        print('      %-16s %5d B 已写入' % (name, len(data)))

    try:
        rex(s, 'os.sync()')
    except Exception:
        pass
    time.sleep(0.8)                 # LFS 落盘静置: 不做这步复位后文件尾部会丢
    for name, (data, want_crc) in files.items():
        got = rex(s, "print(os.stat('%s')[6], ubinascii.crc32(open('%s','rb').read()))"
                  % (name, name))
        assert ('%d %d' % (len(data), want_crc)) in got, \
            '%s 板内校验失败: %r' % (name, got)
    print('      逐文件 CRC 校验通过')

    listing = rex(s, 'print(os.listdir())').strip()
    print('      板内文件:', listing)
    if 'eeprom.img' in listing:
        print('      [注意] 板内已有离线镜像, 本次只覆盖 .py, 镜像保持不变')

    print('[4/4] 软复位, 让 boot.py / main.py 的上电入口生效...')
    try:
        s.write(b'import machine; machine.soft_reset()\x04')
        time.sleep(0.6)
    except OSError:
        pass
finally:
    try:
        s.close()
    except OSError:
        pass

print('''
>>> 固件已装好。接下来:

  1) 接线 (断电操作)
       EEPROM SDA -> GP4      SCL -> GP5
       VCC -> 3V3(OUT)       GND -> GND
       A0/A1/A2 与 WP 全部接 GND  (从机地址 0x50)
       SDA、SCL 各加 4.7k 上拉到 3V3 (裸芯片模块通常已内置)

  2) 只留这一块 Pico 插着, 关闭 Thonny 等占用串口的程序

  3) 打开上位机 (首次会经 CDC 自动激活 HID, 状态灯变绿):
       %s %s

  4) 可选 · 脱离电脑批量烧录:
       在上位机里打开待烧文件, 点「存入设备」把镜像写进 Pico 板载 Flash。
       之后每次断电再上电自动烧一片; GP6 接 GND 上电可跳过本次。
''' % (GUI_PY, HOST))
