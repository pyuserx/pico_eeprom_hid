# boot.py — 保持内置 CDC(REPL), 这里绝不碰 USB。
#
# 实测: 在冷启动阶段(boot.py / main.py 被 MicroPython 自动执行时)重新配置
# machine.USBDevice 会打断主机枚举, 导致整板 USB 挂死(未知 USB 设备)。
# 因此自定义 HID 必须由 PC 上位机经 CDC REPL 一次性下发:
#     import hid_usb; hid_usb.start()
#     import main; main.main()
# PythonEEPROM_v2.py 已把这一步做成后台自动激活, 打开上位机即完成,
# 不用手动敲、不用按 BOOTSEL。每次物理断电重插后在线模式需要重新激活一次。
#
# 离线烧录不需要上面这一步: main.py 在上电时若发现 Flash 里有镜像,
# 会直接烧 EEPROM 并用板载 LED 指示结果, 全程保持纯 CDC、不碰 USBDevice。
# GP6 接 GND 上电 = 跳过本次自动烧录(方便插电脑调试)。
#
# 接线: EEPROM SDA -> GP4, SCL -> GP5, VCC -> 3V3(OUT), GND -> GND
#       地址 A0/A1/A2 全部接 GND => 0x50
