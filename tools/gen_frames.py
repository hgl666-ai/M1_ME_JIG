#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""M1 治具 PC 端协议帧生成/自检脚本

帧格式（IINFO/M1 测试治具PC端通讯协议.xlsx）：
    A5 5B | CMD(2) | SEQ(2) | LEN(2) | DATA(L) | CRC16(2)
    所有多字节字段大端；CRC16 = MODBUS（多项式 0xA001 反射，初值 0xFFFF）

用法：
    python gen_frames.py            # 自检 + 打印样例帧
"""

import sys


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def build(cmd: int, seq: int, data: bytes = b"") -> bytes:
    body = bytes([(cmd >> 8) & 0xFF, cmd & 0xFF,
                  (seq >> 8) & 0xFF, seq & 0xFF,
                  (len(data) >> 8) & 0xFF, len(data) & 0xFF]) + data
    frame = b"\xA5\x5B" + body
    crc = crc16_modbus(frame)
    return frame + bytes([(crc >> 8) & 0xFF, crc & 0xFF])


def hexs(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)


# 已知向量（来自 R1 治具文档，用来校验 CRC 实现与字节序）
KNOWN = [
    (0x0001, 0x0001, b"", "A5 5B 00 01 00 01 00 00 89 0C"),
    (0x0002, 0x0001, b"", "A5 5B 00 02 00 01 00 00 89 48"),
    (0x0012, 0x0001, b"", "A5 5B 00 12 00 01 00 00 4A 89"),
    (0x0020, 0x0001, b"", "A5 5B 00 20 00 01 00 00 8E B0"),
    (0x0010, 0x0001, bytes([0x00, 0x00, 0x00, 0x00, 0xC8, 0x03, 0xE8]),
     "A5 5B 00 10 00 01 00 07 00 00 00 00 C8 03 E8 C3 2F"),
]


def selftest() -> bool:
    ok = True
    for cmd, seq, data, expect in KNOWN:
        got = hexs(build(cmd, seq, data))
        if got != expect:
            ok = False
            print(f"FAIL cmd=0x{cmd:04X}\n  got   : {got}\n  expect: {expect}")
    return ok


def sample_table():
    rows = [
        ("PING",               0x0001, b""),
        ("GET_VERSION",        0x0002, b""),
        ("GET_UID",            0x0003, b""),
        ("MEASURE_ADC 读快照",  0x000E, bytes([0x00])),
        ("MEASURE_ADC 触发测量", 0x000E, bytes([0x01])),
        ("SENSOR_READ 读状态",  0x000F, bytes([0x00])),
        ("SENSOR_READ 立即采",  0x000F, bytes([0x01])),
        ("SENSOR_CONFIG ch0 2000/1700", 0x0021, bytes([0x00, 0x07, 0xD0, 0x06, 0xA4])),
        ("SELF_TEST",          0x0030, b""),
        ("MOTOR_MOVE CW 200步 1000Hz", 0x0010, bytes([0x00, 0x00, 0x00, 0x00, 0xC8, 0x03, 0xE8])),
        ("MOTOR_MOVE CCW 3200步 4000Hz", 0x0010, bytes([0x01, 0x00, 0x00, 0x0C, 0x80, 0x0F, 0xA0])),
        ("MOTOR_STOP",         0x0011, b""),
        ("MOTOR_STATUS",       0x0012, b""),
        ("TMC_QUERY",          0x0020, b""),
        ("TMC_SET_IRUN 16/8",  0x0022, bytes([0x10, 0x08])),
    ]
    print("\n序号统一用 00 01：\n")
    for name, cmd, data in rows:
        f = build(cmd, 0x0001, data)
        print(f"{name:32s} {hexs(f)}")


if __name__ == "__main__":
    print("=== CRC16/组帧自检 ===")
    ok = selftest()
    print("PASS" if ok else "FAIL")
    if not ok:
        sys.exit(1)
    sample_table()
