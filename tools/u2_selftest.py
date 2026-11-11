#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""M1 治具 U2 固件 —— 串口一键自测

用法：
    python u2_selftest.py --port COM4                # 基础自测（不转电机，115200）
    python u2_selftest.py --port COM4 --motor        # 额外转 200 步验证电机
    python u2_selftest.py --port COM4 --baud 460800  # 如果固件改回了 460800

依赖： pyserial（pip install pyserial）

做的事（按顺序）：
    1. PING               —— 链路是否通
    2. GET_VERSION        —— 固件是否在跑、版本对不对
    3. SELF_TEST          —— ADC/对射/VDDA/串口/CRC 的综合位图
    4. MEASURE_ADC(触发)  —— 8 路电压，逐通道打印并标出贴边
    5. SENSOR_READ ×N     —— 对射两路状态（提示你遮挡光路看变化）
    6. TMC_QUERY          —— TMC2209 通信/配置是否正常
    7. MOTOR_MOVE（可选） —— 转 200 步并查到位
"""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    print("缺少 pyserial，请先安装： pip install pyserial")
    sys.exit(2)

# ---------------------------------------------------------------- 协议
HDR = b"\xA5\x5B"
CMD_PING = 0x0001
CMD_VERSION = 0x0002
CMD_ADC = 0x000E
CMD_SENSOR = 0x000F
CMD_MOVE = 0x0010
CMD_STOP = 0x0011
CMD_STATUS = 0x0012
CMD_TMC = 0x0020
CMD_SET_IRUN = 0x0022
CMD_SELFTEST = 0x0030

STATUS = {
    0x00: "OK", 0x01: "BUSY", 0x02: "ERR_SIZE", 0x03: "ERR_FLASH",
    0x04: "ERR_PROG", 0x05: "ERR_TEST", 0x06: "ERR_NO_FW",
    0x07: "ERR_INCOMPLETE", 0x08: "ERR_CRC", 0x09: "ERR_SEQ",
    0x0A: "ERR_COUNT", 0x0B: "ERR_PARAM", 0x0C: "ERR_TIMEOUT",
    0x0D: "ERR_OVERFLOW", 0xFF: "NIY",
}

CH_NAME = ["ADC0/3.3V", "ADC1/3.6V", "ADC2/5V", "ADC3/编码器",
           "ADC4/对射1", "ADC5/对射2", "ADC6/光敏1", "ADC7/光敏2"]


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc & 0xFFFF


def build(cmd: int, seq: int, payload: bytes = b"") -> bytes:
    body = bytes([cmd >> 8, cmd & 0xFF, seq >> 8, seq & 0xFF,
                  len(payload) >> 8, len(payload) & 0xFF]) + payload
    raw = HDR + body
    c = crc16(raw)
    return raw + bytes([c >> 8, c & 0xFF])


def parse_frames(buf: bytes):
    """从字节流里切出所有完整且 CRC 正确的帧"""
    out = []
    i = 0
    while i + 10 <= len(buf):
        if buf[i] != 0xA5 or buf[i + 1] != 0x5B:
            i += 1
            continue
        ln = (buf[i + 6] << 8) | buf[i + 7]
        total = 10 + ln
        if i + total > len(buf):
            break
        raw = buf[i:i + total]
        c = (raw[8 + ln] << 8) | raw[9 + ln]
        if crc16(raw[:8 + ln]) == c:
            out.append({
                "cmd": (raw[2] << 8) | raw[3],
                "seq": (raw[4] << 8) | raw[5],
                "data": raw[8:8 + ln],
            })
            i += total
        else:
            i += 1
    return out


class Link:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.seq = 0

    def close(self):
        self.ser.close()

    def xfer(self, cmd, payload=b"", wait=0.35):
        self.seq = (self.seq % 0xFFFE) + 1
        self.ser.reset_input_buffer()
        self.ser.write(build(cmd, self.seq, payload))
        self.ser.flush()
        buf = b""
        t0 = time.time()
        while time.time() - t0 < wait:
            buf += self.ser.read(256)
            frames = parse_frames(buf)
            for f in frames:
                if f["cmd"] == cmd and f["seq"] == self.seq:
                    return f
        return None


def u16(d, o):
    return (d[o] << 8) | d[o + 1]


def u32(d, o):
    return (d[o] << 24) | (d[o + 1] << 16) | (d[o + 2] << 8) | d[o + 3]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM4")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--motor", action="store_true", help="额外验证电机转动")
    ap.add_argument("--sensor-polls", type=int, default=5)
    args = ap.parse_args()

    results = []

    def record(name, ok, detail=""):
        results.append((name, ok, detail))
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  {detail}" if detail else ""))

    print(f"== M1 U2 自测 ==  {args.port} @ {args.baud} 8N1")
    try:
        link = Link(args.port, args.baud)
    except Exception as e:
        print(f"打不开串口：{e}")
        sys.exit(2)

    try:
        # 1. PING（给 3 次机会，波特率/接线不对时最能说明问题）
        print("\n[1] 链路 PING")
        f = None
        for _ in range(3):
            f = link.xfer(CMD_PING, wait=0.4)
            if f:
                break
        if not f:
            record("PING", False, "无应答 → 检查 H3 接线(TX/RX 对调)、波特率、板子是否上电")
            print("\n中止：链路不通，后面的测试没有意义。")
            return summary(results)
        record("PING", f["data"][0] == 0x00,
               f"状态={STATUS.get(f['data'][0], hex(f['data'][0]))} seq={f['seq']}")

        # 2. 版本
        print("\n[2] 固件版本")
        f = link.xfer(CMD_VERSION)
        if f and len(f["data"]) >= 4 and f["data"][0] == 0x00:
            ver = f"{f['data'][1]}.{f['data'][2]}.{f['data'][3]}"
            record("GET_VERSION", True, f"v{ver}")
        else:
            record("GET_VERSION", False, "应答异常")

        # 3. 自检
        print("\n[3] 综合自检 0x0030")
        f = link.xfer(CMD_SELFTEST, wait=1.0)
        if f and len(f["data"]) >= 6:
            d = f["data"]
            st, mask, vdda, rail, crc_err = d[0], d[1], u16(d, 2), d[4], d[5]
            print(f"      状态={STATUS.get(st, hex(st))}  mask=0x{mask:02X}  "
                  f"VDDA={vdda}mV  贴边掩码=0x{rail:02X}  CRC错={crc_err}")
            bits = [(0, "8路ADC"), (1, "对射1"), (2, "对射2"), (3, "VDDA正常"),
                    (4, "TMC通信+配置"), (5, "电机空闲"), (6, "串口无溢出")]
            for b, name in bits:
                print(f"        bit{b} {name:12s} {'OK' if mask >> b & 1 else 'NG'}")
            record("SELF_TEST", st == 0x00,
                   "全通过" if st == 0x00 else
                   f"未通过（bit4=TMC 未接/未通也属正常，可忽略）")
        else:
            record("SELF_TEST", False, "应答异常")

        # 4. ADC
        print("\n[4] 8 路测试点 ADC（触发一轮）")
        f = link.xfer(CMD_ADC, b"\x01", wait=0.6)
        if f and len(f["data"]) >= 51 and f["data"][0] == 0x00:
            d = f["data"]
            print(f"      回合={u16(d,3)} 耗时={u16(d,5)}ms VDDA={u16(d,7)}mV "
                  f"指纹=0x{u16(d,9):04X}")
            ok_cnt = 0
            for ch in range(8):
                o = 11 + ch * 5
                raw, mv, fl = u16(d, o), u16(d, o + 2), d[o + 4]
                flags = []
                if fl & 0x01: flags.append("有效")
                if fl & 0x02: flags.append("贴边")
                if fl & 0x04: flags.append("对射")
                print(f"      {CH_NAME[ch]:12s} raw={raw:5d}  {mv:5d}mV  " + ",".join(flags))
                if fl & 0x01: ok_cnt += 1
            record("MEASURE_ADC", ok_cnt == 8, f"{ok_cnt}/8 通道有效")
            if u16(d, 7) < 3000 or u16(d, 7) > 3600:
                print("      注意：VDDA 超出 3.0~3.6V，供电可能异常")
        else:
            record("MEASURE_ADC", False, "应答异常")

        # 5. 对射
        print(f"\n[5] 对射状态（{args.sensor_polls} 次采样，可人工遮挡光路观察变化）")
        ok_any = False
        last = None
        for i in range(args.sensor_polls):
            f = link.xfer(CMD_SENSOR, b"\x01", wait=0.5)
            if f and len(f["data"]) >= 20 and f["data"][0] == 0x00:
                d = f["data"]
                s = []
                for ch in range(2):
                    o = 2 + ch * 9
                    raw, mv, blk = u16(d, o), u16(d, o + 2), u16(d, o + 4)
                    rel, blocked = u16(d, o + 6), d[o + 8]
                    s.append(f"ch{ch+1}: {mv:4d}mV 阈值{blk}/{rel} {'遮挡' if blocked else '通光'}")
                line = "   ".join(s)
                if line != last:
                    print(f"      #{i+1}: {line}")
                    last = line
                ok_any = True
            else:
                time.sleep(0.12)
        if ok_any:
            record("SENSOR_READ", True, "两路均有数据（遮挡时应看到“遮挡/通光”翻转）")
        else:
            record("SENSOR_READ", False, "无有效应答")

        # 6. TMC
        print("\n[6] TMC2209 状态")
        f = link.xfer(CMD_TMC, wait=1.0)
        if f and len(f["data"]) >= 17:
            d = f["data"]
            if d[0] == 0x01:
                record("TMC_QUERY", None, "电机忙，稍后再试")
            else:
                alive, cfg = d[1], d[2]
                gconf, chop = u32(d, 3), u32(d, 7)
                drv, sg = u32(d, 11), u16(d, 15)
                print(f"      alive={alive} cfg_ok={cfg} GCONF=0x{gconf:08X} "
                      f"CHOPCONF=0x{chop:08X} DRV_STATUS=0x{drv:08X} SG={sg}")
                record("TMC_QUERY", alive == 1 and cfg == 1,
                       "通信与配置均通过" if alive and cfg else
                       ("通信通但配置读回不一致" if alive else
                        "通信失败 → 查 PA9/PA10 方向与 TMC 供电"))
        else:
            record("TMC_QUERY", False, "应答异常")

        # 7. 电机（可选）
        if args.motor:
            print("\n[7] 电机：CW 200 步 @1000Hz")
            # 先设小电流，避免首次上电电流过大
            f = link.xfer(CMD_SET_IRUN, bytes([12, 6]))
            if f and f["data"][0] == 0x00:
                print("      已设 IRUN/IHOLD = 12/6")
            f = link.xfer(CMD_MOVE, bytes([0, 0, 0, 0, 0xC8, 0x03, 0xE8]))
            if f and f["data"][0] == 0x00:
                t0 = time.time()
                done = 0
                while time.time() - t0 < 3.0:
                    s = link.xfer(CMD_STATUS, wait=0.2)
                    if s and len(s["data"]) >= 10:
                        busy, done = s["data"][1], u32(s["data"], 2)
                        if not busy:
                            break
                    time.sleep(0.05)
                record("MOTOR_MOVE", done >= 200, f"done={done}/200")
            elif f and f["data"][0] == 0x01:
                record("MOTOR_MOVE", False, "BUSY（上一段还没停）")
            else:
                record("MOTOR_MOVE", False, "命令被拒")

    finally:
        link.close()

    return summary(results)


def summary(results):
    print("\n== 汇总 ==")
    for name, ok, detail in results:
        tag = "PASS" if ok is True else ("SKIP" if ok is None else "FAIL")
        print(f"  {tag:4s} {name:14s} {detail}")
    fails = [r for r in results if r[1] is False]
    if fails:
        print(f"\n有 {len(fails)} 项未通过。")
        return 1
    print("\n全部通过。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
