#!/usr/bin/env python3
"""D2XX 直连的 Modbus-RTU 客户端（不依赖 COM 口）。

为什么需要这个
--------------
2026-10-26 现场：USB-485 转接线插上后，Windows 只枚举出
`USB Serial Converter (VID_0403&PID_6001)`，**没有生成 COM 口**
（注册表 SERIALCOMM 为空，pyserial 列 0 个串口）。原因是 VCP 串口
驱动没绑定，只有 D2XX 层可用。

与其装驱动，不如直接用 FTDI 的 D2XX API 收发字节，自己拼 Modbus 帧。
对外接口做成和 pymodbus 的 `read_holding_registers()` 一样，
这样 torque485.py / sensor_rec.py 等现有代码只要换掉 make_client 即可。

用法
----
    from d2xx_modbus import D2xxClient
    c = D2xxClient()                 # 默认取第一颗 FTDI
    c.connect()
    r = c.read_holding_registers(0x54, count=3, slave=1)
    print(r.registers)
    c.close()
"""

import struct
import time


def crc16(data: bytes) -> int:
    """Modbus CRC16（多项式 0xA001，初值 0xFFFF）。"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def build_read(slave: int, addr: int, count: int) -> bytes:
    body = struct.pack(">BBHH", slave, 0x03, addr, count)
    return body + struct.pack("<H", crc16(body))


class FakeResp:
    """模仿 pymodbus 的响应对象，只为兼容 is_err()/registers 两种用法。"""

    def __init__(self, registers=None, error=None):
        self.registers = registers or []
        self._error = error

    def isError(self):
        return self._error is not None

    def __repr__(self):
        return f"FakeResp(registers={self.registers}, error={self._error})"


class D2xxClient:
    def __init__(self, serial_no=None, baud=115200, timeout=0.3, verbose=False):
        self.serial_no = serial_no
        self.baud = baud
        self.timeout = timeout
        self.verbose = verbose
        self.h = None

    # ---- 连接 ----
    def connect(self):
        try:
            import ftd2xx as ftd
        except ImportError:
            raise RuntimeError("缺 ftd2xx：pip install ftd2xx")
        self._ftd = ftd
        devs = ftd.listDevices()
        if not devs:
            raise RuntimeError("D2XX 没找到任何 FTDI 设备（线没插好？）")
        if self.serial_no:
            self.h = ftd.openEx(self.serial_no.encode())
        else:
            self.h = ftd.open(0)
        # ftd2xx 这个包没有把常量导出到模块顶层，用 D2XX 手册的数值兜底：
        #   BITS_8=8, STOP_BITS_1=0, PARITY_NONE=0, FLOW_NONE=0, PURGE_RX=1, PURGE_TX=2
        g = lambda n, d: getattr(ftd, n, d)
        self.h.setBaudRate(self.baud)
        self.h.setDataCharacteristics(g("BITS_8", 8), g("STOP_BITS_1", 0),
                                      g("PARITY_NONE", 0))
        self.h.setFlowControl(g("FLOW_NONE", 0), 0, 0)
        self.h.setTimeouts(int(self.timeout * 1000), int(self.timeout * 1000))
        self.h.purge(g("PURGE_RX", 1) | g("PURGE_TX", 2))
        return True

    def close(self):
        if self.h is not None:
            try:
                self.h.close()
            except Exception:
                pass
            self.h = None

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *exc):
        self.close()

    # ---- 原始收发 ----
    def _write(self, data):
        self.h.write(data)

    def _read_exact(self, n, deadline):
        """读满 n 字节，超时返回已读到的部分。"""
        buf = b""
        while len(buf) < n and time.time() < deadline:
            avail = self.h.getQueueStatus()
            if avail <= 0:
                time.sleep(0.002)
                continue
            chunk = self.h.read(min(avail, n - len(buf)))
            if chunk:
                buf += chunk
        return buf

    # ---- Modbus 功能码 0x03 ----
    def read_holding_registers(self, addr, count=1, slave=None, **kw):
        # 兼容 pymodbus 的 slave / unit / device_id 三种写法
        sid = slave if slave is not None else kw.get("device_id", kw.get("unit", 1))
        if self.h is None:
            self.connect()
        ftd = self._ftd
        self.h.purge(getattr(ftd, "PURGE_RX", 1))
        self._write(build_read(sid, addr, count))
        deadline = time.time() + self.timeout
        # 先读头两字节（地址 + 功能码），再按需读剩余
        head = self._read_exact(2, deadline)
        if len(head) < 2:
            return FakeResp(error="timeout")
        if head[0] != sid:
            return FakeResp(error=f"addr mismatch {head[0]}")
        if head[1] & 0x80:
            tail = self._read_exact(3, deadline)   # 异常码 + CRC
            return FakeResp(error=f"exception code={tail[0] if tail else '?'}")
        if head[1] != 0x03:
            return FakeResp(error=f"func {head[1]}")
        nbytes_b = self._read_exact(1, deadline)
        if not nbytes_b:
            return FakeResp(error="no bytecount")
        nbytes = nbytes_b[0]
        rest = self._read_exact(nbytes + 2, deadline)
        if len(rest) < nbytes + 2:
            return FakeResp(error="short frame")
        payload, crc_b = rest[:nbytes], rest[nbytes:]
        frame = head + nbytes_b + payload
        if crc16(frame) != struct.unpack("<H", crc_b)[0]:
            return FakeResp(error="crc")
        regs = list(struct.unpack(">" + "H" * (nbytes // 2), payload))
        return FakeResp(registers=regs)


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="D2XX 直连 Modbus 自检")
    ap.add_argument("--serial", default=None, help="FTDI 序列号，默认第一颗")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--addr", type=int, default=1, help="从站地址")
    ap.add_argument("--slave-reg", type=int, default=0x54, help="起始寄存器")
    ap.add_argument("--count", type=int, default=3)
    a = ap.parse_args()
    with D2xxClient(a.serial, a.baud) as c:
        for _ in range(3):
            r = c.read_holding_registers(a.slave_reg, a.count, a.addr)
            print(("ERR " + r._error) if r.isError() else r.registers)
            time.sleep(0.2)
