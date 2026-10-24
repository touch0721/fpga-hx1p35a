# -*- coding: utf-8 -*-
"""校验 sim/tb_run.log（tb_uicfgcs520 仿真输出）与 SC500CS 期望序列是否一致。

用法：
    python sim/check_i2c_seq.py [tb_run.log 路径]
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
CFG = os.path.join(ROOT, "user_source", "hdl_source", "uics520_cfg")
LOG = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "tb_run.log")

fails, oks = [], []


def chk(cond, msg):
    (oks if cond else fails).append(msg)


def read(p):
    """读取仿真日志/源文件，兼容 UTF-8 / UTF-16(LE|BE，含无 BOM) / GBK。"""
    b = open(p, "rb").read()
    if b[:2] in (b"\xff\xfe", b"\xfe\xff"):
        return b.decode("utf-16")
    if b[:3] == b"\xef\xbb\xbf":
        return b.decode("utf-8-sig")
    # PowerShell 重定向可能产生无 BOM 的 UTF-16LE：大量隔字节 0x00
    if b.count(0) > len(b) // 4:
        try:
            return b.decode("utf-16-le")
        except UnicodeDecodeError:
            pass
    for enc in ("utf-8", "gbk"):
        try:
            return b.decode(enc)
        except UnicodeDecodeError:
            continue
    return b.decode("latin-1")


# ---------- 期望表 ----------
_full = read(os.path.join(CFG, "uics520reg_720p60.v"))
# ★ 2026-10-15 迁移到 720p60 基线：当前生效的是 uics520reg_720p60
#   （uics520reg_1080p60 仍在同文件里，但已不被例化，只作备用表）
_m = re.search(r"(?ms)^module\s+uics520reg_720p60\b.*?^endmodule", _full)
assert _m, "未找到 uics520reg_720p60 模块"
t = _m.group(0)
size = int(re.search(r"REG_SIZE\s*=\s*9'd(\d+)", t).group(1))
stream_idx = int(re.search(r"STREAM_ON_INDEX\s*=\s*9'd(\d+)", t).group(1))
seq = {}
for m in re.finditer(r"(\d+)\s*:\s*REG_DATA\s*=\s*\{16'h([0-9a-fA-F]{4}),\s*8'h([0-9a-fA-F]{2})", t):
    seq[int(m.group(1))] = (int(m.group(2), 16), int(m.group(3), 16))
expected = [seq[i] for i in range(size)]

# ---------- 仿真日志 ----------
log = read(LOG)
writes = [(int(a, 16), int(d, 16), int(tm), int(dt))
          for a, d, tm, dt in re.findall(
              r"WR idx=(\d+) addr=([0-9a-fA-Fx]+) data=([0-9a-fA-Fx]+) t=(\d+) dt=(\d+)", log)
          ] if False else []
writes = []
for m in re.finditer(r"WR idx=(\d+) addr=([0-9a-fA-F]+) data=([0-9a-fA-F]+) t=(\d+) dt=(\d+)", log):
    writes.append((int(m.group(2), 16), int(m.group(3), 16), int(m.group(4)), int(m.group(5))))
reads = [(int(a, 16), int(d, 16))
         for a, d in re.findall(r"RD idx=\d+ addr=([0-9a-fA-F]+) data=([0-9a-fA-F]+)", log)]

m = re.search(r"CFG_DONE t=\d+ sensor_id=([0-9a-fA-F]+) id_ok=(\d) iic_error=(\d) TOTAL_WRITES=(\d+)", log)
chk(m is not None, "日志中存在 CFG_DONE 行")
if m:
    chk(m.group(1).lower() == "ee27", "Sensor ID = 0x%s（期望 ee27）" % m.group(1))
    chk(m.group(2) == "1", "O_sensor_id_ok = 1")
    chk(m.group(3) == "0", "O_iic_error = 0（无 NACK）")
    chk(int(m.group(4)) == size, "配置写入条数 %s == REG_SIZE %d" % (m.group(4), size))

chk(reads[:2] == [(0x3107, 0xEE), (0x3108, 0x27)],
    "前两次读为 0x3107=0xEE、0x3108=0x27：%s" % reads[:2])

chk(len(writes) >= size, "写序列长度 %d >= %d" % (len(writes), size))
cfg_w = [w[:2] for w in writes[:size]]
chk(cfg_w == expected, "初始化表 %d 条与 uics520reg_720p60.v 逐条一致" % size)
if cfg_w != expected:
    for i, (g, e) in enumerate(zip(cfg_w, expected)):
        if g != e:
            print("  首个不一致 idx=%d 实测 %04x=%02x 期望 %04x=%02x" % (i, g[0], g[1], e[0], e[1]))
            break

# 等待时间：$display 的 %0t 以仿真精度（1 ps）为单位
RESET_WAIT_PS = 5_000_000_000       # 5 ms
STREAM_WAIT_PS = 10_000_000_000     # 10 ms
chk(abs(writes[1][3] - RESET_WAIT_PS) < 200_000_000,
    "软复位后等待 %.3f ms（期望 5 ms）" % (writes[1][3] / 1e9))
chk(writes[stream_idx][:2] == (0x0100, 0x01), "stream on 位于索引 %d" % stream_idx)
chk(stream_idx == size - 2,
    "stream on 是【倒数第二条】（索引 %d），最后一条是后续写 -> POST_STREAM_WRITE 必须为 1" % stream_idx)
chk(writes[size - 1][:2] == (0x302D, 0x00),
    "最后一条 = 0x302d=0x00（原厂要求 stream on 后延时 10 ms 再写）")
chk(abs(writes[size - 1][3] - STREAM_WAIT_PS) < 200_000_000,
    "stream on 之后等待 %.3f ms（期望 10 ms）" % (writes[size - 1][3] / 1e9))
# 0x302d 在 720p60 表里出现两次：前部 0x20（索引 12）、最后一条 0x00
_sd = [i for i, w in enumerate(writes[:size]) if w[0] == 0x302D]
chk(_sd == [12, size - 1], "0x302d 位于索引 %s（0x20 前部 / 0x00 最后一条）" % _sd)

# 分辨率 / 帧率不变式（720p60：1280x720、480 Mbps/Lane、96 MHz PCLK -> 50 fps 消工频闪）
A = {a: v for a, v in expected}
chk(A[0x3208] * 256 + A[0x3209] == 1280, "窗口宽 = 1280（720p60）")
chk(A[0x320A] * 256 + A[0x320B] == 720, "窗口高 = 720")
hts = A[0x320C] * 256 + A[0x320D]
vts = A[0x320E] * 256 + A[0x320F]
chk(hts == 1600, "HTS = %d" % hts)
chk(vts == 1200, "VTS = %d（★ 故意 1200 而非原厂 1000：50 fps 消工频闪烁）" % vts)
chk(A[0x4837] == 0x42, "0x4837 = 0x%02x -> MIPI 480 Mbps/Lane" % A[0x4837])
PCLK = 480e6 * 2 / 10
chk(48.5 < PCLK / (hts * vts) < 51.5,
    "PCLK %.0f MHz -> %.2f fps" % (PCLK / 1e6, PCLK / (hts * vts)))

# AE 序列（TB 驱动 I_AE = 2390 = 720p60 的 VTS=1200 下最大曝光 2*VTS-10）
ae = writes[size:]
exp_ae = [(0x3E00, 0x00), (0x3E01, 0x95), (0x3E02, 0x60),
          (0x3E08, 0x1F), (0x3E09, 0x3F), (0x3E06, 0x00), (0x3E07, 0x80)]
chk([w[:2] for w in ae] == exp_ae, "AE 序列 = %s" % exp_ae)
if ae:
    exposure = (ae[0][1] << 12) | (ae[1][1] << 4) | (ae[2][1] >> 4)
    chk(exposure == 2 * vts - 10, "曝光 = %d = 2*VTS-10 = %d" % (exposure, 2 * vts - 10))
    chk(0x20 <= ae[4][1] <= 0x3F, "ANA FINE GAIN 0x%02x 落在 0x20~0x3f" % ae[4][1])
    chk(ae[3][1] in (0x03, 0x07, 0x0F, 0x1F), "ANA GAIN 0x%02x 为表 2-4 合法粗调" % ae[3][1])

print("=" * 70)
print("仿真日志：%s" % LOG)
print("配置写入 %d 条 + AE 写入 %d 条" % (size, len(ae)))
print("通过 %d 项，失败 %d 项" % (len(oks), len(fails)))
if fails:
    print("\n失败项：")
    for f in fails:
        print("  x", f)
    sys.exit(1)
print("全部检查通过")
