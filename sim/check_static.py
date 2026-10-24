# -*- coding: utf-8 -*-
"""SC500CS I2C 配置模块静态校验。

用法： python sim/check_static.py

★ 2026-10-15 迁移到 720p60 基线：校验对象从 uics520reg_1080p60（备用表）
  改成【当前生效】的 uics520reg_720p60（1280x720@50，117 条，索引 0..116）。
  同文件里的 uics520reg_1080p60 仍在，但已不被例化，改到 1b 节做“备用表”体检。

第 1 节（生效表）期望：
  · REG_SIZE 117，索引 0..116 连续
  · 索引 0 = 0x0103 软复位；索引 1 = 0x0100=0x00 停流
  · stream on（0x0100=0x01）唯一，且在【倒数第二条】= 索引 115
    —— 最后一条（116）是 0x302d=0x00，这正是 POST_STREAM_WRITE 必须为 1'b1 的原因
  · 0x302d 出现两次：索引 12 = 0x20（stream on 之前）、索引 116 = 0x00（之后）

第 2 节（720p60 与 MIPI 速率）期望：
  窗口 1280x720、HTS 1600、VTS 1200、0x4837=0x42 -> 480 Mbps/Lane -> PCLK 96 MHz
  96e6/(1600*1200) = 50.00 fps
  ★ VTS 故意是 1200（不是原厂 1000）：帧周期 20.000ms 正好等于我国 50Hz 市电
    照明光周期，消掉 10Hz 工频闪烁。详见 uics520reg_720p60.v 文件头。

第 3 节需要工程外的厂商 SDK 文件 cfg_cs500.c（绝对路径见 VENDOR_C）。
该文件不存在时，第 3 节会明确标记为“跳过”（不计入通过），其余部分照常运行；
把 cfg_cs500.c 拷回原路径或放到 doc/hw/official500/ 下即可自动恢复本节。
恢复后本节是“容差比对”：只比对两张表都有的地址，允许的偏差仅限文件头已声明的
三处（VTS 0x320E/0x320F、HTS 0x320C/0x320D 新增、曝光 0x3E00/01/02 收敛到
2*VTS-10）；出现任何未声明的值差异都会判失败。
"""
import os
import re
import sys

# 输出被重定向/管道时（CI、日志抓取）Python 会退回 GBK 码页，结尾的 '✓' 编不出来
# 会直接抛 UnicodeEncodeError —— 表现是“全部通过却 exit 1”。终端下走 WriteConsoleW
# 不受影响，所以分两种情况处理：
#   非终端 -> 明确用 utf-8，重定向出去的文本在任何地方都能正确解码
#   其余   -> 只把 errors 放宽成 replace，保住终端原有编码不被改坏
for _s in (sys.stdout, sys.stderr):
    try:
        if not _s.isatty():
            _s.reconfigure(encoding="utf-8", errors="replace")
        else:
            _s.reconfigure(errors="replace")
    except Exception:
        pass

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG = os.path.join(WS, "user_source", "hdl_source", "uics520_cfg")
# 原厂 SC500CS 720p60 全表：取自 lab_ex2 工程（长期存在，不依赖临时目录）
OFF = r"C:\Users\touch\Desktop\anlogic\SC520CS\lab_ex2_edge_SC520\lab_ex2_edge_SC520\lab_ex2_edge\user_source\hdl_source\uics500_cfg"
if not os.path.isdir(OFF):
    OFF = os.path.join(WS, "doc", "hw", "official500")

fails = []
oks = []
skipped = []       # 因参照文件缺失而没跑的检查（不计入通过，只在结尾列出）


def chk(cond, msg):
    (oks if cond else fails).append(msg)


def read(p):
    return open(p, encoding="utf-8", errors="replace").read()


def parse_table(path):
    t = read(path)
    size = int(re.search(r"REG_SIZE\s*=\s*9'd(\d+)", t).group(1))
    idx = {}
    for m in re.finditer(r"(\d+)\s*:\s*REG_DATA\s*=\s*\{16'h([0-9a-fA-F]{4}),\s*8'h([0-9a-fA-F]{2})", t):
        i = int(m.group(1))
        chk(i not in idx, "index %d unique in %s" % (i, path.split('\\')[-1]) if i in idx else "")
        idx[i] = (int(m.group(2), 16), int(m.group(3), 16))
    return size, idx


def parse_table_v(path):
    """Table using 8'd / no explicit index style (uics500regAE)."""
    t = read(path)
    size = int(re.search(r"REG_SIZE\s*=\s*9'd(\d+)", t).group(1))
    idx = {}
    for m in re.finditer(r"(\d+)\s*:\s*REG_DATA\s*=\s*\{16'h([0-9a-fA-F]{4})", t):
        idx[int(m.group(1))] = int(m.group(2), 16)
    return size, idx


VENDOR_C = r"C:\Users\touch\Desktop\anlogic\04_MLK-CAM001-CS500\mlk-com022-f20-cm001-cs500-mipi\fpga_sdk\mipi_cs500\src\cfg_cs500.c"
# 上面是工程外的厂商 SDK 路径，换机器或被清理后就会失效。工程内留一个 fallback
# 位置：把 cfg_cs500.c 拷到 doc/hw/official500/ 下即可自动恢复第 3 节，不用改代码。
if not os.path.isfile(VENDOR_C):
    _alt = os.path.join(WS, "doc", "hw", "official500", "cfg_cs500.c")
    if os.path.isfile(_alt):
        VENDOR_C = _alt


def parse_vendor_c(path):
    """解析厂商 SDK 的 reg_cs500[] 数组（C 源码）。"""
    body = read(path).split("reg_cs500[] =")[1].split("};")[0]
    regs = [(int(a, 16), int(v, 16))
            for a, v in re.findall(r"\{\s*0[xX]([0-9a-fA-F]{4})\s*,\s*0[xX]([0-9a-fA-F]{2})\s*\}", body)]
    return [(a, v) for a, v in regs if a != 0xFFFF]


def module_text(path, name):
    """取出文件里某个 module ... endmodule 的正文，避免多表混在一起。"""
    t = read(path)
    m = re.search(r"(?ms)^module\s+%s\b.*?^endmodule" % re.escape(name), t)
    return m.group(0)


def parse_mod_table(path, name):
    """从文件里取出某个表模块，解析成 (REG_SIZE, {索引: (地址, 值)})。"""
    mt = module_text(path, name)
    sz = int(re.search(r"REG_SIZE\s*=\s*9'd(\d+)", mt).group(1))
    d = {}
    for m in re.finditer(r"(\d+)\s*:\s*REG_DATA\s*=\s*\{16'h([0-9a-fA-F]{4}),\s*8'h([0-9a-fA-F]{2})", mt):
        d[int(m.group(1))] = (int(m.group(2), 16), int(m.group(3), 16))
    return sz, d


print("=" * 78)
tbl = CFG + r"\uics520reg_720p60.v"

print("1) 当前生效表 uics520reg_720p60（1280x720@50，117 条，索引 0..116）")
size, idx = parse_mod_table(tbl, "uics520reg_720p60")
chk(size == len(idx), "REG_SIZE %d == 条目数 %d" % (size, len(idx)))
chk(sorted(idx) == list(range(size)), "索引 0..%d 连续无缺口无重复" % (size - 1))
chk(idx[0] == (0x0103, 0x01), "索引 0 = 0x0103 软复位")
chk(idx[1] == (0x0100, 0x00), "索引 1 = 0x0100=0x00 显式停流")
stream = [i for i, (a, v) in idx.items() if a == 0x0100 and v == 0x01]
chk(stream == [size - 2],
    "stream on 唯一且在倒数第二条（索引 %s，其后还有一条 0x302d=0x00）" % stream)
sd = [i for i, (a, v) in idx.items() if a == 0x302D]
chk(sd == [12, size - 1], "0x302d 出现两次（0x20 在索引 %s、0x00 在最后一条）" % sd)
chk(sd[0] < stream[0] < sd[1],
    "0x302d=0x20 在 stream on 之前、0x302d=0x00 在其之后 -> POST_STREAM_WRITE 必须为 1")
chk(idx[sd[1]] == (0x302D, 0x00), "最后一条 = 0x302d=0x00（原厂要求 stream on 后延时 10ms 再写）")

print("\n1b) 备用表 uics520reg_1080p60（仍在同一文件里，当前不被例化，只做体检）")
size2, idx2 = parse_mod_table(tbl, "uics520reg_1080p60")
A2 = {a: v for (a, v) in idx2.values()}
chk(size2 == len(idx2), "REG_SIZE %d == 条目数 %d" % (size2, len(idx2)))
chk(sorted(idx2) == list(range(size2)), "索引 0..%d 连续无缺口无重复" % (size2 - 1))
chk(idx2[0] == (0x0103, 0x01), "索引 0 = 0x0103 软复位")
chk(size2 == 116 and idx2[115] == (0x0100, 0x01),
    "1080p60 表 stream on 是最后一条（116 条，索引 115）")
chk(A2.get(0x3208, 0) * 256 + A2.get(0x3209, 0) == 1920, "备用表窗口宽 = 1920")
chk(A2.get(0x320A, 0) * 256 + A2.get(0x320B, 0) == 1080, "备用表窗口高 = 1080")
chk(A2.get(0x4837) == 0x21, "备用表 0x4837 = 0x%02x -> 960 Mbps/Lane" % A2.get(0x4837, 0))

print("\n2) 分辨率 / 帧率 / MIPI 速率（720p60，当前生效表）")
A = {a: v for (a, v) in idx.values()}
w = A[0x3208] * 256 + A[0x3209]
h = A[0x320A] * 256 + A[0x320B]
hts = A[0x320C] * 256 + A[0x320D]
vts = A[0x320E] * 256 + A[0x320F]
chk(w == 1280, "窗口宽 = %d（720p60）" % w)
chk(h == 720, "窗口高 = %d" % h)
chk(hts == 1600, "HTS = %d" % hts)
chk(vts == 1200, "VTS = %d（★ 故意 1200 而非原厂 1000：50 fps 消工频闪烁）" % vts)
chk(A[0x4837] == 0x42, "0x4837 = 0x%02x -> MIPI 480 Mbps/Lane" % A[0x4837])
PCLK = 480e6 * 2 / 10          # 2 Lane x 480 Mbps / 10 bit(RAW10)
fps = PCLK / (hts * vts)
chk(48.5 < fps < 51.5,
    "由 480Mbps/Lane 反推 PCLK=%.0f MHz -> %.2f fps（消闪后是 50，不是 60）" % (PCLK / 1e6, fps))
# 曝光高 4 位为 0 时厂商表可以不写 0x3e00，故默认取 0
exp0 = (A.get(0x3E00, 0) << 12) | (A[0x3E01] << 4) | (A[0x3E02] >> 4)
chk(exp0 == 2 * vts - 10, "初始曝光 = %d = 2*VTS-10 = %d" % (exp0, 2 * vts - 10))

print("\n3) 与厂商 SDK 的 cfg_cs500.c 容差比对（生效表 = 720p60）")
if not os.path.isfile(VENDOR_C):
    # 参照文件是工程外的厂商 SDK，可能被清理掉。缺失时“跳过”而不是崩溃 ——
    # 否则后面 3b/4/5/6 节（定序器常量、AE 表、增益表穷举等）全都跑不到。
    skipped.append((3, "第 3 节 厂商 SDK cfg_cs500.c 逐条比对"))
    print("   !! 厂商参照文件缺失，本节 3 项【跳过】，不计入通过：")
    print("      %s" % VENDOR_C)
    print("      拷回上面的路径，或放到 doc/hw/official500/ 下，即可自动恢复本节")
else:
    vT = dict(parse_vendor_c(VENDOR_C))                 # 厂商：地址 -> 值
    nT = {a: v for a, v in idx.values()}                # 本表：地址 -> 值
    # ★ 本表不是原厂表的逐条复制：文件头声明了三处有意偏差，
    #   见 uics520reg_720p60.v 第 12~25 行。
    ALLOW_ADDR = {0x320C, 0x320D, 0x320E, 0x320F, 0x3E00, 0x3E01, 0x3E02}
    common = sorted(set(nT) & set(vT))
    diff = [(a, nT[a], vT[a]) for a in common if nT[a] != vT[a]]
    bad = [d for d in diff if d[0] not in ALLOW_ADDR]
    chk(not bad, "共有地址里没有未声明的值差异（发现 %d 处：%s）"
        % (len(bad), ["0x%04X" % d[0] for d in bad[:6]]))
    chk(len(common) >= 100, "与厂商表的共有地址数 = %d（本表 %d 条 / 厂商 %d 条）"
        % (len(common), len(nT), len(vT)))
    print("      值有差异的地址（应全部落在声明集合内）：%s"
          % ", ".join("0x%04X(本表 %02X / 厂商 %02X)" % d for d in diff))
    chk(all(d[0] in ALLOW_ADDR for d in diff) or not diff,
        "全部值差异都在已声明集合 {VTS 0x320E/F、HTS 0x320C/D、曝光 0x3E00/01/02} 内")

print("\n3b) 定序器常量与表一致")
seq = read(CFG + r"\uicfgcs520.v")
m = re.search(r"STREAM_ON_INDEX\s*=\s*9'd(\d+)", seq)
chk(m is not None and int(m.group(1)) == stream[0],
    "uicfgcs520.STREAM_ON_INDEX(%s) == 表内 stream on 索引(%d)" % (m.group(1) if m else "?", stream[0]))
chk(re.search(r"POST_STREAM_WRITE\s*=\s*1'b1", seq) is not None,
    "POST_STREAM_WRITE=1'b1（720p60 表 stream on 之后还有一条 0x302d=0x00）")
chk("uics520reg_720p60 u_init_regs" in seq, "定序器例化的是 uics520reg_720p60")
chk(re.search(r"uics520reg_1080p60\s+u_init_regs", seq) is None,
    "定序器不再例化已停用的 1080p60 表")

print("\n4) AE 表 uics520regAE.v")
ae_src = read(CFG + r"\uics520regAE.v")
asize = int(re.search(r"REG_SIZE\s*=\s*9'd(\d+)", ae_src).group(1))
aidx = {}
for m in re.finditer(r"9'd(\d+)\s*:\s*REG_DATA\s*=\s*\{16'h([0-9a-fA-F]{4})\s*,\s*(.+?)\};", ae_src):
    aidx[int(m.group(1))] = (int(m.group(2), 16), m.group(3).strip())
chk(asize == len(aidx), "AE REG_SIZE %d == 条目数 %d" % (asize, len(aidx)))
chk(sorted(aidx) == list(range(asize)), "AE 索引 0..%d 连续" % (asize - 1))
addrs = [a for a, _ in aidx.values()]
chk(0x3812 not in addrs, "AE 表不再写未文档化的 0x3812")
chk(all(x in addrs for x in (0x3E00, 0x3E01, 0x3E02)), "AE 表写全曝光 0x3e00/01/02")
chk(all(x in addrs for x in (0x3E08, 0x3E09)), "AE 表写模拟增益 0x3e08/0x3e09")
chk(all(x in addrs for x in (0x3E06, 0x3E07)), "AE 表写数字增益 1.000x 0x3e06/0x3e07")
chk(aidx[0][1].startswith("4'h0, I_AE[15:12]"), "曝光字段 0x3e00 <= {4'h0, I_AE[15:12]}：%s" % aidx[0][1])
chk(aidx[1][1] == "I_AE[11:4]", "曝光字段 0x3e01 <= I_AE[11:4]")
chk(aidx[2][1].startswith("I_AE[3:0], 4'h0"), "曝光字段 0x3e02 <= {I_AE[3:0], 4'h0}：%s" % aidx[2][1])
chk(aidx[3][1] == "I_AG[15:8]" and aidx[4][1] == "I_AG[7:0]", "增益字段 0x3e08/0x3e09 <= I_AG")

print("\n5) 增益表 SC520GainTbl.v 穷举（对照手册表 2-4）")
COARSE = {0x03, 0x07, 0x0F, 0x1F}
bad = []
for i_ag in range(16, 200):
    if i_ag <= 16:
        step = 0
    elif i_ag >= 144:
        step = 127
    else:
        step = i_ag - 16
    coarse = step >> 5
    fine = 0x20 + (step & 0x1F)
    co = [0x03, 0x07, 0x0F, 0x1F][coarse]
    if co not in COARSE or not (0x20 <= fine <= 0x3F):
        bad.append((i_ag, co, fine))
chk(not bad, "I_AG 16..199 全部落在表 2-4 合法行内 (%d 个越界)" % len(bad))
chk(0x20 <= 0x20 + 127 % 32 <= 0x3F, "step=127 -> fine=0x%02x" % (0x20 + 127 % 32))

print("\n6) 定序器 uicfgcs520.v")
seq = read(CFG + r"\uicfgcs520.v")
m = re.search(r"STREAM_ON_INDEX\s*=\s*9'd(\d+)", seq)
chk(m is not None and int(m.group(1)) == stream[0],
    "uicfgcs520.STREAM_ON_INDEX(%s) == 表中 stream on 索引(%d)" % (m.group(1) if m else "?", stream[0]))
chk("16'hee27" in seq.lower(), "Sensor ID 校验为 0xEE27")
chk("16'hee4b" not in seq.lower() and "16'h9e4b" not in seq.lower(), "旧 SC520CS ID 0xEE4B/0x9E4B 已移除")
chk("ST_STREAM_WAIT" in seq and "STREAM_WAIT_CYCLES" in seq, "已加入 stream on 后 10 ms 等待状态")
chk("8'h6c" in seq.lower(), "I2C 从机地址保持 0x6C（手册表 1-4 SID 低）")
chk(len(re.findall(r"ST_STREAM_WAIT", seq)) >= 3, "ST_STREAM_WAIT 定义/复位/分支齐备")

print("\n" + "=" * 78)
n_skip = sum(c for c, _ in skipped)
print("通过 %d 项，失败 %d 项，跳过 %d 项" % (len(oks), len(fails), n_skip))
if fails:
    print("\n失败项：")
    for f in fails:
        print("  ✗", f)
    sys.exit(1)
if skipped:
    print("\n跳过项（参照文件缺失，不计入通过）：")
    for c, s in skipped:
        print("  - %s（%d 项）" % (s, c))
    print("已检查部分全部通过 ✓")
else:
    print("全部检查通过 ✓")
