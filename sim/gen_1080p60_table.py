# -*- coding: utf-8 -*-
"""
gen_1080p60_table.py — 从厂商 SDK 的 cfg_cs500.c 生成 1080p60 寄存器表模块，
追加到 uics520reg_720p60.v（保留原 720p60 模块以便回退），并同步 uicfgcs520.v。

厂商表结构：116 条，索引 0 = 软复位，索引 115 = stream on（最后一条），无 post-stream 写入。
"""
import os
import re
import sys

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG = os.path.join(WS, "user_source", "hdl_source", "uics520_cfg")
TBL = os.path.join(CFG, "uics520reg_720p60.v")
SEQ = os.path.join(CFG, "uicfgcs520.v")
SRC = r"C:\Users\touch\Desktop\anlogic\04_MLK-CAM001-CS500\mlk-com022-f20-cm001-cs500-mipi\fpga_sdk\mipi_cs500\src\cfg_cs500.c"

# ---------------- 1) 解析厂商 C 表 ----------------
txt = open(SRC, encoding="utf-8", errors="replace").read()
body = txt.split("reg_cs500[] =")[1].split("};")[0]
regs = [(int(a, 16), int(v, 16))
        for a, v in re.findall(r"\{\s*0[xX]([0-9a-fA-F]{4})\s*,\s*0[xX]([0-9a-fA-F]{2})\s*\}", body)]
# 末尾的 0xFFFF 是 C 代码的结束标记，不是寄存器
regs = [(a, v) for a, v in regs if a != 0xFFFF]
print("厂商表解析出 %d 条" % len(regs))

# 校验结构
assert regs[0] == (0x0103, 0x01), "索引 0 不是软复位"
so = [i for i, (a, v) in enumerate(regs) if a == 0x0100 and v == 0x01]
assert so == [len(regs) - 1], "stream on 不在最后一条：%s" % so
print("  stream on 在索引 %d（最后一条）" % so[0])
print("  0x3208/09 = 0x%02x%02x, 0x320a/0b = 0x%02x%02x, VTS 0x%02x%02x, HTS 0x%02x%02x" % (
    regs[[a for a, _ in regs].index(0x3208)][1], regs[[a for a, _ in regs].index(0x3209)][1],
    regs[[a for a, _ in regs].index(0x320A)][1], regs[[a for a, _ in regs].index(0x320B)][1],
    regs[[a for a, _ in regs].index(0x320E)][1], regs[[a for a, _ in regs].index(0x320F)][1],
    regs[[a for a, _ in regs].index(0x320C)][1], regs[[a for a, _ in regs].index(0x320D)][1]))

# 曝光初值改成手册表 2-2 的上限 2*VTS-10（厂商写的是 2246，比上限大 2）
VTS = (regs[[a for a, _ in regs].index(0x320E)][1] << 8) | regs[[a for a, _ in regs].index(0x320F)][1]
EXP_MAX = 2 * VTS - 10
def rep(addr, val):
    i = [k for k, (a, _) in enumerate(regs) if a == addr]
    assert len(i) == 1, "0x%04x 出现 %d 次" % (addr, len(i))
    old = regs[i[0]][1]
    regs[i[0]] = (addr, val)
    return old
o1 = rep(0x3E01, (EXP_MAX >> 4) & 0xFF)
o2 = rep(0x3E02, (EXP_MAX & 0x0F) << 4)
print("  VTS=%d -> 曝光上限 %d（厂商 0x%02x/0x%02x -> 本表 0x%02x/0x%02x）"
      % (VTS, EXP_MAX, o1, o2, (EXP_MAX >> 4) & 0xFF, (EXP_MAX & 0x0F) << 4))

# ---------------- 2) 生成 Verilog 模块 ----------------
L = []
L.append("")
L.append("// ============================================================================")
L.append("//  SC500CS 原厂 1080p60 初始化表（1920x1080 @ 60fps、2-Lane RAW10、960 Mbps/Lane）")
L.append("// ----------------------------------------------------------------------------")
L.append("//  来源：厂商 SDK 04_MLK-CAM001-CS500/.../fpga_sdk/mipi_cs500/src/cfg_cs500.c")
L.append("//        的 reg_cs500[] 数组，由 sim/gen_1080p60_table.py 自动解析生成，逐条一致。")
L.append("//")
L.append("//  关键值：")
L.append("//    0x3208/0x3209 = 0x0780 -> 宽 1920")
L.append("//    0x320a/0x320b = 0x0438 -> 高 1080")
L.append("//    0x320c/0x320d = 0x0b24 -> HTS 2852")
L.append("//    0x320e/0x320f = 0x0467 -> VTS 1127 -> 60 fps（PCLK 192 MHz）")
L.append("//    0x4837        = 0x21   -> MIPI 960 Mbps/Lane（720p60 表是 0x42=480Mbps）")
L.append("//    0x3e01/0x3e02         -> 曝光初值 = 2*VTS-10 = %d（厂商原值 2246，超上限 2，已收敛到上限）" % EXP_MAX)
L.append("//")
L.append("//  序列结构（供 uicfgcs520 状态机识别）：")
L.append("//    索引 0            ：软复位 0x0103=0x01，之后等待 5 ms")
L.append("//    索引 %d           ：stream on 0x0100=0x01（**最后一条**，之后无写入）" % (len(regs) - 1))
L.append("//    0x302d=0x20 在前部（索引 %d），0x302d=0x00 在索引 %d" %
         ([a for a, _ in regs].index(0x302D), len(regs) - 1 - [a for a, _ in reversed(regs)].index(0x302D)))
L.append("// ============================================================================")
L.append("module uics520reg_1080p60 (")
L.append("    input  wire [8:0]  REG_INDEX,")
L.append("    output reg  [23:0] REG_DATA,")
L.append("    output wire [8:0]  REG_SIZE")
L.append(");")
L.append("    // stream on 所在索引，uicfgcs520 依据它决定写完最后一条即配置完成")
L.append("    localparam [8:0] STREAM_ON_INDEX = 9'd%d;" % (len(regs) - 1))
L.append("")
L.append("    assign REG_SIZE = 9'd%d;" % len(regs))
L.append("")
L.append("    always @(*) begin")
L.append("        case (REG_INDEX)")
for i, (a, v) in enumerate(regs):
    tag = ""
    if i == 0:
        tag = " // 软复位"
    elif a == 0x0100 and v == 0x00:
        tag = " // 停流"
    elif a == 0x0100 and v == 0x01:
        tag = " // stream on（最后一条）"
    elif a == 0x302D:
        tag = " // 原厂序列固定值"
    elif a == 0x320E:
        tag = " // VTS = 0x%04x = %d" % (VTS, VTS)
    elif a == 0x320C:
        tag = " // HTS = 0x0b24 = 2852"
    elif a == 0x3208:
        tag = " // 宽 1920"
    elif a == 0x320A:
        tag = " // 高 1080"
    elif a == 0x4837:
        tag = " // MIPI 960 Mbps/Lane"
    elif a == 0x3E01:
        tag = " // 曝光 = %d = 2*VTS-10" % EXP_MAX
    L.append("            %3d: REG_DATA = {16'h%04x, 8'h%02x};%s" % (i, a, v, tag))
L.append("            default: REG_DATA = 24'h000000;")
L.append("        endcase")
L.append("    end")
L.append("endmodule")
MOD = "\n".join(L) + "\n"

# ---------------- 3) 追加到表文件（先删掉上一次生成的块） ----------------
t = open(TBL, encoding="utf-8").read().replace("\r\n", "\n")
BEG = "// ===== BEGIN uics520reg_1080p60"
END = "// ===== END uics520reg_1080p60"
if BEG in t:
    t = re.sub(re.escape(BEG) + r".*?" + re.escape(END) + r"\n?", "", t, flags=re.S)
    print("已移除旧的 1080p60 块")
if not t.endswith("\n"):
    t += "\n"
t += "\n" + BEG + " (由 sim/gen_1080p60_table.py 生成，请勿手改) =====\n" + MOD + END + "\n"
open(TBL, "w", encoding="utf-8", newline="\n").write(t)
nmod = len(re.findall(r"(?m)^module\s+(\w+)", t))
print("已写入 %s：模块 = %s" % (os.path.basename(TBL), re.findall(r"(?m)^module\s+(\w+)", t)))

# ---------------- 4) 同步定序器 ----------------
s = open(SEQ, encoding="utf-8").read().replace("\r\n", "\n")
rep4 = [
    (r"uics520reg_720p60 u_init_regs \(", "uics520reg_1080p60 u_init_regs ("),
    (r"localparam \[8:0\] STREAM_ON_INDEX = 9'd\d+;",
     "localparam [8:0] STREAM_ON_INDEX = 9'd%d;" % (len(regs) - 1)),
]
for pat, newv in rep4:
    s2, n = re.subn(pat, newv, s, count=1)
    assert n == 1, "定序器替换失败: %s" % pat
    s = s2
# post-stream-on 等待改成可选（1080p60 表 stream on 是最后一条，之后无写入）
old_wait = "end else if (!ae_table_active && (reg_index == STREAM_ON_INDEX)) begin"
new_wait = "end else if (!ae_table_active && POST_STREAM_WRITE && (reg_index == STREAM_ON_INDEX)) begin"
assert old_wait in s, "post-stream-on 分支未匹配"
s = s.replace(old_wait, new_wait)
old_lp = "    localparam [8:0] STREAM_ON_INDEX = 9'd%d;" % (len(regs) - 1)
new_lp = (old_lp + "\n"
          "    // 1080p60 原厂表里 stream on 是最后一条，之后不再写寄存器 -> 关掉这个等待\n"
          "    localparam POST_STREAM_WRITE = 1'b0;")
assert old_lp in s
s = s.replace(old_lp, new_lp)
# 更新头部注释
s = s.replace("uics520reg_720p60：", "uics520reg_1080p60：").replace("uics520reg_720p60.", "uics520reg_1080p60.")
open(SEQ, "w", encoding="utf-8", newline="\n").write(s)
print("已同步 %s：STREAM_ON_INDEX=%d, POST_STREAM_WRITE=0, 例化 uics520reg_1080p60" % (os.path.basename(SEQ), len(regs) - 1))
