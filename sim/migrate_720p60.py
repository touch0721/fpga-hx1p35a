# -*- coding: utf-8 -*-
"""migrate_720p60.py — 把工程从 1080p60 还原成原始工程的 720p60

依据：原始工程 lab_ex1_mipi_hdmi_SC520（本工作区上一级目录的 zip，
      已抽到 sim/_tmp/orig/）+ doc/sc500cs_i2c_adaptation.md §3 / §5 / §13

8 项改动：
  1 PLL.v            CLKC4_DIV 10->20（75 MHz）、CLKC5_DIV 2->4（375 MHz）
  2 uivtc            H 1280/1650/1390/1430、V 720/750/725/730
  3 IMG_WIDTH/HEIGHT 全工程 1920/1080 -> 1280/720（例化 + 参数默认值）
  4 11'd1080         -> 11'd720（MIPI 每帧行数校验）
  5 GBX_X0/Y0 字面常量 1920/1080 -> 1280/720
  6 传感器表         uics520reg_1080p60 -> uics520reg_720p60，POST_STREAM_WRITE 0->1
  7 .sdc             mipi_rx_ck_pad period -> 4.167 / waveform 0.000 2.083
  8 D-PHY 参数       HS_EQUALIZER / HS_VGA_GAIN -> "0dB"

用法: python sim/migrate_720p60.py [--dry]
      --dry 只报告将要改什么，不写文件
"""
import os
import re
import sys

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

DRY = "--dry" in sys.argv
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
H = os.path.join(ROOT, "user_source", "hdl_source")

# (文件, 说明, 正则, 替换)  —— 每项都会校验命中条数
RULES = [
    ("user_source/ip_source/PLL/PLL.v", "1 PLL 像素分频 10->20",
     r"(\.CLKC4_DIV\s*\(\s*)10(\s*\))", r"\g<1>20\g<2>", 1),
    ("user_source/ip_source/PLL/PLL.v", "1 PLL 串行分频 2->4",
     r"(\.CLKC5_DIV\s*\(\s*)2(\s*\))", r"\g<1>4\g<2>", 1),

    ("user_source/hdl_source/design_top_wrapper.v", "2 uivtc H_ActiveSize",
     r"(\.H_ActiveSize\s*\(\s*)1920(\s*\))", r"\g<1>1280\g<2>", 1),
    ("user_source/hdl_source/design_top_wrapper.v", "2 uivtc H_FrameSize",
     r"(\.H_FrameSize\s*\(\s*)2223(\s*\))", r"\g<1>1650\g<2>", 1),
    ("user_source/hdl_source/design_top_wrapper.v", "2 uivtc H_SyncStart",
     r"(\.H_SyncStart\s*\(\s*)2008(\s*\))", r"\g<1>1390\g<2>", 1),
    ("user_source/hdl_source/design_top_wrapper.v", "2 uivtc H_SyncEnd",
     r"(\.H_SyncEnd\s*\(\s*)2052(\s*\))", r"\g<1>1430\g<2>", 1),
    ("user_source/hdl_source/design_top_wrapper.v", "2 uivtc V_ActiveSize",
     r"(\.V_ActiveSize\s*\(\s*)1080(\s*\))", r"\g<1>720\g<2>", 1),
    ("user_source/hdl_source/design_top_wrapper.v", "2 uivtc V_FrameSize",
     r"(\.V_FrameSize\s*\(\s*)1125(\s*\))", r"\g<1>750\g<2>", 1),
    ("user_source/hdl_source/design_top_wrapper.v", "2 uivtc V_SyncStart",
     r"(\.V_SyncStart\s*\(\s*)1084(\s*\))", r"\g<1>725\g<2>", 1),
    ("user_source/hdl_source/design_top_wrapper.v", "2 uivtc V_SyncEnd",
     r"(\.V_SyncEnd\s*\(\s*)1089(\s*\))", r"\g<1>730\g<2>", 1),

    ("user_source/hdl_source/design_top_wrapper.v", "4 MIPI 行数校验",
     r"11'd1080", "11'd720", 1),
    ("user_source/hdl_source/design_top_wrapper.v", "8 D-PHY HS_EQUALIZER",
     r"(\.HS_EQUALIZER\s*\(\s*)\"[^\"]*\"", r'\g<1>"0dB"', 1),
    ("user_source/hdl_source/design_top_wrapper.v", "8 D-PHY HS_VGA_GAIN",
     r"(\.HS_VGA_GAIN\s*\(\s*)\"[^\"]*\"", r'\g<1>"0dB"', 1),

    ("user_source/hdl_source/uics520_cfg/uicfgcs520.v", "6 传感器表 1080p60->720p60",
     r"uics520reg_1080p60(\s+u_init_regs)", r"uics520reg_720p60\g<1>", 1),
    ("user_source/hdl_source/uics520_cfg/uicfgcs520.v", "6 POST_STREAM_WRITE 0->1",
     r"(localparam POST_STREAM_WRITE\s*=\s*)1'b0", r"\g<1>1'b1", 1),
]

# 3/5 项：全工程的 IMG_* 参数与 GBX 常量（扫这些文件）
FILES_ALL = [
    "user_source/hdl_source/design_top_wrapper.v",
    "user_source/hdl_source/isp/isp_top.v",
    "user_source/hdl_source/isp/sobel_edge_128.v",
    "user_source/hdl_source/hdmi_mixer.v",
]
BULK = [
    ("3 IMG_WIDTH/IMG_W = 1920 -> 1280",
     r"(parameter integer IMG_W(?:IDTH)?\s*=\s*)1920", r"\g<1>1280"),
    ("3 IMG_HEIGHT/IMG_H = 1080 -> 720",
     r"(parameter integer IMG_H(?:EIGHT)?\s*=\s*)1080", r"\g<1>720"),
    ("3 parameter IN_HEIGHT = 1080 -> 720",
     r"(parameter integer IN_HEIGHT\s*=\s*)1080", r"\g<1>720"),
    ("3 .IMG_WIDTH(1920) -> 1280",
     r"(\.IMG_WIDTH\s*\(\s*)1920(\s*\))", r"\g<1>1280\g<2>"),
    ("3 .IMG_HEIGHT(1080) -> 720",
     r"(\.IMG_HEIGHT\s*\(\s*)1080(\s*\))", r"\g<1>720\g<2>"),
    ("3 .IMG_W(1920) -> 1280",
     r"(\.IMG_W\s*\(\s*)1920(\s*\))", r"\g<1>1280\g<2>"),
    ("3 .IMG_H(1080) -> 720",
     r"(\.IMG_H\s*\(\s*)1080(\s*\))", r"\g<1>720\g<2>"),
    ("5 GBX_X0 字面常量 -> 1280",
     r"(localparam \[11:0\] GBX_X0 = \(12'd)1920", r"\g<1>1280"),
    ("5 GBX_Y0 字面常量 -> 720",
     r"(localparam \[11:0\] GBX_Y0 = \(12'd)1080", r"\g<1>720"),
]


def read_any(p):
    b = open(p, "rb").read()
    for enc in ("utf-8", "gbk"):
        try:
            return b.decode(enc), enc
        except UnicodeDecodeError:
            continue
    return b.decode("latin-1"), "latin-1"


total_ok = total_bad = 0
changed_files = {}

print("=== 720p60 迁移%s ===" % ("（试运行，不写文件）" if DRY else ""))
print("--- 逐项规则 ---")
for rel, desc, pat, rep, expect in RULES:
    p = os.path.join(ROOT, rel)
    if not os.path.isfile(p):
        print("  !! 文件不存在: %s" % rel)
        total_bad += 1
        continue
    txt, enc = read_any(p)
    new, n = re.subn(pat, rep, txt)
    mark = "OK " if n == expect else "!! "
    print("  %s %-34s 命中 %d（期望 %d）  %s" % (mark, desc, n, expect, rel))
    if n != expect:
        total_bad += 1
    elif n:
        changed_files[p] = (new, enc)
        total_ok += 1

print("--- 批量项（IMG_* / GBX 常量）---")
for rel in FILES_ALL:
    p = os.path.join(ROOT, rel)
    if not os.path.isfile(p):
        continue
    # ★ 必须接着"逐项规则已经改好的内容"继续改，不能从磁盘重读 ——
    #   重读会把前面打好的补丁覆盖掉（这个 bug 让 uivtc 那 8 项一直没生效过）
    if p in changed_files:
        txt = changed_files[p][0]
        enc = changed_files[p][1]
    else:
        txt, enc = read_any(p)
    hits = []
    for desc, pat, rep in BULK:
        txt, n = re.subn(pat, rep, txt)
        if n:
            hits.append("%s x%d" % (desc.split(" ", 1)[1], n))
    if hits:
        print("  OK  %-46s %s" % (os.path.basename(rel), "; ".join(hits)))
        total_ok += 1
        if p in changed_files:
            changed_files[p] = (txt, changed_files[p][1])
        else:
            changed_files[p] = (txt, enc)

print("--- .sdc（MIPI 时钟周期）---")
for rel in ("td_project/camera_to_dsi_display.sdc", "user_source/constraints_source/timing.sdc"):
    p = os.path.join(ROOT, rel)
    if not os.path.isfile(p):
        continue
    txt, enc = read_any(p)
    old = re.search(r"-name \{mipi_rx_ck_pad\}[^\n]*", txt)
    new, n = re.subn(
        r"(-name \{mipi_rx_ck_pad\}\s*-period\s*)[\d.]+(\s*-waveform\s*\{)[\d.\s]+(\})",
        r"\g<1>4.167\g<2>0.000 2.083\g<3>", txt)
    if n:
        print("  OK  %s" % rel)
        print("      旧: %s" % (old.group(0).strip() if old else "?"))
        print("      新: %s" % re.search(r"-name \{mipi_rx_ck_pad\}[^\n]*", new).group(0).strip())
        total_ok += 1
        changed_files[p] = (new, enc)
    else:
        # 这个文件本来就不含 MIPI 约束（如 constraints_source/timing.sdc），只提示
        print("  ·  %s 里没有 mipi_rx_ck_pad 行（正常，跳过）" % rel)

print("\n--- 写入 ---")
if DRY:
    print("  试运行，未写文件")
else:
    for p, (txt, enc) in changed_files.items():
        open(p, "w", encoding=enc if enc != "latin-1" else "utf-8", newline="\n").write(txt)
        print("  已写 %s" % os.path.relpath(p, ROOT))

print("\n应用 %d 项，异常 %d 项" % (total_ok, total_bad))
if total_bad:
    print("★ 有异常项，请先看上面带 !! 的行")
    sys.exit(1)
print("★ 下一步必须跑：python sim/inline_sobel.py（sobel 改了）")
print("                python sim/inline_tile_recog.py")
print("                python sim/check_ports.py")
