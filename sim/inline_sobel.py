# -*- coding: utf-8 -*-
"""
inline_sobel.py — 把 sobel_edge_128.v 的模块内联进 isp_top.v 末尾

为什么内联：TD 打开工程时会用内存里的旧文件列表覆盖 .al，
外部往 .al 加新文件条目会被回滚，综合时报
"instantiate unknown module ... is a black box"。
blc_raw40 / axis_gamma_rgb96 都是这么内联的，所以不依赖 .al。

重复执行安全：先删掉上一次内联进来的区块（用 BEGIN/END 标记）再重新追加，
保证 isp_top.v 里的副本始终与 sobel_edge_128.v 一致。

用法：python sim/inline_sobel.py
"""
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "user_source", "hdl_source", "isp", "sobel_edge_128.v")
DST = os.path.join(ROOT, "user_source", "hdl_source", "isp", "isp_top.v")

BEGIN = "// ===== BEGIN inline sobel_edge_128.v (由 sim/inline_sobel.py 生成，请勿手改) ====="
END = "// ===== END inline sobel_edge_128.v ====="


def read_any(p):
    b = open(p, "rb").read()
    for enc in ("utf-8", "gbk"):
        try:
            return b.decode(enc)
        except UnicodeDecodeError:
            continue
    return b.decode("latin-1")


src = read_any(SRC).replace("\r\n", "\n").replace("\r", "\n")
dst = read_any(DST).replace("\r\n", "\n").replace("\r", "\n")

# 去掉源文件里的 `timescale（isp_top.v 里已有，重复会改变后续模块的时间单位）
src_body = re.sub(r"(?m)^\s*`timescale[^\n]*\n", "", src).strip("\n")

# 先移除旧的区块
if BEGIN in dst:
    dst = re.sub(re.escape(BEGIN) + r".*?" + re.escape(END) + r"\n?", "", dst, flags=re.S)
    print("已移除旧的内联区块")

if not dst.endswith("\n"):
    dst += "\n"

block = "\n" + BEGIN + "\n" + src_body + "\n" + END + "\n"
dst += block

open(DST, "w", encoding="utf-8", newline="\n").write(dst)

n_mod = len(re.findall(r"(?m)^module\s+(\w+)", dst))
names = re.findall(r"(?m)^module\s+(\w+)", dst)
print("已内联 -> %s" % DST)
print("  当前模块总数 = %d" % n_mod)
print("  模块: %s" % ", ".join(names))
need = ["sobel_edge_128", "sobel_line_ram", "binarize_4", "rgb_to_ycbcr_4", "data96_128_pa"]
miss = [m for m in need if m not in names]
if miss:
    print("  !! 缺少: %s" % miss)
    sys.exit(1)
dup = [m for m in set(names) if names.count(m) > 1]
if dup:
    print("  !! 重复定义: %s" % dup)
    sys.exit(1)
print("  OK")
