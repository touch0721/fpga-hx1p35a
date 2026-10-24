# -*- coding: utf-8 -*-
"""inline_tile_recog.py — 把 tile_recog_onchip.v 内联进 design_top_wrapper.v 末尾

为什么内联：TD 打开工程时会用内存里的旧文件列表覆盖 .al，
外部往 .al 加新文件条目会被回滚（本项目踩过两次）。
frame_diff_onchip / sobel_edge_128 都是这么内联的，所以不依赖 .al。

重复执行安全：先删掉上一次内联进来的区块（用 BEGIN/END 标记）再重新追加，
保证 design_top_wrapper.v 里的副本始终与 tile_recog_onchip.v 一致。

用法：python sim/inline_tile_recog.py
"""
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "user_source", "hdl_source", "tile_recog_onchip.v")
DST = os.path.join(ROOT, "user_source", "hdl_source", "design_top_wrapper.v")

BEGIN = "// ===== BEGIN inline tile_recog_onchip.v (由 sim/inline_tile_recog.py 生成，请勿手改) ====="
END = "// ===== END inline tile_recog_onchip.v ====="


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

# 去掉源文件里的 `timescale（design_top_wrapper.v 里已有，重复会改变后续模块的时间单位）
src_body = re.sub(r"(?m)^\s*`timescale[^\n]*\n", "", src).strip("\n")

if BEGIN in dst:
    dst = re.sub(re.escape(BEGIN) + r".*?" + re.escape(END) + r"\n?", "", dst, flags=re.S)
    print("已移除旧的内联区块")

if not dst.endswith("\n"):
    dst += "\n"

block = "\n" + BEGIN + "\n" + src_body + "\n" + END + "\n"
dst += block

open(DST, "w", encoding="utf-8", newline="\n").write(dst)

names = re.findall(r"(?m)^module\s+(\w+)", dst)
print("已内联 -> %s" % DST)
print("  当前模块总数 = %d" % len(names))
print("  模块: %s" % ", ".join(names))
for need in ("design_top_wrapper", "frame_diff_onchip", "tile_recog_onchip"):
    if need not in names:
        print("  !! 缺少: %s" % need)
        sys.exit(1)
dup = [m for m in set(names) if names.count(m) > 1]
if dup:
    print("  !! 重复定义: %s" % dup)
    sys.exit(1)
print("  OK")
