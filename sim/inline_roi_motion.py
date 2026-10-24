# -*- coding: utf-8 -*-
"""
把 roi_motion_onchip.v 内联进 design_top_wrapper.v（规则 1：模块不进 .al 文件列表）。

用法：
    python sim/inline_roi_motion.py

内联位置：wrapper 里 "// ===== BEGIN inline roi_motion_onchip.v" 标记之间。
改了 user_source/hdl_source/roi_motion_onchip.v 之后必须跑一次，然后跑
    python sim/check_inline_fdiff.py roi_motion_onchip user_source/hdl_source/design_top_wrapper.v user_source/hdl_source/roi_motion_onchip.v
"""
import io
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "user_source", "hdl_source", "roi_motion_onchip.v")
DST = os.path.join(ROOT, "user_source", "hdl_source", "design_top_wrapper.v")

BEGIN = "// ===== BEGIN inline roi_motion_onchip.v"
END = "// ===== END inline roi_motion_onchip.v"


def main():
    src = io.open(SRC, encoding="utf-8").read()
    dst = io.open(DST, encoding="utf-8").read()
    if BEGIN not in dst or END not in dst:
        raise SystemExit("wrapper 里找不到内联标记，请先手工加上一对 BEGIN/END 标记")
    head, rest = dst.split(BEGIN, 1)
    _, tail = rest.split(END, 1)
    blk = (BEGIN + " (由 sim/inline_roi_motion.py 生成，请勿手改) =====\n"
           + src + "\n" + END + " =====")
    io.open(DST, "w", encoding="utf-8", newline="\n").write(head + blk + tail)
    print("已内联 roi_motion_onchip.v -> design_top_wrapper.v (%d 字符)" % len(src))


if __name__ == "__main__":
    main()
