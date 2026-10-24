# -*- coding: utf-8 -*-
"""内联副本一致性校验 —— 防止"改了源文件忘了重新内联"。

用法：
    python sim/check_inline_fdiff.py
        -> 默认校验 frame_diff_onchip.v 是否已同步内联进 design_top_wrapper.v

    python sim/check_inline_fdiff.py <模块名> <外层文件> <源文件>
        -> 通用形式，例如：
           python sim/check_inline_fdiff.py sobel_edge_128 ^
                  user_source/hdl_source/isp/isp_top.v ^
                  user_source/hdl_source/isp/sobel_edge_128.v

为什么需要它（AGENTS.md 规则 2）：
    本项目约定不往 td_project/*.al 加新文件（TD 会用内存里的文件列表覆盖它），
    所以这两个模块都是【内联】进外层文件的：
        frame_diff_onchip.v  ->  user_source/hdl_source/design_top_wrapper.v
        sobel_edge_128.v     ->  user_source/hdl_source/isp/isp_top.v
                                 （这一份由 sim/inline_sobel.py 生成）
    内联副本是复制出来的，源文件改了而副本没跟上时，TD 编译的仍是旧代码；
    更麻烦的是仿真直接喂源文件会通过、上板行为却不对 —— 极难查。
    本脚本把两边 module 区块逐行比对（忽略空白与注释），失同步直接报失败。

比对范围只取 module ... endmodule，所以像 inline_sobel.py 剥掉的 `timescale
（位于 module 之外）不会被误报。
"""
import os
import re
import sys

# 终端下 Python 走 WriteConsoleW，中文与 ✓ 都正常；
# 但输出被重定向/管道时（CI、日志抓取）会退回 GBK 码页，'✓' 编不出来直接抛
# UnicodeEncodeError —— 表现是"检查全过却 exit 1"。这里做两件事：
#   非终端 -> 明确用 utf-8（重定向的文本在任何地方都能正确解码）
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

DEFAULT = (
    "frame_diff_onchip",
    os.path.join("user_source", "hdl_source", "design_top_wrapper.v"),
    os.path.join("user_source", "hdl_source", "frame_diff_onchip.v"),
)

fails = []
oks = []


def chk(cond, msg):
    (oks if cond else fails).append(msg)


def read_any(p):
    """按 utf-8 / gbk 依次尝试解码（TD 里的文件编码不统一）。"""
    b = open(p, "rb").read()
    for enc in ("utf-8", "gbk"):
        try:
            return b.decode(enc)
        except UnicodeDecodeError:
            continue
    return b.decode("latin-1")


def strip_comments(text):
    """把 // 与 /* */ 注释替换成空白，保留换行位置，字符串字面量不误伤。"""
    out = []
    i, n = 0, len(text)
    in_block = in_str = False
    while i < n:
        c = text[i]
        nxt = text[i + 1] if i + 1 < n else ""
        if in_block:
            if c == "*" and nxt == "/":
                in_block = False
                out.append("  ")
                i += 2
                continue
            out.append("\n" if c == "\n" else " ")
            i += 1
            continue
        if in_str:
            out.append(c)
            if c == "\\" and nxt:
                out.append(nxt)
                i += 2
                continue
            if c == '"':
                in_str = False
            i += 1
            continue
        if c == "/" and nxt == "*":
            in_block = True
            out.append("  ")
            i += 2
            continue
        if c == "/" and nxt == "/":
            while i < n and text[i] != "\n":
                out.append(" ")
                i += 1
            continue
        if c == '"':
            in_str = True
        out.append(c)
        i += 1
    return "".join(out)


def module_region(path, module):
    """返回该文件里 module <module> ... endmodule 区块的 (起始行, 结束行, 行列表)。
    行列表元素为 (原始行号, 去注释后的代码)。找不到返回 None。"""
    code = strip_comments(read_any(path)).replace("\r\n", "\n").replace("\r", "\n")
    lines = code.split("\n")
    start = end = None
    for idx, s in enumerate(lines):
        if re.match(r"\s*module\s+%s\b" % re.escape(module), s):
            if start is None:
                start = idx
        elif start is not None and re.match(r"\s*endmodule\b", s):
            end = idx
            break
    if start is None:
        return None
    if end is None:
        return (start + 1, None, [(i + 1, s) for i, s in enumerate(lines[start:], start)])
    return (start + 1, end + 1, [(i + 1, lines[i]) for i in range(start, end + 1)])


def norm(region_lines):
    """去掉空白与空行，得到可比对的 (原始行号, 归一化文本) 序列。"""
    res = []
    for ln, s in region_lines:
        s = re.sub(r"\s+", " ", s).strip()
        if s:
            res.append((ln, s))
    return res


def main():
    if len(sys.argv) == 1:
        module, wrapper, source = DEFAULT
    elif len(sys.argv) == 4:
        module, wrapper, source = sys.argv[1], sys.argv[2], sys.argv[3]
    else:
        print(__doc__)
        return 2

    wrapper = os.path.join(WS, wrapper) if not os.path.isabs(wrapper) else wrapper
    source = os.path.join(WS, source) if not os.path.isabs(source) else source

    print("=" * 78)
    print("内联一致性校验：module %s" % module)
    print("=" * 78)
    print("  外层文件（含内联副本）: %s" % wrapper)
    print("  源文件                : %s" % source)

    for p in (wrapper, source):
        if not os.path.isfile(p):
            print("  ✗ 文件不存在: %s" % p)
            return 1

    # 外层文件里同名 module 只能有一个，重复定义 TD 会报错
    wcode = read_any(wrapper)
    n_def = len(re.findall(r"(?m)^\s*module\s+%s\b" % re.escape(module), wcode))
    chk(n_def >= 1, "%s 里找到 module %s 的定义（%d 处）" % (os.path.basename(wrapper), module, n_def))
    chk(n_def <= 1, "%s 里 module %s 未重复定义（实际 %d 处）" % (os.path.basename(wrapper), module, n_def))
    if n_def != 1:
        print("\n通过 %d 项，失败 %d 项" % (len(oks), len(fails)))
        for f in fails:
            print("  ✗", f)
        return 1

    wr = module_region(wrapper, module)
    sr = module_region(source, module)
    chk(wr is not None and sr is not None,
        "两边都能定位到 module %s ... endmodule 区块" % module)
    if wr is None or sr is None:
        print("\n通过 %d 项，失败 %d 项" % (len(oks), len(fails)))
        for f in fails:
            print("  ✗", f)
        return 1

    w_end = "文件末尾" if wr[1] is None else "第 %d 行" % wr[1]
    s_end = "文件末尾" if sr[1] is None else "第 %d 行" % sr[1]
    print("  内联副本: 第 %d 行 .. %s" % (wr[0], w_end))
    print("  源文件  : 第 %d 行 .. %s" % (sr[0], s_end))
    print("  区块行数: 内联 %d 行 / 源文件 %d 行" % (len(wr[2]), len(sr[2])))

    A, B = norm(wr[2]), norm(sr[2])
    print("  有效行数（去空行/注释）: 内联 %d / 源文件 %d" % (len(A), len(B)))

    same = [s for _, s in A] == [s for _, s in B]
    chk(same, "内联副本与源文件逐行一致（忽略空白与注释）")

    if not same:
        import difflib
        sm = difflib.SequenceMatcher(None, [s for _, s in A], [s for _, s in B],
                                     autojunk=False)
        diffs = []
        for tag, i1, i2, j1, j2 in sm.get_opcodes():
            if tag == "equal":
                continue
            if tag == "replace":
                for k in range(max(i2 - i1, j2 - j1)):
                    a = A[i1 + k] if i1 + k < i2 else None
                    b = B[j1 + k] if j1 + k < j2 else None
                    diffs.append((tag, a, b))
            elif tag == "delete":
                for k in range(i1, i2):
                    diffs.append((tag, A[k], None))
            else:
                for k in range(j1, j2):
                    diffs.append((tag, None, B[k]))
        print("\n  共 %d 处差异，列出前 20 处：" % len(diffs))
        for tag, a, b in diffs[:20]:
            pa = "内联 p%d" % a[0] if a else "内联   -"
            pb = "源 p%d" % b[0] if b else "源   -"
            print("    [%s] %s: %s" % (tag, pa, a[1] if a else "(无)"))
            print("    [%s] %s: %s" % (tag, pb, b[1] if b else "(无)"))
        print("\n  ➜ 修复：把源文件 module 区块整段复制过去覆盖内联副本"
              "（frame_diff_onchip 见 AGENTS.md 规则 2；sobel 直接跑 python sim/inline_sobel.py）")

    print("\n" + "=" * 78)
    print("通过 %d 项，失败 %d 项" % (len(oks), len(fails)))
    if fails:
        print("\n失败项：")
        for f in fails:
            print("  ✗", f)
        return 1
    print("全部检查通过 ✓")
    return 0


if __name__ == "__main__":
    sys.exit(main())
