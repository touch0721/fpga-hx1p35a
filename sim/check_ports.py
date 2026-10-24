# -*- coding: utf-8 -*-
"""check_ports.py — 例化端口名 vs 模块定义，确定性比对（不靠仿真器）

为什么需要这个：
    往模块上加/改端口时，只要有一处漏改（只改了内联副本没改源文件、
    或漏了顶层包装模块），iverilog 的单文件检查查不出来 —— 子模块"未知"时
    它根本不做端口检查，要等到 TD 综合才报
        ERROR: cannot find port 'X' on this module
    ★ 这个问题真实发生过一次（加 O_cb_out/O_cr_out 时漏了 isp_top 包装层）。
    本脚本把两侧端口名直接对一遍，秒级出结果、不依赖任何工具链。

数据来源：td_project/*.al 里的 <File Path="..."> 列表（**只读**，不修改）。
    .enc.v / .sv / 厂商 IP 跳过（无法可靠解析）。

用法：python sim/check_ports.py [-v]
      -v  同时打印每个模块的端口数，以及"未被例化"的模块
退出码：0 = 无不匹配；1 = 有不匹配
"""
import os
import re
import sys

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(errors="replace") if _s.isatty() else _s.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VERBOSE = "-v" in sys.argv

KW = set("""module endmodule input output inout wire reg logic signed unsigned
parameter localparam assign always initial begin end case endcase if else for
while function endfunction task endtask generate endgenerate genvar integer
real time defparam specify endspecify primitive endprimitive table endtable
`timescale `default_nettype `include `define `ifdef `endif `else `undef
and or nand nor xor xnor not buf bufif0 bufif1 nmos pmos cmos rcmos rnmos rpmos
tri triand trior tri0 tri1 trireg supply0 supply1 wand wor""".split())

COMMENT = re.compile(r"//[^\n]*|/\*.*?\*/", re.S)


def read_any(p):
    b = open(p, "rb").read()
    for enc in ("utf-8", "gbk"):
        try:
            return b.decode(enc)
        except UnicodeDecodeError:
            continue
    return b.decode("latin-1")


def match_paren(t, i):
    """t[i] == '(' -> 返回配对 ')' 的下标，找不到返回 -1"""
    d = 0
    while i < len(t):
        c = t[i]
        if c == "(":
            d += 1
        elif c == ")":
            d -= 1
            if d == 0:
                return i
        i += 1
    return -1


def split_top(s):
    """按顶层逗号切分（不切 [] 内的）"""
    out, d, cur = [], 0, ""
    for c in s:
        if c == "[":
            d += 1
        elif c == "]":
            d -= 1
        if c == "," and d == 0:
            out.append(cur)
            cur = ""
        else:
            cur += c
    out.append(cur)
    return out


LAST_ID = re.compile(r"([A-Za-z_]\w*)\s*(?:=\s*[^,=]*)?$")


def parse_ports(plist):
    """ANSI 端口表 -> 端口名列表"""
    names = []
    for seg in split_top(plist):
        seg = seg.strip()
        if not seg:
            continue
        m = LAST_ID.search(seg)
        if not m:
            continue
        nm = m.group(1)
        # 排除"只有方向/类型没有名字"的段
        if nm.lower() in KW:
            continue
        names.append(nm)
    return names


def portlist_span(txt, pos):
    """模块名之后，找端口表 ( ... ) 的区间；★ 必须先跳过 #( 参数表 )，
    否则会把参数名当成端口名（这个坑真实踩过）。"""
    i = pos
    while i < len(txt) and txt[i] in " \t\r\n":
        i += 1
    if i < len(txt) and txt[i] == "#":
        j = txt.find("(", i)
        if j < 0:
            return None
        k = match_paren(txt, j)
        if k < 0:
            return None
        i = k + 1
    j = txt.find("(", i)
    if j < 0:
        return None
    k = match_paren(txt, j)
    if k < 0:
        return None
    return j + 1, k


MOD_RE = re.compile(r"(?m)^[ \t]*module\s+([A-Za-z_]\w*)")
INST_RE = re.compile(
    r"(?m)^[ \t]*([A-Za-z_]\w*)[ \t]*"
    r"(?:#\s*\((?:[^()]|\([^()]*\))*\)[ \t]*)?"      # 可选参数表
    r"([A-Za-z_]\w*)[ \t]*\(")


def al_files():
    al = os.path.join(ROOT, "td_project")
    cand = [f for f in os.listdir(al) if f.endswith(".al")]
    if not cand:
        print("找不到 .al；请从工程根目录运行")
        sys.exit(2)
    txt = read_any(os.path.join(al, sorted(cand)[0]))
    files = []
    for m in re.finditer(r'<File\s+Path="([^"]+)"', txt):
        p = os.path.normpath(os.path.join(al, m.group(1)))
        files.append(p)
    return sorted(cand)[0], files


al_name, files = al_files()
mods = {}          # module -> (相对路径, [端口名])
dup = []
skipped = []
for p in files:
    rel = os.path.relpath(p, ROOT)
    if not os.path.isfile(p):
        skipped.append((rel, "文件不存在"))
        continue
    if p.endswith(".enc.v") or p.endswith(".sv"):
        skipped.append((rel, "加密/SystemVerilog"))
        continue
    src = read_any(p)
    txt = COMMENT.sub(" ", src)
    for m in MOD_RE.finditer(txt):
        name = m.group(1)
        sp = portlist_span(txt, m.end())
        if not sp:
            continue
        j, k = sp
        ports = parse_ports(txt[j:k])
        if name in mods:
            dup.append((name, mods[name][0], rel))
        else:
            mods[name] = (rel, ports)

errors, warns, checked, unknown = [], [], 0, set()
for p in files:
    if not os.path.isfile(p) or p.endswith(".enc.v") or p.endswith(".sv"):
        continue
    rel = os.path.relpath(p, ROOT)
    txt = COMMENT.sub(" ", read_any(p))
    for m in INST_RE.finditer(txt):
        modname, instname = m.group(1), m.group(2)
        if modname in KW or modname not in mods:
            if modname not in KW:
                unknown.add(modname)
            continue
        k = match_paren(txt, m.end() - 1)
        if k < 0:
            continue
        conn = txt[m.end():k]
        if "." not in conn:                     # 位置连接，跳过
            continue
        want = set(mods[modname][1])
        got = set(re.findall(r"\.\s*([A-Za-z_]\w*)\s*\(", conn))
        checked += 1
        for nm in sorted(got - want):
            errors.append("%s: 例化 %s 连了模块 %s 里不存在的端口 '%s'"
                          % (rel, instname, modname, nm))
        for nm in sorted(want - got):
            warns.append("%s: 例化 %s 未连接模块 %s 的端口 '%s'"
                         % (rel, instname, modname, nm))

print("端口一致性检查（来源：td_project/%s）" % al_name)
print("  解析到模块 %d 个，检查了 %d 处例化" % (len(mods), checked))
if VERBOSE:
    for name in sorted(mods):
        print("    %-32s %3d 端口   %s" % (name, len(mods[name][1]), mods[name][0]))
if skipped:
    print("  跳过 %d 个文件：" % len(skipped))
    for rel, why in skipped[:6]:
        print("    %-50s %s" % (rel, why))
if dup:
    print("  !! 同名模块定义在多处（会导致综合取错版本）：")
    for name, f1, f2 in dup:
        print("    %-28s %s  <->  %s" % (name, f1, f2))
if unknown and VERBOSE:
    print("  未在本文件集里定义的例化（黑盒/IP）：%s" % ", ".join(sorted(unknown)))

if errors:
    print("\n  ✗ 端口不存在（这就是 TD 报 'cannot find port' 的原因）%d 处：" % len(errors))
    for e in errors[:40]:
        print("    " + e)
    if len(errors) > 40:
        print("    ... 其余 %d 处省略" % (len(errors) - 40))
else:
    print("\n例化端口全部存在 ✓")
if warns:
    print("\n  · 以下 %d 个模块端口未连接（Verilog 合法，通常是故意留空）%s："
          % (len(warns), "" if VERBOSE else "，-v 看全部"))
    for w in (warns if VERBOSE else warns[:5]):
        print("    " + w)
sys.exit(1 if errors else 0)
