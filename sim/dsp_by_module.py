# -*- coding: utf-8 -*-
"""从 TD 门级网表里统计 DSP 原语按模块的分布 —— 用来定位 DSP 超限是谁造成的。

用法: python sim/_tmp/dsp_by_module.py <netlist.v>
"""
import re
import sys

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

path = sys.argv[1] if len(sys.argv) > 1 else None
if not path:
    print("用法: dsp_by_module.py <netlist.v>")
    sys.exit(2)

PRIM = re.compile(r"^\s*PH1P_PHY_(DSPMULT|DSPSIOU)_V2\s*$")
INST = re.compile(r"^\s*\\(\S+?)\s*\(")

cnt = {}
tot = 0
lines = open(path, "r", encoding="utf-8", errors="replace").read().splitlines()
for i, ln in enumerate(lines):
    if not PRIM.match(ln):
        continue
    name = None
    for j in range(i + 1, min(i + 7, len(lines))):
        m = INST.match(lines[j])
        if m:
            name = m.group(1)
            break
    if not name:
        name = "(未解析)"
    top = name.split("/")[0]
    sub = "/".join(name.split("/")[:2])
    cnt[top] = cnt.get(top, 0) + 1
    cnt["  -> " + sub] = cnt.get("  -> " + sub, 0) + 1
    tot += 1

print("DSP 原语总数: %d" % tot)
print("%-46s %s" % ("模块", "DSP 数"))
for k in sorted(cnt, key=lambda x: (-cnt[x], x)):
    print("%-46s %d" % (k, cnt[k]))
