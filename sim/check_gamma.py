# -*- coding: utf-8 -*-
"""独立复算 axis_gamma_rgb96 的 16 段折线，检查单调性/端点/文档中的检查点。"""
import sys

BASE  = [0, 37, 60, 79, 97, 113, 129, 143, 157, 171, 184, 197, 209, 221, 233, 244]
DELTA = [37, 23, 19, 18, 16, 16, 14, 14, 14, 13, 13, 12, 12, 12, 12, 11]


def gamma07(v):
    if v == 0xFF:
        return 0xFF
    seg = v >> 4
    interp = DELTA[seg] * (v & 0xF) + 8
    return BASE[seg] + (interp >> 4)


curve = [gamma07(v) for v in range(256)]
fails = []


def chk(c, m):
    if not c:
        fails.append(m)


chk(gamma07(0x00) == 0x00, "gamma(0x00)=0（保黑）实际 0x%02x" % gamma07(0))
chk(gamma07(0xFF) == 0xFF, "gamma(0xFF)=0xFF（保白）实际 0x%02x" % gamma07(255))
chk(all(curve[i] <= curve[i + 1] for i in range(255)), "全曲线单调不减")
chk(all(curve[i] < 256 for i in range(256)), "无溢出（<=255）")

print("检查点（模块注释里写的值）:")
for v, exp in [(0x40, 0x61), (0x80, 0x9D), (0xC0, 0xD1),
               (0x7C, 0x9A), (0x3C, 0x5D), (0xFC, 0xFC)]:
    got = gamma07(v)
    ok = (got == exp)
    print("  gamma(0x%02x) = 0x%02x  (期望 0x%02x)  %s  提亮 x%.2f"
          % (v, got, exp, "OK" if ok else "MISMATCH", got / float(v)))
    chk(ok, "检查点 gamma(0x%02x)=0x%02x 实际 0x%02x" % (v, exp, got))

print("\n提亮倍率采样:")
for v in [8, 16, 32, 64, 96, 128, 160, 192, 224, 240]:
    print("  0x%02x -> 0x%02x   x%.2f" % (v, gamma07(v), gamma07(v) / float(v)))

# 与理想 gamma 0.7 的比较
print("\n与理想 y = 255*(x/255)^0.7 的最大偏差:")
mx, mxv = 0, 0
for v in range(1, 256):
    ideal = 255.0 * (v / 255.0) ** 0.7
    d = abs(gamma07(v) - ideal)
    if d > mx:
        mx, mxv = d, v
print("  最大偏差 %.2f LSB（在 x=0x%02x）" % (mx, mxv))
chk(mx < 6.0, "与理想 0.7 gamma 偏差 <6 LSB，实际 %.2f" % mx)

print("\n" + "=" * 60)
if fails:
    print("失败 %d 项：" % len(fails))
    for f in fails:
        print("  x", f)
    sys.exit(1)
print("gamma 曲线全部检查通过")
