# -*- coding: utf-8 -*-
"""素材图 -> FPGA 模板包（阶段 5 工具链 / 灰度 SAD，支持分组不同分辨率）。

用法:
    python sim/make_tile_templates.py [素材目录] [输出目录] [方案] [排除]
    方案格式:  "32=Pin*,Sou*;48=Man*,Ton,Nan,Shaa,Pei,Chun,Hatsu"
               （每类取第一个匹配到的尺寸；未匹配到的用最后一组的尺寸）
    排除:      逗号分隔的通配符，如 "Haku"（白板这类无图案的牌不进模板库，
               改用"空白率"规则判定）
    例:
    python sim/make_tile_templates.py sim/_tmp/tiles_src sim/out \
        "32=Pin*,Sou*;48=Man*,Ton,Nan,Shaa,Pei,Chun,Hatsu" "Haku"

处理: 灰度 -> 裁内容外接框 -> 等比缩放到该组尺寸(白底) -> 对比度拉伸(1%~99% 分位)
      => 8 位灰度模板（给 SAD） + 同图 Otsu 二值化 => 1 位模板（给 TB/退路）
输出: tile_tmpl_pkg.v               每组一个灰度 ROM + 一个二值 ROM + 颜色签名表
      tile_tmpl_g<S>_gray.mem       每组 $readmemh（32 位字装 4 个 8 位像素）
      tile_tmpl_g<S>_bin.mem        每组 $readmemh（1 位，每字一行）
      tile_tmpl_preview_g<S>.png    每组联系表
      tile_tmpl_report.txt          组内可分性（跨组距离无意义，故按组算）
"""
import fnmatch
import os
import sys

try:
    from PIL import Image, ImageDraw
except ImportError:
    print("需要 Pillow（DSH 自带 python 已包含）")
    sys.exit(2)

for _s in (sys.stdout, sys.stderr):
    try:
        if not _s.isatty():
            _s.reconfigure(encoding="utf-8", errors="replace")
        else:
            _s.reconfigure(errors="replace")
    except Exception:
        pass

SRC = sys.argv[1] if len(sys.argv) > 1 else os.path.join("sim", "_tmp", "tiles_src")
OUT = sys.argv[2] if len(sys.argv) > 2 else os.path.join("sim", "out")
PLAN_STR = sys.argv[3] if len(sys.argv) > 3 else ""
EXCL_STR = sys.argv[4] if len(sys.argv) > 4 else ""
ORDER_STR = sys.argv[5] if len(sys.argv) > 5 else ""
ORDER = [x.strip() for x in ORDER_STR.split(",") if x.strip()]
EXTS = (".png", ".jpg", ".jpeg", ".bmp", ".gif", ".webp")
PPW = 4                                  # 每个 32 位字装几个 8 位像素


def order_key(fn):
    """给了 ORDER 就按它排类号（便于 RTL 用区间判类），否则按文件名字母序。"""
    nm = os.path.splitext(fn)[0]
    if ORDER:
        try:
            return (0, ORDER.index(nm), "")
        except ValueError:
            return (1, 0, nm)
    return (0, 0, nm)

# ---- 解析方案 ----
PLAN = []


def parse_part(part):
    """'32=Pin*,Sou*' 或 '32:45=Man*,Ton' —— 冒号后是垂直裁剪百分比（只取上半部）。"""
    size_s, _, pats_s = part.partition("=")
    size_s = size_s.strip()
    vcrop = 100
    if ":" in size_s:
        size_s, _, v_s = size_s.partition(":")
        vcrop = int(v_s)
    return int(size_s), vcrop, [p.strip() for p in pats_s.split(",") if p.strip()]


if PLAN_STR:
    for part in PLAN_STR.split(";"):
        part = part.strip()
        if part:
            PLAN.append(parse_part(part))
if not PLAN:
    PLAN = [(32, 100, ["Pin*", "Sou*"]),
            (32, 45, ["Man*", "Ton", "Nan", "Shaa", "Pei", "Chun", "Hatsu"])]
EXCL = [p.strip() for p in EXCL_STR.split(",") if p.strip()]


def group_of(name):
    for size, vcrop, pats in PLAN:
        for p in pats:
            if fnmatch.fnmatch(name, p):
                return size, vcrop
    return PLAN[-1][0], PLAN[-1][1]


def excluded(name):
    return any(fnmatch.fnmatch(name, p) for p in EXCL)


os.makedirs(OUT, exist_ok=True)
files = sorted((f for f in os.listdir(SRC) if f.lower().endswith(EXTS)), key=order_key)
if not files:
    print("素材目录里没有图片:", SRC)
    sys.exit(1)
print("素材 %s   共 %d 张" % (SRC, len(files)))
print("方案: " + "  ".join("%d(%d%%高)=%s" % (s, v, ",".join(p)) for s, v, p in PLAN))
if EXCL:
    print("排除: " + ",".join(EXCL) + "（这些类不进模板库，改用空白率规则判定）")


def otsu(px):
    hist = [0] * 256
    for v in px:
        hist[v] += 1
    total, sum_all = len(px), sum(i * h for i, h in enumerate(hist))
    sum_b, w_b, best_t, best_v = 0.0, 0, 128, -1.0
    for t in range(256):
        w_b += hist[t]
        if w_b == 0:
            continue
        w_f = total - w_b
        if w_f == 0:
            break
        sum_b += t * hist[t]
        var = w_b * w_f * ((sum_b / w_b) - ((sum_all - sum_b) / w_f)) ** 2
        if var > best_v:
            best_v, best_t = var, t
    return best_t


def stretch(px, lo_pct=1.0, hi_pct=99.0):
    srt = sorted(px)
    lo = srt[max(0, int(len(srt) * lo_pct / 100.0))]
    hi = srt[min(len(srt) - 1, int(len(srt) * hi_pct / 100.0))]
    if hi <= lo:
        return list(px)
    return [max(0, min(255, int(round((v - lo) * 255.0 / (hi - lo))))) for v in px]


def color_sig(rgb_img, mask):
    px = rgb_img.convert("RGB")
    w, h = px.size
    bins, n = [0, 0, 0, 0], 0
    for y in range(h):
        for x in range(w):
            if not mask[y * w + x]:
                continue
            r, g, b = px.getpixel((x, y))
            n += 1
            mx, mn = max(r, g, b), min(r, g, b)
            if mx - mn < 40:
                bins[3] += 1
            elif mx == r:
                bins[0] += 1
            elif mx == g:
                bins[1] += 1
            else:
                bins[2] += 1
    if n == 0:
        return [0, 0, 0, 0]
    return [min(255, int(round(255.0 * c / n))) for c in bins]


def prep(path, S, vcrop=100):
    """返回 (8 位灰度列表, 1 位列表, Otsu 阈值, RGB 内容图) —— 尺寸 SxS。
    vcrop<100 时只取内容框的上部 vcrop%（万子的数字在牌面上半部，
    下半部的"萬"字所有万子都一样，含进来会稀释差异）。"""
    im = Image.open(path)
    rgb = im.convert("RGB")
    g = im.convert("L")
    w, h = g.size
    vals = list(g.getdata())
    bg = max(set(vals), key=vals.count)
    thr0 = max(30, bg - 45)
    pts = [(x, y) for y in range(h) for x in range(w) if g.getpixel((x, y)) < thr0]
    if pts:
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        y1 = max(ys)
        if vcrop < 100:
            y1 = min(ys) + max(1, int((max(ys) - min(ys) + 1) * vcrop / 100.0)) - 1
        box = (max(0, min(xs) - 1), max(0, min(ys) - 1),
               min(w - 1, max(xs) + 1), min(h - 1, y1 + 1))
        g2 = g.crop((box[0], box[1], box[2] + 1, box[3] + 1))
        rgb2 = rgb.crop((box[0], box[1], box[2] + 1, box[3] + 1))
    else:
        g2, rgb2 = g, rgb
    cw, ch = g2.size
    sc = float(S) / max(cw, ch)
    nw, nh = max(1, int(round(cw * sc))), max(1, int(round(ch * sc)))
    canvas = Image.new("L", (S, S), 255)
    canvas.paste(g2.resize((nw, nh), Image.LANCZOS), ((S - nw) // 2, (S - nh) // 2))
    gpx = stretch(list(canvas.getdata()))
    thr = otsu(gpx)
    bpx = [1 if v <= thr else 0 for v in gpx]
    return gpx, bpx, thr, rgb2


# ---------------- 逐类预处理，按组归集 ----------------
groups = {}          # size -> [(cls_idx, name, gpx, bpx, thr, rgb2)]
skipped = []
for fn in files:
    nm = os.path.splitext(fn)[0]
    if excluded(nm):
        skipped.append(nm)
        continue
    S, VC = group_of(nm)
    gpx, bpx, thr, rgb2 = prep(os.path.join(SRC, fn), S, VC)
    groups.setdefault(S, []).append((nm, gpx, bpx, thr, rgb2))

# 全局类号（跨组统一编号，便于 RTL 用一套 class id）
cls_id = {}
for S in sorted(groups):
    for (nm, *_rest) in groups[S]:
        cls_id[nm] = len(cls_id) + 1
N = len(cls_id)

# ---------------- 打包 ----------------
packed = {}   # size -> [(cls, name, gray_words, bin_words, sig)]
for S in sorted(groups):
    WORDS_ROW = (S + PPW - 1) // PPW
    lst = []
    for (nm, gpx, bpx, thr, rgb2) in groups[S]:
        gw = []
        for y in range(S):
            for wx in range(WORDS_ROW):
                wd = 0
                for k in range(PPW):
                    x = wx * PPW + k
                    v = gpx[y * S + x] if x < S else 0
                    wd |= (v & 0xFF) << (8 * (PPW - 1 - k))
                gw.append(wd)
        WB = (S + 31) // 32          # 二值 ROM：每行需要几个 32 位字
        bw = []
        for y in range(S):
            row = [0] * WB
            for x in range(S):
                if bpx[y * S + x]:
                    row[x // 32] |= (1 << (31 - (x % 32)))
            bw.extend(row)
        lst.append((cls_id[nm], nm, gw, bw, color_sig(rgb2.resize((S, S), Image.LANCZOS), bpx), thr))
    packed[S] = lst

# ---------------- Verilog 包 ----------------
vp = os.path.join(OUT, "tile_tmpl_pkg.v")
with open(vp, "w", encoding="utf-8", newline="\n") as f:
    f.write("// 由 sim/make_tile_templates.py 生成，请勿手改\n")
    f.write("// 分组分辨率模板库，共 %d 类（类 0 = 未知/拒识）\n" % N)
    for S in sorted(packed):
        f.write("//   尺寸 %d : %s\n" % (S, ", ".join("%d=%s" % (c, n) for c, n, *_ in packed[S])))
    if skipped:
        f.write("//   已排除(用空白率规则判定): %s\n" % ", ".join(skipped))
    f.write("\n")
    for S in sorted(packed):
        W = (S + PPW - 1) // PPW
        CN = len(packed[S])
        f.write("// ===== 尺寸 %d：%d 类，地址 = row*%d + word（每字 %d 像素，高位在前）=====\n"
                % (S, CN, W, PPW))
        f.write("module tile_tmpl_rom_g%d_gray (\n    input  wire        I_clk,\n"
                "    input  wire [%d:0] I_addr,\n    output reg  [31:0] O_data\n);\n"
                % (S, max(0, (S * W * CN - 1).bit_length() - 1)))
        f.write("    reg [31:0] M [0:%d];\n    initial begin\n" % (S * W * CN - 1))
        for (c, nm, gw, bw, sg, thr) in packed[S]:
            f.write("        // 类 %d = %s\n" % (c, nm))
            for r, wv in enumerate(gw):
                f.write("        M[%d] = 32'h%08x;\n" % ((c - 1) * S * W + r, wv))
        f.write("    end\n    always @(posedge I_clk) O_data <= M[I_addr];\nendmodule\n\n")
        WB = (S + 31) // 32
        f.write("module tile_tmpl_rom_g%d_bin (\n    input  wire        I_clk,\n"
                "    input  wire [%d:0] I_addr,\n    output reg  [31:0] O_data\n);\n"
                % (S, max(0, (S * WB * CN - 1).bit_length() - 1)))
        f.write("    reg [31:0] M [0:%d];\n    initial begin\n" % (S * WB * CN - 1))
        for (c, nm, gw, bw, sg, thr) in packed[S]:
            for r, wv in enumerate(bw):
                f.write("        M[%d] = 32'h%08x;\n" % ((c - 1) * S * WB + r, wv))
        f.write("    end\n    always @(posedge I_clk) O_data <= M[I_addr];\nendmodule\n\n")
    f.write("// 颜色签名：4 桶(红/绿/蓝/无彩) x 8 位；未知类返回 0\n")
    f.write("module tile_color_tbl (\n    input  wire [7:0]  I_cls,\n    output reg  [31:0] O_sig\n);\n"
            "    always @(*) begin\n        case (I_cls)\n")
    for S in sorted(packed):
        for (c, nm, gw, bw, sg, thr) in packed[S]:
            f.write("            8'd%-3d: O_sig = 32'h%08x;   // %s\n"
                    % (c, (sg[0] << 24) | (sg[1] << 16) | (sg[2] << 8) | sg[3], nm))
    f.write("            default: O_sig = 32'h0;\n        endcase\n    end\nendmodule\n")

# ---------------- $readmemh + 预览 ----------------
for S in sorted(packed):
    W = (S + PPW - 1) // PPW
    with open(os.path.join(OUT, "tile_tmpl_g%d_gray.mem" % S), "w", encoding="utf-8", newline="\n") as f:
        for (c, nm, gw, bw, sg, thr) in packed[S]:
            for wv in gw:
                f.write("%08x\n" % wv)
    with open(os.path.join(OUT, "tile_tmpl_g%d_bin.mem" % S), "w", encoding="utf-8", newline="\n") as f:
        for (c, nm, gw, bw, sg, thr) in packed[S]:
            for wv in bw:
                f.write("%08x\n" % wv)

    show = min(S, 48)
    cell = show * 3
    pad, lab = 6, 14
    cols = min(9, len(packed[S]))
    rows = (len(packed[S]) + cols - 1) // cols
    sheet = Image.new("RGB", (cols * (cell + pad) + pad, rows * (cell + pad + lab) + pad), (245, 245, 245))
    dr = ImageDraw.Draw(sheet)
    for i, (c, nm, gw, bw, sg, thr) in enumerate(packed[S]):
        cx = pad + (i % cols) * (cell + pad)
        cy = pad + (i // cols) * (cell + pad + lab)
        tmp = Image.new("L", (S, S))
        tmp.putdata(groups[S][i][1])
        sheet.paste(tmp.convert("RGB").resize((cell, cell), Image.NEAREST), (cx, cy))
        dr.text((cx, cy + cell + 1), "%d %s" % (c, nm), fill=(0, 0, 0))
    sheet.save(os.path.join(OUT, "tile_tmpl_preview_g%d.png" % S))

# ---------------- 组内可分性报告 ----------------
rp = os.path.join(OUT, "tile_tmpl_report.txt")
with open(rp, "w", encoding="utf-8", newline="\n") as f:
    f.write("模板可分性报告（按组计算，跨组尺寸不同、距离无意义）\n")
    f.write("=" * 88 + "\n")
    tot_g = tot_b = 0
    for S in sorted(packed):
        lst = packed[S]
        gp = [g[1] for g in groups[S]]      # 每类的灰度像素（S*S）
        bp = [g[2] for g in groups[S]]      # 每类的二值像素（S*S）
        f.write("\n### 尺寸 %dx%d ，%d 类\n" % (S, S, len(lst)))
        f.write("类号 名称         前景%%   Otsu   最近类 平均差(类号)   最近类汉明\n")
        pairs_g, pairs_b = [], []
        for i in range(len(lst)):
            for j in range(i + 1, len(lst)):
                pairs_g.append((sum(abs(x - y) for x, y in zip(gp[i], gp[j])), i, j))
                pairs_b.append((sum(1 for x, y in zip(bp[i], bp[j]) if x != y), i, j))
        pairs_g.sort()
        pairs_b.sort()
        gm, bm = {}, {}
        for d, i, j in pairs_g:
            gm.setdefault(i, (d, j))
            gm.setdefault(j, (d, i))
        for d, i, j in pairs_b:
            bm.setdefault(i, (d, j))
            bm.setdefault(j, (d, i))
        for i, (c, nm, gw, bw, sg, thr) in enumerate(lst):
            pct = 100.0 * sum(bp[i]) / (S * S)
            gd, gj = gm.get(i, (0, -1))
            f.write("%4d %-12s %5.1f  %5d   %6.1f (vs %-8s)  %5d\n"
                    % (c, nm, pct, thr, gd / float(S * S),
                       lst[gj][1] if gj >= 0 else "-", bm.get(i, (0,))[0]))
        if pairs_g:
            f.write("  组内最接近的 6 对（平均绝对差 0..255）：\n")
            for d, i, j in pairs_g[:6]:
                f.write("     %-10s vs %-10s : %6.2f\n"
                        % (lst[i][1], lst[j][1], d / float(S * S)))
            f.write("  组内最小 %.2f / 平均 %.2f\n"
                    % (pairs_g[0][0] / float(S * S),
                       sum(d for d, _, _ in pairs_g) / len(pairs_g) / float(S * S)))
        g_bits = len(lst) * S * ((S + PPW - 1) // PPW) * 32
        b_bits = len(lst) * S * S
        tot_g += g_bits
        tot_b += b_bits
        f.write("  用量: 灰度 %d 位 = %.1f Kbit -> %d 块 ERAM20K ; 二值 %d 位 = %.1f Kbit -> %d 块\n"
                % (g_bits, g_bits / 1024.0, -(-g_bits // 20480),
                   b_bits, b_bits / 1024.0, -(-b_bits // 20480)))
    f.write("\n" + "=" * 88 + "\n")
    f.write("合计: 灰度库 %d 位 = %.1f Kbit -> %d 块 ERAM20K ; 二值库 %d 位 = %.1f Kbit -> %d 块\n"
            % (tot_g, tot_g / 1024.0, -(-tot_g // 20480), tot_b, tot_b / 1024.0, -(-tot_b // 20480)))
    if skipped:
        f.write("已排除: %s（用\"前景占比 < 2%%\"规则判定）\n" % ", ".join(skipped))

print("\n输出:")
for f2 in sorted(os.listdir(OUT)):
    p = os.path.join(OUT, f2)
    if os.path.isfile(p):
        print("   %-32s %9d 字节" % (p, os.path.getsize(p)))
print("\n合计: 灰度库 %.1f Kbit -> %d 块 ERAM20K ; 二值库 %.1f Kbit -> %d 块"
      % (tot_g / 1024.0, -(-tot_g // 20480), tot_b / 1024.0, -(-tot_b // 20480)))
print("类号: " + ", ".join("%d=%s" % (v, k) for k, v in sorted(cls_id.items(), key=lambda kv: kv[1])))
