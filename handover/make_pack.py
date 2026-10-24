# -*- coding: utf-8 -*-
"""
make_pack.py — 从当前工程生成单文件移植包 handover/sc500cs_cam_pack.v

做四件事：
  1. 抽取 7 个模块：I2C 主机、I2C 定序器、初始化表、AE 表、增益表、BLC、Gamma
  2. 定序器可移植化改写：
       - 去掉对按键/AE 控制器的依赖：新增 AE_DEFAULT / AG_DEFAULT / AE_ONLINE 参数，
         AE_ONLINE=0 时配置完成后自动写一次 AE 表（原来的 ae_set 就是干这个的）
       - 5ms/10ms 延时改为由 CLK_HZ 参数换算
  3. 统一改名加 sc500cs_ 前缀，避免与目标工程已有模块重名
  4. 输出单文件（含目录、模块分段注释、移植说明头）

用法：  python handover/make_pack.py
"""
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "user_source", "hdl_source")
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sc500cs_cam_pack.v")

FILES = {
    "i2c_master": os.path.join(SRC, "uics500_cfg", "uii2c.v"),
    "seq":        os.path.join(SRC, "uics520_cfg", "uicfgcs520.v"),
    "reg_init":   os.path.join(SRC, "uics520_cfg", "uics520reg_720p60.v"),
    "reg_ae":     os.path.join(SRC, "uics520_cfg", "uics520regAE.v"),
    "gain":       os.path.join(SRC, "uics520_cfg", "SC520GainTbl.v"),
    "isp":        os.path.join(SRC, "isp", "isp_top.v"),
}

RENAME = [
    ("uics520reg_720p60", "sc500cs_reg_init"),
    ("uics520regAE",      "sc500cs_reg_ae"),
    ("SC520GainTbl",      "sc500cs_gain_tbl"),
    ("uicfgcs520",        "sc500cs_cfg"),
    ("uii2c",             "sc500cs_i2c_master"),
    ("axis_gamma_rgb96",  "sc500cs_gamma_rgb96"),
    ("blc_raw40",         "sc500cs_blc_raw40"),
]


def read_any(path):
    b = open(path, "rb").read()
    txt = None
    for enc in ("utf-8", "gbk"):
        try:
            txt = b.decode(enc)
            break
        except UnicodeDecodeError:
            continue
    if txt is None:
        txt = b.decode("latin-1")
    # 统一成 LF，避免 Windows 文本模式下 \r\n 被写成 \r\r\n（产生空行）
    return txt.replace("\r\n", "\n").replace("\r", "\n")


def extract_module(text, name):
    """取出 `module <name> ... endmodule` 整段。"""
    m = re.search(r"(?m)^module\s+%s\b.*?^endmodule" % re.escape(name), text, re.S)
    if not m:
        print("FAIL 找不到模块 %s" % name)
        sys.exit(1)
    return m.group(0)


def sub_once(text, pat, repl, label, flags=re.S):
    new, n = re.subn(pat, repl, text, count=1, flags=flags)
    if n != 1:
        print("FAIL [%s] 匹配 %d 次（期望 1）" % (label, n))
        sys.exit(1)
    return new


def rename_all(text):
    for a, b in RENAME:
        text = text.replace(a, b)
    return text


# ---------------------------------------------------------------- 读源文件
txt = {k: read_any(v) for k, v in FILES.items()}

seq = extract_module(txt["seq"], "uicfgcs520")
i2c = extract_module(txt["i2c_master"], "uii2c")
reg_init = extract_module(txt["reg_init"], "uics520reg_720p60")
reg_ae = extract_module(txt["reg_ae"], "uics520regAE")
gain = extract_module(txt["gain"], "SC520GainTbl")
blc = extract_module(txt["isp"], "blc_raw40")
gam = extract_module(txt["isp"], "axis_gamma_rgb96")

# ------------------------------------------------- 定序器可移植化改写
seq = sub_once(
    seq,
    r"module uicfgcs520 #\(\s*parameter integer CLK_DIV = 16'd239\s*\) \(",
    """module sc500cs_cfg #(
    // I2C SCL 分频：SCL 频率 = CLK_HZ / (2*(CLK_DIV+1))
    parameter integer CLK_DIV    = 16'd238,
    // I_clk 频率，用于把 5 ms / 10 ms 延时换算成周期数（换主频时只改这里）
    parameter integer CLK_HZ     = 23809524,
    // ---- 曝光/增益默认值（无按键方案）----
    // 曝光单位 1/2 行，合法范围 1 .. 2*VTS-10；VTS=1000 时上限 1990
    parameter [15:0]  AE_DEFAULT = 16'd1990,
    // 增益档位 16..144 -> 模拟增益 1.000x..15.750x（见数据手册表 2-4）
    parameter [15:0]  AG_DEFAULT = 16'd144,
    // 0 = 不用按键/外部 AE，配置完成后自动按上面两个常量写一次 AE 表
    // 1 = 由外部提供 I_ae / I_ag，并在 I_ae_req 有效时写 AE 表
    parameter         AE_ONLINE  = 1'b0
) (""",
    "定序器模块头",
)

seq = sub_once(
    seq,
    r"(localparam \[7:0\] CAM_ID = 8'h6c;)",
    r"""\1

    // ---- AE 取值与触发 ----------------------------------------------
    // AE_ONLINE=0：忽略 I_ae/I_ag/I_ae_req，配置完成后自动用 AE_DEFAULT/AG_DEFAULT
    //              写一次 AE 表（等价于原工程里 ae_set 上电后的那一次写入）。
    // AE_ONLINE=1：由外部给 I_ae/I_ag，I_ae_req 有效时写一次 AE 表。
    wire [15:0] ae_use  = AE_ONLINE ? I_ae : AE_DEFAULT;
    wire [15:0] ag_use  = AE_ONLINE ? I_ag : AG_DEFAULT;
    reg         ae_oneshot;                    // AE_ONLINE=0 时的一次性写请求
    wire        ae_trig = AE_ONLINE ? I_ae_req : ae_oneshot;""",
    "AE 取值逻辑",
)

seq = sub_once(
    seq,
    r"localparam integer RESET_WAIT_CYCLES  = 119048; // 5 ms  at 23\.809524 MHz\s*\n"
    r"\s*localparam integer STREAM_WAIT_CYCLES = 238095; // 10 ms at 23\.809524 MHz",
    "localparam integer RESET_WAIT_CYCLES  = CLK_HZ / 200; // 5 ms\n"
    "    localparam integer STREAM_WAIT_CYCLES = CLK_HZ / 100; // 10 ms",
    "延时周期换算",
)

seq = sub_once(
    seq,
    r"    // 该常量必须与 uics520reg_720p60\.v 内的 STREAM_ON_INDEX 保持一致。",
    "    // 该常量必须与 sc500cs_reg_init 内的 STREAM_ON_INDEX 保持一致。",
    "STREAM_ON_INDEX 注释",
)

seq = sub_once(
    seq,
    r"(            ae_table_active  <= 1'b0;)",
    r"\1\n            ae_oneshot       <= 1'b0;",
    "复位 ae_oneshot",
)

seq = sub_once(
    seq,
    r"(                        if \(reg_index == init_reg_size\) begin\s*\n"
    r"                            O_cfg_done <= 1'b1;\s*\n"
    r"                            O_ae_cfg_done <= 1'b1;)",
    r"\1\n                            ae_oneshot <= AE_ONLINE ? 1'b0 : 1'b1;",
    "配置完成后置 oneshot",
)

seq = sub_once(
    seq,
    r"                    end else if \(I_ae_req\) begin\s*\n"
    r"                        ae_table_active <= 1'b1;\s*\n"
    r"                        O_ae_cfg_done <= 1'b0;\s*\n"
    r"                        reg_index <= 9'd0;\s*\n"
    r"                    end",
    "                    end else if (ae_trig) begin\n"
    "                        ae_table_active <= 1'b1;\n"
    "                        O_ae_cfg_done <= 1'b0;\n"
    "                        reg_index <= 9'd0;\n"
    "                        ae_oneshot <= 1'b0;\n"
    "                    end",
    "ST_IDLE 用 ae_trig",
)

seq = sub_once(seq, r"\.I_AG\(I_ag\),", ".I_AG(ag_use),", "增益表输入")
seq = sub_once(seq, r"\.I_AE\(I_ae\),", ".I_AE(ae_use),", "AE 表输入")

# ---------------------------------------------------------------- 改名
seq = rename_all(seq)
i2c = rename_all(i2c)
reg_init = rename_all(reg_init)
reg_ae = rename_all(reg_ae)
gain = rename_all(gain)
blc = rename_all(blc)
gam = rename_all(gam)

# 头注释里"本工程原命名"的说法已不适用，改掉
seq = seq.replace(
    "SC500CS 摄像头 I2C 配置定序器（模块名沿用工程原命名，寄存器内容已适配 SC500CS）。",
    "SC500CS 摄像头 I2C 配置定序器（无按键版：曝光/增益由参数给出）。",
)

HEADER = r"""// ============================================================================
//  sc500cs_cam_pack.v  —  SC500CS 摄像头配置 + 黑电平校正 + Gamma  单文件移植包
// ============================================================================
//  本文件由 lab_ex1_mipi_hdmi(SC520CS) 工程整理而成，可直接复制到新工程使用。
//  全部模块名统一加 sc500cs_ 前缀，避免与目标工程已有模块重名。
//
//  ---------------------------------------------------------------------------
//  包含模块（按依赖顺序）
//  ---------------------------------------------------------------------------
//   1. sc500cs_i2c_master   I2C 主机控制器（位级时序，开漏 SDA，支持随机读）
//   2. sc500cs_reg_init     初始化寄存器表 117 条（1280x720@60，480 Mbps/Lane）
//   3. sc500cs_reg_ae       曝光/增益表 7 条
//   4. sc500cs_gain_tbl     增益档位 -> {0x3e08,0x3e09}
//   5. sc500cs_cfg          I2C 定序器（读 ID -> 写 117 条 -> 写 AE 表）
//   6. sc500cs_blc_raw40    黑电平校正（RAW10 域，放在 demosaic 之前）
//   7. sc500cs_gamma_rgb96  Gamma 校正（RGB888 域，放在 AWB 之后）
//
//  ---------------------------------------------------------------------------
//  典型接法
//  ---------------------------------------------------------------------------
//    !!! 注意三者的位置和时钟域都不同，不要串在一起 !!!
//
//    [24MHz 域]  sc500cs_cfg          -> O_cam_scl / IO_cam_sda 接摄像头
//    [CSI 时钟域] I_raw_tdata(40bit) -> sc500cs_blc_raw40 -> demosaic
//    [CSI 时钟域] awb 输出(96bit)     -> sc500cs_gamma_rgb96 -> 96转128 -> DDR
//
//  最省事的用法（不需要任何按键或外部 AE 控制器）：
//      sc500cs_cfg #(
//          .CLK_DIV   ( CLK_HZ/100000 - 1 ),   // SCL ~100 kHz
//          .CLK_HZ    ( 23809524          ),   // 与 I_clk 实际频率一致
//          .AE_DEFAULT( 16'd1990          ),   // 曝光：1/2 行为单位，VTS=1000 时上限 1990
//          .AG_DEFAULT( 16'd144           ),   // 增益档位：144 -> 15.750x（表 2-4 上限）
//          .AE_ONLINE ( 1'b0              )    // 不用按键
//      ) u_cam_cfg (
//          .I_clk(I_clk_24m), .I_rst_n(rst_n),
//          .I_ae_req(1'b0), .I_ae(16'd0), .I_ag(16'd0),   // AE_ONLINE=0 时这三个脚被忽略
//          .O_cam_scl(O_cam_scl), .IO_cam_sda(IO_cam_sda),
//          .O_cfg_done(cam_cfg_done), .O_ae_cfg_done(),
//          .O_sensor_id(), .O_sensor_id_ok(), .O_iic_error()
//      );
//
//  ---------------------------------------------------------------------------
//  移植必查项
//  ---------------------------------------------------------------------------
//   1. I_clk 频率：不是 23.809524 MHz 就改 CLK_HZ（5ms/10ms 延时与 SCL 都靠它）
//   2. I2C 从机地址：CAM_ID = 0x6C 对应 SID 引脚为低。SID 拉高时为 7'h10 -> 0x20
//   3. Sensor ID：校验值 0xEE27（SC500CS 数据手册表 1-6）。换料号要同步改
//   4. 分辨率/帧率：改 sc500cs_reg_init 的 0x3208/09（宽）、0x320a/0b（高）、
//      0x320e/0f（VTS）；PCLK = HTS*VTS*fps，Lane 速率 = PCLK*10/2
//   5. LUT 例化处要跟着改：demosaic 的 IMG_WIDTH/IMG_HEIGHT/BAYER_MODE、
//      awb 的 IMG_WIDTH/IMG_HEIGHT
//   6. 时钟约束：MIPI 时钟周期 = 1/(Lane速率/2)。480 Mbps/Lane -> 4.167 ns
//   7. BLC 偏置 15（10bit 域）；Gamma 曲线见 sc500cs_gamma_rgb96 内的 BASE/DELTA 表
//
//  ---------------------------------------------------------------------------
//  验证
//  ---------------------------------------------------------------------------
//   同目录下 tb_sc500cs_pack.v 是自检 testbench（含 I2C 从机模型），
//   不依赖任何本工程文件，可直接在新工程里跑：
//       iverilog -g2012 -o tb.vvp tb_sc500cs_pack.v sc500cs_cam_pack.v
//       vvp tb.vvp
//   全部输出 PASS 即说明：ID 校验、117 条写序列、5ms/10ms 时序、
//   自动 AE 写入、BLC 算术、Gamma 映射都正确。
//
//  ---------------------------------------------------------------------------
//  来源与依据
//  ---------------------------------------------------------------------------
//   - SC500CS 数据手册 V1.8（表 1-2/1-3/1-4/1-6/1-7/1-9、表 2-1~2-10）
//   - 安路官方 lab_ex2_edge 工程内 SC500CS 原厂 720p60 全表（116 条）
//   - 厂商 SDK cfg_cs500.c（交叉验证 I2C 地址与寄存器族）
// ============================================================================

`timescale 1ns / 1ps

"""

TOC = """
// ============================================================================
//  [1/7] sc500cs_i2c_master — I2C 主机控制器
// ============================================================================
"""

parts = [
    HEADER,
    "\n// ============================================================================\n"
    "//  [1/7] sc500cs_i2c_master — I2C 主机控制器（原 uii2c.v）\n"
    "// ============================================================================\n",
    i2c,
    "\n// ============================================================================\n"
    "//  [2/7] sc500cs_reg_init — 初始化寄存器表 117 条\n"
    "//        1280x720@60 / 24MHz MCLK / 2-Lane RAW10 / 480 Mbps per Lane\n"
    "//        索引 0 = 软复位，索引 115 = stream on，索引 116 = 0x302d=0\n"
    "// ============================================================================\n",
    reg_init,
    "\n// ============================================================================\n"
    "//  [3/7] sc500cs_reg_ae — 曝光/增益表 7 条\n"
    "//        曝光 {0x3e00,0x3e01,0x3e02[7:4]}；模拟增益 0x3e08/0x3e09；数字增益 1.000x\n"
    "// ============================================================================\n",
    reg_ae,
    "\n// ============================================================================\n"
    "//  [4/7] sc500cs_gain_tbl — 增益档位 -> {0x3e08, 0x3e09}\n"
    "//        I_AG 16..144 映射 1.000x..15.750x，与数据手册表 2-4 的 128 行一一对应\n"
    "// ============================================================================\n",
    gain,
    "\n// ============================================================================\n"
    "//  [5/7] sc500cs_cfg — I2C 定序器（无按键版）\n"
    "//        读 0x3107/0x3108 校验 ID=0xEE27 -> 写 117 条 -> 写 AE 表\n"
    "//        参数 AE_DEFAULT/AG_DEFAULT/AE_ONLINE 控制曝光增益，无需按键\n"
    "// ============================================================================\n",
    seq,
    "\n// ============================================================================\n"
    "//  [6/7] sc500cs_blc_raw40 — 黑电平校正（RAW10，4 像素/拍）\n"
    "//        接在 demosaic 之前；逐通道减偏置并下钳位到 0，一级流水\n"
    "// ============================================================================\n",
    blc,
    "\n// ============================================================================\n"
    "//  [7/7] sc500cs_gamma_rgb96 — Gamma 校正（RGB888，4 像素/拍）\n"
    "//        接在 AWB 之后、96->128bit 打包之前；16 段折线近似 y=255*(x/255)^0.7\n"
    "//        0x00->0x00(保黑)、0x40->0x61、0x80->0x9d、0xc0->0xd1、0xff->0xff(保白)\n"
    "// ============================================================================\n",
    gam,
    "\n",
]

os.makedirs(os.path.dirname(OUT), exist_ok=True)
with open(OUT, "w", encoding="utf-8", newline="\n") as f:
    f.write("".join(parts))

n_mod = len(re.findall(r"(?m)^module\s", "".join(parts)))
print("已生成 %s" % OUT)
print("  模块数 = %d（应为 7）" % n_mod)
print("  行数   = %d" % "".join(parts).count("\n"))
for k in ("sc500cs_i2c_master", "sc500cs_reg_init", "sc500cs_reg_ae",
          "sc500cs_gain_tbl", "sc500cs_cfg", "sc500cs_blc_raw40",
          "sc500cs_gamma_rgb96"):
    print("  %-22s %s" % (k, "OK" if re.search(r"(?m)^module\s+%s\b" % k, "".join(parts)) else "缺失!"))
for k in ("uii2c", "uicfgcs520", "uics520reg", "SC520GainTbl", "axis_gamma_rgb96", "blc_raw40"):
    if re.search(r"(?m)^module\s+%s\b" % k, "".join(parts)):
        print("  !! 旧模块名残留: %s" % k)
