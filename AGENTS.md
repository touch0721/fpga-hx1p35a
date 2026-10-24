# 本工作区说明（给 AI 助手）

**⚠️ 动手之前，先完整读一遍：**

    handover/720p60迁移与识别_交接包.txt        ← ★ 最新（§13 阶段2/瘦身、§15 固定 ROI 检测、
                                               §16 固定 14 牌位、**§17 推翻全部识别方案：
                                               删识别 + SW3 二值化 + 牌位固定红线**）

参考文档（都还在，但要看清楚哪部分已过期）：

    handover/帧差检测与HDMI时序_工作交接.txt    1080p60 时代的上下文；
                                               ★ 只有第 9 节（综合 merge_equal off + TimingUltra）仍有效，
                                                 其它现状描述已被 720p60 还原取代
    handover/麻将牌识别_RTL设计方案.txt         识别模块设计文档；
                                               ★ 阶段 2 的 F3（8x8 块 + 行扫描跨行合并）已被
                                                 交接包 §13 的 4x4 位图 + 列投影实现取代

---

## 一句话现状

安路 TD 工程 `lab_ex1_mipi_hdmi`（PH1P35MDG324），已还原成 **720p60**：

| 项目 | 当前值 |
|---|---|
| 采集 | SC500CS 1280x720（VTS=1200 -> 50 fps，消工频闪），MIPI 480 Mbps/Lane，2 Lane RAW10 |
| PLL | VCO 1500 MHz；clk0 100 MHz、clk1 23.81 MHz、**像素 75 MHz、串行 375 MHz** |
| HDMI 输出 | uivtc 1280x720 / HTS 1650 / VTS 750；`hdmi_tx.v` 里 HTOTAL 1650、**VIC 4** |
| 时序（最近一次布局布线） | Setup WNS **+1262 ps**、0 违规端点；Hold WNS +20 ps、0 违规 |
| | 逐时钟 SWNS：clk0 +1.782 / clk2 **+1.589** / clk5 **+1.262**（其余全正） |
| 资源（那一版） | LUT 17429(82.15%) / REG 19865 / **SLICE 19118(90.11%)** / ERAM 58 / **DSP 33** |
| 图像通路 | Sobel 保留（不参与显示）；投影法 bin_proj_bound 已停用（常量输出） |
| **识别 / 帧差 / ROI 检测** | ★★ **全部不例化**（§17：用户推翻了所有识别方案）—— 硬件里没有任何识别逻辑；模块源码与内联副本仍在仓库里，恢复只要把 assign 换回例化 |
| **SW3** | ★ **二值化显示**：亮度 `(R+2G+B)/4 < BIN_THR(150)` -> 暗=白、亮=黑（阈值在 mixer 里改）。SW1/SW2/SW4 现在是空功能 |
| **屏幕叠加** | ★ 三条红的东西：**13 张牌红框**（`x 56..1062, y 240..351`，右边界已随红线加宽 15）+ **16 条牌位竖线**（**第 8 条起整体右移 15**：`56,132,208,284,360,436,512,603,679,755,831,907,983,1059` + 小框 `1111/1183`）+ 左上角**一位数字** |
| **识别（第一版，§18）** | ★ 只接**最左第一个牌位（槽 0）**：网格交点采样（x/y 每 5 像素一个交点，采样区 `x 66..121, y 250..341` 内缩 10 px），**白 1 / 黑 0**，判据 **全 1 = 白板**。OSD：**1 = 识别到，0 = 没识别到**。只存一个"全白"标志（不存矩阵），约 40~60 LUT |
| OSD | **只 1 个字符**（`OSD_NC=1`，`OSD_EN=1'b1`），就是上面那个 0/1 |
| bit | ★ 已留档：[artifacts/20261015-720p60-sobel-fulldiff-recog/](artifacts/20261015-720p60-sobel-fulldiff-recog/README.md)（含 bit + 时序/资源报告 + 复现脚本，git 标签 `state-board-20261015-720p60-sobel-fulldiff-recog`）。原始产物在 `td_project/.../phy_recog2_cli/`（该目录被 .gitignore 排除，不入库） |

★ **用户的目标是【削减需要资源】**：任何新功能优先"用 FF/ERAM 换 LUT"（FF 现在只用 ~47%，
  ERAM 53.7%），因为器件瓶颈是 **slice ≈ LUT + 进位链**。判断一个改动先问"LUT 省了吗"。
★★ **§16/§17 两轮改了代码，但【没跑回归、没提交 git】** —— 当时 shell 沙箱坏了
  （`windows-acl-run: --temp is not an existing directory`，所有 pwsh 命令都失败）。
  接手第一件事：修 shell -> 跑 `sim/check_ports.py` + 三份内联同步 + 三个 TB + 联合编译，
  全绿再提交（提交信息英文 ASCII）。
★ §17 之后硬件里**没有任何识别/帧差/ROI 检测逻辑**（都不例化 = 综合剪掉，0 资源）：
  屏上只有摄像头画面 + 14 个牌位的固定红线；SW3 = 二值化。
  模块源码（tile_recog_onchip.v / roi_motion_onchip.v / frame_diff_onchip.v）与
  wrapper 里的三份内联副本**都还在**，只是没例化 —— 恢复只要把常量 assign 换回例化。

★ **90.11% slice 已经很紧**：再加新功能（比如 11.5 的 16x16 模板匹配，估 600~900 LUT）
  会逼近 95%，届时要先删东西：C2 减配（列投影 1 像素列 -> 2 像素列，约省 500 slice）、
  NSLOT 16->12、删 Sobel 通路、或 11.3 行缓冲深度参数化。
★ 识别模块的瘦身原则：**只算【选中槽】那一张牌**（阶段 1 累加器 + 阶段 2 位图/投影都是
  一份，不是 16 份）—— 这是从 106% slice 降到 90% 的那一刀，改回 16 槽就会装不下。

★ 综合收敛靠两条参数，**RTL 一行没改**：`rtl merge_equal off`（阻止 4 个 HDMI lane 的
等价逻辑被合并成一个寄存器，扇出 40 → 10）+ 布局布线 TimingUltra。详见旧交接文档第 9 节。
参数已由用户在 TD 的 **"More Options"** 里填好，TD 自己写进了 `.al`
（`<RtlProperty>/<PlaceProperty>/<RouteProperty>` + `<UserParams><param>`），
所以**在 TD 里点 Run 也会带上这些参数**，不会再退回默认档。
改参数请继续走 More Options，**绝不手写 `.al`**（TD 要把它解析成带类型的 XML）。

---

## 五条硬性规则

0. ★★ **不要跑综合/布局/bitgen —— 用户自己跑。**
   AI 只负责：改 RTL/约束/脚本、跑仿真与静态检查（iverilog、`sim/check_*.py`）、写交接文档、提交 git。
   **不要**调用 `td_commands_prompt.exe`、不要建 `syn_*_cli/phy_*_cli` 之类的 run 目录、
   不要跑 `launch_runs`。改完代码把「改了什么 + 怎么验证 + 期望看到什么」讲清楚，编译交给用户。
   （原因：用户要在 TD GUI 里自己控制综合/布局与烧录节奏，AI 跑一遍要 3~15 分钟且会占用工程状态。）

1. **绝不编辑 `td_project/*.al`** —— TD 会用内存里的文件列表覆盖它。
   要加代码就把模块**内联**进 `design_top_wrapper.v` 或 `isp/isp_top.v`。

2. **改了源文件必须重新内联**，改完必须跑 `sim/check_ports.py`：
   | 改了 | 怎么内联 |
   |---|---|
   | `frame_diff_onchip.v` | 手工替换 `design_top_wrapper.v` 里的内联副本 |
   | `tile_recog_onchip.v` | `python sim/inline_tile_recog.py` |
   | `roi_motion_onchip.v` | `python sim/inline_roi_motion.py` |
   | `sobel_edge_128.v` | `python sim/inline_sobel.py` |
   ★ **加/改端口时，要改的是源文件，不是内联副本** —— 手改副本有两个后果：
   与源文件脱同步（下次跑 inline 脚本被冲掉）、漏掉包装层端口，TD 综合时报
   `ERROR: cannot find port 'XXX' on this module`。

3. **不要擅自改显示时序参数**（`uivtc` 的 `H_FrameSize` 等）和**二值帧差阈值 `DIFF_THR`**。
   ★ 分辨率迁移不止改 IMG_*：还必须同步
   `hdmi_tx.v` 的 HTOTAL/VTOTAL/HACTIVE/VACTIVE/**VIDEO_VIC**（漏了它显示器报无信号）、
   `uics520_cfg/uicfgcs520.v` 的 `POST_STREAM_WRITE` 与例化的表名、
   `td_project/camera_to_dsi_display.sdc` 的 MIPI 时钟周期、
   `design_top_wrapper.v` 的 `S_mipi_last_line_error` 行数常量、GBX_X0/Y0。

4. **算术 `*` 一律手写成移位加法** —— 器件 DSP 上限 40，基线已用 **33，只剩 7 个余量**。
   TD 会把 Verilog 的 `*`（**包括"乘以常量"**）映射成 DSP：实测识别模块里 6 个 `*`
   就把 DSP 推到 54，布局器直接报 `PHY-7027 model type (DSP) exceed the design
   limits` + `PHY-9002 Placer initialization failed`（PHY-9146/9147/9179 是陪衬）。
   ★ 同样危险的是**把 ISP 里已经死掉的乘法器引出**：引出 `s1_cb/s1_cr` 会让
     `rgb_to_ycbcr_4` 里 24 个常量乘法器复活，DSP 33 → 49。要色彩信息就取 ISP 的
     `O_col4`（比较器算出的色系码，零乘法）。
   ★ 定位工具：`python sim/dsp_by_module.py <门级网表>` 按模块列出 DSP 分布。

5. **改 PLL 要认清真值源** —— TD 编译时会**重新生成** `user_source/ip_source/PLL/PLL.v`，
   它是按 `user_source/ip_source/PLL/.params_PLL.txt` 里的 `CLK4_REQUEST_FREQ_DISP` 生成的。
   只改 `PLL.v` 会被覆盖。要改频率就改 `.params_PLL.txt`（并让 `PLL.v` 的分频比/注释一致）。
   判断方法：编译后看 Clock Summary 里 clk2 是不是期望的频率。

---

## 阶段 2 细粒度计数的实现要点（本轮新增，改它之前先看这里）

* 位置：`tile_recog_onchip.v` 第 ⑧ 节（内联副本在 `design_top_wrapper.v` 末尾）。
* **只数【选中槽】那一张牌**：位图 20x24 = 480 位、列投影 80x7 位，各只有一份。
* 粒度：**4x4 像素 = 1 格**（一拍 = 一个格列，4 行 = 一个格行）。一张立牌 ≈ 17x23 格。
* **筒 = 连通块数**：位图 + 8 邻域连通域标记，用"即时重标号"（当场把输的标签在上/本两行
  里改成赢的、尺寸并过去、输的清零）——不是"合并表 + 收尾并查集"，收尾只数尺寸 >= DOT_MIN 的标签。
* **条 = 列投影峰数**：逐像素列投影，门限 = max(固定底噪 BAR_MIN_V, 最大峰/2)，
  峰宽 >= BAR_MIN_W、峰距 < BAR_MIN_G 的两个峰算一个。
* 帧末顺序：`F_RUN`（算选中槽特征，**6 拍**；C1 瘦身后不再是 16 槽 x 6 拍）-> `F_WAITG`
  -> `G`（清 -> 480 格连通域 -> 统计 -> 40~80 列投影 -> 锁存）-> `F_DONE`（切分 FSM 才开始重切）。
  一次约 610 拍，720p60 消隐有 9600 拍。
* ★ 三个已踩的坑（再改这里务必留意）：
  1. **位图/投影的清零必须放在扫描【之后】**（放前面会让 CCL 只看到空图）。
  2. **合并/重标号必须用 `g_occ` 门控**：空格子的左上/上右邻居会同时非零，
     不门控就会把两根竹条并成一根（实测 5 条并成 1 条）。
  3. **对参数做部分选择会得到 x**（`NSLOT[3:0]`、`LAB_MAX[4:0]`）：iverilog 不报错，
     只是让整个模块静默失效（前者让槽号恒 0，后者让帧末 FSM 死循环）。
     要比较就先 `localparam` 定宽再比。
* 计数结果走 OSD 的 `P`/`G` 两行（见下方 OSD 表）。

---

## 工具路径

```
iverilog : C:\Users\touch\.vscode\extensions\iccoderagenticverilogplatform.iccoder-2.0.9-win32-x64\tools\iverilog\win\bin\
python   : C:\Users\touch\.dsh\dsh-runtimes\dsh-primary-runtime\dependencies\python\python.exe
TD CLI   : "D:\anlogic TD\bin\td_commands_prompt.exe"（严格 stdin 喂 Tcl；见交接包 §9）
```

跑 iverilog 前先 `New-Item -ItemType Directory -Force "sim\_tmp"` 并设 `TMP`/`TEMP`。
PowerShell 里不要用内联 `python -c`（引号/中文必炸），写成 `.py` 文件再执行。
控制台看中文输出前先 `[Console]::OutputEncoding = [Text.Encoding]::UTF8`。

---

## 改完必做

```
python sim/check_ports.py            # ★ 先跑这个：例化端口名 vs 模块定义逐一对
python sim/inline_tile_recog.py      # 改了 tile_recog_onchip.v 才需要
python sim/inline_roi_motion.py      # 改了 roi_motion_onchip.v 才需要
python sim/inline_sobel.py           # 改了 sobel_edge_128.v 才需要
python sim/check_inline_fdiff.py                                     # frame_diff
python sim/check_inline_fdiff.py tile_recog_onchip ^
       user_source/hdl_source/design_top_wrapper.v ^
       user_source/hdl_source/tile_recog_onchip.v                    # tile_recog
python sim/check_inline_fdiff.py roi_motion_onchip ^
       user_source/hdl_source/design_top_wrapper.v ^
       user_source/hdl_source/roi_motion_onchip.v                    # roi_motion
python sim/check_inline_fdiff.py sobel_edge_128 ^
       user_source/hdl_source/isp/isp_top.v ^
       user_source/hdl_source/isp/sobel_edge_128.v                   # sobel
python sim/check_static.py           # 44 通过 / 0 失败 / 3 跳过（厂商 cfg_cs500.c 缺失时后 3 项跳过，正常）
iverilog -g2012 -o fd.vvp sim/tb_fdiff.v user_source/hdl_source/frame_diff_onchip.v
vvp fd.vvp                           # 期望 FDIF ALL CHECKS PASSED
iverilog -g2012 -o tr.vvp sim/tb_tile_recog.v user_source/hdl_source/tile_recog_onchip.v
vvp tr.vvp                           # 期望 TILE RECOG ALL CHECKS PASSED（41 项）
iverilog -g2012 -o roi.vvp sim/tb_roi_motion.v user_source/hdl_source/roi_motion_onchip.v
vvp roi.vvp                          # 期望 ROI MOTION ALL CHECKS PASSED（16 项）
iverilog -g2012 -t null -o nul user_source/hdl_source/design_top_wrapper.v
                                     # 判据：syntax error 必须为 0（Unknown module type 正常）
iverilog -g2012 -t null -o nul user_source/hdl_source/design_top_wrapper.v ^
         user_source/hdl_source/isp/isp_top.v user_source/hdl_source/hdmi_mixer.v
                                     # 判据：**非 "Unknown module type" 的错误必须为 0**
                                     #   （不要用关键词过滤！掏空模块体那类错 iverilog 只当警告）
```

★ `sim/tb_multi.v` 测的是**已停用**的投影法模块（`bin_proj_bound` 被掏空成常量输出），
  现在跑必然 4 项失败 —— 这是**预期**，不是回归，不必修。

I2C 时序回归是**两步** —— 日志是仿真产物，被 `.gitignore` 排除，不会随仓库带过来，
直接跑检查器会报 FileNotFoundError：

```
iverilog -g2012 -o sim/tb_uicfgcs520.vvp ^
         -y user_source/hdl_source/uics520_cfg ^
         -y user_source/hdl_source/uics500_cfg ^
         sim/tb_uicfgcs520.v ^
         user_source/hdl_source/uics520_cfg/uics520reg_720p60.v
vvp sim/tb_uicfgcs520.vvp > sim/tb_run.log
python sim/check_i2c_seq.py          # 期望通过 24 项
```

（`^` 是 cmd 的续行符；PowerShell 里换成反引号或写成一行。）

★ 末尾那个显式列出的文件**不能省**：模块名 `uics520reg_1080p60` 与它所在的
文件名 `uics520reg_720p60.v` 不一致，而 `-y` 库查找靠"文件名 = 模块名"匹配，
省掉就报 `Unknown module type: uics520reg_1080p60`。

---

## 固定 ROI 触发检测（roi_motion_onchip，§15 引入）★ §17 已解除例化（代码保留，未例化 = 0 资源）

* 两个 ROI（都按 8 像素块边界对齐，**与 mixer 画的红框坐标完全一致**）：
  主框 `x 56..1047, y 240..351`（≈13 张牌那排）；小框 `x 1096..1167, y 240..351`。
* 粒度是 **8 像素宽的"列条"**（8 × ROI 高 112 = 896 像素）：主框 **124 条**、小框 **9 条**。
  ★ 不是 8×8 块：按 (块行,块列) 存要 14×133 = 1862 项，光地址译码就上千 LUT。
* 列条平均 = `累加 >> AVG_SH`（AVG_SH = ceil(log2(8×ROI 高)) = 10，**近似整除**，全屏统一缩放）。
* 判据：`|平均−背景| > DIFF_THR` 记 1 条 → 帧末 `条数 ≥ THR_x` → present；`HOLD` 帧防抖。
* 背景：**KEY4 拨上抓（空桌）、拨下冻结检测**；`O_bg_rdy` 没抓好前不报 present。SW4 = 总使能。
* 存储用寄存器不用 ERAM（133 × 26 位 ≈ 3.4K FF）；FF 富余，ERAM 留给别处。
* ★ 三个已踩的坑：
  1. **结算触发要"块行最后一行"**（`brow == CR_H && line[2:0] == 3'b111`）：只判 brow 会让
     ROI 末块行的 8 个行末各触发一次，把只累了一半的值读走清零。
  2. **4 像素 Y 的拍和要 10 位**：4×255 = 1020，9 位会被截成 mod 512（800 → 288）。
  3. **阈值量纲跟着粒度走**：列条上限只有 124 / 9，照 8×8 块的 1736 估出来的阈值永远触发不了。
* 屏上怎么看：**红框 = 没牌，绿框 = 有牌**；OSD 的 `A`/`B` 两行就是两路的列条计数（标阈值用）。

---

## OSD（§18：只显示 1 个字符）

`OSD_EN = 1'b1`、`OSD_NC = 12'd1` —— 左上角画**一位数字**：**1 = 槽 0 的采样交点全为白（判为白板）**，
**0 = 至少有黑点（没识别到）**。字符码在 mixer 的 `osd_ch = sam_flag ? 8'h31 : 8'h30` 里。
★ 要恢复原来的多行文字（F/M/N/C/A/P/G 或 A/P/G），需要同时把 `OSD_NC` 改回 5、
区域高度改回 48/112，并恢复对应的数据源（识别模块要重新例化）。
