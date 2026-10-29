/*********************************************************************
* 文件：BASIC_HelloWorld.c
* 说明：远程数据采集系统 —— 显示端（B 板）主界面
*
*   数据来源：协议栈 proto.c；当前由 fakesrc.c 提供假数据帧
*             （后续换成 USART6 接收即可直接接 A 板）。
*
*   界面：
*     - 顶部触摸按钮：时基档位循环、CH1/CH2/D1/D2/D3 开关
*     - 中部：10 格 × 8 格栅格 + CH1(蓝)/CH2(红) 模拟波形
*     - 下部：D1/D2/D3 三路数字波形（与模拟通道共用时间刻度）
*     - 底部：测量结果（频率、峰峰值、脉宽）
*
*   交互：每点一次按钮，本地状态立即更新，同时通过 USART6
*         向 A 板下发 9 字节控制帧（协议 V1.1）。
*
* 屏幕：480x272，ARGB8888，帧缓冲 SDRAM @0xC0000000
*********************************************************************/

#include "main.h"
#include "GUI.h"
#include "proto.h"
#include "fakesrc.h"
#include "touch.h"
#include "uart_rx.h"
#include "log_uart.h"
#include "export.h"
#include "usart.h"
#include "net_udp.h"
#include "lwip.h"

/* ==================== 数据源选择 ====================
   三者互斥，取消注释其中一个：
     RX_SOURCE_UART：USART6 接 A 板数据（921600 8N1，PC6/PC7），适用于 10ms/5ms 档。
     RX_SOURCE_UDP ：以太网 UDP 接 A 板数据（监听 5005），1ms 及更快档、≥5m 验收必须用这条。
     都不定义      ：使用板内假数据源 fakesrc，单板即可完整自测波形与测量。
   三条路径共用同一个协议解析器 proto，切换不影响其它任何功能。
   ==================================================== */
#define RX_SOURCE_UART
// #define RX_SOURCE_UDP    /* 无网口/U 盘网卡时先注释，用假数据自测 */

#if defined(RX_SOURCE_UART) || defined(RX_SOURCE_UDP)
  #define RX_SOURCE_LINK      /* 有真实对端，不再伪造数据帧与状态帧回执 */
#endif

/* ---------------- 界面布局 ---------------- */
#define SCR_W            480
#define SCR_H            272

#define TOP_H             20          /* 顶部按钮栏高度 */

#define WAVE_Y0           22          /* 模拟通道区起始 y */
#define WAVE_H           130          /* 模拟通道区高度（原 140，让出 10px 给时间轴标签） */
#define DIG_Y0           164          /* 数字通道区起始 y */
#define DIG_LANE_H        26          /* 每路数字通道高度 */
#define MEAS_Y0          244          /* 底部测量信息区起始 y（留 28px，两行不贴边） */

#define DIV_X             10          /* 水平 10 格 */
#define PTS_PER_DIV       50          /* 每格 50 点 */
#define TOTAL_PTS        (DIV_X * PTS_PER_DIV)    /* 一屏 500 点 */

/* 各档位下单点采样间隔（µs），与协议 GEAR 定义一致 */
static const uint16_t dt_us_tab[5] = { 200, 100, 20, 10, 2 };
static const char *gear_txt[5] = { "10ms", "5ms", "1ms", "500us", "100us" };

/* ---------------- 触摸按钮区域 ---------------- */
typedef struct {
  uint16_t x0, x1;
  uint8_t  id;        /* 0=档位；1..5=通道；6=F-；7=F+；8=频率显示区（点开键盘） */
} btn_t;

static const btn_t buttons[] = {
  {   0,  70, 0 },     /* 档位 */
  {  74, 104, 1 },     /* CH1 */
  { 106, 136, 2 },     /* CH2 */
  { 138, 174, 3 },     /* D1  */
  { 176, 212, 4 },     /* D2  */
  { 214, 250, 5 },     /* D3  */
  { 254, 284, 6 },     /* F-  */
  { 288, 318, 7 },     /* F+  */
  { 320, 377, 8 },     /* W: 频率显示区 —— 点击弹出数字键盘 */
  { 400, 470, 9 },     /* SAVE 导出菜单 */
};
#define BTN_NUM   (sizeof(buttons) / sizeof(buttons[0]))

/* ---------------- 触发设置 ----------------
   与 A 板的约定：触发点固定在每帧第 0 个采样点（屏上为每格左边缘），
   预触发深度 0；500ms 内无触发时 A 板自动转自由运行。
   显示端只负责下发配置和显示 A 板的回执。
--------------------------------------------------------- */
#define TRIG_X0     252                 /* 底部测量行右侧的指示区 */
#define TRIG_X1     474
#define TRIG_Y0     (MEAS_Y0 - 1)
#define TRIG_Y1     (MEAS_Y0 + 9)

#define TG_X0        60                 /* 设置面板 */
#define TG_Y0        40
#define TG_X1       420
#define TG_Y1       232
#define TG_ROW1_Y    76
#define TG_ROW2_Y   112
#define TG_ROW3_Y   148
#define TG_ROW_H     30
#define TG_VALUE_X  160
#define TG_VALUE_W  150
#define TG_ADJ_W     40
#define TG_MINUS_X  316
#define TG_PLUS_X   372
#define TG_CLOSE_X  170
#define TG_CLOSE_Y  184
#define TG_CLOSE_W  140

static const char *trig_src_txt[6]  = { "OFF", "CH1", "CH2", "D1", "D2", "D3" };
static const char *trig_mode_txt[3] = { "RISE", "FALL", "ANY" };

/* ---------------- 坐标轴 ----------------
   参照真实示波器：格子里不逐格标数字，只在边缘稀疏标注，避免小屏拥挤。
   Y 轴固定 3 个电压值（不随档位变）；X 轴每个档位只标 首/中/尾 3 个时间值。
--------------------------------------------------------- */
static const char *axis_mid[5] = { "50ms", "25ms", "5ms", "2.5ms", "500us" };
static const char *axis_end[5] = { "100ms", "50ms", "10ms", "5ms", "1ms" };

#define AXIS_X_MID   222              /* 屏幕中点（10 格的中点在第 5 格线右侧） */
#define AXIS_X_END   446              /* 右端，留 34px 给 5~6 字符标签 */
#define AXIS_Y_XLAB  153              /* X 轴标签顶部 y，紧贴波形区底边 152 */

/* ---------------- 本地状态 ---------------- */
static uint8_t g_gear = GEAR_10MS;


/* 档位命令的待确认目标（0xFF = 无待确认请求）。
   B 的档位按钮必须能发出 0~4 全部值，否则 A 实现 CMD01 后无法验证
   GEAR1→GEAR0（A 拒绝 2~4 时，若每次都按 A 的回显递增，就永远发不出 0）。
   做法：每次按键在“上次请求值”上递增并保持 1.2s——
     A 回显等于请求值 → 立即清除（切换成功）；
     超时仍不等 → 清除（A 安全拒绝），下次按键重新从 A 的实际档位递增。
   屏幕档位与时间轴始终取 A 的 GEAR，不显示请求值。 */
#define GEAR_REQ_HOLD_MS  1200
static uint8_t  g_gear_req      = 0xFF;
static uint32_t g_gear_req_tick = 0;
static uint8_t g_mask = MASK_CH1 | MASK_CH2 | MASK_D1 | MASK_D2 | MASK_D3;
static uint8_t g_touch_down = 0;
static uint16_t g_wave_freq = 1000;     /* 矩形波频率（Hz），默认 1kHz，范围 100~10000 */

/* ---- 触发设置（本地值；收到 A 板状态帧后以其回执为准） ---- */
static uint8_t g_trig_src   = 0;        /* 0=OFF 1=CH1 2=CH2 3=D1 4=D2 5=D3 */
static uint8_t g_trig_mode  = 0;        /* 0=上升沿 1=下降沿 2=任意边沿 */
static uint8_t g_trig_level = 128;      /* 0~255，线性映射 0~VREF */
static uint8_t g_trig_open  = 0;        /* 触发设置面板是否弹出 */

/* ---- 数字键盘状态 ---- */
static uint8_t g_kp_open = 0;           /* 键盘是否弹出 */
static char    g_kp_buf[7];             /* 输入缓冲，最多 6 位数字 */
static uint8_t g_kp_len = 0;

/* ---- 导出菜单状态 ---- */
static uint8_t  g_menu_open = 0;        /* 导出菜单是否弹出 */

/* ---------------- 有效通道掩码 ----------------
   A 板数据帧里的 CH_MASK 表示它采了哪些通道，本地 g_mask 表示用户想看哪些。
   两者取交集：任一为关即不显示。
   这样在 A 板尚未实现 CMD 0x02（通道开关）的阶段，B 端仍能本地关掉未接信号的
   通道，避免浮空噪声压住要观察的那一路。
--------------------------------------------------------- */
static uint8_t cur_mask(void)
{
  uint8_t m = proto_stat()->ch_mask;

  return m ? (uint8_t)(m & g_mask) : g_mask;
}

/* ---- 最近一帧的状态：导出 BMP 时用它重绘同一画面 ---- */
static uint32_t g_last_frame   = 0;
static uint8_t  g_last_ts_ok   = 1;
static uint8_t  g_last_link_ok = 1;
#if defined(RX_SOURCE_UDP)
static uint8_t  g_last_phy_up  = 0;     /* 以太网 PHY 是否 link（区分“网线没通”和“通了没数据”） */
#endif
static uint8_t  g_fps          = 0;     /* 实测渲染帧率，用于验收“≥20 帧/秒” */
static uint16_t g_rx_fps       = 0;     /* 实测接收帧率（有效数据帧/秒），联调用 */
/* 临时性能诊断：绘制四阶段耗时(ms)：0=Begin拷贝 1=清屏+波形 2=文字控件 3=End等VSYNC */
static uint8_t  g_dbg_t[4]     = { 0, 0, 0, 0 };

/* 导出菜单布局 */
#define MN_X0       120
#define MN_Y0        80
#define MN_X1       360
#define MN_Y1       210
#define MN_BTN_Y0   115
#define MN_BTN_H     36
#define MN_BTN_W     96
#define MN_CAN_Y    168
#define MN_CSV_X    135
#define MN_BMP_X    249
#define MN_CAN_X    180
#define MN_CAN_W    120

/* 键盘布局常量（弹窗占满屏宽 30~450，高 30~260） */
#define KP_X0        30
#define KP_Y0        30
#define KP_X1       450
#define KP_Y1       258
#define KP_ROW_Y0    96          /* 第一行按键顶部 y */
#define KP_ROW_H     36          /* 按键高度 */
#define KP_ROW_GAP    2
#define KP_COL0_X    45
#define KP_COL_W     85
#define KP_COL_GAP    5
#define KP_FN_X     320          /* 功能列起点 */
#define KP_FN_W     120

/* ---------------- 数据缓冲 ---------------- */
static uint16_t ch1[TOTAL_PTS];     /* raw12 */
static uint16_t ch2[TOTAL_PTS];     /* raw12 */
static uint8_t  dig[TOTAL_PTS];     /* 每点 DIG 字节 */
static uint16_t wpos = 0;           /* 环形缓冲下一个写入位置 */

/* 线性副本：环形缓冲按时间顺序展开后的结果（0 = 最老，TOTAL_PTS-1 = 最新）。
   每帧绘制前由 make_linear_copy() 定格一次；绘制与测量、导出都只读它。 */
static uint16_t s_lin1[TOTAL_PTS];
static uint16_t s_lin2[TOTAL_PTS];
static uint8_t  s_lind[TOTAL_PTS];

/* ---------------- 环形缓冲工具 ---------------- */
static void ring_push(uint16_t r1, uint16_t r2, uint8_t d)
{
  ch1[wpos] = r1;
  ch2[wpos] = r2;
  dig[wpos] = d;
  wpos = (uint16_t)((wpos + 1) % TOTAL_PTS);
}

/* ---- 线性副本（时间顺序：最老 → 最新） ----
   绘制与测量只读这份副本。UDP 模式下渲染过程会穿插轮询收包，
   frame_in() 仍会继续改写原始环形缓冲；若直接读环，同一屏的前后
   半部分会取自不同时刻（画面撕裂、测量值突变）。所以每帧绘制前
   先把窗口定格一次，绘制中进来的新数据留到下一帧。 */
static void make_linear_copy(void);

/* ---------------- 帧入环 ----------------
   由 proto 在每帧 CRC 通过后回调，保证「一次轮询解析出的多帧」全部入环。
   旧做法用 good 计数在主循环里只取最后一帧：真实链路一次轮询会有多帧，
   其余帧被丢掉，500 点窗口就成了间隔 66ms 的非连续片段拼接。
--------------------------------------------------------- */
static uint16_t s_last_r1;
static uint16_t s_last_r2;
static uint8_t  s_last_d;

/* SEQ 断档：不跨缺口拼接，用上一有效样本铺满，等新数据覆盖 */
static void ring_fill(uint16_t r1, uint16_t r2, uint8_t d)
{
  uint16_t k;

  for (k = 0; k < TOTAL_PTS; k++) {
    ch1[k] = r1;
    ch2[k] = r2;
    dig[k] = d;
  }
  wpos = 0;
}

static void frame_in(const proto_frame_t *f)
{
  uint16_t pt;

  if (f->gap) {
    ring_fill(s_last_r1, s_last_r2, s_last_d);
  }

  for (pt = 0; pt < PROTO_N_POINTS; pt++) {
    const uint8_t *p = &f->payload[pt * PROTO_POINT_LEN];
    uint16_t r1 = (uint16_t)((p[PNT_CH1_L] | ((uint16_t)p[PNT_CH1_H] << 8)) >> 4);
    uint16_t r2 = (uint16_t)((p[PNT_CH2_L] | ((uint16_t)p[PNT_CH2_H] << 8)) >> 4);
    uint8_t  d  = (uint8_t)(p[PNT_DIG] & (DIG_D1 | DIG_D2 | DIG_D3));

    ring_push(r1, r2, d);
    s_last_r1 = r1;
    s_last_r2 = r2;
    s_last_d  = d;
  }
}

/* ---------------- 渲染期间轮询以太网 ----------------
   1ms 档 A 板每秒发 1000 帧（260B/包），而一次整屏绘制要 20~40ms。
   若绘制全程不轮询，硬件只有 4 个 RX 描述符、12 个零拷贝接收池，
   几毫秒就溢满，后续帧被 DMA 直接丢弃（屏上 RX 远低于 1000、DROP 持续涨）。
   这里在耗时的绘制循环里按 1ms 节拍穿插轮询：ethernetif_input 内部是
   do/while，一次调用会把 DMA 里已挂起的帧全部取完，所以每个时间片一次即可。
   UART 模式是中断接收 + 8KB 环形缓冲，不需要这段。 */
#if defined(RX_SOURCE_UDP)
static uint32_t s_poll_tick = 0xFFFFFFFFUL;

static void link_poll(void)
{
  uint32_t now = HAL_GetTick();

  if (now == s_poll_tick) {
    return;                     /* 同一毫秒内只轮询一次 */
  }
  s_poll_tick = now;
  MX_LWIP_Process();
}
#else
#define link_poll()   do { } while (0)
#endif

static int raw12_to_y(uint16_t raw)
{
  int32_t v = (int32_t)raw * WAVE_H / 4095L;
  return WAVE_Y0 + WAVE_H - (int)v;
}

static int idx_to_x(int i)
{
  return i * (SCR_W - 1) / (TOTAL_PTS - 1);
}

static uint32_t cur_dt_us(void)
{
  uint8_t g = proto_stat()->gear;
  return (g < 5) ? dt_us_tab[g] : dt_us_tab[0];
}

/* ---------------- 波形直接写帧缓冲 ----------------
   emWin 在 32bpp 非缓存 SDRAM 上逐点/逐列调用约 0.75us/像素，5 路 500 点合计
   30~40ms，是 15fps 的主要来源。波形只有 5 种纯色、且每帧整屏重绘，因此
   直接对当前后台缓冲做 32 位写入，同一画面降到 1~2ms。
   颜色换算：GUI_COLOR 是 0x00BBGGRR，帧缓冲是 0xAARRGGBB（见 export.c 的 BMP 解码）。
   栅格、文字、弹出层仍然走 emWin，它们写在波形之后。 */
extern uint32_t LCDConf_GetDrawBufferAddr(void);

#define FB_PIX(c) (0xFF000000UL | (((uint32_t)(c) & 0x0000FFUL) << 16) | \
                   ((uint32_t)(c) & 0x00FF00UL) |                         \
                   (((uint32_t)(c) >> 16) & 0x000000FFUL))

static uint32_t *s_fb;              /* 当前后台绘制缓冲（MULTIBUF_Begin 之后有效） */

static void fb_setup(void)
{
  s_fb = (uint32_t *)LCDConf_GetDrawBufferAddr();
}

static void fb_vline(int x, int y0, int y1, uint32_t c)
{
  uint32_t *p;
  int y;

  if (x < 0 || x >= SCR_W) {
    return;
  }
  if (y0 > y1) { int t = y0; y0 = y1; y1 = t; }
  if (y0 < 0) { y0 = 0; }
  if (y1 > SCR_H - 1) { y1 = SCR_H - 1; }

  p = s_fb + (uint32_t)y0 * SCR_W + (uint32_t)x;
  for (y = y0; y <= y1; y++) {
    *p = c;
    p += SCR_W;
  }
}

static void fb_hline(int x0, int x1, int y, uint32_t c)
{
  uint32_t *p;
  int x;

  if (y < 0 || y >= SCR_H) {
    return;
  }
  if (x0 > x1) { int t = x0; x0 = x1; x1 = t; }
  if (x0 < 0) { x0 = 0; }
  if (x1 > SCR_W - 1) { x1 = SCR_W - 1; }

  p = s_fb + (uint32_t)y * SCR_W + (uint32_t)x0;
  for (x = x0; x <= x1; x++) {
    *p++ = c;
  }
}

/* 清屏也改直写帧缓冲：
   GUI_Clear 在 32bpp 非缓存 SDRAM 上要 4~5ms，中途无法让出 CPU，
   是整帧里单次最长的不可打断点（超过 4 个 RX 描述符能撑的时间）。
   逐行填充后每 32 行插一次收包轮询，单段压到 0.5ms 量级。
   每帧全屏重绘，语义与 GUI_Clear(GUI_BLACK) 等价。 */
#define FB_BLACK   0xFF000000UL

static void fb_clear(void)
{
  uint32_t *p = s_fb;
  int y;

  for (y = 0; y < SCR_H; y++) {
    int x;

    for (x = 0; x < SCR_W; x++) {
      p[x] = FB_BLACK;
    }
    p += SCR_W;

    if ((y & 0x1F) == 0) {
      link_poll();
    }
  }
}

#ifndef RX_SOURCE_LINK
/* 假数据源模式：没有 A 板，直接伪造一帧状态帧，让触发面板能看到回执。
   真实链路（UART/UDP）下由 A 板按协议回状态帧。 */
static void fake_status_reply(void)
{
  uint8_t  buf[PROTO_STATUS_LEN];
  uint16_t i;

  proto_build_status(buf, g_gear, g_mask, g_trig_src, g_trig_mode,
                     g_trig_level, g_wave_freq);
  for (i = 0; i < PROTO_STATUS_LEN; i++) {
    proto_feed(buf[i]);
  }
}
#endif

/* ---------------- 控制帧下发 ----------------
   UDP 模式走 5006 端口，UART 模式走 USART6；两边的帧内容完全一致。
   UDP 链路未连接时 net_udp_send 会静默丢弃，等 link 建立后由主循环补发握手。
--------------------------------------------------------- */
static void send_ctrl(uint8_t cmd, uint16_t param)
{
  uint8_t buf[PROTO_CTRL_LEN];

  proto_build_ctrl(buf, cmd, param);

#if defined(RX_SOURCE_UDP)
  net_udp_send(buf, PROTO_CTRL_LEN);
#else
  HAL_UART_Transmit(&huart6, buf, PROTO_CTRL_LEN, 20);
#endif

#ifndef RX_SOURCE_LINK
  fake_status_reply();        /* 自测模式：模拟 A 板回执 */
#endif
}

/* 档位按钮：真实链路下按“上次请求值”递增，保证 0~4 都能发出去；
   自测模式下 g_gear 本身就是数据源的档位，直接递增即可。 */
static void gear_next(void)
{
#if defined(RX_SOURCE_LINK)
  uint8_t base = (g_gear_req != 0xFF) ? g_gear_req : g_gear;
  uint8_t next = (uint8_t)((base + 1) % 5);

  g_gear_req      = next;
  g_gear_req_tick = HAL_GetTick();
  send_ctrl(CMD_GEAR, next);
#else
  g_gear = (uint8_t)((g_gear + 1) % 5);
  send_ctrl(CMD_GEAR, g_gear);
#endif
}

/* ---------------- 测量 ----------------
   频率采用「相邻上升沿间隔取平均」而不是「数上升沿个数 / 窗口时长」：
   后者在窗口内只有 3~4 个周期时量化误差很大（78Hz 会算成 60/70Hz）。
   平均周期法只要窗口内能捕捉到 2 个上升沿，就能得到准确频率。
   周期数不足 2 个时返回 0，界面显示 "---" 而不是误导性的 0Hz。
--------------------------------------------------------- */
static void meas_analog(const uint16_t *buf, uint16_t *vpp_mv, uint32_t *freq_hz)
{
  uint16_t k;
  uint16_t mn = 4095, mx = 0, mid;
  int32_t  last_rise = -1;
  uint32_t sum_pts = 0, cnt = 0;
  uint32_t dt = cur_dt_us();

  for (k = 0; k < TOTAL_PTS; k++) {
    uint16_t v = buf[k];
    if (v < mn) { mn = v; }
    if (v > mx) { mx = v; }
    if ((k & 0x3F) == 0) { link_poll(); }   /* 测量只读快照，穿插收包安全 */
  }

  *vpp_mv = (uint16_t)(((uint32_t)(mx - mn) * 3300UL) / 4095UL);

  /* 信号几乎无变化（如通道关闭或直流），无法测频 */
  if ((uint16_t)(mx - mn) < 50) {
    *freq_hz = 0;
    return;
  }

  mid = (uint16_t)(((uint32_t)mn + mx) / 2U);
  for (k = 1; k < TOTAL_PTS; k++) {
    uint16_t p = buf[(uint16_t)(k - 1)];
    uint16_t c = buf[k];
    if (p < mid && c >= mid) {                 /* 中点上升穿越 */
      if (last_rise >= 0) {
        sum_pts += (uint32_t)((int32_t)k - last_rise);
        cnt++;
      }
      last_rise = (int32_t)k;
    }
  }

  if (cnt > 0 && dt != 0) {
    uint32_t period_us = (sum_pts * dt) / cnt;
    *freq_hz = (period_us != 0) ? (1000000UL / period_us) : 0;
  } else {
    *freq_hz = 0;
  }
}

static void meas_digital(uint8_t mask, uint32_t *freq_hz, uint32_t *pw_us)
{
  uint16_t k;
  int32_t  last_rise = -1;
  uint32_t sum_pts = 0, cnt = 0;
  uint16_t high_len = 0, last_high = 0;
  uint8_t  prev;
  uint32_t dt = cur_dt_us();

  prev = (s_lind[0] & mask) ? 1 : 0;
  if (prev) { high_len = 1; }

  for (k = 1; k < TOTAL_PTS; k++) {
    uint8_t cur = (s_lind[k] & mask) ? 1 : 0;

    if (cur && !prev) {                        /* 上升沿 */
      if (last_rise >= 0) {
        sum_pts += (uint32_t)((int32_t)k - last_rise);
        cnt++;
      }
      last_rise = (int32_t)k;
      high_len = 1;
    } else if (cur && prev) {                  /* 持续高电平 */
      high_len++;
    } else if (!cur && prev) {                 /* 下降沿，记录完整高电平宽度 */
      last_high = high_len;
      high_len = 0;
    }
    prev = cur;
  }

  if (cnt > 0 && dt != 0) {
    uint32_t period_us = (sum_pts * dt) / cnt;
    *freq_hz = (period_us != 0) ? (1000000UL / period_us) : 0;
  } else {
    *freq_hz = 0;
  }
  *pw_us = (uint32_t)last_high * dt;
}

/* ---------------- 栅格 ---------------- */
static void draw_grid(void)
{
  int i;

  GUI_SetColor(GUI_DARKGRAY);

  for (i = 0; i <= DIV_X; i++) {
    int x = i * (SCR_W - 1) / DIV_X;
    GUI_DrawVLine(x, WAVE_Y0, WAVE_Y0 + WAVE_H);
    link_poll();
  }
  for (i = 0; i <= 8; i++) {
    int y = WAVE_Y0 + i * WAVE_H / 8;
    GUI_DrawHLine(y, 0, SCR_W - 1);
    link_poll();
  }
  for (i = 0; i <= 3; i++) {
    int y = DIG_Y0 + i * DIG_LANE_H;
    GUI_DrawHLine(y, 0, SCR_W - 1);
    link_poll();
  }
}

/* ---------------- 模拟通道 ----------------
   逐点 GUI_DrawLine 在 32bpp 非缓存 SDRAM 上约 0.75us/像素，
   两路 500 段要 45ms（实测），直接导致帧率掉到 12。
   改为「每像素列一次竖直填充」：
     - 500 个采样点映射到 480 列，同列多点取 min/max（等价于真实示波器的包络）
     - 与上一列的连接点合并，保证跨列不断线
     - 填充直接写帧缓冲（见 fb_vline），不再经 emWin/DMA2D 逐列调用
   画面形态与原折线一致，耗时从几十毫秒降到 1ms 量级。
--------------------------------------------------------- */
static int16_t s_col_min[SCR_W];
static int16_t s_col_max[SCR_W];

static void draw_analog(const uint16_t *buf, GUI_COLOR color)
{
  int i, col;
  int prev_y = -1;

  for (col = 0; col < SCR_W; col++) {
    s_col_min[col] = -1;
    s_col_max[col] = -1;
  }

  for (i = 0; i < TOTAL_PTS; i++) {
    int y = raw12_to_y(buf[i]);

    col = idx_to_x(i);
    if (s_col_min[col] < 0) {
      s_col_min[col] = (int16_t)y;
      s_col_max[col] = (int16_t)y;
    } else {
      if (y < s_col_min[col]) { s_col_min[col] = (int16_t)y; }
      if (y > s_col_max[col]) { s_col_max[col] = (int16_t)y; }
    }
  }

  {
    uint32_t c = FB_PIX(color);

    for (col = 0; col < SCR_W; col++) {
      int lo, hi;

      if (s_col_min[col] < 0) {
        continue;
      }

      lo = s_col_min[col];
      hi = s_col_max[col];

      /* 与上一列的端点相连，跨列不出现断点 */
      if (prev_y >= 0) {
        if (prev_y < lo) { lo = prev_y; }
        if (prev_y > hi) { hi = prev_y; }
      }

      fb_vline(col, lo, hi, c);

      prev_y = (s_col_min[col] + s_col_max[col]) / 2;

      if ((col & 0x1F) == 0) {
        link_poll();            /* 每 32 列让出一次收包机会 */
      }
    }
  }
}

/* ---------------- 数字通道 ----------------
   画法：低电平画在泳道底部，高电平画在泳道顶部，跳变沿用竖线连接。
   按屏幕列取电平，把连续同电平合并成一段后直接写帧缓冲：
   原来 500 个采样点逐个调 GUI_DrawHLine（一帧三路约 1800 次 emWin 调用），
   合并后每路只剩"电平段数 + 跳变沿数"，且走 32 位直写，耗时降到 1ms 以内。 */
static void draw_digital(void)
{
  static const GUI_COLOR col[3]  = { GUI_GREEN, GUI_YELLOW, GUI_MAGENTA };
  static const uint8_t   bitm[3] = { DIG_D1, DIG_D2, DIG_D3 };   /* DIG 字节位，非 CH_MASK */
  uint8_t m = cur_mask();
  int ch;

  for (ch = 0; ch < 3; ch++) {
    int lane = DIG_Y0 + ch * DIG_LANE_H;
    int y_hi = lane + 4;
    int y_lo = lane + DIG_LANE_H - 6;
    uint32_t c = FB_PIX(col[ch]);
    int colx, run_start = 0, run_y = -1;

    /* 通道关闭时不绘制。
       注意：开关判据用 CH_MASK 位（MASK_D1<<ch），取电平用 DIG 位（bitm[ch]）。
       两套位定义不同——DIG 里 D1 在 bit0，而 CH_MASK 里 D1 在 bit2；
       若这里误用 bitm[ch] 判开关，会变成“D1 跟着 CH1、D3 跟着 D1”。 */
    if ((m & (uint8_t)(MASK_D1 << ch)) == 0) {
      continue;
    }

    for (colx = 0; colx < SCR_W; colx++) {
      /* 该像素列对应的采样点（与 idx_to_x 互逆，四舍五入） */
      uint16_t k = (uint16_t)(((uint32_t)colx * (TOTAL_PTS - 1) + (SCR_W - 1) / 2) /
                              (SCR_W - 1));
      int y = (s_lind[k] & bitm[ch]) ? y_hi : y_lo;

      if (run_y < 0) {
        run_y = y;
        run_start = colx;
      } else if (y != run_y) {
        fb_hline(run_start, colx - 1, run_y, c);
        fb_vline(colx, run_y, y, c);        /* 跳变沿 */
        run_y = y;
        run_start = colx;
      }

      if ((colx & 0x1F) == 0) {
        link_poll();            /* 每 32 列让出一次收包机会 */
      }
    }
    fb_hline(run_start, SCR_W - 1, run_y, c);
  }
}

/* ---------------- 顶部触摸按钮栏 ---------------- */
/* 简易长度计算，避免引入 string.h */
static int strlen_(const char *s)
{
  int n = 0;
  while (s[n]) { n++; }
  return n;
}

static void draw_button(int x0, int y0, int x1, int y1,
                        const char *label, uint8_t on, GUI_COLOR color)
{
  GUI_SetColor(on ? color : GUI_DARKGRAY);
  GUI_FillRect(x0, y0, x1, y1);

  GUI_SetColor(on ? GUI_BLACK : GUI_LIGHTGRAY);
  GUI_DrawRect(x0, y0, x1, y1);

  GUI_SetFont(&GUI_Font8x16);
  GUI_SetTextMode(GUI_TM_TRANS);
  GUI_SetColor(on ? GUI_BLACK : GUI_LIGHTGRAY);
  GUI_DispStringAt(label,
                   x0 + ((x1 - x0 + 1) - strlen_(label) * 8) / 2,
                   y0 + 1);

  link_poll();            /* 顶栏 9 个按钮逐个让出收包机会 */
}

static void draw_topbar(uint32_t frame, uint8_t ts_ok, uint8_t link_ok)
{
  proto_stat_t *st = proto_stat();
  uint8_t g = (st->gear < 5) ? st->gear : g_gear;
  uint8_t m = cur_mask();
  char buf[4];

  /* 档位按钮 */
  draw_button(buttons[0].x0, 0, buttons[0].x1, TOP_H - 2, gear_txt[g], 1, GUI_CYAN);

  /* 通道按钮 */
  buf[0] = '1'; buf[1] = 0;
  draw_button(buttons[1].x0, 0, buttons[1].x1, TOP_H - 2, buf, (m & MASK_CH1) ? 1 : 0, GUI_BLUE);

  buf[0] = '2'; buf[1] = 0;
  draw_button(buttons[2].x0, 0, buttons[2].x1, TOP_H - 2, buf, (m & MASK_CH2) ? 1 : 0, GUI_RED);

  draw_button(buttons[3].x0, 0, buttons[3].x1, TOP_H - 2, "D1", (m & MASK_D1) ? 1 : 0, GUI_GREEN);
  draw_button(buttons[4].x0, 0, buttons[4].x1, TOP_H - 2, "D2", (m & MASK_D2) ? 1 : 0, GUI_YELLOW);
  draw_button(buttons[5].x0, 0, buttons[5].x1, TOP_H - 2, "D3", (m & MASK_D3) ? 1 : 0, GUI_MAGENTA);

  /* 矩形波频率按钮 */
  draw_button(buttons[6].x0, 0, buttons[6].x1, TOP_H - 2, "F-", 1, GUI_LIGHTGRAY);
  draw_button(buttons[7].x0, 0, buttons[7].x1, TOP_H - 2, "F+", 1, GUI_LIGHTGRAY);

  /* 导出按钮 */
  draw_button(buttons[9].x0, 0, buttons[9].x1, TOP_H - 2, "SAVE", 1, GUI_ORANGE);

  /* 右侧状态信息（6x8 字体） */
  GUI_SetFont(&GUI_Font6x8);
  GUI_SetTextMode(GUI_TM_TRANS);

  /* 当前矩形波频率 */
  GUI_SetColor(GUI_CYAN);
  GUI_DispStringAt("W:", 322, 2);
  GUI_DispDecAt((I32)g_wave_freq, 334, 2, 4);
  GUI_DispStringAt("Hz", 362, 2);

  /* 触摸状态：T=OK，E+错误码=异常 */
  if (ts_ok) {
    GUI_SetColor(GUI_GREEN);
    GUI_DispStringAt("T", 380, 2);
  } else {
    GUI_SetColor(GUI_RED);
    GUI_DispStringAt("E", 380, 2);
    GUI_DispDecAt((I32)touch_error(), 386, 2, 1);
  }

  /* 数据源/链路状态：
       S=假数据自测  U=USART6 有数据  E=以太网有数据
       P=以太网 PHY 已 link 但 500ms 内没收到有效帧（A 没发包 / IP 不通 / 端口不对）
       L=以太网 PHY 未 link（网线没插、对端没上电）
     P 和 L 分开，联调时能一眼判断是物理层还是数据层的问题。 */
#if defined(RX_SOURCE_UDP)
  if (link_ok) {
    GUI_SetColor(GUI_GREEN);
    GUI_DispStringAt("E", 392, 2);
  } else if (g_last_phy_up) {
    GUI_SetColor(GUI_ORANGE);
    GUI_DispStringAt("P", 392, 2);
  } else {
    GUI_SetColor(GUI_RED);
    GUI_DispStringAt("L", 392, 2);
  }
#elif defined(RX_SOURCE_UART)
  GUI_SetColor(link_ok ? GUI_GREEN : GUI_RED);
  GUI_DispStringAt(link_ok ? "U" : "L", 392, 2);
#else
  GUI_SetColor(GUI_CYAN);
  GUI_DispStringAt("S", 392, 2);
#endif
}

/* ---------------- 触发指示 ----------------
   底部测量行右侧显示当前触发配置；边框绿色表示 A 板已回执确认，
   灰色表示只是本地设置（A 板还没回状态帧）。
--------------------------------------------------------- */
static uint16_t trig_level_mv(void)
{
  return (uint16_t)(((uint32_t)g_trig_level * 3300UL) / 255UL);
}

static void draw_trig_indicator(void)
{
  const proto_status_t *st = proto_status();
  uint8_t m = cur_mask();
  uint8_t confirmed;
  uint8_t src_off;

  /* 数字触发源时电平参数被 A 板忽略，回显值可能被清零；
     因此只比较源与方式，避免绿框永远不亮（模拟源仍三项全比） */
  if (g_trig_src >= 3) {
    confirmed = (uint8_t)(st->valid &&
                          st->trig_src  == g_trig_src &&
                          st->trig_mode == g_trig_mode);
  } else {
    confirmed = (uint8_t)(st->valid &&
                          st->trig_src   == g_trig_src &&
                          st->trig_mode  == g_trig_mode &&
                          st->trig_level == g_trig_level);
  }

  /* 触发源通道被关闭：A 板按自由运行处理（补充条款第 9 条），显示 AUTO */
  src_off = (uint8_t)(g_trig_src >= 1 && g_trig_src <= 5 &&
                      (m & (uint8_t)(1u << (g_trig_src - 1))) == 0);

  GUI_SetFont(&GUI_Font6x8);
  GUI_SetTextMode(GUI_TM_TRANS);

  GUI_SetColor(confirmed ? GUI_GREEN : GUI_DARKGRAY);
  GUI_DrawRect(TRIG_X0, TRIG_Y0, TRIG_X1, TRIG_Y1);

  GUI_SetColor(g_trig_src ? GUI_YELLOW : GUI_GRAY);
  GUI_DispStringAt("TRIG", TRIG_X0 + 4, TRIG_Y0 + 1);
  GUI_DispStringAt(trig_src_txt[g_trig_src < 6 ? g_trig_src : 0],
                   TRIG_X0 + 31, TRIG_Y0 + 1);

  if (g_trig_src == 0) {
    return;
  }

  GUI_DispStringAt(trig_mode_txt[g_trig_mode < 3 ? g_trig_mode : 0],
                   TRIG_X0 + 50, TRIG_Y0 + 1);

  if (src_off) {                               /* 源通道关闭：自由运行 */
    GUI_SetColor(GUI_DARKGRAY);
    GUI_DispStringAt("AUTO", TRIG_X0 + 80, TRIG_Y0 + 1);
  } else if (g_trig_src <= 2) {                /* 模拟通道：显示电平 */
    GUI_SetColor(g_trig_src ? GUI_YELLOW : GUI_GRAY);
    GUI_DispDecAt((I32)trig_level_mv(), TRIG_X0 + 80, TRIG_Y0 + 1, 4);
    GUI_DispStringAt("mV", TRIG_X0 + 106, TRIG_Y0 + 1);
  } else {                                     /* 数字通道：只看逻辑沿 */
    GUI_DispStringAt("LOGIC", TRIG_X0 + 80, TRIG_Y0 + 1);
  }
}

/* ---------------- 触发电平线 ----------------
   在波形区画一条虚线，高度对应触发电平；颜色跟随触发源通道。
   数字通道作触发源时不画（协议规定电平参数对数字触发无效）。
   触发点的时间位置不画标记：约定触发点 = 每帧第 0 点 = 每格左边缘，
   屏上 10 个格子左边缘都是触发点，单独标一个反而误导。
--------------------------------------------------------- */
static void draw_trig_level(void)
{
  uint16_t raw, y;
  int x;

  if (g_trig_src != 1 && g_trig_src != 2) {
    return;                     /* OFF 或数字通道：不画电平线 */
  }
  /* 触发源通道被关掉时，没有信号可触发，线也一并隐藏，避免被误认为波形 */
  if ((g_trig_src == 1) && ((cur_mask() & MASK_CH1) == 0)) {
    return;
  }
  if ((g_trig_src == 2) && ((cur_mask() & MASK_CH2) == 0)) {
    return;
  }

  /* level(0~255) → ADC 码 → 屏幕 y（与波形同一映射） */
  raw = (uint16_t)(((uint32_t)g_trig_level * 4095UL) / 255UL);
  y   = (uint16_t)raw12_to_y(raw);

  GUI_SetColor((g_trig_src == 1) ? GUI_BLUE : GUI_RED);
  for (x = 0; x < SCR_W; x += 8) {
    int xe = x + 3;
    if (xe > SCR_W - 1) { xe = SCR_W - 1; }
    GUI_DrawHLine((int)y, x, xe);
    link_poll();
  }

  /* 右侧 T 标记：黑底保证压在波形上也读得清 */
  {
    int ty = (int)y - 8;
    if (ty < WAVE_Y0) { ty = WAVE_Y0; }
    GUI_SetBkColor(GUI_BLACK);
    GUI_SetTextMode(GUI_TM_NORMAL);
    GUI_SetColor((g_trig_src == 1) ? GUI_BLUE : GUI_RED);
    GUI_DispStringAt("T", SCR_W - 48, ty);
    GUI_DispDecAt((I32)trig_level_mv(), SCR_W - 42, ty, 4);
    GUI_SetTextMode(GUI_TM_TRANS);
  }
}

/* ---------------- 坐标轴标签 ----------------
   Y 轴：3.3V / 1.6V / 0V（固定，0~3.3V 分 8 格，约 0.41V/格）
   X 轴：0 / 中点 / 全屏时长（随档位变化）
   全部带黑底，压在波形和栅格上也清晰。
--------------------------------------------------------- */
static void axis_text(int x, int y, const char *s, GUI_COLOR c)
{
  GUI_SetBkColor(GUI_BLACK);
  GUI_SetTextMode(GUI_TM_NORMAL);
  GUI_SetColor(c);
  GUI_DispStringAt(s, x, y);
  GUI_SetTextMode(GUI_TM_TRANS);

  link_poll();
}

static void draw_axis(void)
{
  uint8_t g = proto_stat()->gear;

  if (g > 4) { g = 0; }

  GUI_SetFont(&GUI_Font6x8);

  /* Y 轴：顶部、中部、底部 */
  axis_text(1, WAVE_Y0 + 1,              "3.3V", GUI_YELLOW);
  axis_text(1, WAVE_Y0 + WAVE_H / 2 - 4, "1.6V", GUI_YELLOW);
  axis_text(1, WAVE_Y0 + WAVE_H - 8,     "0V",   GUI_YELLOW);

  /* X 轴：左端 0，中点，右端全屏时长 */
  axis_text(1, AXIS_Y_XLAB, "0", GUI_CYAN);
  axis_text(AXIS_X_MID, AXIS_Y_XLAB, axis_mid[g], GUI_CYAN);
  axis_text(AXIS_X_END, AXIS_Y_XLAB, axis_end[g], GUI_CYAN);
}

/* ---------------- 底部测量信息 ---------------- */
static void draw_meas(void)
{
  uint16_t vpp1 = 0, vpp2 = 0;
  uint32_t f1 = 0, f2 = 0;
  uint32_t fd[3], pw[3];
  uint8_t  ch;
  uint8_t  m = cur_mask();

  GUI_SetFont(&GUI_Font6x8);
  GUI_SetTextMode(GUI_TM_TRANS);

  draw_trig_indicator();        /* 数据未就绪时也要能配置触发 */

  if (proto_stat()->good < 12) {
    GUI_SetColor(GUI_GRAY);
    GUI_DispStringAt("measuring...", 2, MEAS_Y0 + 4);
    return;
  }

  if (m & MASK_CH1) { meas_analog(s_lin1, &vpp1, &f1); }
  if (m & MASK_CH2) { meas_analog(s_lin2, &vpp2, &f2); }

  /* 行 1：模拟通道（频率为 0 表示窗口内周期不足，显示 --- 避免误读） */
  GUI_SetColor(GUI_BLUE);
  GUI_DispStringAt("1:", 2, MEAS_Y0 + 1);
  GUI_SetColor(GUI_WHITE);
  if (m & MASK_CH1) {
    GUI_DispDecAt((I32)vpp1, 14, MEAS_Y0 + 1, 4);
    GUI_DispStringAt("mV", 40, MEAS_Y0 + 1);
    if (f1 != 0) { GUI_DispDecAt((I32)f1, 62, MEAS_Y0 + 1, 4); }
    else         { GUI_DispStringAt("---", 62, MEAS_Y0 + 1);     }
    GUI_DispStringAt("Hz", 88, MEAS_Y0 + 1);
  } else {
    GUI_DispStringAt("--- OFF ---", 14, MEAS_Y0 + 1);
  }

  GUI_SetColor(GUI_RED);
  GUI_DispStringAt("2:", 128, MEAS_Y0 + 1);
  GUI_SetColor(GUI_WHITE);
  if (m & MASK_CH2) {
    GUI_DispDecAt((I32)vpp2, 140, MEAS_Y0 + 1, 4);
    GUI_DispStringAt("mV", 166, MEAS_Y0 + 1);
    if (f2 != 0) { GUI_DispDecAt((I32)f2, 188, MEAS_Y0 + 1, 4); }
    else         { GUI_DispStringAt("---", 188, MEAS_Y0 + 1);     }
    GUI_DispStringAt("Hz", 214, MEAS_Y0 + 1);
  } else {
    GUI_DispStringAt("--- OFF ---", 140, MEAS_Y0 + 1);
  }

  /* 行 2：数字通道 */
  for (ch = 0; ch < 3; ch++) {
    int x = 2 + ch * 158;
    uint8_t on = (m & (MASK_D1 << ch)) ? 1 : 0;

    link_poll();

    GUI_SetColor((ch == 0) ? GUI_GREEN : (ch == 1) ? GUI_YELLOW : GUI_MAGENTA);
    GUI_DispStringAt("D", x, MEAS_Y0 + 11);
    GUI_DispDecAt((I32)(ch + 1), x + 6, MEAS_Y0 + 11, 1);
    GUI_DispStringAt(":", x + 12, MEAS_Y0 + 11);

    GUI_SetColor(GUI_WHITE);
    if (on) {
      /* meas_digital 与 dig[] 比较，必须用 DIG 字节位（不是 CH_MASK 位） */
      meas_digital((uint8_t)(DIG_D1 << ch), &fd[ch], &pw[ch]);
      if (fd[ch] != 0) { GUI_DispDecAt((I32)fd[ch], x + 18, MEAS_Y0 + 11, 4); }
      else             { GUI_DispStringAt("---", x + 18, MEAS_Y0 + 11);      }
      GUI_DispStringAt("Hz", x + 46, MEAS_Y0 + 11);
      GUI_DispDecAt((I32)pw[ch], x + 64, MEAS_Y0 + 11, 5);
      GUI_DispStringAt("us", x + 98, MEAS_Y0 + 11);
    } else {
      GUI_DispStringAt("--- OFF ---", x + 18, MEAS_Y0 + 11);
    }
  }
}

/* ---------------- 链路统计条 ----------------
   紧贴屏幕底边（y=264，仅 8px 高）显示 SEQ、CRC 错误、丢帧数和链路状态，
   用于验收时观察链路健壮性。CRC/DROP 为 0 时灰色，非 0 时告警色。
--------------------------------------------------------- */
static void draw_link_stats(void)
{
  const proto_stat_t   *st = proto_stat();
  const proto_status_t *ss = proto_status();

  GUI_SetFont(&GUI_Font6x8);
  GUI_SetTextMode(GUI_TM_TRANS);

  GUI_SetColor(GUI_DARKGRAY);
  GUI_DispStringAt("SEQ", 2, 264);
  GUI_SetColor(GUI_CYAN);
  GUI_DispDecAt((I32)st->last_seq, 22, 264, 5);

  GUI_SetColor(GUI_DARKGRAY);
  GUI_DispStringAt("CRC", 58, 264);
  GUI_SetColor(st->crc_err ? GUI_RED : GUI_GRAY);
  GUI_DispDecAt((I32)st->crc_err, 78, 264, 4);

  link_poll();

  GUI_SetColor(GUI_DARKGRAY);
  GUI_DispStringAt("DROP", 108, 264);
  GUI_SetColor(st->drop ? GUI_YELLOW : GUI_GRAY);
  GUI_DispDecAt((I32)st->drop, 132, 264, 4);

  GUI_SetColor(GUI_DARKGRAY);
  GUI_DispStringAt("LINK", 162, 264);
  GUI_SetColor(g_last_link_ok ? GUI_GREEN : GUI_RED);
  GUI_DispStringAt(g_last_link_ok ? "OK" : "--", 186, 264);

  /* 接收帧率（联调用）：每秒收到的有效数据帧数，与渲染帧率 FPS 分开看 */
  GUI_SetColor(GUI_DARKGRAY);
  GUI_DispStringAt("RX", 206, 264);
  GUI_SetColor(GUI_CYAN);
  GUI_DispDecAt((I32)g_rx_fps, 222, 264, 4);

  /* 实测帧率：任务书提高部分（3）要求 ≥20 帧/秒，这里给出屏上依据 */
  GUI_SetColor(GUI_DARKGRAY);
  GUI_DispStringAt("FPS", 252, 264);
  GUI_SetColor(g_fps >= 20 ? GUI_GREEN : GUI_ORANGE);
  GUI_DispDecAt((I32)g_fps, 276, 264, 3);

  link_poll();

  /* 临时性能诊断：B/W/T/E 四阶段耗时(ms) */
  GUI_SetFont(&GUI_Font6x8);
  GUI_SetColor(GUI_DARKGRAY);  GUI_DispStringAt("B", 300, 264);
  GUI_SetColor(GUI_WHITE);     GUI_DispDecAt((I32)g_dbg_t[0], 306, 264, 2);
  GUI_SetColor(GUI_DARKGRAY);  GUI_DispStringAt("W", 320, 264);
  GUI_SetColor(GUI_WHITE);     GUI_DispDecAt((I32)g_dbg_t[1], 326, 264, 2);
  GUI_SetColor(GUI_DARKGRAY);  GUI_DispStringAt("T", 340, 264);
  GUI_SetColor(GUI_WHITE);     GUI_DispDecAt((I32)g_dbg_t[2], 346, 264, 2);
  GUI_SetColor(GUI_DARKGRAY);  GUI_DispStringAt("E", 360, 264);
  GUI_SetColor(GUI_WHITE);     GUI_DispDecAt((I32)g_dbg_t[3], 366, 264, 2);

  link_poll();

  /* A 板状态帧回执（验收记录用，无需挂调试器）：
     ST = 有效状态帧累计数，G = 状态帧 GEAR，M = 状态帧 CH_MASK（十六进制）。
     收到 CMD 0x06 或 A 板主动上报时 ST 递增；G/M 为 A 板当前实际设置。 */
  {
    const char *hex = "0123456789ABCDEF";
    char mb[3];

    GUI_SetColor(GUI_DARKGRAY);
    GUI_DispStringAt("ST", 382, 264);
    GUI_SetColor(ss->valid ? GUI_CYAN : GUI_GRAY);
    GUI_DispDecAt((I32)ss->count, 394, 264, 4);

    GUI_SetColor(GUI_DARKGRAY);
    GUI_DispStringAt("G", 420, 264);
    GUI_SetColor(ss->valid ? GUI_CYAN : GUI_GRAY);
    if (ss->valid) {
      GUI_DispDecAt((I32)ss->gear, 426, 264, 1);
    } else {
      GUI_DispStringAt("-", 426, 264);
    }

    GUI_SetColor(GUI_DARKGRAY);
    GUI_DispStringAt("M", 433, 264);
    GUI_SetColor(ss->valid ? GUI_CYAN : GUI_GRAY);
    if (ss->valid) {
      mb[0] = hex[(ss->ch_mask >> 4) & 0x0F];
      mb[1] = hex[ss->ch_mask & 0x0F];
      mb[2] = 0;
      GUI_DispStringAt(mb, 439, 264);
    } else {
      GUI_DispStringAt("--", 439, 264);
    }
  }
}

/* ---------------- 数字键盘（设置矩形波频率） ----------------
   弹出式键盘：点击顶栏 "W:xxxxHz" 打开，输入 100~10000 后按 OK 生效。
   布局（4 列 × 4 行）：
       1   2   3  | DEL
       4   5   6  | CLR
       7   8   9  | CAN
          0       | OK
--------------------------------------------------------- */
static void draw_key(int x0, int y0, int x1, int y1, const char *label, GUI_COLOR c)
{
  GUI_SetColor(c);
  GUI_FillRect(x0, y0, x1, y1);
  GUI_SetColor(GUI_BLACK);
  GUI_DrawRect(x0, y0, x1, y1);
  GUI_SetFont(&GUI_Font8x16);
  GUI_SetTextMode(GUI_TM_TRANS);
  GUI_SetColor(GUI_BLACK);
  GUI_DispStringAt(label, x0 + ((x1 - x0 + 1) - strlen_(label) * 8) / 2, y0 + 9);

  link_poll();            /* 键盘/触发面板/导出菜单的按键逐个让出收包 */
}

static void draw_keypad(void)
{
  int r, c;
  int y;
  char num[2];

  if (!g_kp_open) {
    return;
  }

  /* 弹窗底板 + 边框 */
  GUI_SetColor(GUI_LIGHTGRAY);
  GUI_FillRect(KP_X0, KP_Y0, KP_X1, KP_Y1);
  GUI_SetColor(GUI_WHITE);
  GUI_DrawRect(KP_X0, KP_Y0, KP_X1, KP_Y1);
  GUI_DrawRect(KP_X0 + 1, KP_Y0 + 1, KP_X1 - 1, KP_Y1 - 1);

  /* 标题 */
  GUI_SetFont(&GUI_Font8x16);
  GUI_SetTextMode(GUI_TM_TRANS);
  GUI_SetColor(GUI_BLACK);
  GUI_DispStringAt("Set Rect Wave Freq (100-10000 Hz)", KP_X0 + 10, KP_Y0 + 6);

  /* 输入显示区 */
  GUI_SetColor(GUI_WHITE);
  GUI_FillRect(KP_X0 + 10, KP_Y0 + 28, KP_X1 - 10, KP_Y0 + 56);
  GUI_SetColor(GUI_BLACK);
  GUI_DrawRect(KP_X0 + 10, KP_Y0 + 28, KP_X1 - 10, KP_Y0 + 56);

  GUI_SetFont(&GUI_Font8x16);
  GUI_SetColor(GUI_BLACK);
  if (g_kp_len == 0) {
    GUI_SetColor(GUI_GRAY);
    GUI_DispStringAt("input...", KP_X0 + 20, KP_Y0 + 32);
  } else {
    uint8_t i;
    for (i = 0; i < g_kp_len; i++) {
      num[0] = g_kp_buf[i];
      num[1] = 0;
      GUI_DispStringAt(num, KP_X0 + 20 + i * 10, KP_Y0 + 32);
    }
    GUI_SetColor(GUI_BLUE);
    GUI_DispStringAt("Hz", KP_X0 + 24 + g_kp_len * 10, KP_Y0 + 32);
  }

  /* 数字键 1~9 */
  for (r = 0; r < 3; r++) {
    y = KP_ROW_Y0 + r * (KP_ROW_H + KP_ROW_GAP);
    for (c = 0; c < 3; c++) {
      int x = KP_COL0_X + c * (KP_COL_W + KP_COL_GAP);
      num[0] = (char)('1' + r * 3 + c);
      num[1] = 0;
      draw_key(x, y, x + KP_COL_W, y + KP_ROW_H, num, GUI_WHITE);
    }
  }

  /* 0 键（占满三列） */
  y = KP_ROW_Y0 + 3 * (KP_ROW_H + KP_ROW_GAP);
  draw_key(KP_COL0_X, y,
           KP_COL0_X + 3 * KP_COL_W + 2 * KP_COL_GAP, y + KP_ROW_H,
           "0", GUI_WHITE);

  /* 功能键列 */
  draw_key(KP_FN_X, KP_ROW_Y0 + 0 * (KP_ROW_H + KP_ROW_GAP),
           KP_FN_X + KP_FN_W, KP_ROW_Y0 + 0 * (KP_ROW_H + KP_ROW_GAP) + KP_ROW_H,
           "DEL", GUI_LIGHTGRAY);

  draw_key(KP_FN_X, KP_ROW_Y0 + 1 * (KP_ROW_H + KP_ROW_GAP),
           KP_FN_X + KP_FN_W, KP_ROW_Y0 + 1 * (KP_ROW_H + KP_ROW_GAP) + KP_ROW_H,
           "CLR", GUI_LIGHTGRAY);

  draw_key(KP_FN_X, KP_ROW_Y0 + 2 * (KP_ROW_H + KP_ROW_GAP),
           KP_FN_X + KP_FN_W, KP_ROW_Y0 + 2 * (KP_ROW_H + KP_ROW_GAP) + KP_ROW_H,
           "CAN", GUI_LIGHTGRAY);

  draw_key(KP_FN_X, KP_ROW_Y0 + 3 * (KP_ROW_H + KP_ROW_GAP),
           KP_FN_X + KP_FN_W, KP_ROW_Y0 + 3 * (KP_ROW_H + KP_ROW_GAP) + KP_ROW_H,
           "OK", GUI_GREEN);
}

/* 键盘触摸：返回 1 表示事件已被键盘消费 */
static uint8_t handle_keypad_tap(uint16_t x, uint16_t y)
{
  int r, c;

  /* 弹窗范围外：视为关闭 */
  if (x < KP_X0 || x > KP_X1 || y < KP_Y0 || y > KP_Y1) {
    g_kp_open = 0;
    return 1;
  }

  /* 数字键区 1~9 */
  for (r = 0; r < 3; r++) {
    int ry0 = KP_ROW_Y0 + r * (KP_ROW_H + KP_ROW_GAP);
    if (y < ry0 || y > ry0 + KP_ROW_H) {
      continue;
    }
    for (c = 0; c < 3; c++) {
      int cx0 = KP_COL0_X + c * (KP_COL_W + KP_COL_GAP);
      if (x >= cx0 && x <= cx0 + KP_COL_W) {
        if (g_kp_len < 5) {                       /* 最多 5 位（10000） */
          g_kp_buf[g_kp_len++] = (char)('1' + r * 3 + c);
          g_kp_buf[g_kp_len] = 0;
        }
        return 1;
      }
    }
    /* 功能列（前 3 行） */
    if (x >= KP_FN_X && x <= KP_FN_X + KP_FN_W) {
      if (r == 0) {                               /* DEL */
        if (g_kp_len > 0) { g_kp_len--; g_kp_buf[g_kp_len] = 0; }
      } else if (r == 1) {                        /* CLR */
        g_kp_len = 0;
        g_kp_buf[0] = 0;
      } else {                                    /* CAN */
        g_kp_open = 0;
      }
      return 1;
    }
    return 1;                                     /* 行内空白，吞掉事件 */
  }

  /* 第 4 行：0 键 + OK */
  {
    int ry0 = KP_ROW_Y0 + 3 * (KP_ROW_H + KP_ROW_GAP);
    if (y >= ry0 && y <= ry0 + KP_ROW_H) {
      if (x >= KP_FN_X && x <= KP_FN_X + KP_FN_W) {
        /* OK：解析并限幅到 100~10000 */
        if (g_kp_len > 0) {
          uint32_t v = 0;
          uint8_t  i;
          for (i = 0; i < g_kp_len; i++) {
            v = v * 10 + (uint32_t)(g_kp_buf[i] - '0');
          }
          if (v < 100)    { v = 100; }
          if (v > 10000)  { v = 10000; }
          g_wave_freq = (uint16_t)v;
          send_ctrl(CMD_WAVE_FREQ, g_wave_freq);
        }
        g_kp_open = 0;
      } else if (x >= KP_COL0_X &&
                 x <= KP_COL0_X + 3 * KP_COL_W + 2 * KP_COL_GAP) {
        if (g_kp_len < 5) {
          g_kp_buf[g_kp_len++] = '0';
          g_kp_buf[g_kp_len] = 0;
        }
      }
      return 1;
    }
  }

  return 1;
}

/* ---------------- 触发设置面板 ----------------
   点击底部右侧触发指示区弹出；点面板外或 CLOSE 关闭。
   每次修改立即下发对应命令（0x03/0x04/0x05），并由 A 板状态帧回执确认。
--------------------------------------------------------- */
static void draw_trig_panel(void)
{
  int y;

  if (!g_trig_open) {
    return;
  }

  GUI_SetColor(GUI_LIGHTGRAY);
  GUI_FillRect(TG_X0, TG_Y0, TG_X1, TG_Y1);
  GUI_SetColor(GUI_WHITE);
  GUI_DrawRect(TG_X0, TG_Y0, TG_X1, TG_Y1);
  GUI_DrawRect(TG_X0 + 1, TG_Y0 + 1, TG_X1 - 1, TG_Y1 - 1);

  GUI_SetFont(&GUI_Font8x16);
  GUI_SetTextMode(GUI_TM_TRANS);
  GUI_SetColor(GUI_BLACK);
  GUI_DispStringAt("Trigger Setting", TG_X0 + 12, TG_Y0 + 8);

  /* 行 1：触发源（点击循环切换） */
  y = TG_ROW1_Y;
  GUI_SetColor(GUI_BLACK);
  GUI_DispStringAt("Source", TG_X0 + 14, y + 8);
  draw_key(TG_VALUE_X, y, TG_VALUE_X + TG_VALUE_W, y + TG_ROW_H,
           trig_src_txt[g_trig_src < 6 ? g_trig_src : 0],
           g_trig_src ? GUI_YELLOW : GUI_WHITE);

  /* 行 2：触发方式 */
  y = TG_ROW2_Y;
  GUI_SetColor(GUI_BLACK);
  GUI_DispStringAt("Mode", TG_X0 + 14, y + 8);
  draw_key(TG_VALUE_X, y, TG_VALUE_X + TG_VALUE_W, y + TG_ROW_H,
           trig_mode_txt[g_trig_mode < 3 ? g_trig_mode : 0], GUI_CYAN);

  /* 行 3：触发电平（数字触发源时该参数被 A 板忽略） */
  y = TG_ROW3_Y;
  GUI_SetColor(GUI_BLACK);
  GUI_DispStringAt("Level", TG_X0 + 14, y + 8);
  GUI_SetColor(GUI_WHITE);
  GUI_DispDecAt((I32)trig_level_mv(), TG_VALUE_X, y + 8, 4);
  GUI_SetColor(GUI_BLACK);
  GUI_DispStringAt("mV", TG_VALUE_X + 40, y + 8);

  draw_key(TG_MINUS_X, y, TG_MINUS_X + TG_ADJ_W, y + TG_ROW_H, "-", GUI_LIGHTGRAY);
  draw_key(TG_PLUS_X,  y, TG_PLUS_X  + TG_ADJ_W, y + TG_ROW_H, "+", GUI_LIGHTGRAY);

  GUI_SetFont(&GUI_Font6x8);
  GUI_SetColor(GUI_DARKGRAY);
  GUI_DispStringAt("Trigger point = left edge of each grid (pre-trigger 0)",
                   TG_X0 + 12, TG_Y1 - 14);

  draw_key(TG_CLOSE_X, TG_CLOSE_Y, TG_CLOSE_X + TG_CLOSE_W, TG_CLOSE_Y + 26,
           "CLOSE", GUI_GREEN);
}

/* 触发电平步进：dir=-1 减小、+1 增大；步进 16 码 ≈ 0.21V */
static void trig_level_step(int8_t dir)
{
  if (dir < 0) {
    g_trig_level = (uint8_t)((g_trig_level >= 16) ? (g_trig_level - 16) : 0);
  } else {
    g_trig_level = (uint8_t)((g_trig_level <= 239) ? (g_trig_level + 16) : 255);
  }
  send_ctrl(CMD_TRIG_LEVEL, g_trig_level);
}

/* 判断点是否落在触发面板的电平 -/+ 按钮上：返回 -1 / +1 / 0 */
static int8_t trig_adj_hit(uint16_t x, uint16_t y)
{
  if (!g_trig_open) {
    return 0;
  }
  if (y < TG_ROW3_Y || y > TG_ROW3_Y + TG_ROW_H) {
    return 0;
  }
  if (x >= TG_MINUS_X && x <= TG_MINUS_X + TG_ADJ_W) {
    return -1;
  }
  if (x >= TG_PLUS_X && x <= TG_PLUS_X + TG_ADJ_W) {
    return 1;
  }
  return 0;
}

static uint8_t handle_trig_tap(uint16_t x, uint16_t y)
{
  /* 面板范围外：关闭 */
  if (x < TG_X0 || x > TG_X1 || y < TG_Y0 || y > TG_Y1) {
    g_trig_open = 0;
    return 1;
  }

  /* 行 1：触发源 */
  if (y >= TG_ROW1_Y && y <= TG_ROW1_Y + TG_ROW_H &&
      x >= TG_VALUE_X && x <= TG_VALUE_X + TG_VALUE_W) {
    g_trig_src = (uint8_t)((g_trig_src + 1) % 6);
    send_ctrl(CMD_TRIG_SRC, g_trig_src);
    return 1;
  }

  /* 行 2：触发方式 */
  if (y >= TG_ROW2_Y && y <= TG_ROW2_Y + TG_ROW_H &&
      x >= TG_VALUE_X && x <= TG_VALUE_X + TG_VALUE_W) {
    g_trig_mode = (uint8_t)((g_trig_mode + 1) % 3);
    send_ctrl(CMD_TRIG_MODE, g_trig_mode);
    return 1;
  }

  /* 行 3：触发电平 - / +（步进 16 ≈ 0.21V；按住可连发） */
  {
    int8_t d = trig_adj_hit(x, y);
    if (d != 0) {
      trig_level_step(d);
      return 1;
    }
  }

  /* CLOSE */
  if (y >= TG_CLOSE_Y && y <= TG_CLOSE_Y + 26 &&
      x >= TG_CLOSE_X && x <= TG_CLOSE_X + TG_CLOSE_W) {
    g_trig_open = 0;
    return 1;
  }

  return 1;
}

/* ---------------- 导出菜单 ----------------
   点击顶栏 SAVE 弹出，选择导出 CSV 或 BMP。
   导出数据经 USART1（ST-Link 虚拟串口）发到 PC。
--------------------------------------------------------- */
static void make_linear_copy(void)
{
  uint16_t base = wpos;
  uint16_t i;

  for (i = 0; i < TOTAL_PTS; i++) {
    uint16_t j = (uint16_t)((base + i) % TOTAL_PTS);
    s_lin1[i] = ch1[j];
    s_lin2[i] = ch2[j];
    s_lind[i] = dig[j];
  }
}

/* 导出期间显示提示（在 Begin/End 之间调用） */
static void show_exporting(const char *what)
{
  GUI_MULTIBUF_Begin();
  GUI_SetColor(GUI_BLACK);
  GUI_Clear();
  GUI_SetFont(&GUI_Font8x16);
  GUI_SetTextMode(GUI_TM_TRANS);
  GUI_SetColor(GUI_CYAN);
  GUI_DispStringAt("Exporting", 180, 110);
  GUI_SetColor(GUI_WHITE);
  GUI_DispStringAt(what, 180, 130);
  GUI_SetColor(GUI_GRAY);
  GUI_DispStringAt("via USB (ST-Link COM)", 130, 160);
  GUI_MULTIBUF_End();
}

static void do_export_csv(void)
{
  make_linear_copy();
  show_exporting("CSV ...");
  (void)export_csv(s_lin1, s_lin2, s_lind, TOTAL_PTS, proto_stat()->gear);
}

/* 整帧重绘函数，定义在 handle_tap 之后 */
static void draw_screen(void);

static void do_export_bmp(void)
{
  uint32_t fb;

  /* 菜单此刻还开着，直接导出会把弹窗一起拍进去。
     先关菜单、重绘一帧干净画面，等 LTDC 换页完成后再取缓冲地址。 */
  g_menu_open = 0;
  draw_screen();
  HAL_Delay(40);

  fb = export_lcd_fb();          /* 此时扫描的已是干净画面 */
  show_exporting("BMP ...");
  (void)export_bmp(fb);
}

static void draw_menu(void)
{
  if (!g_menu_open) {
    return;
  }

  GUI_SetColor(GUI_LIGHTGRAY);
  GUI_FillRect(MN_X0, MN_Y0, MN_X1, MN_Y1);
  GUI_SetColor(GUI_WHITE);
  GUI_DrawRect(MN_X0, MN_Y0, MN_X1, MN_Y1);
  GUI_DrawRect(MN_X0 + 1, MN_Y0 + 1, MN_X1 - 1, MN_Y1 - 1);

  GUI_SetFont(&GUI_Font8x16);
  GUI_SetTextMode(GUI_TM_TRANS);
  GUI_SetColor(GUI_BLACK);
  GUI_DispStringAt("Export Waveform", MN_X0 + 12, MN_Y0 + 10);

  draw_key(MN_CSV_X, MN_BTN_Y0, MN_CSV_X + MN_BTN_W, MN_BTN_Y0 + MN_BTN_H,
           "CSV", GUI_CYAN);
  draw_key(MN_BMP_X, MN_BTN_Y0, MN_BMP_X + MN_BTN_W, MN_BTN_Y0 + MN_BTN_H,
           "BMP", GUI_MAGENTA);

  GUI_SetFont(&GUI_Font6x8);
  GUI_SetColor(GUI_GRAY);
  GUI_DispStringAt("-> PC via USB COM", MN_X0 + 12, MN_BTN_Y0 + MN_BTN_H + 8);

  draw_key(MN_CAN_X, MN_CAN_Y, MN_CAN_X + MN_CAN_W, MN_CAN_Y + MN_BTN_H,
           "CANCEL", GUI_WHITE);
}

static uint8_t handle_menu_tap(uint16_t x, uint16_t y)
{
  /* 弹窗范围外：关闭 */
  if (x < MN_X0 || x > MN_X1 || y < MN_Y0 || y > MN_Y1) {
    g_menu_open = 0;
    return 1;
  }

  /* CSV / BMP */
  if (y >= MN_BTN_Y0 && y <= MN_BTN_Y0 + MN_BTN_H) {
    if (x >= MN_CSV_X && x <= MN_CSV_X + MN_BTN_W) {
      g_menu_open = 0;
      do_export_csv();
      return 1;
    }
    if (x >= MN_BMP_X && x <= MN_BMP_X + MN_BTN_W) {
      g_menu_open = 0;
      do_export_bmp();
      return 1;
    }
  }

  /* CANCEL */
  if (y >= MN_CAN_Y && y <= MN_CAN_Y + MN_BTN_H) {
    if (x >= MN_CAN_X && x <= MN_CAN_X + MN_CAN_W) {
      g_menu_open = 0;
      return 1;
    }
  }

  return 1;
}

/* ---------------- 触摸处理 ---------------- */
static void handle_tap(uint16_t x, uint16_t y)
{
  uint8_t i;

  /* 键盘弹出时，所有触摸都交给键盘 */
  if (g_kp_open) {
    (void)handle_keypad_tap(x, y);
    return;
  }

  /* 导出菜单弹出时，所有触摸都交给菜单 */
  if (g_menu_open) {
    (void)handle_menu_tap(x, y);
    return;
  }

  /* 触发设置面板弹出时，所有触摸都交给面板 */
  if (g_trig_open) {
    (void)handle_trig_tap(x, y);
    return;
  }

  if (y > TOP_H) {
    /* 底部右侧触发指示区：点击打开设置面板，并请求 A 板上报当前状态 */
    if (x >= TRIG_X0 && x <= TRIG_X1 && y >= TRIG_Y0 && y <= TRIG_Y1) {
      g_trig_open = 1;
      send_ctrl(CMD_HANDSHAKE, 0);
    }
    return;                 /* 其余区域只有顶部按钮区响应 */
  }

  for (i = 0; i < BTN_NUM; i++) {
    if (x >= buttons[i].x0 && x <= buttons[i].x1) {
      if (buttons[i].id == 0) {
        /* 档位循环：10ms → 5ms → 1ms → 500us → 100us */
        gear_next();
      } else if (buttons[i].id == 6) {
        /* 矩形波频率 -100Hz */
        if (g_wave_freq >= 200) {
          g_wave_freq -= 100;
        }
        send_ctrl(CMD_WAVE_FREQ, g_wave_freq);
      } else if (buttons[i].id == 7) {
        /* 矩形波频率 +100Hz */
        if (g_wave_freq <= 9900) {
          g_wave_freq += 100;
        }
        send_ctrl(CMD_WAVE_FREQ, g_wave_freq);
      } else if (buttons[i].id == 8) {
        /* 打开数字键盘，预填当前频率 */
        uint16_t v = g_wave_freq;
        uint8_t  n = 0;
        char     tmp[6];

        g_kp_len = 0;
        if (v == 0) { v = 1; }
        while (v > 0 && n < 5) {          /* 拆成十进制数字 */
          tmp[n++] = (char)('0' + (v % 10));
          v /= 10;
        }
        while (n > 0) {                    /* 反转写入缓冲 */
          g_kp_buf[g_kp_len++] = tmp[--n];
        }
        g_kp_buf[g_kp_len] = 0;
        g_kp_open = 1;
      } else if (buttons[i].id == 9) {
        /* 打开导出菜单 */
        g_menu_open = 1;
      } else {
        /* 以“屏上实际显示的通道”（A 掩码 ∩ 本地掩码）为基准取反：
             - A 未实现 CMD02 时 cur_mask() == g_mask，与直接 g_mask ^= bit 等价；
             - A 实现 CMD02 后，若两边掩码有偏差（典型：B 复位后 g_mask 回到 0x1F，
               而 A 仍保持上次的 0x1E），按显示状态取反能一次点击就拉齐，
               不会出现“按两下才生效”或误开其它通道。 */
        uint8_t bit = (uint8_t)(1u << (buttons[i].id - 1));
        uint8_t m   = cur_mask();

        g_mask = (uint8_t)((m & bit) ? (m & (uint8_t)~bit) : (m | bit));
        send_ctrl(CMD_CHMASK, g_mask);
      }
      return;
    }
  }
}

/* ---------------- 整帧绘制 ----------------
   主循环和 BMP 导出共用：Begin → 画完 → End 整屏切换，避免撕裂。
   键盘/菜单是弹层，画在最后。
   ------------------------------------------ */
static void draw_screen(void)
{
  uint32_t t0, t1, t2, t3;

  t0 = HAL_GetTick();
  GUI_MULTIBUF_Begin();
  fb_setup();             /* 取本次绘制的后台缓冲地址（波形直写用） */
  make_linear_copy();     /* 定格本帧数据：绘制期间收包不再影响画面 */
  t1 = HAL_GetTick();

  GUI_SetBkColor(GUI_BLACK);
  fb_clear();             /* 清屏直写帧缓冲，内部每 32 行让出一次收包 */
  draw_grid();
  if (cur_mask() & MASK_CH1) { draw_analog(s_lin1, GUI_BLUE); }
  if (cur_mask() & MASK_CH2) { draw_analog(s_lin2, GUI_RED);  }
  draw_digital();
  draw_trig_level();      /* 触发电平虚线，压在波形之上 */
  t2 = HAL_GetTick();

  draw_axis();            /* 坐标轴标签最后画，带黑底保证可读 */
  link_poll();
  draw_topbar(g_last_frame, g_last_ts_ok, g_last_link_ok);
  link_poll();
  draw_meas();
  link_poll();
  draw_link_stats();      /* 底部 8px 链路统计条 */
  link_poll();
  draw_keypad();          /* 键盘弹出时覆盖在波形之上 */
  draw_menu();            /* 导出菜单放在最上层 */
  draw_trig_panel();      /* 触发设置面板 */
  t3 = HAL_GetTick();

  GUI_MULTIBUF_End();

  /* 临时性能诊断：B=Begin（缓冲区拷贝），W=清屏+波形，T=文字/控件，E=End（等 VSYNC） */
  g_dbg_t[0] = (uint8_t)(t1 - t0);
  g_dbg_t[1] = (uint8_t)(t2 - t1);
  g_dbg_t[2] = (uint8_t)(t3 - t2);
  g_dbg_t[3] = (uint8_t)(HAL_GetTick() - t3);
}

/* ---------------- 主任务 ----------------
   循环不再用 HAL_Delay(50) 阻塞：以太网是轮询模式，必须高频调用
   MX_LWIP_Process() 才能及时取走 DMA 里的帧（1ms 档 A 板每秒发 1000 帧）。
   因此改成「数据入口每轮都转，渲染按 40ms 节拍做」，屏幕约 25 帧/s
   （任务书提高部分要求 ≥20 帧/s，留 5 帧余量）。
--------------------------------------------------------- */
void MainTask(void)
{
#if defined(RX_SOURCE_LINK)
  /* 真实链路（UART/UDP）：没有本地造帧，不需要帧缓冲 */
#else
  uint8_t  frame_buf[PROTO_DATA_LEN];
  uint16_t i;
  uint32_t last_fake = 0;
#endif
  uint32_t frame = 0;
  uint16_t tx, ty;
  uint8_t  ts_ok;
  uint8_t  key_prev = 0;
  uint32_t last_good = 0;
  uint32_t last_status = 0;
  uint32_t last_good_tick = 0;
  uint32_t last_draw = 0;
  uint32_t fps_cnt = 0;
  uint32_t fps_tick = 0;
  uint32_t rx_prev = 0;
  uint32_t repeat_ms = 0;
  uint8_t  link_ok = 1;
  int8_t   repeat_dir = 0;
#if defined(RX_SOURCE_UDP)
  uint8_t  phy_up_prev = 0;
#endif

#if defined(RX_SOURCE_UDP)
  net_udp_init();         /* 必须在 main() 的 MX_LWIP_Init() 之后 */
  proto_reset();
#elif defined(RX_SOURCE_UART)
  proto_reset();
  uart_rx_start();
#else
  fakesrc_init();
  proto_reset();
#endif
  proto_set_frame_cb(frame_in);   /* 每帧有效数据帧立即入环 */
  ts_ok = touch_init();
  log_uart_init();        /* USART1 经 ST-Link 虚拟串口，用于导出文件到 PC */

  /* 上电握手：请求 A 板上报当前档位/通道/触发配置（自测模式会立刻伪造回执） */
  send_ctrl(CMD_HANDSHAKE, 0);

  /* 板载按键 B1 = PI11：下拉输入，按下接 VDD 为高电平。
     作为触摸之外的备用档位切换手段（任务书允许按键调节）。 */
  {
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOI, &gpio);
  }

  GUI_SetBkColor(GUI_BLACK);
  GUI_Clear();

  fps_tick = HAL_GetTick();
  last_draw = fps_tick;

  for (;;) {
    /* --- 数据入口（每轮都转，频率远高于渲染） --- */
#if defined(RX_SOURCE_UDP)
    MX_LWIP_Process();          /* 轮询以太网：收包在回调里喂给 proto_feed */
#elif defined(RX_SOURCE_UART)
    uart_rx_poll(proto_feed);
#endif

#ifndef RX_SOURCE_LINK
    /* --- 假数据：每 50ms 造一帧，模拟 A 板 20 帧/秒 --- */
    if ((uint32_t)(HAL_GetTick() - last_fake) >= 50) {
      last_fake = HAL_GetTick();
      fakesrc_tick(frame_buf, g_gear, g_mask);
      for (i = 0; i < PROTO_DATA_LEN; i++) {
        proto_feed(frame_buf[i]);
      }
    }
#endif

    /* --- 触摸：按下沿触发一次；停在触发面板电平 +/- 上则连发 --- */
    {
      if (touch_read(&tx, &ty)) {
        if (!g_touch_down) {
          g_touch_down = 1;
          repeat_dir = 0;
          repeat_ms  = 0;
          handle_tap(tx, ty);
        } else {
          int8_t   dir = trig_adj_hit(tx, ty);
          uint32_t now = HAL_GetTick();

          if (dir == 0) {
            repeat_dir = 0;                 /* 手指移出 +/-：停止连发 */
            repeat_ms  = 0;
          } else if (dir != repeat_dir) {
            repeat_dir = dir;               /* 换到另一个键：重新计时 */
            repeat_ms  = now + 400;
          } else if (repeat_ms != 0 &&
                     (int32_t)(now - repeat_ms) >= 0) {
            trig_level_step(dir);           /* 按住 400ms 后每 100ms 一步 */
            repeat_ms = now + 100;
          }
        }
      } else {
        g_touch_down = 0;
        repeat_dir = 0;
        repeat_ms  = 0;
      }
    }

    /* --- 板载按键 B1：按一次切换一个档位 --- */
    {
      uint8_t key = (HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_11) == GPIO_PIN_SET) ? 1 : 0;
      if (key && !key_prev) {
        gear_next();
      }
      key_prev = key;
    }

    /* --- A 板状态帧回执：以其为准同步触发配置 --- */
    if (proto_status()->valid && proto_status()->count != last_status) {
      last_status  = proto_status()->count;
      g_trig_src   = proto_status()->trig_src;
      g_trig_mode  = proto_status()->trig_mode;
      g_trig_level = proto_status()->trig_level;
    }

#if defined(RX_SOURCE_LINK)
    /* 档位以 A 板数据帧里的 GEAR 为准：屏幕文字、时间轴、测量都用它。
       只在真实链路同步：fakesrc 模式下 g_gear 是数据源，不能被旧帧回写。 */
    if (proto_stat()->gear < 5) {
      g_gear = proto_stat()->gear;
    }

    /* 档位请求确认：A 回显等于请求值 = 切换成功，立即清掉；
       超过 GEAR_REQ_HOLD_MS 仍未回显 = A 安全拒绝，下次按键从 A 的实际档位重新递增。 */
    if (g_gear_req != 0xFF) {
      if (proto_stat()->gear == g_gear_req) {
        g_gear_req = 0xFF;
      } else if ((uint32_t)(HAL_GetTick() - g_gear_req_tick) >= GEAR_REQ_HOLD_MS) {
        g_gear_req = 0xFF;
      }
    }
#endif

    /* 数据帧已在 proto 解析完成时由 frame_in() 回调入环，
       这里不再按 good 计数取"最后一帧"（会丢掉同一次轮询的其余帧）。 */

    /* --- 链路监测：连续 500ms 没有新有效帧则判定断开（时间基准） --- */
    if (proto_stat()->good != last_good) {
      last_good = proto_stat()->good;
      last_good_tick = HAL_GetTick();
      link_ok = 1;
    } else if ((uint32_t)(HAL_GetTick() - last_good_tick) >= 500) {
      link_ok = 0;
    }

#if defined(RX_SOURCE_UDP)
    /* --- 以太网：物理链路刚建立时补发一次握手（上电时 link 通常还没起来） --- */
    {
      uint8_t up = net_udp_link_up();

      if (up && !phy_up_prev) {
        send_ctrl(CMD_HANDSHAKE, 0);
      }
      if (!up) {
        link_ok = 0;
      }
      phy_up_prev = up;
    }
#endif

    /* --- 渲染：40ms 节拍 = 25 帧/s，与数据入口解耦，给"≥20fps"留余量 ---
       若单次绘制耗时超过 40ms，实际帧率会低于 25，屏上 FPS 会给出实测值。 */
    if ((uint32_t)(HAL_GetTick() - last_draw) >= 40) {
      last_draw = HAL_GetTick();

      g_last_frame   = frame;
      g_last_ts_ok   = ts_ok;
      g_last_link_ok = link_ok;
#if defined(RX_SOURCE_UDP)
      g_last_phy_up  = phy_up_prev;
#endif
      draw_screen();

      frame++;
      fps_cnt++;
    }

    /* --- 每满 1 秒统计一次：渲染帧率 + 接收帧率 --- */
    {
      uint32_t now = HAL_GetTick();

      if ((uint32_t)(now - fps_tick) >= 1000) {
        uint32_t dt = now - fps_tick;

        g_fps = (uint8_t)((fps_cnt * 1000U) / dt);

        /* 接收帧率：有效数据帧增量 / 实际经过时间（A 发了多少、B 收到多少，联调必需） */
        g_rx_fps = (uint16_t)(((proto_stat()->good - rx_prev) * 1000U) / dt);
        rx_prev  = proto_stat()->good;

        fps_cnt  = 0;
        fps_tick = now;
      }
    }
  }
}

/*************************** End of file ****************************/