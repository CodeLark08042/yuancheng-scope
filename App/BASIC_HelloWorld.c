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

/* ==================== 数据源选择 ====================
   默认（注释掉）：使用板内假数据源 fakesrc，单板即可完整自测波形与测量。
   取消注释：使用 USART6 接收 A 板真实数据（921600 8N1，PC6/PC7）。
   两条路径共用同一个协议解析器 proto，切换不影响其它任何功能。
   ==================================================== */
/* #define RX_SOURCE_UART */

/* ---------------- 界面布局 ---------------- */
#define SCR_W            480
#define SCR_H            272

#define TOP_H             20          /* 顶部按钮栏高度 */

#define WAVE_Y0           22          /* 模拟通道区起始 y */
#define WAVE_H           140          /* 模拟通道区高度 */
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

/* ---------------- 本地状态 ---------------- */
static uint8_t g_gear = GEAR_10MS;
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

/* ---- 最近一帧的状态：导出 BMP 时用它重绘同一画面 ---- */
static uint32_t g_last_frame   = 0;
static uint8_t  g_last_ts_ok   = 1;
static uint8_t  g_last_link_ok = 1;

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

/* ---------------- 环形缓冲工具 ---------------- */
static void ring_push(uint16_t r1, uint16_t r2, uint8_t d)
{
  ch1[wpos] = r1;
  ch2[wpos] = r2;
  dig[wpos] = d;
  wpos = (uint16_t)((wpos + 1) % TOTAL_PTS);
}

static uint16_t idx_of(uint16_t k)
{
  return (uint16_t)((wpos + k) % TOTAL_PTS);
}

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

#ifndef RX_SOURCE_UART
/* 假数据源模式：没有 A 板，直接伪造一帧状态帧，让触发面板能看到回执。
   真实 UART 模式下由 A 板按协议回状态帧。 */
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

/* ---------------- 控制帧下发 ---------------- */
static void send_ctrl(uint8_t cmd, uint16_t param)
{
  uint8_t buf[PROTO_CTRL_LEN];

  proto_build_ctrl(buf, cmd, param);
  HAL_UART_Transmit(&huart6, buf, PROTO_CTRL_LEN, 20);

#ifndef RX_SOURCE_UART
  fake_status_reply();        /* 自测模式：模拟 A 板回执 */
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
    uint16_t v = buf[idx_of(k)];
    if (v < mn) { mn = v; }
    if (v > mx) { mx = v; }
  }

  *vpp_mv = (uint16_t)(((uint32_t)(mx - mn) * 3300UL) / 4095UL);

  /* 信号几乎无变化（如通道关闭或直流），无法测频 */
  if ((uint16_t)(mx - mn) < 50) {
    *freq_hz = 0;
    return;
  }

  mid = (uint16_t)(((uint32_t)mn + mx) / 2U);
  for (k = 1; k < TOTAL_PTS; k++) {
    uint16_t p = buf[idx_of((uint16_t)(k - 1))];
    uint16_t c = buf[idx_of(k)];
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

  prev = (dig[idx_of(0)] & mask) ? 1 : 0;
  if (prev) { high_len = 1; }

  for (k = 1; k < TOTAL_PTS; k++) {
    uint8_t cur = (dig[idx_of(k)] & mask) ? 1 : 0;

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
  }
  for (i = 0; i <= 8; i++) {
    int y = WAVE_Y0 + i * WAVE_H / 8;
    GUI_DrawHLine(y, 0, SCR_W - 1);
  }
  for (i = 0; i <= 3; i++) {
    int y = DIG_Y0 + i * DIG_LANE_H;
    GUI_DrawHLine(y, 0, SCR_W - 1);
  }
}

/* ---------------- 模拟通道 ---------------- */
static void draw_analog(const uint16_t *buf, GUI_COLOR color)
{
  int i, x = 0, py = 0;

  GUI_SetColor(color);
  for (i = 0; i < TOTAL_PTS; i++) {
    int xi = idx_to_x(i);
    int y  = raw12_to_y(buf[idx_of((uint16_t)i)]);

    if (i == 0) {
      x  = xi;
      py = y;
    } else {
      GUI_DrawLine(x, py, xi, y);
      x  = xi;
      py = y;
    }
  }
}

/* ---------------- 数字通道 ----------------
   画法：低电平画在泳道底部，高电平画在泳道顶部，跳变沿用竖线连接。
   注意：GUI_DrawVLine(x, y0, y1) 要求 y0 <= y1，参数必须有序，
   否则上升沿（从低到高）会画不出来。这里统一做一次大小交换。 */
static void vline_ordered(int x, int y0, int y1)
{
  int t;

  if (y0 > y1) {
    t  = y0;
    y0 = y1;
    y1 = t;
  }
  GUI_DrawVLine(x, y0, y1);
}

static void draw_digital(void)
{
  static const GUI_COLOR col[3]  = { GUI_GREEN, GUI_YELLOW, GUI_MAGENTA };
  static const uint8_t   bitm[3] = { MASK_D1, MASK_D2, MASK_D3 };
  uint8_t m = proto_stat()->ch_mask;
  int ch;

  for (ch = 0; ch < 3; ch++) {
    int lane = DIG_Y0 + ch * DIG_LANE_H;
    int y_hi = lane + 4;
    int y_lo = lane + DIG_LANE_H - 6;
    int i, prev_y;
    int x_prev = 0;

    /* 通道关闭时不绘制 */
    if ((m & bitm[ch]) == 0) {
      continue;
    }

    GUI_SetColor(col[ch]);

    for (i = 0; i < TOTAL_PTS; i++) {
      int xi = idx_to_x(i);
      int y  = (dig[idx_of((uint16_t)i)] & bitm[ch]) ? y_hi : y_lo;

      if (i == 0) {
        prev_y = y;
      } else if (y != prev_y) {
        /* 跳变沿：画在当前列，保证 y0<=y1 */
        vline_ordered(xi, prev_y, y);
        GUI_DrawHLine(y, x_prev, xi);
        prev_y = y;
      } else {
        GUI_DrawHLine(y, x_prev, xi);
      }
      x_prev = xi;
    }
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
}

static void draw_topbar(uint32_t frame, uint8_t ts_ok, uint8_t link_ok)
{
  proto_stat_t *st = proto_stat();
  uint8_t g = (st->gear < 5) ? st->gear : g_gear;
  uint8_t m = st->ch_mask ? st->ch_mask : g_mask;
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

  /* 数据源：S=假数据自测，U=USART6 实时，L=链路断开 */
#ifdef RX_SOURCE_UART
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
  uint8_t confirmed = (uint8_t)(st->valid &&
                                st->trig_src   == g_trig_src &&
                                st->trig_mode  == g_trig_mode &&
                                st->trig_level == g_trig_level);

  GUI_SetFont(&GUI_Font6x8);
  GUI_SetTextMode(GUI_TM_TRANS);

  GUI_SetColor(confirmed ? GUI_GREEN : GUI_DARKGRAY);
  GUI_DrawRect(TRIG_X0, TRIG_Y0, TRIG_X1, TRIG_Y1);

  GUI_SetColor(g_trig_src ? GUI_YELLOW : GUI_GRAY);
  GUI_DispStringAt("TRIG", TRIG_X0 + 4, TRIG_Y0 + 1);
  GUI_DispStringAt(trig_src_txt[g_trig_src < 6 ? g_trig_src : 0],
                   TRIG_X0 + 28, TRIG_Y0 + 1);

  if (g_trig_src == 0) {
    return;
  }

  GUI_DispStringAt(trig_mode_txt[g_trig_mode < 3 ? g_trig_mode : 0],
                   TRIG_X0 + 50, TRIG_Y0 + 1);

  if (g_trig_src <= 2) {                       /* 模拟通道：显示电平 */
    GUI_DispDecAt((I32)trig_level_mv(), TRIG_X0 + 80, TRIG_Y0 + 1, 4);
    GUI_DispStringAt("mV", TRIG_X0 + 106, TRIG_Y0 + 1);
  } else {                                     /* 数字通道：只看逻辑沿 */
    GUI_DispStringAt("LOGIC", TRIG_X0 + 80, TRIG_Y0 + 1);
  }
}

/* ---------------- 底部测量信息 ---------------- */
static void draw_meas(void)
{
  uint16_t vpp1 = 0, vpp2 = 0;
  uint32_t f1 = 0, f2 = 0;
  uint32_t fd[3], pw[3];
  uint8_t  ch;
  uint8_t  m = proto_stat()->ch_mask;

  GUI_SetFont(&GUI_Font6x8);
  GUI_SetTextMode(GUI_TM_TRANS);

  draw_trig_indicator();        /* 数据未就绪时也要能配置触发 */

  if (proto_stat()->good < 12) {
    GUI_SetColor(GUI_GRAY);
    GUI_DispStringAt("measuring...", 2, MEAS_Y0 + 4);
    return;
  }

  if (m & MASK_CH1) { meas_analog(ch1, &vpp1, &f1); }
  if (m & MASK_CH2) { meas_analog(ch2, &vpp2, &f2); }

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

    GUI_SetColor((ch == 0) ? GUI_GREEN : (ch == 1) ? GUI_YELLOW : GUI_MAGENTA);
    GUI_DispStringAt("D", x, MEAS_Y0 + 11);
    GUI_DispDecAt((I32)(ch + 1), x + 6, MEAS_Y0 + 11, 1);
    GUI_DispStringAt(":", x + 12, MEAS_Y0 + 11);

    GUI_SetColor(GUI_WHITE);
    if (on) {
      meas_digital((uint8_t)(MASK_D1 << ch), &fd[ch], &pw[ch]);
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

  /* 行 3：触发电平 - / +（步进 16 ≈ 0.21V） */
  if (y >= TG_ROW3_Y && y <= TG_ROW3_Y + TG_ROW_H) {
    if (x >= TG_MINUS_X && x <= TG_MINUS_X + TG_ADJ_W) {
      g_trig_level = (uint8_t)((g_trig_level >= 16) ? (g_trig_level - 16) : 0);
      send_ctrl(CMD_TRIG_LEVEL, g_trig_level);
      return 1;
    }
    if (x >= TG_PLUS_X && x <= TG_PLUS_X + TG_ADJ_W) {
      g_trig_level = (uint8_t)((g_trig_level <= 239) ? (g_trig_level + 16) : 255);
      send_ctrl(CMD_TRIG_LEVEL, g_trig_level);
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
/* 导出用的线性副本（环形缓冲 → 时间顺序） */
static uint16_t s_lin1[TOTAL_PTS];
static uint16_t s_lin2[TOTAL_PTS];
static uint8_t  s_lind[TOTAL_PTS];

static void make_linear_copy(void)
{
  uint16_t i;

  for (i = 0; i < TOTAL_PTS; i++) {
    uint16_t j = idx_of(i);
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
        g_gear = (uint8_t)((g_gear + 1) % 5);
        send_ctrl(CMD_GEAR, g_gear);
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
        uint8_t bit = (uint8_t)(1u << (buttons[i].id - 1));
        g_mask ^= bit;
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
  GUI_MULTIBUF_Begin();

  GUI_SetBkColor(GUI_BLACK);
  GUI_Clear();
  draw_grid();
  if (proto_stat()->ch_mask & MASK_CH1) { draw_analog(ch1, GUI_BLUE); }
  if (proto_stat()->ch_mask & MASK_CH2) { draw_analog(ch2, GUI_RED);  }
  draw_digital();
  draw_topbar(g_last_frame, g_last_ts_ok, g_last_link_ok);
  draw_meas();
  draw_keypad();          /* 键盘弹出时覆盖在波形之上 */
  draw_menu();            /* 导出菜单弹出时覆盖在最上层 */
  draw_trig_panel();      /* 触发设置面板 */

  GUI_MULTIBUF_End();
}

/* ---------------- 主任务 ---------------- */
void MainTask(void)
{
#ifdef RX_SOURCE_UART
  /* UART 模式不需要帧缓冲 */
#else
  uint8_t  frame_buf[PROTO_DATA_LEN];
  uint16_t i;
#endif
  uint32_t frame = 0;
  uint16_t tx, ty;
  uint8_t  ts_ok;
  uint8_t  key_prev = 0;
  uint32_t last_good = 0;
  uint32_t last_status = 0;
  uint32_t stall = 0;
  uint8_t  link_ok = 1;

#ifdef RX_SOURCE_UART
  proto_reset();
  uart_rx_start();
#else
  fakesrc_init();
  proto_reset();
#endif
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

  for (;;) {
    /* --- 触摸（上升沿触发一次） --- */
    if (touch_read(&tx, &ty)) {
      if (!g_touch_down) {
        g_touch_down = 1;
        handle_tap(tx, ty);
      }
    } else {
      g_touch_down = 0;
    }

    /* --- 板载按键 B1：按一次切换一个档位 --- */
    {
      uint8_t key = (HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_11) == GPIO_PIN_SET) ? 1 : 0;
      if (key && !key_prev) {
        g_gear = (uint8_t)((g_gear + 1) % 5);
        send_ctrl(CMD_GEAR, g_gear);
      }
      key_prev = key;
    }

    /* --- 取数据：假数据源自造 或 USART6 实时接收 --- */
#ifdef RX_SOURCE_UART
    uart_rx_poll(proto_feed);
#else
    fakesrc_tick(frame_buf, g_gear, g_mask);
    for (i = 0; i < PROTO_DATA_LEN; i++) {
      proto_feed(frame_buf[i]);
    }
#endif

    /* --- A 板状态帧回执：以其为准同步触发配置 --- */
    if (proto_status()->valid && proto_status()->count != last_status) {
      last_status  = proto_status()->count;
      g_trig_src   = proto_status()->trig_src;
      g_trig_mode  = proto_status()->trig_mode;
      g_trig_level = proto_status()->trig_level;
    }

    /* --- 有新帧 → 50 点灌入环形缓冲 --- */
    if (proto_frame_ready()) {
      const proto_frame_t *f = proto_frame();
      uint16_t pt;

      for (pt = 0; pt < PROTO_N_POINTS; pt++) {
        const uint8_t *p = &f->payload[pt * PROTO_POINT_LEN];
        uint16_t w1 = (uint16_t)(p[PNT_CH1_L] | ((uint16_t)p[PNT_CH1_H] << 8));
        uint16_t w2 = (uint16_t)(p[PNT_CH2_L] | ((uint16_t)p[PNT_CH2_H] << 8));

        ring_push((uint16_t)(w1 >> 4),
                  (uint16_t)(w2 >> 4),
                  (uint8_t)(p[PNT_DIG] & (MASK_D1 | MASK_D2 | MASK_D3)));
      }
    }

    /* --- 链路监测：连续 500ms 没有新有效帧则判定断开 --- */
    if (proto_stat()->good != last_good) {
      last_good = proto_stat()->good;
      stall = 0;
      link_ok = 1;
    } else {
      stall++;
      if (stall >= 10) {          /* 10 × 50ms = 500ms */
        link_ok = 0;
      }
    }

    /* --- 绘制到后台缓冲，结束后整屏切换，消除闪烁 --- */
    g_last_frame   = frame;
    g_last_ts_ok   = ts_ok;
    g_last_link_ok = link_ok;
    draw_screen();

    frame++;
    HAL_Delay(50);      /* 显示刷新约 20 帧/s */
  }
}

/*************************** End of file ****************************/