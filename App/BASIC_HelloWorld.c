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
  uint8_t  id;        /* 0 = 档位；1..5 = 通道 */
} btn_t;

static const btn_t buttons[] = {
  {   0,  70, 0 },     /* 档位 */
  {  74, 104, 1 },     /* CH1 */
  { 106, 136, 2 },     /* CH2 */
  { 138, 174, 3 },     /* D1  */
  { 176, 212, 4 },     /* D2  */
  { 214, 250, 5 },     /* D3  */
};
#define BTN_NUM   (sizeof(buttons) / sizeof(buttons[0]))

/* ---------------- 本地状态 ---------------- */
static uint8_t g_gear = GEAR_10MS;
static uint8_t g_mask = MASK_CH1 | MASK_CH2 | MASK_D1 | MASK_D2 | MASK_D3;
static uint8_t g_touch_down = 0;

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

/* ---------------- 控制帧下发 ---------------- */
static void send_ctrl(uint8_t cmd, uint16_t param)
{
  uint8_t buf[PROTO_CTRL_LEN];

  proto_build_ctrl(buf, cmd, param);
  HAL_UART_Transmit(&huart6, buf, PROTO_CTRL_LEN, 20);
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

  /* 右侧统计信息（6x8 字体） */
  GUI_SetFont(&GUI_Font6x8);
  GUI_SetTextMode(GUI_TM_TRANS);

  GUI_SetColor(GUI_WHITE);
  GUI_DispStringAt("F", 256, 2);
  GUI_DispDecAt((I32)(frame % 100000), 262, 2, 5);

  GUI_DispStringAt("dp", 296, 2);
  GUI_DispDecAt((I32)st->drop, 308, 2, 4);

  GUI_DispStringAt("ce", 334, 2);
  GUI_DispDecAt((I32)st->crc_err, 346, 2, 4);

  /* 触摸驱动状态 */
  if (ts_ok) {
    GUI_SetColor(GUI_GREEN);
    GUI_DispStringAt("TS", 370, 2);
  } else {
    GUI_SetColor(GUI_RED);
    GUI_DispStringAt("TE", 370, 2);
    GUI_DispDecAt((I32)touch_error(), 382, 2, 1);
  }

  /* 数据源：F=假数据自测，U=USART6 实时接收；link_ok=0 表示超过 500ms 无有效帧 */
#ifdef RX_SOURCE_UART
  GUI_SetColor(link_ok ? GUI_GREEN : GUI_RED);
  GUI_DispStringAt(link_ok ? "U OK" : "U LOST", 396, 2);
#else
  GUI_SetColor(GUI_CYAN);
  GUI_DispStringAt("F SIM", 396, 2);
#endif
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

/* ---------------- 触摸处理 ---------------- */
static void handle_tap(uint16_t x, uint16_t y)
{
  uint8_t i;

  if (y > TOP_H) {
    return;                 /* 只有顶部按钮区响应 */
  }

  for (i = 0; i < BTN_NUM; i++) {
    if (x >= buttons[i].x0 && x <= buttons[i].x1) {
      if (buttons[i].id == 0) {
        /* 档位循环：10ms → 5ms → 1ms → 500us → 100us */
        g_gear = (uint8_t)((g_gear + 1) % 5);
        send_ctrl(CMD_GEAR, g_gear);
      } else {
        uint8_t bit = (uint8_t)(1u << (buttons[i].id - 1));
        g_mask ^= bit;
        send_ctrl(CMD_CHMASK, g_mask);
      }
      return;
    }
  }
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
    GUI_MULTIBUF_Begin();

    GUI_SetBkColor(GUI_BLACK);
    GUI_Clear();
    draw_grid();
    if (proto_stat()->ch_mask & MASK_CH1) { draw_analog(ch1, GUI_BLUE); }
    if (proto_stat()->ch_mask & MASK_CH2) { draw_analog(ch2, GUI_RED);  }
    draw_digital();
    draw_topbar(frame, ts_ok, link_ok);
    draw_meas();

    GUI_MULTIBUF_End();

    frame++;
    HAL_Delay(50);      /* 显示刷新约 20 帧/s */
  }
}

/*************************** End of file ****************************/