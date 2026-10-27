/*********************************************************************
* 文件：export.c
* 说明：CSV / BMP 导出实现，见 export.h。
*
*   设计要点：
*     1) 不占用大块内存：CSV 逐行拼装后发送，BMP 逐行生成后发送。
*     2) 串口 921600 baud，CSV 约 20KB / 0.2s，BMP 约 383KB / 4.3s。
*     3) BMP 从 LTDC 当前扫描的帧缓冲读取（三缓冲中正在显示的那一块），
*        因此导出的是导出瞬间屏幕上真实的画面，不含"Exporting"提示。
*********************************************************************/
#include "main.h"
#include "log_uart.h"
#include "export.h"
#include "proto.h"       /* DIG_D1/D2/D3：DIG 字节位定义（bit0~2，协议第 3.1 节） */

#define SCR_W            480
#define SCR_H            272
#define FB_ADDR          0xC0000000UL   /* 第 0 块帧缓冲（兜底用） */
#define FB_ADDR_END      0xC0200000UL   /* 三缓冲总占用上限 */

/* LTDC 层 1 当前帧缓冲地址寄存器（LTDC_LxCFBAR，偏移 0xAC） */
#define LTDC_L1CFBAR     (*(volatile uint32_t *)0x400168ACUL)

static const uint16_t dt_us_tab[5] = { 200, 100, 20, 10, 2 };

/* 行缓冲：BMP 一行 480 像素 × 3 字节 = 1440 字节（正好 4 字节对齐） */
static uint8_t s_row[SCR_W * 3];

/* 无符号整数转十进制字符串，返回写入长度 */
static int u32_to_str(uint32_t v, char *out)
{
  char tmp[12];
  int  n = 0;
  int  i = 0;

  if (v == 0) {
    out[0] = '0';
    out[1] = 0;
    return 1;
  }
  while (v > 0 && n < 11) {
    tmp[n++] = (char)('0' + (v % 10));
    v /= 10;
  }
  while (n > 0) {
    out[i++] = tmp[--n];
  }
  out[i] = 0;
  return i;
}

/* 追加字符串，返回新的写入位置 */
static uint32_t app_str(uint8_t *dst, uint32_t pos, const char *s)
{
  while (*s) {
    dst[pos++] = (uint8_t)(*s++);
  }
  return pos;
}

/* 追加无符号数 */
static uint32_t app_u32(uint8_t *dst, uint32_t pos, uint32_t v)
{
  char buf[12];
  u32_to_str(v, buf);
  return app_str(dst, pos, buf);
}

uint32_t export_csv(const uint16_t *ch1, const uint16_t *ch2,
                    const uint8_t *dig, uint16_t n, uint8_t gear)
{
  static uint8_t line[160];
  uint32_t pos;
  uint32_t sent = 0;
  uint16_t k;
  uint32_t dt;

  if (gear > 4) { gear = 0; }
  dt = dt_us_tab[gear];

  /* --- 表头：便于 Excel 直接打开理解 --- */
  pos = 0;
  pos = app_str(line, pos, "# Remote Data Acquisition System - Waveform Export\r\n");
  sent += pos; log_uart_write(line, pos);

  pos = 0;
  pos = app_str(line, pos, "# Gear=");
  pos = app_str(line, pos, gear == 0 ? "10ms/div" :
                        gear == 1 ? "5ms/div"  :
                        gear == 2 ? "1ms/div"  :
                        gear == 3 ? "500us/div" : "100us/div");
  pos = app_str(line, pos, ", dt_us=");
  pos = app_u32(line, pos, dt);
  pos = app_str(line, pos, ", points=");
  pos = app_u32(line, pos, n);
  pos = app_str(line, pos, ", pts_per_div=50\r\n");
  sent += pos; log_uart_write(line, pos);

  pos = 0;
  pos = app_str(line, pos, "# Vref_mV=3300 (nominal), raw12 range 0..4095\r\n");
  sent += pos; log_uart_write(line, pos);

  pos = 0;
  pos = app_str(line, pos, "idx,time_us,CH1_raw,CH1_mV,CH2_raw,CH2_mV,D1,D2,D3\r\n");
  sent += pos; log_uart_write(line, pos);

  /* --- 数据行 --- */
  for (k = 0; k < n; k++) {
    uint16_t r1 = ch1[k];
    uint16_t r2 = ch2[k];
    uint8_t  d  = dig[k];

    pos = 0;
    pos = app_u32(line, pos, k);
    pos = app_str(line, pos, ",");
    pos = app_u32(line, pos, (uint32_t)k * dt);
    pos = app_str(line, pos, ",");
    pos = app_u32(line, pos, r1);
    pos = app_str(line, pos, ",");
    pos = app_u32(line, pos, (uint32_t)r1 * 3300UL / 4095UL);
    pos = app_str(line, pos, ",");
    pos = app_u32(line, pos, r2);
    pos = app_str(line, pos, ",");
    pos = app_u32(line, pos, (uint32_t)r2 * 3300UL / 4095UL);
    pos = app_str(line, pos, ",");
    pos = app_u32(line, pos, (d & DIG_D1) ? 1 : 0);
    pos = app_str(line, pos, ",");
    pos = app_u32(line, pos, (d & DIG_D2) ? 1 : 0);
    pos = app_str(line, pos, ",");
    pos = app_u32(line, pos, (d & DIG_D3) ? 1 : 0);
    pos = app_str(line, pos, "\r\n");

    log_uart_write(line, pos);
    sent += pos;
  }

  return sent;
}

/* 写 16 位小端（BMP 头用） */
static void put_u16(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
}

/* 写 32 位小端 */
static void put_u32(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
  p[2] = (uint8_t)((v >> 16) & 0xFF);
  p[3] = (uint8_t)((v >> 24) & 0xFF);
}

uint32_t export_lcd_fb(void)
{
  uint32_t fb = LTDC_L1CFBAR;

  if (fb < FB_ADDR || fb >= FB_ADDR_END) {
    fb = FB_ADDR;
  }
  return fb;
}

uint32_t export_bmp(uint32_t fb_addr)
{
  uint8_t  hdr[54];
  uint32_t row_bytes = SCR_W * 3;             /* 1440，已 4 字节对齐 */
  uint32_t img_bytes = row_bytes * SCR_H;
  uint32_t file_size = 54 + img_bytes;
  uint32_t sent = 0;
  uint32_t fb = fb_addr;
  int      y;

  /* --- BITMAPFILEHEADER (14 字节) --- */
  hdr[0] = 'B';
  hdr[1] = 'M';
  put_u32(&hdr[2], file_size);
  put_u16(&hdr[6], 0);                        /* 保留 */
  put_u16(&hdr[8], 0);
  put_u32(&hdr[10], 54);                      /* 像素数据偏移 */

  /* --- BITMAPINFOHEADER (40 字节) --- */
  put_u32(&hdr[14], 40);                      /* 头大小 */
  put_u32(&hdr[18], SCR_W);                   /* 宽 */
  put_u32(&hdr[22], SCR_H);                   /* 高（正数=自底向上） */
  put_u16(&hdr[26], 1);                       /* 平面数 */
  put_u16(&hdr[28], 24);                      /* 位深 */
  put_u32(&hdr[30], 0);                       /* 不压缩 */
  put_u32(&hdr[34], img_bytes);               /* 图像大小 */
  put_u32(&hdr[38], 2835);                    /* 水平 DPI ≈ 72 */
  put_u32(&hdr[42], 2835);                    /* 垂直 DPI */
  put_u32(&hdr[46], 0);                       /* 调色板数 */
  put_u32(&hdr[50], 0);                       /* 重要颜色数 */

  log_uart_write(hdr, 54);
  sent += 54;

  /* --- 像素数据：BMP 自底向上，BGR 顺序 --- */
  for (y = SCR_H - 1; y >= 0; y--) {
    const volatile uint32_t *src =
        (const volatile uint32_t *)(fb) + (uint32_t)y * SCR_W;
    int x;

    for (x = 0; x < SCR_W; x++) {
      uint32_t px = src[x];                   /* ARGB8888 */
      s_row[x * 3 + 0] = (uint8_t)((px >> 0)  & 0xFF);   /* B */
      s_row[x * 3 + 1] = (uint8_t)((px >> 8)  & 0xFF);   /* G */
      s_row[x * 3 + 2] = (uint8_t)((px >> 16) & 0xFF);   /* R */
    }
    log_uart_write(s_row, row_bytes);
    sent += row_bytes;
  }

  return sent;
}

/*************************** End of file ****************************/