/*********************************************************************
* 文件：fakesrc.c
* 说明：A 板数据源模拟器实现。
*
*   核心思想：用 32 位相位累加器按「绝对时间」推进，
*             每点推进量 = f * dt_us * 2^32 / 1e6。
*   这样无论时基档位如何变化，信号频率都保持恒定，
*   而屏幕上每格内的波形周期数会随时基真实变化——
*   与真实采集端的表现完全一致。
*********************************************************************/
#include "proto.h"
#include "fakesrc.h"

/* 32 点正弦表，幅度 ±1000 */
static const int16_t sin32[32] = {
     0,  195,  383,  556,  707,  831,  924,  981,
  1000,  981,  924,  831,  707,  556,  383,  195,
     0, -195, -383, -556, -707, -831, -924, -981,
 -1000, -981, -924, -831, -707, -556, -383, -195
};

/* 各档位单点采样间隔（µs） */
static const uint16_t dt_tab[5] = { 200, 100, 20, 10, 2 };

/* 相位累加器（2^32 = 一个完整周期） */
static uint32_t ph1, ph2, phd1, phd2, phd3;
static uint16_t g_seq;

/* 把频率与采样间隔换算成每点的相位增量 */
static uint32_t phase_inc(uint32_t freq_hz, uint32_t dt_us)
{
  uint64_t v = (uint64_t)freq_hz * (uint64_t)dt_us;
  v = (v << 32) / 1000000ULL;
  return (uint32_t)v;
}

void fakesrc_init(void)
{
  ph1 = ph2 = phd1 = phd2 = phd3 = 0;
  g_seq = 0;
}

/* 归一化值(-1000..+1000) + 中心偏置 → raw12 */
static uint16_t norm_to_raw(int32_t norm, int32_t center)
{
  int32_t v = center + (norm * 1024L) / 1000L;
  if (v < 0)    { v = 0; }
  if (v > 4095) { v = 4095; }
  return (uint16_t)v;
}

uint16_t fakesrc_tick(uint8_t *out, uint8_t gear, uint8_t ch_mask)
{
  uint8_t  points[PROTO_N_POINTS][PROTO_POINT_LEN];
  uint32_t dt;
  uint32_t i1, i2, id1, id2, id3;
  uint16_t i;

  if (gear > 4) { gear = 0; }
  dt = dt_tab[gear];

  i1  = phase_inc(FAKE_FREQ_CH1, dt);
  i2  = phase_inc(FAKE_FREQ_CH2, dt);
  id1 = phase_inc(FAKE_FREQ_D1,  dt);
  id2 = phase_inc(FAKE_FREQ_D2,  dt);
  id3 = phase_inc(FAKE_FREQ_D3,  dt);

  for (i = 0; i < PROTO_N_POINTS; i++) {
    uint16_t raw1 = 0, raw2 = 0;
    uint8_t  dig  = 0;
    uint16_t wire1, wire2;

    ph1  += i1;
    ph2  += i2;
    phd1 += id1;
    phd2 += id2;
    phd3 += id3;

    if (ch_mask & MASK_CH1) {
      /* 32 步/周期正弦 */
      raw1 = norm_to_raw(sin32[(ph1 >> 27) & 31], 2048);
    }
    if (ch_mask & MASK_CH2) {
      /* 64 步/周期三角波 */
      uint32_t k = (ph2 >> 26) & 63;
      int32_t  v = (k < 32) ? (-1000 + (int32_t)k * 2000L / 31L)
                           : ( 1000 - (int32_t)(k - 32) * 2000L / 31L);
      raw2 = norm_to_raw(v, 2048);
    }
    if ((ch_mask & MASK_D1) && (phd1 & 0x80000000UL)) { dig |= MASK_D1; }
    if ((ch_mask & MASK_D2) && (phd2 & 0x80000000UL)) { dig |= MASK_D2; }
    if ((ch_mask & MASK_D3) && (phd3 & 0x80000000UL)) { dig |= MASK_D3; }

    /* 协议编码：wire16 = raw12 << 4，小端 */
    wire1 = (uint16_t)(raw1 << 4);
    wire2 = (uint16_t)(raw2 << 4);
    points[i][PNT_CH1_L] = (uint8_t)(wire1 & 0xFF);
    points[i][PNT_CH1_H] = (uint8_t)((wire1 >> 8) & 0xFF);
    points[i][PNT_CH2_L] = (uint8_t)(wire2 & 0xFF);
    points[i][PNT_CH2_H] = (uint8_t)((wire2 >> 8) & 0xFF);
    points[i][PNT_DIG]   = dig;
  }

  proto_build_data(out, g_seq, gear, ch_mask, points);

  return g_seq++;
}

/*************************** End of file ****************************/