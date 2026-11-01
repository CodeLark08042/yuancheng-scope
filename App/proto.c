/*********************************************************************
* 文件：proto.c
* 说明：远程数据采集系统通信协议 V1.1 实现
*       - CRC-16/CCITT-FALSE（软件实现，测试向量 "123456789" → 0x29B1）
*       - 字节流帧同步状态机（数据/控制/状态三种帧）
*       - SEQ 循环丢帧统计
*       所有多字节字段按小端序（低字节先发）。
*********************************************************************/
#include "proto.h"

/* 命令超时/内部状态 */
typedef enum {
  ST_HEAD0 = 0,
  ST_HEAD1,
  ST_BODY
} proto_state_t;

static proto_frame_t g_frame;
static proto_stat_t  g_stat;
static proto_status_t g_status;
/* 最近一帧有效状态帧的原始字节，供联调时核对字段/CRC（Live Watch 直接查看） */
static uint8_t       g_status_raw[PROTO_STATUS_LEN];
static proto_frame_cb_t g_frame_cb;   /* 每帧有效数据帧的回调（显示端注册） */
static proto_state_t g_state = ST_HEAD0;
static uint8_t       g_rxbuf[PROTO_DATA_LEN];
static uint16_t      g_len;      /* 当前帧已收字节数 */
static uint16_t      g_want;     /* 当前帧期望总长 */

/* ------------------------------------------------------------------
 * CRC-16/CCITT-FALSE
 * poly=0x1021, init=0xFFFF, RefIn/RefOut=false, XorOut=0x0000
 * 标准测试向量：ASCII "123456789" → 0x29B1（线上先发 B1 再发 29）
 *
 * 查表实现：逐位算 258 字节约 2 万周期（约 75us）。G3/G4 档每秒近 1800 帧，
 * 光 CRC 就要吃掉一半 CPU，界面因此卡顿；查表后每字节一次查表异或，
 * 耗时降到约 1/8。表按上面的多项式静态生成，算法与逐位版完全一致。
 * -----------------------------------------------------------------*/
static const uint16_t s_crc_tab[256] = {
  0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
  0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
  0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
  0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
  0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
  0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
  0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
  0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
  0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
  0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
  0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
  0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
  0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
  0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
  0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
  0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
  0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
  0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
  0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
  0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
  0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
  0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
  0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
  0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
  0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
  0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
  0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
  0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
  0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
  0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
  0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
  0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0,
};

uint16_t proto_crc16(const uint8_t *buf, uint32_t len)
{
  uint16_t crc = 0xFFFF;
  uint32_t i;

  for (i = 0; i < len; i++) {
    crc = (uint16_t)((crc << 8) ^ s_crc_tab[((crc >> 8) ^ buf[i]) & 0xFF]);
  }
  return crc;
}

/* 根据 TYPE 判定帧总长 */
static uint16_t frame_len_for_type(uint8_t type)
{
  switch (type) {
    case PROTO_TYPE_DATA:   return PROTO_DATA_LEN;
    case PROTO_TYPE_CTRL:   return PROTO_CTRL_LEN;
    case PROTO_TYPE_STATUS: return PROTO_STATUS_LEN;
    default:                return 0;
  }
}

/* 一帧收满后的统一校验与分发 */
static void frame_complete(void)
{
  uint8_t type = g_rxbuf[2];
  uint16_t crc_len = g_want - 2;
  uint16_t calc, rcvd;

  calc = proto_crc16(g_rxbuf, crc_len);
  rcvd = (uint16_t)(g_rxbuf[crc_len] | ((uint16_t)g_rxbuf[crc_len + 1] << 8));

  if (calc != rcvd) {
    g_stat.crc_err++;
    return;   /* 校验失败，丢弃，重新同步 */
  }

  if (type == PROTO_TYPE_DATA) {
    uint16_t seq;
    uint16_t expected_len = PROTO_DATA_LEN;
    (void)expected_len;

    seq = (uint16_t)(g_rxbuf[OFS_SEQ_L] | ((uint16_t)g_rxbuf[OFS_SEQ_H] << 8));

    /* SEQ 循环比较：计算实际跳变，累加入 drop */
    g_frame.gap = 0;
    if (g_stat.good > 0) {
      uint16_t gap = (uint16_t)(seq - g_stat.last_seq);   /* 无符号，天然处理回绕 */
      if (gap > 1) {
        g_stat.drop += (uint32_t)(gap - 1);
        g_frame.gap   = 1;
      }
    }
    g_stat.last_seq = seq;
    g_stat.good++;

    /* 提取整帧内容供显示端使用 */
    g_frame.ready   = 1;
    g_frame.seq[0]  = g_rxbuf[OFS_SEQ_L];
    g_frame.seq[1]  = g_rxbuf[OFS_SEQ_H];
    g_frame.gear    = g_rxbuf[OFS_GEAR];
    g_frame.ch_mask = g_rxbuf[OFS_CH_MASK];
    {
      uint16_t i;
      for (i = 0; i < PROTO_PAYLOAD_LEN; i++) {
        g_frame.payload[i] = g_rxbuf[OFS_PAYLOAD + i];
      }
    }
    g_stat.gear    = g_frame.gear;
    g_stat.ch_mask = g_frame.ch_mask;

    /* 通知显示端：本帧已完整解析，立即入环（一次轮询的多帧都不会丢） */
    if (g_frame_cb != 0) {
      g_frame_cb(&g_frame);
    }
  } else if (type == PROTO_TYPE_STATUS) {
    /* A 板回执：记录当前档位、通道、触发配置和矩形波频率 */
    {
      uint16_t i;
      for (i = 0; i < PROTO_STATUS_LEN; i++) {
        g_status_raw[i] = g_rxbuf[i];
      }
    }
    g_status.gear       = g_rxbuf[OFS_ST_GEAR];
    g_status.ch_mask    = g_rxbuf[OFS_ST_CH_MASK];
    g_status.trig_src   = g_rxbuf[OFS_ST_TRIG_SRC];
    g_status.trig_mode  = g_rxbuf[OFS_ST_TRIG_MODE];
    g_status.trig_level = g_rxbuf[OFS_ST_TRIG_LEVEL];
    g_status.wave_freq  = (uint16_t)(g_rxbuf[OFS_ST_FREQ_L] |
                                     ((uint16_t)g_rxbuf[OFS_ST_FREQ_H] << 8));
    g_status.valid = 1;
    g_status.count++;
  }
  /* 控制帧：显示端不接收，忽略 */
}

/* ---------------- API ---------------- */

void proto_reset(void)
{
  g_state = ST_HEAD0;
  g_len   = 0;
  g_want  = 0;
  g_frame.ready = 0;
  g_frame.gap   = 0;
  g_stat.good   = 0;
  g_stat.crc_err = 0;
  g_stat.drop   = 0;
  g_stat.last_seq = 0;
  g_status.valid = 0;
  g_status.count = 0;
}

void proto_feed(uint8_t b)
{
  switch (g_state) {
    case ST_HEAD0:
      if (b == PROTO_HEAD0) {
        g_rxbuf[0] = b;
        g_len = 1;
        g_state = ST_HEAD1;
      }
      break;

    case ST_HEAD1:
      if (b == PROTO_HEAD1) {
        g_rxbuf[1] = b;
        g_len = 2;
        g_state = ST_BODY;
      } else {
        /* 首个头字节后再遇到非 0x55：视为重新开始找 0xAA */
        g_state = ST_HEAD0;
        proto_feed(b);   /* 递归一次，处理 "AA AA 55" 场景 */
      }
      break;

    case ST_BODY:
      g_rxbuf[g_len++] = b;
      if (g_len >= 3) {
        /* 初次判定总长（基于 TYPE） */
        if (g_want == 0) {
          g_want = frame_len_for_type(g_rxbuf[2]);
          if (g_want == 0) {   /* 非法 TYPE，重新同步 */
            g_state = ST_HEAD0;
            g_len = 0;
            g_want = 0;
            return;
          }
        }
      }
      if (g_len >= g_want) {
        frame_complete();
        g_state = ST_HEAD0;   /* 与下一帧无缝衔接 */
        g_len   = 0;
        g_want  = 0;
      }
      break;
  }
}

void proto_set_frame_cb(proto_frame_cb_t cb)
{
  g_frame_cb = cb;
}

int proto_frame_ready(void)
{
  return g_frame.ready;
}

const proto_frame_t *proto_frame(void)
{
  return &g_frame;
}

proto_stat_t *proto_stat(void)
{
  return &g_stat;
}

const proto_status_t *proto_status(void)
{
  return &g_status;
}

const uint8_t *proto_status_raw(void)
{
  return g_status_raw;
}

/* 组一帧控制帧（B→A），返回总长（固定 9） */
uint16_t proto_build_ctrl(uint8_t *out, uint8_t cmd, uint16_t param)
{
  uint16_t crc;

  out[0] = PROTO_HEAD0;
  out[1] = PROTO_HEAD1;
  out[2] = PROTO_TYPE_CTRL;
  out[3] = cmd;
  out[4] = (uint8_t)(param & 0xFF);        /* PARAM_L */
  out[5] = (uint8_t)((param >> 8) & 0xFF); /* PARAM_H */
  out[6] = 0x00;                           /* RESERVED */
  crc = proto_crc16(out, 7);               /* 覆盖 0..6 */
  out[7] = (uint8_t)(crc & 0xFF);
  out[8] = (uint8_t)((crc >> 8) & 0xFF);
  return PROTO_CTRL_LEN;
}

/* 组一帧状态帧（A 板用 / 假数据源自测），返回总长（固定 12） */
uint16_t proto_build_status(uint8_t *out, uint8_t gear, uint8_t ch_mask,
                            uint8_t trig_src, uint8_t trig_mode, uint8_t trig_level,
                            uint16_t wave_freq)
{
  uint16_t crc;

  out[0] = PROTO_HEAD0;
  out[1] = PROTO_HEAD1;
  out[2] = PROTO_TYPE_STATUS;
  out[OFS_ST_GEAR]       = gear;
  out[OFS_ST_CH_MASK]    = ch_mask;
  out[OFS_ST_TRIG_SRC]   = trig_src;
  out[OFS_ST_TRIG_MODE]  = trig_mode;
  out[OFS_ST_TRIG_LEVEL] = trig_level;
  out[OFS_ST_FREQ_L]     = (uint8_t)(wave_freq & 0xFF);
  out[OFS_ST_FREQ_H]     = (uint8_t)((wave_freq >> 8) & 0xFF);

  crc = proto_crc16(out, 10);              /* 覆盖 0..9 */
  out[10] = (uint8_t)(crc & 0xFF);
  out[11] = (uint8_t)((crc >> 8) & 0xFF);
  return PROTO_STATUS_LEN;
}

/* 组一帧数据帧（A 板用 / 自测用），返回总长（固定 260） */
uint16_t proto_build_data(uint8_t *out, uint16_t seq, uint8_t gear,
                          uint8_t ch_mask,
                          const uint8_t (*points)[PROTO_POINT_LEN])
{
  uint16_t crc;
  uint16_t i;

  out[0] = PROTO_HEAD0;
  out[1] = PROTO_HEAD1;
  out[2] = PROTO_TYPE_DATA;
  out[OFS_SEQ_L] = (uint8_t)(seq & 0xFF);
  out[OFS_SEQ_H] = (uint8_t)((seq >> 8) & 0xFF);
  out[OFS_GEAR] = gear;
  out[OFS_CH_MASK] = ch_mask;
  out[OFS_N] = PROTO_N_POINTS;

  for (i = 0; i < PROTO_N_POINTS; i++) {
    uint16_t j;
    for (j = 0; j < PROTO_POINT_LEN; j++) {
      out[OFS_PAYLOAD + i * PROTO_POINT_LEN + j] = points[i][j];
    }
  }

  crc = proto_crc16(out, PROTO_DATA_LEN - 2);   /* 覆盖 0..257 */
  out[OFS_CRC_L] = (uint8_t)(crc & 0xFF);
  out[OFS_CRC_H] = (uint8_t)((crc >> 8) & 0xFF);
  return PROTO_DATA_LEN;
}

/*************************** End of file ****************************/