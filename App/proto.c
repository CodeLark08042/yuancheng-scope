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
static proto_state_t g_state = ST_HEAD0;
static uint8_t       g_rxbuf[PROTO_DATA_LEN];
static uint16_t      g_len;      /* 当前帧已收字节数 */
static uint16_t      g_want;     /* 当前帧期望总长 */

/* ------------------------------------------------------------------
 * CRC-16/CCITT-FALSE
 * poly=0x1021, init=0xFFFF, RefIn/RefOut=false, XorOut=0x0000
 * 标准测试向量：ASCII "123456789" → 0x29B1（线上先发 B1 再发 29）
 * -----------------------------------------------------------------*/
uint16_t proto_crc16(const uint8_t *buf, uint32_t len)
{
  uint16_t crc = 0xFFFF;
  uint32_t i;

  for (i = 0; i < len; i++) {
    int bit;

    crc ^= ((uint16_t)buf[i]) << 8;
    for (bit = 0; bit < 8; bit++) {
      if (crc & 0x8000U) {
        crc = (uint16_t)((crc << 1) ^ 0x1021U);
      } else {
        crc = (uint16_t)(crc << 1);
      }
    }
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
    if (g_stat.good > 0) {
      uint16_t gap = (uint16_t)(seq - g_stat.last_seq);   /* 无符号，天然处理回绕 */
      if (gap > 1) {
        g_stat.drop += (uint32_t)(gap - 1);
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
  }
  /* 控制帧 / 状态帧：显示端目前不主动推送，暂只计数/忽略内容 */
}

/* ---------------- API ---------------- */

void proto_reset(void)
{
  g_state = ST_HEAD0;
  g_len   = 0;
  g_want  = 0;
  g_frame.ready = 0;
  g_stat.good   = 0;
  g_stat.crc_err = 0;
  g_stat.drop   = 0;
  g_stat.last_seq = 0;
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