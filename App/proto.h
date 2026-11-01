/*********************************************************************
* 文件：proto.h
* 说明：远程数据采集系统通信协议 V1.1 —— 解析/组帧/CRC 公共头
*       依据《通信协议规范 V1.1》：260 字节数据帧 / 9 字节控制帧 /
*       12 字节状态帧；CRC-16/CCITT-FALSE；多字节字段小端序。
*********************************************************************/
#ifndef PROTO_H
#define PROTO_H

#include <stdint.h>

/* 帧头 */
#define PROTO_HEAD0          0xAA
#define PROTO_HEAD1          0x55

/* 帧类型 */
#define PROTO_TYPE_DATA      0x01
#define PROTO_TYPE_CTRL      0x02
#define PROTO_TYPE_STATUS    0x03

/* 固定长度 */
#define PROTO_DATA_LEN       260
#define PROTO_CTRL_LEN       9
#define PROTO_STATUS_LEN     12

/* 数据帧字段偏移 */
#define OFS_SEQ_L            3
#define OFS_SEQ_H            4
#define OFS_GEAR             5
#define OFS_CH_MASK          6
#define OFS_N                7
#define OFS_PAYLOAD          8
#define OFS_CRC_L            (PROTO_DATA_LEN - 2)
#define OFS_CRC_H            (PROTO_DATA_LEN - 1)

/* 负载参数 */
#define PROTO_N_POINTS       50
#define PROTO_POINT_LEN      5
#define PROTO_PAYLOAD_LEN    (PROTO_N_POINTS * PROTO_POINT_LEN)   /* 250 */

/* 采样点内偏移 */
#define PNT_CH1_L            0
#define PNT_CH1_H            1
#define PNT_CH2_L            2
#define PNT_CH2_H            3
#define PNT_DIG              4

/* 通道掩码（CH_MASK 字段，协议第 3 节：bit0=CH1 bit1=CH2 bit2=D1 bit3=D2 bit4=D3） */
#define MASK_CH1             0x01
#define MASK_CH2             0x02
#define MASK_D1              0x04
#define MASK_D2              0x08
#define MASK_D3              0x10

/* 数字采样位（数据帧 DIG 字节，协议第 3.1 节：bit0=D1 bit1=D2 bit2=D3）
   注意与上面的 MASK_D* 区分：
     MASK_D* 用于 CH_MASK（通道开关），协议规定 D1 在 bit2；
     DIG_D*  用于采样点的 DIG 字节，协议规定 D1 在 bit0。
   两者是不同字段，位位置不同，不可混用。 */
#define DIG_D1               0x01
#define DIG_D2               0x02
#define DIG_D3               0x04

/* 时基档位 */
#define GEAR_10MS            0
#define GEAR_5MS             1
#define GEAR_1MS             2
#define GEAR_500US           3
#define GEAR_100US           4

/* 状态帧字段偏移（A→B，固定 12 字节） */
#define OFS_ST_GEAR          3
#define OFS_ST_CH_MASK       4
#define OFS_ST_TRIG_SRC      5
#define OFS_ST_TRIG_MODE     6
#define OFS_ST_TRIG_LEVEL    7
#define OFS_ST_FREQ_L        8
#define OFS_ST_FREQ_H        9

/* 控制帧命令（B→A） */
#define CMD_GEAR             0x01
#define CMD_CHMASK           0x02
#define CMD_TRIG_SRC         0x03
#define CMD_TRIG_MODE        0x04
#define CMD_TRIG_LEVEL       0x05
#define CMD_HANDSHAKE        0x06
#define CMD_WAVE_FREQ        0x07

/* 解析统计 */
typedef struct {
  uint32_t good;          /* 累计有效数据帧数 */
  uint32_t crc_err;       /* CRC 校验失败计数 */
  uint32_t drop;          /* SEQ 跳变检测到的丢帧数 */
  uint32_t last_seq;      /* 上一帧 SEQ */
  uint16_t gear;          /* 当前时基 */
  uint8_t  ch_mask;       /* 当前通道掩码 */
} proto_stat_t;

/* 供显示端读取的整帧结果 */
typedef struct {
  uint8_t      ready;                 /* 收到一帧有效数据帧 = 1 */
  uint8_t      seq[2];                /* 原始 SEQ 字节 */
  uint8_t      gear;
  uint8_t      ch_mask;
  uint8_t      gap;                   /* 与上一有效帧 SEQ 不连续（内部标志，不上线） */
  uint8_t      payload[PROTO_PAYLOAD_LEN];   /* 50 点 × 5 字节 */
} proto_frame_t;

/* 每解析出一帧有效数据帧就回调一次（在 proto_feed 调用上下文执行）。
   显示端借此把"一次轮询解析出的多帧"全部取走，而不是只留最后一帧。 */
typedef void (*proto_frame_cb_t)(const proto_frame_t *f);

/* 状态帧解析结果（A 板回报的当前设置） */
typedef struct {
  uint8_t  valid;        /* 收到过有效状态帧 = 1 */
  uint32_t count;        /* 有效状态帧累计数，用于判断“有新回执” */
  uint8_t  gear;
  uint8_t  ch_mask;
  uint8_t  trig_src;     /* 0=关闭 1=CH1 2=CH2 3=D1 4=D2 5=D3 */
  uint8_t  trig_mode;    /* 0=上升沿 1=下降沿 2=任意边沿 */
  uint8_t  trig_level;   /* 0~255，线性映射 0~VREF */
  uint16_t wave_freq;    /* 当前矩形波频率，Hz */
} proto_status_t;

/* ---- API ---- */
uint16_t proto_crc16(const uint8_t *buf, uint32_t len);
void     proto_reset(void);           /* 清空解析器状态 */
void     proto_feed(uint8_t b);       /* 喂一个字节，内部状态机 */
void     proto_set_frame_cb(proto_frame_cb_t cb);  /* 注册每帧回调（可传 0 取消） */
int      proto_frame_ready(void);     /* 是否有新的有效数据帧 */
const proto_frame_t *proto_frame(void);
proto_stat_t *proto_stat(void);
const proto_status_t *proto_status(void);   /* 最近一次状态帧（A 板回执） */
const uint8_t *proto_status_raw(void);      /* 最近一次状态帧的原始 12 字节（联调核对用） */

/* 组一帧控制帧（B→A），返回帧长（9） */
uint16_t proto_build_ctrl(uint8_t *out, uint8_t cmd, uint16_t param);

/* 组一帧状态帧（A 板用 / 假数据源自测），返回帧长（12） */
uint16_t proto_build_status(uint8_t *out, uint8_t gear, uint8_t ch_mask,
                            uint8_t trig_src, uint8_t trig_mode, uint8_t trig_level,
                            uint16_t wave_freq);

/* 组一帧数据帧（A 板用，B 板自测也可用） */
uint16_t proto_build_data(uint8_t *out, uint16_t seq, uint8_t gear,
                          uint8_t ch_mask,
                          const uint8_t (*points)[PROTO_POINT_LEN]);

#endif /* PROTO_H */