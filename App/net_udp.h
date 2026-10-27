/*********************************************************************
* 文件：net_udp.h
* 说明：以太网 UDP 链路（协议 V1.1 的 UDP 模式）
*       - A→B：目的端口 5005，一个数据报 = 一帧（260 字节数据帧或 12 字节状态帧）
*       - B→A：目的端口 5006，9 字节控制帧
*       - 静态 IP：A=192.168.1.10，B=192.168.1.20
*
*   使用（裸机 NO_SYS=1）：
*     1) main.c 里 MX_LWIP_Init() 已由 CubeMX 调用
*     2) MainTask 开头调用 net_udp_init()
*     3) 主循环每轮调用 MX_LWIP_Process()，收包会回调进 proto_feed
*********************************************************************/
#ifndef NET_UDP_H
#define NET_UDP_H

#include <stdint.h>

#define NET_UDP_RX_PORT     5005      /* B 监听，收 A 的数据/状态帧 */
#define NET_UDP_TX_PORT     5006      /* A 监听，收 B 的控制帧 */
#define NET_A_IP0           192
#define NET_A_IP1           168
#define NET_A_IP2           1
#define NET_A_IP3           10

/* 建立 UDP 收发通道（需在 MX_LWIP_Init 之后调用） */
void net_udp_init(void);

/* 把一帧控制帧发往 A 板 5006 端口；链路未连接或内存不足时静默丢弃 */
void net_udp_send(const uint8_t *data, uint16_t len);

/* 物理链路是否已连接（PHY link） */
uint8_t net_udp_link_up(void);

/* 统计：长度合法的数据报数 / 长度非法被丢弃的数据报数 */
uint32_t net_udp_rx_ok(void);
uint32_t net_udp_rx_bad(void);

#endif /* NET_UDP_H */
