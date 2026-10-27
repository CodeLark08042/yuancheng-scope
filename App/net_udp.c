/*********************************************************************
* 文件：net_udp.c
* 说明：以太网 UDP 链路实现（裸机 lwIP raw API）
*
*   收：绑定 5005，一个数据报必须恰好是一帧（260 字节数据帧 / 12 字节状态帧）。
*       长度不符的数据报按协议 §9 直接丢弃并计数；合法数据报逐字节喂给
*       proto_feed()，帧同步/CRC/SEQ 仍由 proto 状态机负责。
*   发：同一个 pcb 发出的包源端口即 5005，目的 192.168.1.10:5006。
*
*   注意：
*     - 回调在 MX_LWIP_Process() 的主循环上下文执行，无并发问题。
*     - 接收缓冲来自 ethernetif 的零拷贝 RX_POOL，HAL 已在 RxLinkCallback
*       里做过 D-Cache 无效化，这里可以直接读 payload。
*     - 发送缓冲来自 lwIP 堆（0x20048000），该区已在 main.c 的 MPU 里
*       配置为非缓存，因此不需要手工清理 D-Cache。
*********************************************************************/
#include "net_udp.h"
#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "proto.h"

static struct udp_pcb *s_pcb;
static ip_addr_t       s_a_ip;
static uint32_t        s_rx_ok;
static uint32_t        s_rx_bad;

/* 收到一个数据报：长度合法才喂给协议状态机 */
static void udp_recv_cb(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                        const ip_addr_t *addr, u16_t port)
{
  (void)arg;
  (void)pcb;
  (void)addr;
  (void)port;

  if (p == NULL) {
    return;
  }

  if (p->tot_len == PROTO_DATA_LEN || p->tot_len == PROTO_STATUS_LEN) {
    struct pbuf *q;

    for (q = p; q != NULL; q = q->next) {
      const uint8_t *d = (const uint8_t *)q->payload;
      u16_t i;

      for (i = 0; i < q->len; i++) {
        proto_feed(d[i]);
      }
    }
    s_rx_ok++;
  } else {
    s_rx_bad++;                 /* 长度错误：整包丢弃 */
  }

  pbuf_free(p);
}

void net_udp_init(void)
{
  if (s_pcb != NULL) {
    return;
  }

  s_pcb = udp_new();
  if (s_pcb == NULL) {
    return;
  }

  if (udp_bind(s_pcb, IP_ADDR_ANY, NET_UDP_RX_PORT) != ERR_OK) {
    udp_remove(s_pcb);
    s_pcb = NULL;
    return;
  }

  IP4_ADDR(&s_a_ip, NET_A_IP0, NET_A_IP1, NET_A_IP2, NET_A_IP3);
  udp_recv(s_pcb, udp_recv_cb, NULL);
}

void net_udp_send(const uint8_t *data, uint16_t len)
{
  struct pbuf *p;
  err_t err;

  if (s_pcb == NULL || data == NULL || len == 0) {
    return;
  }
  if (netif_default == NULL || !netif_is_link_up(netif_default)) {
    return;                     /* 链路未连接，不去占堆 */
  }

  p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
  if (p == NULL) {
    return;
  }

  err = pbuf_take(p, data, len);
  if (err == ERR_OK) {
    (void)udp_sendto(s_pcb, p, &s_a_ip, NET_UDP_TX_PORT);
  }

  pbuf_free(p);
}

uint8_t net_udp_link_up(void)
{
  return (uint8_t)((netif_default != NULL) && netif_is_link_up(netif_default));
}

uint32_t net_udp_rx_ok(void)
{
  return s_rx_ok;
}

uint32_t net_udp_rx_bad(void)
{
  return s_rx_bad;
}

/*************************** End of file ****************************/
