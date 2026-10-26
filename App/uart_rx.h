/*********************************************************************
* 文件：uart_rx.h
* 说明：USART6 接收模块 —— 把 A 板发来的字节流可靠搬运到协议解析器。
*
*   设计：
*     - 每字节中断（HAL_UART_Receive_IT 链式），921600 baud 下约 92k 次/s，
*       F7@216MHz 占用 CPU <10%，可接受（UART 仅用于 10ms/5ms 档）。
*     - 收到的字节先入 2048 字节软件环形缓冲，再由主循环弹出喂给
*       proto_feed()。这样协议解析只在主循环上下文执行，无中断竞争。
*     - 帧同步（AA 55）、CRC、SEQ 由 proto_feed 状态机负责，本模块只负责搬运。
*
*   使用：
*     主循环中：uart_rx_poll(&proto_feed);
*********************************************************************/
#ifndef UART_RX_H
#define UART_RX_H

#include <stdint.h>

/* 启动接收：配置 USART6 NVIC 并开始第一字节中断 */
void uart_rx_start(void);

/* 主循环轮询：把环形缓冲中新到的字节逐个交给 feed()（通常为 proto_feed）
   直至缓冲为空 */
void uart_rx_poll(void (*feed)(uint8_t));

/* 自上次调用以来收到的总字节数（用于统计/观察，可选） */
uint32_t uart_rx_bytes(void);

#endif /* UART_RX_H */