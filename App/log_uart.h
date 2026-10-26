/*********************************************************************
* 文件：log_uart.h
* 说明：调试/导出串口 —— USART1，经板载 ST-Link 虚拟串口连到 PC。
*
*   硬件：STM32F746G-DISCOVERY 的 ST-Link/V2-1 自带 VCP，
*         接 USART1（TX=PA9, RX=PB7, AF7）。
*         插上 CN14 的 USB 线，PC 上即出现一个 COM 口。
*
*   用途：
*     1) 导出 CSV / BMP 文件内容到 PC（任务书"通过USB接口获取文件"）
*     2) 调试信息打印
*
*   注意：与 USART6（A 板数据链路）完全独立，互不影响。
*********************************************************************/
#ifndef LOG_UART_H
#define LOG_UART_H

#include <stdint.h>

/* 初始化 USART1 @ 921600 8N1 */
void log_uart_init(void);

/* 发送原始字节 */
void log_uart_write(const uint8_t *buf, uint32_t len);

/* 发送以 0 结尾的字符串 */
void log_uart_str(const char *s);

#endif /* LOG_UART_H */