/*********************************************************************
* 文件：uart_rx.c
* 说明：USART6 接收模块实现 —— 见 uart_rx.h。
*
*   USART6 引脚：RX = PC7（已在 CubeMX 生成的 usart.c 中配置为 AF8）
*   中断向量：USART6_IRQHandler（需要在 stm32f7xx_it.c 中添加，
*   并 extern huart6）。
*********************************************************************/
#include "main.h"
#include "usart.h"
#include "uart_rx.h"

/* 5ms 档 A 板约 200 帧/s（260B/帧）≈52kB/s；一次绘制约 66ms 不轮询，
   期间积压约 3.4KB。原 2048 字节必溢出，改为 8192 留一倍余量。 */
#define RX_RING_SIZE   8192

static uint8_t s_ring[RX_RING_SIZE];
static volatile uint16_t s_rd = 0;     /* 读指针（主循环） */
static volatile uint16_t s_wr = 0;     /* 写指针（中断回调） */
static uint8_t s_rx_byte;
static volatile uint32_t s_total = 0;

/* 当且仅当 USART6 空闲时使用；回调里再次发起单字节接收，形成链式 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6) {
    uint16_t next = (uint16_t)((s_wr + 1) % RX_RING_SIZE);

    /* 环形仍有余量才写入；否则丢字节（信号已断开或主循环太慢） */
    if (next != s_rd) {
      s_ring[s_wr] = s_rx_byte;
      s_wr = next;
    }

    /* 继续接收下一字节 */
    if (HAL_UART_Receive_IT(huart, &s_rx_byte, 1) != HAL_OK) {
      /* 重新发起一次，防止链断 */
      (void)HAL_UART_Receive_IT(huart, &s_rx_byte, 1);
    }
  }
}

void uart_rx_start(void)
{
  /* 使能 USART6 中断：接收用 NVIC 优先级 5（低于屏幕/触摸，避免动画被打断） */
  HAL_NVIC_SetPriority(USART6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART6_IRQn);

  s_rd = 0;
  s_wr = 0;

  /* 清掉上电/初始化期间 A 板持续发送留下的 ORE/FE/NE/PE，
     否则第一个中断就是错误，HAL 会直接结束接收。 */
  __HAL_UART_CLEAR_OREFLAG(&huart6);
  __HAL_UART_CLEAR_FEFLAG(&huart6);
  __HAL_UART_CLEAR_NEFLAG(&huart6);
  __HAL_UART_CLEAR_PEFLAG(&huart6);

  (void)HAL_UART_Receive_IT(&huart6, &s_rx_byte, 1);
}

/* UART 错误恢复：
   ORE（溢出）在 HAL 里属于 blocking error，会 UART_EndRxTransfer()：
   RxState 置 READY 并关闭 RXNEIE，之后不再收任何字节，屏上表现为
   “U 闪一下 → 500ms 后 LINK--、RX=0”。这里清标志并重新挂接收。
   FE/NE/PE 属于非阻塞错误，清标志后原接收链继续。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6) {
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    if (huart->RxState == HAL_UART_STATE_READY) {
      (void)HAL_UART_Receive_IT(huart, &s_rx_byte, 1);
    }
  }
}

void uart_rx_poll(void (*feed)(uint8_t))
{
  /* 先把两个 volatile 指针各读一次存入局部变量，
     避免在同一个表达式里同时读 s_rd / s_wr——
     IAR Pa082：volatile 访问顺序在 C 标准中未定义。 */
  uint16_t rd = s_rd;
  uint16_t wr = s_wr;

  while (rd != wr) {
    uint8_t b = s_ring[rd];
    rd = (uint16_t)((rd + 1) % RX_RING_SIZE);
    s_total++;
    if (feed) {
      feed(b);
    }
    /* 中断可能又收到新字节，重新取一次写指针 */
    wr = s_wr;
  }

  s_rd = rd;
}

uint32_t uart_rx_bytes(void)
{
  return s_total;
}

/*************************** End of file ****************************/