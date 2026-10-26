/*********************************************************************
* 文件：log_uart.c
* 说明：USART1（经 ST-Link VCP）实现，见 log_uart.h。
*
*   为什么单独用 USART1 而不是复用 USART6：
*     USART6 是 A 板数据链路（921600，PC6/PC7），方向是接收；
*     USART1 是 ST-Link 虚拟串口，用于向 PC 导出文件。
*     两者职责分离，互不干扰。
*********************************************************************/
#include "main.h"
#include "log_uart.h"

static UART_HandleTypeDef s_huart1;
static uint8_t            s_ready = 0;

void log_uart_init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* PA9 = USART1_TX */
  gpio.Pin       = GPIO_PIN_9;
  gpio.Mode      = GPIO_MODE_AF_PP;
  gpio.Pull      = GPIO_PULLUP;
  gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &gpio);

  /* PB7 = USART1_RX（本工程只发送，但一并配置便于将来调试） */
  gpio.Pin       = GPIO_PIN_7;
  gpio.Mode      = GPIO_MODE_AF_PP;
  gpio.Pull      = GPIO_PULLUP;
  gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOB, &gpio);

  s_huart1.Instance                    = USART1;
  s_huart1.Init.BaudRate               = 921600;
  s_huart1.Init.WordLength             = UART_WORDLENGTH_8B;
  s_huart1.Init.StopBits               = UART_STOPBITS_1;
  s_huart1.Init.Parity                 = UART_PARITY_NONE;
  s_huart1.Init.Mode                   = UART_MODE_TX_RX;
  s_huart1.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
  s_huart1.Init.OverSampling           = UART_OVERSAMPLING_16;
  s_huart1.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
  s_huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  /* 注意：HAL_UART_MspInit 是 usart.c 中的统一实现，
     内部只处理 USART6，对 USART1 不做任何事，因此不会冲突。 */
  if (HAL_UART_Init(&s_huart1) == HAL_OK) {
    s_ready = 1;
  }
}

void log_uart_write(const uint8_t *buf, uint32_t len)
{
  /* 分块发送，避免单次超时过长 */
  const uint32_t CHUNK = 4096;
  uint32_t off = 0;

  if (!s_ready || buf == 0 || len == 0) {
    return;
  }

  while (off < len) {
    uint32_t n = (len - off > CHUNK) ? CHUNK : (len - off);
    (void)HAL_UART_Transmit(&s_huart1, (uint8_t *)(buf + off), (uint16_t)n, 1000);
    off += n;
  }
}

void log_uart_str(const char *s)
{
  uint32_t n = 0;

  if (!s_ready || s == 0) {
    return;
  }
  while (s[n] != 0) { n++; }
  if (n > 0) {
    log_uart_write((const uint8_t *)s, n);
  }
}

/*************************** End of file ****************************/