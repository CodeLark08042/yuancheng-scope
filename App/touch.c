/*********************************************************************
* 文件：touch.c
* 说明：FT5336 电容触摸驱动实现（I2C3，7 位地址 0x70）
*
*   FT5336 / FT6206 寄存器：
*     0x02  TD_STATUS   低 4 位 = 当前触摸点数
*     0x03  P1_XH       bit3:0 = X 高 4 位
*     0x04  P1_XL       X 低 8 位
*     0x05  P1_YH       bit3:0 = Y 高 4 位
*     0x06  P1_YL       Y 低 8 位
*     0xA4  G_MODE      0x00 = 轮询模式，0x01 = 中断触发模式
*     0xA8  CHIP_ID     FT5336 应为 0x51
*
*   注意事项：
*     1) 本板面板为 480x272 横屏，触摸控制器原生坐标系旋转 90°，
*        必须交换 X/Y（见 ST BSP tsOrientation = TS_SWAP_XY）。
*     2) I2C3 时序必须用 ST BSP 的手工值 0x40912732，CubeMX 自动
*        生成值会导致读不到芯片。
*     3) 初始化时尝试 FT5336(0x70) 与 FT6206(0x38) 两种地址，
*        并做一次总线恢复，避免上次异常导致 SDA 锁死。
*********************************************************************/
#include "main.h"
#include "touch.h"
#include "i2c.h"

#define FT_REG_TD         0x02
#define FT_REG_P1XH       0x03
#define FT_REG_GMODE      0xA4
#define FT_REG_ID         0xA8

#define FT5336_ADDR8      (0x70 << 1)
#define FT6206_ADDR8      (0x38 << 1)

static uint8_t s_ready = 0;      /* 触摸芯片是否可用 */
static uint8_t s_addr8 = 0;      /* 实际探测到的 8 位地址 */
static uint8_t s_err   = 0;      /* 诊断码 */

/* I2C 总线恢复：若从机把 SDA 拉死，手动发 9 个 SCL 脉冲 + STOP */
static void i2c_bus_recover(void)
{
  GPIO_InitTypeDef gpio = {0};
  int i;

  gpio.Pin   = GPIO_PIN_7 | GPIO_PIN_8;      /* SCL=PH7, SDA=PH8 */
  gpio.Mode  = GPIO_MODE_OUTPUT_OD;
  gpio.Pull  = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOH, &gpio);

  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_8, GPIO_PIN_SET);   /* 释放 SDA */

  for (i = 0; i < 9; i++) {
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_Delay(1);
  }

  /* 生成 STOP：SCL 高时 SDA 由低变高 */
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_Delay(1);

  /* 恢复 AF4 复用 */
  gpio.Mode      = GPIO_MODE_AF_OD;
  gpio.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(GPIOH, &gpio);
}

/* 把 HAL 错误码翻译成 1..4 的诊断码 */
static uint8_t map_err(uint32_t e)
{
  if (e & HAL_I2C_ERROR_BERR)     { return 1; }
  if (e & HAL_I2C_ERROR_AF)       { return 2; }
  if (e & HAL_I2C_ERROR_TIMEOUT)  { return 3; }
  if (e != HAL_I2C_ERROR_NONE)    { return 4; }
  return 0;
}

static uint8_t ft_read_addr(uint8_t addr8, uint16_t reg, uint8_t *buf, uint16_t len)
{
  return (HAL_I2C_Mem_Read(&hi2c3, addr8, reg,
                           I2C_MEMADD_SIZE_8BIT, buf, len, 30) == HAL_OK) ? 1 : 0;
}

static uint8_t ft_write_addr(uint8_t addr8, uint16_t reg, uint8_t val)
{
  return (HAL_I2C_Mem_Write(&hi2c3, addr8, reg,
                            I2C_MEMADD_SIZE_8BIT, &val, 1, 30) == HAL_OK) ? 1 : 0;
}

static uint8_t probe(uint8_t addr8)
{
  uint8_t id = 0;

  if (!ft_read_addr(addr8, FT_REG_ID, &id, 1)) {
    s_err = map_err(HAL_I2C_GetError(&hi2c3));
    return 0;
  }

  /* 读到全 0 或全 FF 视为无效（常见于总线未连通时读回固定电平） */
  if (id == 0x00 || id == 0xFF) {
    s_err = 4;
    return 0;
  }

  s_addr8 = addr8;
  return 1;
}

uint8_t touch_init(void)
{
  static const uint8_t cand[2] = { FT5336_ADDR8, FT6206_ADDR8 };
  uint8_t i;

  s_ready = 0;
  s_err   = 0;

  i2c_bus_recover();

  for (i = 0; i < 2; i++) {
    if (probe(cand[i])) {
      /* 切到轮询模式，避免依赖 INT 引脚 */
      (void)ft_write_addr(cand[i], FT_REG_GMODE, 0x00);
      s_ready = 1;
      return 1;
    }

    /* 失败后复位 I2C 外设，清掉可能卡住的状态机，再试下一个地址 */
    (void)HAL_I2C_DeInit(&hi2c3);
    MX_I2C3_Init();
  }

  return 0;
}

uint8_t touch_read(uint16_t *x, uint16_t *y)
{
  uint8_t st = 0;
  uint8_t d[4];
  uint16_t raw_x, raw_y;

  if (!s_ready) {
    return 0;
  }

  if (!ft_read_addr(s_addr8, FT_REG_TD, &st, 1)) {
    return 0;
  }
  if ((st & 0x0F) == 0) {       /* 无触摸点 */
    return 0;
  }

  if (!ft_read_addr(s_addr8, FT_REG_P1XH, d, 4)) {
    return 0;
  }

  raw_x = (uint16_t)((((uint16_t)d[0] & 0x0F) << 8) | d[1]);
  raw_y = (uint16_t)((((uint16_t)d[2] & 0x0F) << 8) | d[3]);

  /* 面板为横屏，控制器原生坐标旋转 90°，必须交换 X/Y */
  *x = raw_y;
  *y = raw_x;

  if (*x >= 480 || *y >= 272) {
    return 0;
  }

  return 1;
}

uint8_t touch_error(void)
{
  return s_err;
}

/*************************** End of file ****************************/