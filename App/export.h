/*********************************************************************
* 文件：export.h
* 说明：波形数据导出 —— CSV 文本 与 BMP 图片，经 USART1(ST-Link VCP)
*       发送到 PC，由串口助手保存为文件。
*
*   任务书对应条款：提高部分(4)
*     "可以将当前屏幕显示的模拟波形和数字波形存储为 CSV 格式数据文件
*      和 BMP 格式图片文件，可以在PC机上通过USB或以太网接口获取该文件"
*
*   CSV 通过串口直接发送（文本，约 20KB，0.2 秒发完）
*   BMP 逐行生成发送（24 位色，480x272，约 383KB，4 秒发完）
*********************************************************************/
#ifndef EXPORT_H
#define EXPORT_H

#include <stdint.h>

/* 导出一屏数据为 CSV 文本。
   ch1/ch2 : 500 点 raw12 值（线性排列，时间从旧到新）
   dig     : 500 点数字通道字节
   n       : 点数
   gear    : 时基档位 0..4
   返回实际发送的字节数 */
uint32_t export_csv(const uint16_t *ch1, const uint16_t *ch2,
                    const uint8_t *dig, uint16_t n, uint8_t gear);

/* 读取 LTDC 当前正在扫描的帧缓冲地址（三缓冲中正在显示的那一块）。
   注意：必须在 GUI_MULTIBUF_End 改变可视缓冲之前调用。 */
uint32_t export_lcd_fb(void);

/* 把指定帧缓冲的画面导出为 24 位 BMP。
   fb_addr : 由 export_lcd_fb() 取得的缓冲地址
   返回实际发送的字节数 */
uint32_t export_bmp(uint32_t fb_addr);

#endif /* EXPORT_H */