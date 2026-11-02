# 远程数据采集系统 —— 显示端（B 板）工程说明

STM32F746G-DISCOVERY 上的示波器式显示端工程。默认用板内假数据源自测，
取消一个宏即可切换为 USART6 接收采集端（A 板）的真实数据。

- MCU：STM32F746NGH6（32F746GDISCOVERY / MB1191，480×272 电容触摸屏）
- 工具链：IAR EWARM（8.32.1 验证通过）
- 生成工具：STM32CubeMX 6.18.1 + STM32Cube FW_F7 V1.17.4
- GUI：STemWin 5.28（`Middlewares/STemWin/Lib/STemWin_CM7_wc32.a`）
- 版本：`df56944`（2026-11，显示 / 交互 / 测量 / CSV-BMP 导出均已双板实测）

---

## 1. 快速开始

1. 用 IAR 打开 `EWARM/Project.eww`
2. `Project → Rebuild All`
3. `Ctrl+D` 下载，然后**拔插一次 USB 或按板载复位键**（只按 Ctrl+D 不会运行）
4. 屏幕应显示：顶栏按钮 + 栅格 + 蓝/红双路模拟波 + 绿/黄/紫三路数字波 + 底部测量值

顶栏右侧 `S` 表示当前是假数据自测，`U` 表示 USART6 实时数据。

---

## 2. 换一块板子必查：SDRAM 时钟使能引脚

同型号板子存在**不同硬件 revision**，`FMC_SDCKE0` 的物理引脚不同：

- 本工程按 **PC3** 配置（见 `Core/Src/fmc.c` 的 `USER CODE BEGIN FMC_MspInit 1`）
- **如果屏幕全黑**，先怀疑这一点：把该处的 `GPIOC / GPIO_PIN_3` 改为 `GPIOH / GPIO_PIN_2` 再试
- 其余显示参数已调好，**不要改**：像素时钟 9.6MHz（PLLSAI 192/5/4）、
  DE 低有效、ARGB8888 三缓冲（帧缓冲 `0xC0000000`）、SDRAM 挂 MPU 非缓存、
  I2C3 时序 `0x40912732`

---

## 3. 目录结构（App 层）

| 文件 | 作用 |
|---|---|
| `App/BASIC_HelloWorld.c` | 主界面：布局、按钮、键盘、导出菜单、主循环 |
| `App/proto.c/.h` | 协议解析（AA 55 状态机 + CRC-16/CCITT-FALSE）、组帧 |
| `App/fakesrc.c/.h` | 假数据源：32 位相位累加器生成正弦/三角/方波 |
| `App/touch.c/.h` | FT5336 触摸驱动（I2C3 轮询，含总线恢复） |
| `App/net_udp.c/.h` | 以太网 UDP 收发（A→B 5005，B→A 5006） |
| `App/uart_rx.c/.h` | USART6 中断接收 + 8192 字节环形缓冲 |
| `App/log_uart.c/.h` | USART1（ST-Link 虚拟串口）导出 CSV/BMP |
| `App/export.c/.h` | CSV 行拼装、BMP 24 位逐行生成 |

`Core/`、`Drivers/` 由 CubeMX 生成（含手写补丁，均在 `USER CODE` 区）。

---

## 4. 数据通路

```
fakesrc_tick() ─┐
                ├─ proto_build_data() → proto_feed() → 协议解析
USART6 接收 ────┘                                      ↓
                                     500 点环形缓冲（ch1/ch2/dig）
                                                   ↓
                                  波形绘制 + 自动测量 + CSV/BMP 导出
```

- **以太网 UDP 模式**（当前默认）：`#define RX_SOURCE_UDP`，A=`192.168.1.10`、B=`192.168.1.20`，
  A→B 5005、B→A 5006；1ms 及更快的档位必须走这条路
- **USART6 模式**：改为 `#define RX_SOURCE_UART`（921600 8N1，PC6/PC7，TX/RX 交叉、共地），
  适用于 10ms / 5ms 档
- **假数据模式**：两个宏都注释掉，直接用板内 `fakesrc` 自测界面与测量

---

## 5. 协议要点（V1.1）

- **数据帧 260 字节**：`AA 55 01 | SEQ_L SEQ_H | GEAR | CH_MASK | N | 负载(50×5) | CRC_L CRC_H`
- **每点 5 字节**：`CH1_L CH1_H CH2_L CH2_H DIG`，模拟值编码为 `raw12 << 4`，接收端 `raw12 = wire16 >> 4`
- **DIG 位**：bit0=D1、bit1=D2、bit2=D3（协议 V1.1 §3.1；`DIG_D1=0x01`）
  注意与 `CH_MASK` 区分：`CH_MASK` 里 D1 在 bit2（`MASK_D1=0x04`），两个字段位序不同
- **控制帧 9 字节**（B→A）；**状态帧 12 字节**
- **CRC-16/CCITT-FALSE**：poly `0x1021`、init `0xFFFF`，测试向量 `"123456789" → 0x29B1`
- 多字节字段**小端序**

控制帧命令：档位 `0x01`、通道掩码 `0x02`、触发源 `0x03`、触发方式 `0x04`、
触发电平 `0x05`、握手 `0x06`、矩形波频率 `0x07`（七条 B 端均已下发，A 端已实现并双板验证）。

---

## 6. 时基与测量

- 5 档：10ms / 5ms / 1ms / 500µs / 100µs（每格 50 点、满屏 10 格 = 500 点）
- 单点间隔：200 / 100 / 20 / 10 / 2 µs；整屏窗口 = 500 × dt（即 100ms ~ 1ms）
- 峰峰值：`(max − min) × 3300 / 4095` mV
- 频率：取**相邻上升沿间隔的平均值**再换算（不是数个数），窗口不足时显示 `---`
- 脉宽：高电平点数 × dt

---

## 7. 交互

- 顶栏：档位循环、CH1、CH2、D1、D2、D3、F−、F+、`W:xxxxHz`（点开数字键盘，100~10000Hz）、SAVE
- 触摸：FT5336 @ I2C3，7 位地址 `0x70`，驱动内已做 X/Y 交换
- 板载按键 B1（PI11）：按一次切一个档位（触摸之外的备用手段）

---

## 8. 导出 CSV / BMP

- 通道：**USART1（PA9/PB7）经板载 ST-Link 虚拟串口**，921600 8N1
- 步骤：板上点 `SAVE` → 选 `CSV` 或 `BMP` → PC 端用 Tera Term `File → Log`，
  **必须勾选 Binary**，保存为 `.csv` / `.bmp`
- CSV：4 行注释表头 + 500 行数据（idx, time_us, CH1_raw, CH1_mV, CH2_raw, CH2_mV, D1~D3）
- BMP：固定 391,734 字节（54 字节头 + 480×3×272），导出的是当前实际显示画面，不含 "Exporting" 提示
- 数据先存在于 SDRAM 波形缓冲，导出即"通过 USB 获取文件"，无需 TF 卡
- 已实测：CSV 与 BMP 均能从板上导出并在 PC 端正常打开

---

## 9. 在采集端（A 板）上怎么用

- **只想先看界面**：什么都不用改，直接编译下载，跑假数据
- **想显示自己采的数据**：把 `fakesrc_tick()` 换成自己的 ADC 采样，
  继续调用 `proto_build_data()` + `proto_feed()` 即可，UI/测量/导出都不用动
- 注意：模拟量保持 `raw12`（0~4095），点数与 `dt` 必须和当前档位一致（50 点/帧）
- **不要改** LTDC / FMC / SDRAM / I2C3 配置，那些是这块板子踩坑调通的

---

## 10. 已知坑（高频）

1. IAR 里 `Ctrl+D` 只下载不运行，必须 `F5` 或物理复位
2. **CubeMX 重新生成会重写 `EWARM/yuancheng.ewp`**：App 组、`..\App` 与 STemWin 的
   include 路径、STemWin 静态库都会丢。生成前备份该文件，生成后拷回；
   若新增了外设源文件，再手动把那 1 个文件挂进 App 组
3. `.ioc` 里 `SDCKE0` 仍是 PH2（代码已补 PC3），这是唯一"打开 CubeMX 会看到不一致"的点
4. `.ioc` 里 LTDC 层格式是 RGB565，运行时由 `LCDConf.c` 重配为 ARGB8888（答辩按 ARGB8888 说）
5. `GUI_Sprintf` 在精简库里没有定义，用 `GUI_DispDecAt` / `GUI_DispStringAt`
6. IAR 警告 Pa082：对 volatile 变量在同一表达式里多次访问，先各自读入局部变量
7. 串口助手多半只能存文本，BMP 必须用二进制日志（Tera Term 勾 Binary）

---

## 11. HTML 原型

`scope_preview.html` 双击用浏览器打开，是与实机逐像素一致的界面原型，
支持切档位、开关通道、设置频率，点 SAVE 可直接下载 CSV/BMP 预览。

---

## 12. 给 IDE 中的 AI 助手（重要上下文）

- 关键约束：LTDC 9.6MHz / DE Low / ARGB8888 / 帧缓冲 `0xC0000000`；
  SDRAM SDCLK ÷3、CAS=2、模式寄存器 `0x0220`、刷新率 `0x0451`；
  SDCKE0 实物为 PC3；I2C3 时序 `0x40912732`；USART6 921600
- 手写补丁位置：`fmc.c` 的 `USER CODE BEGIN FMC_Init 2` / `FMC_MspInit 1`，
  `ltdc.c` 的 NVIC，`uart_rx.c` 的 NVIC
- 修改 App 层不受 CubeMX 影响；修改外设配置必须同步 `.ioc`，否则下次生成会被覆盖
