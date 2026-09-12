# TC387 GTM 模拟 I2S 采集状态说明

日期：2026-05-08

## 当前目标

目标是在 TC387 上接收 ES8388 的 LINEIN ADC 数据，得到可用于讯飞识别的 `16 kHz / 16 bit / mono PCM`。

当前硬件连接：

- `BCK`: `P11_6`
- `WS/LRCK`: `P11_2`
- `ADC SDOUT`: `P11_9`
- `MCLK`: `P11_11`
- ES8388 I2C 地址：`0x10`

ES8388 I2C 通信和 LINEIN 初始化基本正常；目前主要问题在 TC387 侧如何稳定接收 I2S 数据。

## 当前判断

TC387 没有专用 I2S 外设。官方建议的 I2S 从机接收路线是：

`GTM TIM -> ARU -> PSM FIFO -> DMA -> RAM`

这条路线方向是对的，但我们当前已经尝试的方案还不是完整官方路线。

当前代码中主要尝试的是：

`BCK 触发 GTM TIM -> DMA 快照 MODULE_P11.IN.U -> CPU 软件解析 WS/SDOUT`

这个方案能证明 DMA 可以被 BCK 触发，但它只是“高速 GPIO 快照”，不是 TIM 硬件完成 I2S 位移/压缩。因此即使采样数看起来接近，PCM 仍可能因为边沿、相位、丢请求、声道同步错误而破音。

## 已尝试方案和结论

### CPU 轮询 GPIO

结果：

- 能采到非零数据。
- 采样率不稳定。
- 音频有尖锐噪声、破音、机器声。

结论：

CPU 轮询不能稳定跟住 BCK 边沿，不适合作为最终方案。

### CPU1 单独采集

结果：

- 比 CPU0 轮询略好。
- 采样率曾提升到约 14 kHz 左右。
- 音频仍破，讯飞返回空文本。

结论：

多核只能减少干扰，不能解决 I2S 边沿采样和位对齐问题。

### QSPI 从机接收

结果：

- BCK 接 QSPI 时钟、SDOUT 接输入后，能收到数据。
- 尝试过 16/32 bit、leading/trailing、不同高低位截取、WS 同步。
- 因缺少可靠的 WS/片选同步，数据边界不稳定。
- QFN 封装限制了飞线，难以补充合适 SLSI/片选输入。

结论：

当前硬件条件下，QSPI 路线受限，不适合继续作为主线。

### GTM TIM + DMA 快照 GPIO

结果：

- TIM 可以由 `P11_6/BCK` 触发 DMA。
- DMA 能把 `MODULE_P11.IN.U` 搬到 RAM。
- `pulse` 模式比 `pulseNotify` 更接近预期。
- 仍出现过 `gtm_lost`、采样率偏差、PCM 全 0、PCM 破音等问题。

结论：

这条路线证明了 DMA 触发链路可用，但还不是官方建议的 `TIM/ARU/FIFO/DMA` 接收结构。它可以作为诊断和临时方案，但可靠性风险较高。

## 已掌握的重要资料

### 1. TC38x ARU Write Address

用户手册 `26.4.1 ARU Write Address Overview` 已确认：

- `TIM0_WRADDR[0..7]`: `0x001..0x008`
- `TIM1_WRADDR[0..7]`: `0x009..0x010`
- `TIM2_WRADDR[0..7]`: `0x011..0x018`
- `TIM3_WRADDR[0..7]`: `0x019..0x020`
- `F2A0_WRADDR[0..7]`: `0x051..0x058`
- `F2A1_WRADDR[0..7]`: `0x059..0x060`

因此当前引脚若使用 `TIM2`：

- `P11_2 / WS / TIM2_CH1` 的 ARU write address 是 `0x012`
- `P11_6 / BCK / TIM2_CH3` 的 ARU write address 是 `0x014`
- `P11_9 / SDOUT / TIM2_CH4` 的 ARU write address 是 `0x015`

若使用 `TIM3`：

- `P11_2 / WS / TIM3_CH1` 的 ARU write address 是 `0x01A`
- `P11_6 / BCK / TIM3_CH3` 的 ARU write address 是 `0x01C`
- `P11_9 / SDOUT / TIM3_CH4` 的 ARU write address 是 `0x01D`

### 2. ARU Port Partitioning

用户手册 `26.4.2 ARU Port Partitioning` 已确认：

- `PSM0` 连接 `ARU-0 port`
- `PSM1` 连接 `ARU-1 port`

因此优先考虑 `PSM0`，因为 `TIM2/TIM3` 输出可由 `PSM0` 通过 ARU0 读取。

### 3. ARU Read ID

用户手册 `26.4.3 ARU Read ID` 已确认：

- `PSM0 channel 0` 的 ARU read ID 是 `3`
- `PSM0 channel 1` 的 ARU read ID 是 `5`
- `PSM0 channel 2` 的 ARU read ID 是 `7`
- `PSM0 channel 3` 的 ARU read ID 是 `9`
- 依此类推，PSM0 偶数间隔出现在 ARU0 侧

注意：

- `ARU write address` 是数据源写入 ARU 的地址。
- `ARU read ID` 是数据目的模块在 ARU 端口上的读槽位。
- 这两个概念不能混用。

### 4. 本地 iLLD 可用接口

本地 iLLD 中已经有 PSM/TIM 基础接口，不需要修改库函数：

- `IfxGtm_Psm_F2a_setAruReadAddress`
- `IfxGtm_Psm_F2a_setTransferMode`
- `IfxGtm_Psm_F2a_setTransferDirection`
- `IfxGtm_Psm_F2a_enableStream`
- `IfxGtm_Psm_Fifo_setChannelStartAddress`
- `IfxGtm_Psm_Fifo_setChannelEndAddress`
- `IfxGtm_Psm_Fifo_setChannelSize`
- `IfxGtm_Psm_Fifo_setChannelUpperWatermark`
- `IfxGtm_Psm_Fifo_getChannelFillLevel`
- `IfxGtm_Psm_Fifo_getChannelWritePtr`
- `IfxGtm_Psm_Fifo_getChannelReadPtr`
- `IfxGtm_Psm_Fifo_getChannelSrcPointer`
- `IfxGtm_Psm_Afd_getChannelPointer`
- TIM 支持 `enableAruRouting`、`GPR0/GPR1` 选择、`inputEvent`、`bitCompression`

### 5. TIM Bit Compression Mode (TBCM)

用户手册 `28.13.4.2.5 TIM Bit Compression Mode (TBCM)` 已确认：

- TBCM 可以把一个 TIM sub-module 内所有 filtered input signals 合成为一个并行 `m bit` 数据字。
- 这个并行数据字可以路由到 ARU。
- 在 TBCM 下，`CNTS` 用来选择哪个输入边沿触发采样。
- `CNTS` bit `0..m-1` 用来选择各 TIM filter 的 rising edge detect 信号作为采样事件。
- `CNTS` bit `8..(7+m)` 用来选择各 TIM filter 的 falling edge detect 信号作为采样事件。
- 多个事件可以 OR 组合。
- 采样事件发生时，输入 `F_IN(0)..F_IN(m-1)` 会按通道顺序采样成并行数据并放入 `GPR1`。
- `GPR0` 保存 timestamp。
- `ARU_EN=1` 时，`GPR1` 的采样数据会和 `GPR0` timestamp 一起通过 ARU 输出。
- TBCM 不依赖 `CLK_SEL`。
- 在 TBCM 中，`GPR1_SEL` 不适用，手册说明 `EGPR1_SEL=1, GPR1_SEL=01` 会使用 `TIM_INP_VAL`，其他情况使用 TIM filter `F_OUT`。

对当前 I2S 采集的意义：

- 可以用 `BCK` 的边沿作为采样事件。
- 在每个 BCK 边沿，硬件同时采样 `WS` 和 `SDOUT` 的电平。
- 这比 DMA 快照 `MODULE_P11.IN.U` 更接近官方推荐的 TIM 硬件采样路线。

当前引脚对应 TIM2 channel：

- `P11_2 / WS` = `TIM2_CH1`
- `P11_6 / BCK` = `TIM2_CH3`
- `P11_9 / SDOUT` = `TIM2_CH4`

因此若用 `TIM2_CH4` 配 TBCM：

- 用 BCK 上升沿触发采样，`CNTS` 大概率设置为 `1 << 3`。
- 用 BCK 下降沿触发采样，`CNTS` 大概率设置为 `1 << (8 + 3)`。
- `GPR1` 中至少需要观察：
  - bit 1：`WS / TIM2_CH1`
  - bit 3：`BCK / TIM2_CH3`
  - bit 4：`SDOUT / TIM2_CH4`

这一步可以先只做诊断：不接 PSM/FIFO/DMA，直接读 `GPR1`、`IRQ_NOTIFY.NEWVAL` 和计数，确认 BCK 触发时 `WS/SDOUT` 是否被硬件采到。

### 6. TIM Channel CTRL 关键位

用户手册 `28.13.8.1 Register TIM[i]_CH[x]_CTRL` 已确认：

- `TIM_EN` bit 0：使能 channel。使能后会把 `ECNT/CNT/GPR0/GPR1` 复位到 reset value。
- `TIM_MODE` bits `3:1`：
  - `000b`: TPWM
  - `001b`: TPIM
  - `010b`: TIEM
  - `011b`: TIPM
  - `100b`: TBCM
  - `101b`: TGPS
  - `110b`: TSSM
- `OSM` bit 4：one-shot mode。
- `ARU_EN` bit 5：`GPR0/GPR1` 是否路由到 ARU。
- `CICTRL` bit 6：
  - `0`: channel x 使用 `TIM_IN(x)`
  - `1`: channel x 使用 `TIM_IN(x-1)`，x=0 时使用 `TIM_IN(m-1)`
- `TBU0_SEL` bit 7：选择写入 `GPR0/GPR1` 的 TBU_TS0 位段。
- `GPR0_SEL` bits `9:8`：选择 `GPR0` 输入源。
- `GPR1_SEL` bits `11:10`：选择 `GPR1` 输入源；TBCM 中手册特别说明行为不同。
- `CNTS_SEL` bit 12：TBCM 中该功能 disabled。
- `DSL` bit 13：
  - TSSM 中定义 shift direction。
  - 其他模式中常用于选择 active signal level。
- `ISL` bit 14：
  - `0`: 使用 `DSL` 选择 active level。
  - `1`: 忽略 `DSL`，TIEM 中把两种边沿都当作 active edge。
- `ECNT_RESET` bit 15：某些模式中控制 counter reset；TSSM 中定义 shift register 初始极性。
- `FLT_EN` bit 16：
  - `0`: filter disabled，`F_IN` 直接路由到 `F_OUT`
  - `1`: filter enabled
- `EXT_CAP_EN` bit 19：
  - `0`: external capture disabled
  - `1`: external capture enabled
- `CLK_SEL` bits `26:24`：选择 CMU clock；TBCM 不依赖它。
- `FR_ECNT_OFL` bit 27：扩展 edge counter overflow 行为。
- `EGPR0_SEL` bit 28 / `EGPR1_SEL` bit 29：扩展 `GPR0_SEL/GPR1_SEL`。
- `TOCTRL` bits `31:30`：timeout control。

TBCM 诊断建议初值：

- `TIM_MODE = 100b`
- `ARU_EN = 0` 做本地 GPR 诊断，接 PSM 时再置 `1`
- `CICTRL = 0`
- `FLT_EN = 0`
- `EXT_CAP_EN = 0`
- `OSM = 0`
- `TIM_EN = 1`

### 7. TIM Channel CNTS / IRQ 关键位

用户手册 `28.13.8.7 Register TIM[i]_CH[x]_CNTS` 已确认：

- `CNTS` bits `23:0`：counter shadow register。
- `ECNT` bits `31:24`：edge counter，只读。
- `CNTS` 内容随 TIM channel mode 有不同含义。
- `CNTS` 只在 `TIPM/TBCM/TGPS/TSSM` 中可写。
- `ECNT` 会统计输入 filtered edge，rising/falling 都计数；奇偶值可反映检测到 rising 或 falling edge，bit0 也包含输入电平信息。

用户手册 `28.13.8.8 Register TIM[i]_CH[x]_IRQ_NOTIFY` 已确认：

- `NEWVAL` bit 0：channel 检测到新的 measurement value。
- `ECNTOFL` bit 1：edge counter overflow。
- `CNTOFL` bit 2：SMU counter overflow。
- `GPROFL` bit 3：`GPR0/GPR1` data overflow，表示旧数据未读时新数据已到。
- `TODET` bit 4：timeout reached。
- `GLITCHDET` bit 5：glitch detected。
- 写 `1` 清对应 notify bit，读访问不改变。

用户手册 `28.13.8.9 Register TIM[i]_CH[x]_IRQ_EN` 已确认：

- `NEWVAL_IRQ_EN` bit 0：使 `NEWVAL` interrupt visible outside GTM。
- `GPROFL_IRQ_EN` bit 3：使 GPR overflow interrupt visible outside GTM。

TBCM 第一版诊断可以不依赖外部中断，只轮询：

- `IRQ_NOTIFY.NEWVAL`
- `IRQ_NOTIFY.GPROFL`
- `GPR1`
- `ECNT`

若 `GPROFL` 很快置位，说明 CPU 读不及时，但也说明 TBCM 正在产生新数据。

## 当前还缺的资料

要做完整官方路线，还需要这些手册页面或截图。

### 1. PSM F2A / FIFO 详细说明

TIM TBCM、`CTRL`、`CNTS` 的关键资料已经基本确认。现在缺口主要转移到 PSM/FIFO。

需要 PSM 章节中这些内容：

- `F2A ARU to FIFO` 的工作流程
- `ARU_RD_FIFO.ADDR` 应该填 TIM write address，还是其他映射值
- `STR_CFG.DIR`
- `STR_CFG.TMODE`
- `F2A stream` 与 `FIFO channel` 的对应关系
- FIFO RAM 的 start/end/size/watermark 配置要求
- AFD `BUF_ACC` 是否是 DMA 应该读取的源地址

核心问题：

- PSM0 如何从 `TIM2_CH4` 的 ARU 输出读数据。
- FIFO channel 满水位后如何稳定触发 DMA。
- DMA 源地址应该用 FIFO channel 的 AFD buffer，还是 FIFO RAM/其他寄存器。

### 2. FIFO Watermark DMA 说明

需要这些内容：

- FIFO upper watermark / lower watermark 触发条件
- FIFO channel SRC 如何配置给 DMA
- DMA 每次请求应该搬 1 word 还是一批 word
- FIFO 读操作如何推进 read pointer
- 是否需要 hysteresis mode

### 3. ES8388 I2S 格式确认

还需要确认 ES8388 当前输出格式：

- I2S 标准格式还是 left-justified
- 是否存在 1 bit delay
- slot width 是 16 bit 还是 32 bit
- ADC 是 stereo 输出还是 mono 输出
- 左声道/右声道分别对应 WS 的哪个电平
- 当前 MCLK 是否应该是 `4.096 MHz` 而不是日志里的约 `4.000 MHz`

这部分最好通过逻辑分析仪同时量：

- `MCLK P11_11`
- `BCK P11_6`
- `WS P11_2`
- `SDOUT P11_9`

## 已基本解决的资料缺口

以下内容之前属于未知项，现在已经有足够信息做第一版 TIM TBCM 诊断：

- `TIM_MODE = 100b` 是 TBCM。
- TBCM 可以用 `CNTS` 选择 BCK 边沿作为采样事件。
- TBCM 会把同一个 TIM sub-module 内的输入电平采样到 `GPR1`。
- `ARU_EN=1` 会把 `GPR0/GPR1` 路由到 ARU。
- `IRQ_NOTIFY.NEWVAL` 可以作为诊断事件。
- `IRQ_NOTIFY.GPROFL` 可以用来判断 GPR 是否被覆盖。
- 第一版可以先不接 PSM/DMA，直接读 `GPR1` 验证 `WS/SDOUT` 是否随 BCK 被硬件采样。

## 建议的下一步

### 第一步：先做 TIM TBCM 本地诊断

先新增一个独立诊断开关，例如：

`GUIMAI_GTM_TBCM_DIAG`

目标只验证：

- `BCK / TIM2_CH3` 的上升沿或下降沿是否能触发 TBCM 采样。
- `IRQ_NOTIFY.NEWVAL` 是否持续置位。
- `GPR1` bit1 是否对应 `WS`。
- `GPR1` bit4 是否对应 `SDOUT`。
- `GPROFL` 是否大量出现。

这一阶段不追求音频可听，只追求链路跑通。

### 第二步：再做 TIM -> ARU -> PSM FIFO

TBCM 本地诊断跑通后，再新增：

`GUIMAI_GTM_ARU_FIFO_DIAG`

目标验证：

- TIM 是否能把 `GPR0/GPR1` 送进 ARU。
- PSM0 是否能从 ARU 读到 TIM2_CH4 的输出。
- FIFO fill level 是否增加。
- FIFO write pointer 是否变化。
- FIFO watermark/SRC 是否触发。

### 第三步：跑通 PSM FIFO 后再接 DMA

如果 FIFO fill/write pointer 正常，再加：

`PSM FIFO/AFD -> DMA -> RAM`

先小 buffer，避免再次触发 RAM 链接不足。建议先用 512 或 1024 words 做诊断。

### 第四步：确认数据格式后再解 PCM

等确认 TIM 输出到 FIFO 的数据格式后，再决定：

- 是否需要软件按 bit 拼 16 bit PCM
- 是否已经由 TBCM/TSSM 压缩成 word
- 是否需要处理 I2S 1-bit delay
- 取 WS=0 还是 WS=1
- 取高 16 bit 还是低 16 bit

## 目前不能确定的点

目前还不能确定：

- `TIM2_CH4` 配 TBCM 时，`GPR1` 的 bit 与 `TIM2_CHx` 的实际对应是否完全按预期。
- 应该使用 BCK 上升沿还是下降沿采样 SDOUT。
- PSM F2A 配置中的 read address 应该如何精确填写才能读到 TIM2_CH4 数据。
- FIFO 到 DMA 的最佳搬运粒度。
- 当前 ES8388 的 MCLK/BCK/WS 是否已经严格满足 16 kHz 音频要求。

## 结论

现在的主要问题不是“DMA 有没有用”，而是还没有走到官方定义的完整 I2S 从机模拟链路。

已确认：

- 当前引脚具备进入 GTM TIM 的条件。
- ARU 地址映射已经找到。
- PSM0 走 ARU0，理论上可接 TIM 输出。
- 本地 iLLD 有 PSM/FIFO/F2A 接口，可以先不改库函数。

下一步最需要的是 TIM 模式和 PSM FIFO 的详细手册页。拿到这些后，可以先做 `TIM -> ARU -> PSM FIFO` 诊断版；诊断跑通后再接 DMA 和 PCM 解码。

2026-05-08 更新：

- TIM TBCM、`CTRL`、`CNTS`、`IRQ_NOTIFY`、`IRQ_EN` 的关键资料已经补齐。
- 现在可以先做 `TIM2 TBCM` 本地诊断版。
- 完整 `TIM -> ARU -> PSM FIFO -> DMA` 仍需要 PSM/F2A/FIFO 细节。
