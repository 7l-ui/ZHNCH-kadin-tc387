# GUIMAI 音频采集调试记录

## 背景

目标是在 TC387 上从 ES8388 的 LINEIN 获取 16 kHz / 16 bit / mono PCM，并通过 WiFi 发给 PC 端 `server_xfyun/main.py` 调用讯飞识别。

当前硬件连接：

- BCK: `P11_6`
- LRCK/WS: `P11_2`
- ADC SDOUT: `P11_9`
- MCLK: `P11_11`
- ES8388 I2C 地址: `0x10`

ES8388 初始化和 I2C 通信正常，LINEIN 源当前使用 `SRC_B(0x0A=0x50)`。

## 已尝试方案

### 1. CPU 直接采 GPIO 解 I2S

做法：CPU 轮询 BCK/WS/SDOUT，根据 I2S 时序拼 16 bit PCM。

现象：

- 能采到非零数据。
- 采样率不稳定，曾出现约 2 kHz、10 kHz、14 kHz 等。
- 音频表现为尖锐噪声、破音、机器声。

判断：

- 纯 CPU 轮询很难稳定卡住 BCK 边沿。
- 即使用 CPU1 单独采集，也只能缓解，不能根治时序抖动和丢边沿。

### 2. CPU1 分担采集

做法：录音热循环放到 CPU1，CPU0 负责 UI/按键/发送等。

现象：

- 采样率提升到约 14 kHz 左右。
- 串口日志仍有 UI/按键输出穿插。
- 音频仍破，讯飞返回空文本。

判断：

- 多核分工能减少干扰，但 GPIO 软解 I2S 仍然不够稳。
- 问题不是简单的 CPU0 忙，而是采样边沿和位对齐无法可靠保证。

### 3. QSPI Slave 接 I2S

做法：尝试用 QSPI1 作为从机，BCK 作为时钟，SDOUT 作为输入。

现象：

- QSPI 能收到数据的版本中，采样率约 15.3 kHz 左右。
- 但数据仍表现为破音/高 ZCR，讯飞无法识别。
- 尝试 WS 同步、leading/trailing、16/32 bit、高低 16 bit 等配置，结果不稳定。

限制：

- QFN 封装无法飞线到可用的 SLSI/片选脚。
- 没有可靠的硬件帧同步，QSPI 很难稳定对齐 I2S 左右声道边界。

判断：

- 如果硬件能提供合适的片选/WS 同步脚，QSPI/SSC 类外设会比 GPIO 软解更合理。
- 当前板子引脚条件下，这条路受限。

### 4. GTM TIM + DMA 截图 GPIO

做法：用 `GTM TIM2_3` 监听 `P11_6` BCK，触发 DMA 把 `MODULE_P11.IN.U` 搬到 RAM，再由 CPU 从快照中解析 `P11_2` WS 和 `P11_9` SDOUT。

已经验证：

- DMA 确实能被 BCK 触发。
- `pulseNotify` 会导致触发异常，`gtm_words` 暴涨，PCM 全 0。
- 改为 `pulse` 后触发频率回到 BCK 量级。
- `ICH-only` 版本能采到非零数据，说明 DMA 搬运链路可行。

遇到的问题：

- 单块 DMA 完成后重新 `disable -> 改地址 -> enable`，期间会丢 BCK 请求，`gtm_lost` 较高。
- 双缓冲 8192 words 时 RAM 不够，链接器报 RAM 区域不足；改 4096 words 后能链接。
- continuous + circular ring 版本没有按预期产生半区完成信号。
- 轮询 `DADR` 的版本目前没有触发处理，表现为 `gtm_chunks=0`。
- 即使采到非零数据，音频仍有破音风险，可能还存在 BCK 边沿、WS 声道、电平采样相位、I2S 1-bit delay 等问题。

判断：

- GTM+DMA 是当前不改硬件条件下最有希望的软件救急方案。
- 但它本质是“DMA 高速截图 GPIO，再软件解 I2S”，不是硬件 I2S 接收，可靠性和相位容错较差。
- 若要继续推进，应先把 `gtm_lost` 降低，再调 I2S 相位。

## 当前主要问题

1. 采样率仍不稳定，目标是稳定接近 16 kHz。
2. `gtm_lost` 偏高时会丢 BCK，导致音频时间轴被压缩或断裂。
3. PCM 波形可能存在位错或通道错，表现为高 RMS、高 ZCR、破音。
4. 讯飞返回空文本，说明当前音频仍不是可识别语音。

## 后续建议

优先级从高到低：

1. 继续优化 GTM+DMA 分块，减少或消除 `gtm_lost`。
2. 在稳定采样率后，依次测试：
   - BCK leading / trailing
   - WS=0 / WS=1
   - I2S 延迟 0 bit / 1 bit
   - 取前 16 bit / 后 16 bit
3. 给 DMA buffer 加少量诊断，统计 WS 半周期长度、SDOUT 的 1 比例、前几个样本值。
4. 如果仍无法得到正常语音，优先考虑硬件方案：让 TC387 的可用串行外设直接接收 ES8388 ADC SDOUT，并提供可靠 WS/片选同步。

## 结论

当前方案不是完全不可行，但属于救急路线。  
最稳妥的最终方案仍然是硬件外设直接接收音频流；若受 QFN 和引脚限制无法改线，GTM+DMA 可以继续尝试，但需要接受调试成本和可靠性风险。
