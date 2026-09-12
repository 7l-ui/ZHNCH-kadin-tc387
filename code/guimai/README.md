# GUIMAI 语音链路移植说明

本目录包含 TC387 + ES8388 语音识别链路。当前已验证的采集基线是：

- ES8388 codec master，16 kHz / 16 bit / mono
- BCK: `P11_6 / TIM2_3`
- WS/LRCK: `P11_2`
- ADC SDOUT: `P11_9`
- MCLK: `P11_11`
- GTM TIM2_3 在 BCK 下降沿触发 DMA
- DMA 快照 `MODULE_P11.IN.U`，软件解析 WS/SDOUT
- I2S 参数：`edge=trailing`，`left_ws=1`，`delay=0`，`byteswap=1`

## 文件分工

- `guimai_voice_config.h`: 移植配置入口，优先改这里。
- `guimai_board.h`: 对外接口声明。
- `guimai_board.c`: ES8388 初始化、GTM-DMA 采集、PCM 打包、WiFi 传输、命令回传解析。
- `server_xfyun/main.py`: PC 端 TCP 服务器和讯飞 IAT 转发。

## 旧工程接入方式

如果继续使用本模块内置的 KEY3/UI 触发，保持默认配置：

```c
#define GUIMAI_VOICE_EXTERNAL_TRIGGER  (0)
```

入口保持不变：

```c
guimai_board_init();

while (1) {
    guimai_board_task();
}
```

如果启用了 CPU1 录音紧循环，还需要在 CPU1 周期任务或主循环中调用：

```c
guimai_board_record_core_task();
```

## 已有控制工程接入方式

移植到已有控制代码时，建议关闭内置按键/UI 触发，由控制工程主动开始和停止录音：

```c
#define GUIMAI_VOICE_EXTERNAL_TRIGGER  (1)
```

控制工程调用：

```c
guimai_board_init();

while (1) {
    guimai_board_task();              // WiFi 接收、发送收尾、命令解析仍需要跑
    guimai_board_record_core_task();  // 使用 CPU1 采集时必须跑在对应核心

    if (voice_key_pressed) {
        guimai_voice_start_record();
    }

    if (voice_key_released) {
        guimai_voice_stop_record();
    }

    uint8 cmd;
    while (guimai_voice_get_command(&cmd)) {
        control_handle_voice_command(cmd);
    }
}
```

`guimai_voice_get_command()` 会返回服务器识别后的命令字节。原 LoRa 透传逻辑仍保留；如果已有控制工程不需要本模块初始化 LoRa，可配置：

```c
#define GUIMAI_VOICE_INIT_LORA         (0)
```

## 移植时优先检查

1. DMA 通道 `GUIMAI_GTM_DMA_CHANNEL` 是否与原控制工程冲突。
2. `TIM2_3 / P11_6` 是否已被其他功能占用。
3. `P11_2/P11_6/P11_9` 是否仍在同一个端口快照 `MODULE_P11.IN.U` 内。
4. `GUIMAI_RECORD_STORE_SAMPLES` 的 RAM 占用是否能接受，默认 80000 samples 约 160 KB。
5. WiFi 目标 IP、SSID、密码是否已改为目标环境配置。

若 RAM 紧张，先把录音缓存降到 3 秒左右：

```c
#define GUIMAI_RECORD_STORE_SAMPLES    (48000U)
```

## 不建议先改的基线参数

这些参数是当前已验证成功识别的基线，移植初期不要改：

```c
#define GUIMAI_GTM_DMA_SAMPLE_LEADING_EDGE (0)
#define GUIMAI_GTM_DMA_LEFT_WS_LEVEL       (1U)
#define GUIMAI_GTM_DMA_I2S_DELAY_BITS      (0U)
#define GUIMAI_RECORD_PCM_TRANSFORM_MODE   (1U)
```

确认新工程能稳定得到 `gtm_lost=0` 并完成讯飞识别后，再考虑清理诊断代码或继续拆分模块。
