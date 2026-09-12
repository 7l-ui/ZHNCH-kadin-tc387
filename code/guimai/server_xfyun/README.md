# Guimai 讯飞中转服务（Python）

这个服务用于你当前 `TC387 guimai` 工程：
- 板端通过 TCP 发音频帧（`A1 B2 C3 D4 + flag + 640B`）到电脑；
- 电脑转发到讯飞 IAT WebSocket；
- 可选把识别后的命令码回传给板端。

## 1. 先改板端配置

编辑：
- `TC387_kd1  guimai/code/guimai/guimai_board.h`

重点宏：
- `GUIMAI_WIFI_SSID`
- `GUIMAI_WIFI_PASSWORD`
- `GUIMAI_WIFI_TARGET_IP`：改成运行本 Python 服务的电脑 IP
- `GUIMAI_WIFI_TARGET_PORT`：默认 `8080`

说明：
- 你的 WiFi 必须是 2.4G。
- `GUIMAI_WIFI_TARGET_IP` 需要和板子在同一网段。

## 2. 安装依赖

在本目录执行：

```powershell
python -m pip install -r requirements.txt
```

## 3. 启动服务

```powershell
python main.py --appid <你的APPID> --apikey <你的APIKey> --apisecret <你的APISecret> --send-back-cmd
```

参数说明：
- `--send-back-cmd` 可选。打开后会把识别文本按内置关键词映射成命令码发回板端。
- 如果你只想看识别文本，不发回命令，可以去掉 `--send-back-cmd`。

## 4. 板端运行与触发

- 上电后串口看到 `wifi tcp connected` 说明已经连到服务。
- 拨动你配置的开关脚（默认 `P33_12`）触发开始/结束采样发送。

