# 第21届全国大学生智能汽车竞赛 · 卡丁快跑组 · 全国一等奖代码

> 竞赛：第二十一届全国大学生智能汽车竞赛（2026）
> 组别：卡丁快跑组
> 奖项：全国一等奖
> 主控：Infineon AURIX TC387（TriCore 四核 @ 300MHz）

本项目为卡丁快跑组的完整参赛代码，基于英飞凌 TC387 主控与逐飞（SeekFree）开源库开发，实现了油门踏板有人驾驶、GNSS+INS 组合导航自动驾驶、语音交互等多个竞赛任务的完整控制方案。

**QQ 交流：3170851279** —— 欢迎交流技术问题

**关键词**：卡丁快跑 / 卡丁车 / 全国大学生智能汽车竞赛 第21届 / 智能车 国一代码 / Infineon AURIX TC387 / 逐飞库 / 组合导航 / 硅麦语音识别

## 硬件平台

| 模块 | 型号 / 说明 |
|------|------------|
| 主控 | 英飞凌 TC387（AURIX™ TriCore™ 四核 300MHz） |
| IMU | 逐飞 IMU963RA（陀螺仪 + 加速度计，航向融合） |
| GNSS | 逐飞 GNSS 模块（卫星定位） |
| 地磁计 | DICI 地磁传感器（航向修正） |
| 转向编码器 | SPI 绝对值编码器（4096 线，转向角闭环） |
| 轮速编码器 | 增量编码器 ×2（左右轮速） |
| 语音识别 | 硅麦（MEMS 麦克风）ADC 采集 |
| 无线通信 | LoRa ATK-MW1278D |
| 人机交互 | TLD7002 点阵屏、LED 灯带、多页菜单 UI |
| 数据记录 | SD 卡 + FATFS（数据日志 / 磁力计标定） |
| 遥控 | SBUS 遥控接收机（UART 100000 波特率） |
| 油门踏板 | ADC 采集（踏板行程 → 目标速度） |

## 软件架构

```
code/
├── anjian/      按键 + XL9555 IO 扩展
├── cibian/      磁编码器（MT6701 + SPI 绝对值编码器）
├── dianji/      电机驱动配置
├── es8388/      ES8388 音频 Codec（备用链路）
├── fatfs/       FATFS 文件系统
├── guimai/      硅麦语音识别链路（ADC 采集）
├── IMU/         地磁传感器驱动（dici）
├── led_strip/   LED 灯带驱动
├── lora/        LoRa ATK-MW1278D 无线模块
├── nav/         组合导航：GNSS + 编码器/IMU 航位推算融合，路径跟踪
├── pid/         PID 控制器
├── sd/          SD 卡驱动
├── taban/       油门踏板采集
├── task_ctrl/   竞赛任务状态机
├── TLD7002/     TLD7002 LED 驱动
├── ui/          屏幕菜单 UI（信息/参数/GPS 多页面）
└── yaokong/     SBUS 遥控接收

user/            主函数与中断（TC387 四核分工）
docs/            导航算法使用说明
libraries/       逐飞库 + 英飞凌 AURIX 官方库
```

## 核心算法

- **组合导航**（`code/nav/`）：GNSS 绝对位置 + 编码器/IMU 航位推算（Dead Reckoning）融合，含速度滤波、位置校正与路径跟踪控制，详细用法见 `docs/nav_ins_usage.md`
- **转向闭环**：绝对值编码器测量转向角，PID 闭环 + 打角限幅保护
- **速度控制**：踏板/遥控/任务状态机多源目标速度切换，带故障检测
- **语音识别**：硅麦经 ADC 采集音频，软件滤波后送识别链路
- **任务状态机**（`code/task_ctrl/`）：竞赛各任务的启停调度、方向盘自动扫零等流程控制

## 编译

1. 安装 [AURIX Development Studio](https://www.infineon.com/cms/en/product/promopages/aurix-development-studio/)（内置 TASKING 编译器）
2. File → Import → Existing Projects，选择本工程目录
3. 直接 Build 即可生成可烧录固件

烧录使用 ADS 自带的 Flash 工具或 DAP MiniWiggler。

## 致谢

- [逐飞科技](https://gitee.com/seekfree) 开源库与硬件模块
- 全国大学生智能汽车竞赛组委会

## 开源协议

本项目采用 [MIT License](LICENSE) 开源，仅供学习交流使用。

## 说明

本 README 由 AI 辅助总结生成。
