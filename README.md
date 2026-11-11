# M1 测试治具固件（STM32F103C6T6 + TMC2209）

线材挤出机「入料传感器 + 磁编码器」测试治具的两颗主控固件。

| 目录 | 内容 |
|---|---|
| `U1/` | 主控1 固件（Keil 工程 `U1/MDK-ARM/U1.uvprojx`）：PC 协议 + 8 路测试点 ADC + 对射判定 |
| `U2/` | 主控2 固件（Keil 工程 `U2/MDK-ARM/U2.uvprojx`）：同上 + TMC2209（STEP/DIR/EN + 软件 UART）电机驱动 |
| `docs/` | 硬件引脚、PC 协议、固件结构与验证、自测/串口助手测试手册 |
| `tools/gen_frames.py` | 协议帧生成/CRC 自检，并打印所有样例帧 |
| `test/host/` | 主机端回归测试（gcc 编译真实协议/对射代码，不依赖硬件） |
| `IINFO/` | 原始资料：原理图、协议表、芯片手册（只读） |

## 快速开始

```bat
:: 编译（Keil MDK V5）
"D:\Develop\Keil_MDK\UV4\UV4.exe" -b U1\MDK-ARM\U1.uvprojx -j0 -o U1\MDK-ARM\build.log
"D:\Develop\Keil_MDK\UV4\UV4.exe" -b U2\MDK-ARM\U2.uvprojx -j0 -o U2\MDK-ARM\build.log

:: 主机端回归测试（需要 gcc）
powershell -ExecutionPolicy Bypass -File test\host\run_tests.ps1

:: 生成/校验样例帧
python tools\gen_frames.py
```

产物：`U1/MDK-ARM/U1/U1.hex`、`U2/MDK-ARM/U2/U2.hex`。

## 一句话现状

- 两颗固件 **0 Error / 0 Warning**（U1 Code≈10.1KB，U2 Code≈12.7KB，RAM 各≈3.4KB/10KB）。
- 协议层 4094 项主机端断言全过（含 CRC 错不回 ACK、粘包、超长帧、BUSY 分支）。
- **尚未上板**：串口 115200 实际链路、ADC 标定、对射阈值、TMC2209 单线 UART 都需要实板验证，
  步骤见 `docs/02_固件结构与验证.md` 第 5 节。
