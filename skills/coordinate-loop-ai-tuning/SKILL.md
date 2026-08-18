---
name: coordinate-loop-ai-tuning
description: RoboCup2026 坐标闭环的安全在线 AI 调参、串口遥测、速度—时间图、参数固化与 Task1 回切流程。用户提到坐标闭环、AI 调参、UART5 遥测、速度图、参数寻优、到点过冲、调参后跑全图或恢复 Task1 时使用。
---

# 坐标闭环 AI 调参

执行前读取 `坐标闭环AI调参实施方案.md`、`Mymode/ai_tune.[ch]`、`Mymode/control.[ch]`、`Core/Src/main.c` 与 `tools/ai_tuner.py`。

## 控制链路

保持以下链路：世界系误差经 yaw 旋转为车体系误差 → 轨迹规划生成 `plan_vx/plan_vy` 与 `a_plan` → PID 生成反馈速度 → 加速度前馈与静摩擦补偿叠加 → 制动包络限速 → 麦轮逆解 → X42 速度模式。

将跨模块默认参数放在 `Mymode/control.h`：`Kp`、`Kd`、加速度反馈系数、力到速度前馈系数、静摩擦速度、制动能力。令 `Mymode/ai_tune.c` 从这些宏初始化在线调参基准参数。

保持总速度命令满足 `sqrt(2 × POSITION_COMMAND_BRAKE_DECEL_MPS2 × 距离误差)`。该制动包络覆盖规划速度、前馈速度和 PID 速度。

## 在线调参

确认测试区域为空、里程计已回零、UART5 连接到上位机。使用 UART5 的 2 Mbit/s DMA 链路，帧头为 `0xC3 0x3C`，参数命令为 `0x01`，确认命令为 `0x02`，测试启动命令为 `0x03`。

从项目根目录运行：

```powershell
python tools/ai_tuner.py --port COM20 --point-a 0 0 --point-b -600 600 --speed 1.5 --iterations 12 --csv ai_tune_1p5ms.csv
```

按实际串口修改 `--port`。保持测试速度为 1.5 m/s，直到实测制动距离完成复核。每组参数执行三次 A/B 往返，使用中位数评分。

检查固件保护：最小距离反增 0.50 m、连续 150 ms 距离增加、连续 100 ms 速度达到 6.99 m/s、运动时间达到 8 s。保护触发后恢复已验证参数并发送零速度；上位机将保护帧作为该段评分结束帧。

## 遥测与判定

保留 CSV 中的目标坐标、实际坐标、实测局部速度、速度指令、到点标志和保护标志。调参结束后运行 `tools/plot_ai_tune.py`，生成位置—时间 SVG 与 `*_telemetry_speed.svg`；速度图必须包含实测线速度和速度指令。

检查速度指令峰值、里程计速度峰值、保护标志与往返终点误差。实体终点与遥测终点出现固定 y 轴差值时，校准 y 轴编码器行程比例后再进行参数寻优。

将评分领先且完成所有保护检查的参数写入 `Mymode/control.h`。将 `POSITION_PID_DEFAULT_KP`、`POSITION_PID_DEFAULT_KD`、`POSITION_ACCEL_FEEDBACK_GAIN`、`POSITION_FORCE_TO_SPEED_FF_MPS_PER_N` 与 `POSITION_STATIC_FRICTION_SPEED_MPS` 同步更新。

## Task1 回切

将按键 1 保持为里程计回零入口。零点稳定等待完成后执行 `start_task1()`，服务周期调用 `run_task1()`。按键路径不调用 `ai_tune_arm()`。

## 验证与提交

执行：

```powershell
python -m py_compile tools/ai_tuner.py tools/plot_ai_tune.py
git diff --check
```

提交固件源文件、Keil 工程文件、调参工具、实施方案和项目技能。保留遥测 CSV、SVG、Python 缓存与 Keil 构建产物在工作区。

## 上位机 EXE 构建

`tools/ai_tune_gui.py` 的 EXE 固定使用 `.pack_venv\Scripts\python.exe` 构建。该环境对应 Python 3.13、Matplotlib 与 NumPy 二进制版本，并提供 GUI 运行所需的 `_tkinter` 扩展。

构建前设置 `TCL_LIBRARY=D:\Program Files\Python313\tcl\tcl8.6` 和 `TK_LIBRARY=D:\Program Files\Python313\tcl\tk8.6`。PyInstaller 必须包含 `tkinter`、`_tkinter`、`tkinter.ttk`、`tkinter.filedialog`、`tkinter.messagebox`，并附加 `D:\Program Files\Python313\Lib\tkinter` 到 `tkinter`、Tcl 目录到 `_tcl_data`、Tk 目录到 `_tk_data`；保留 Matplotlib 的完整收集参数。

构建完成后，使用 `pyi-archive_viewer -l tools\dist\AI调参工具.exe` 确认存在 `pyi_rth__tkinter`、`_tkinter.pyd`、`_tcl_data` 与 `_tk_data`。随后启动 EXE 5 秒并确认进程持续运行，再结束该校验进程。
