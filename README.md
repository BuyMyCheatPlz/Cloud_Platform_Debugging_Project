# 项目概述

- 本工程为基于 STM32F405（Keil/MDK）平台的云台/平台控制项目。使用 HAL 驱动、FreeRTOS（CMSIS-RTOS2）和 GM6020 电机，通过遥控器控制 motor2 与 motor4 角度，采用两级 PID（角度环 + 速度环）实现位置闭环。速度环反馈来源为 GM6020 电机编码器自带的 `speed_rpm`，不依赖 IMU。

## 整体架构

- 控制：电机用 CAN（GM6020），遥控用 SBUS 解析，串口回传（VOFA）用于遥测和在线调参。
- 实时任务：按 CMSIS-RTOS2 划分为获取遥控目标、运行 PID、发送遥测三个任务。

## 控制环说明

- 级联结构：上层角度环（外环） + 下层速度环（内环）。
  - 角度环（外环）：以电机编码器累积角度为反馈，计算速度目标。
  - 速度环（内环）：以 GM6020 反馈的 `speed_rpm`（编码器微分速度）为反馈，计算最终电压命令。
- 电机映射：motor2（CAN ID 0x206）、motor4（CAN ID 0x208）。

## 主要文件与关键函数

### PID 与电机

- [Hardwares/CAN_Motor/PID_Controller/Inc/PID_Controller.h](Hardwares/CAN_Motor/PID_Controller/Inc/PID_Controller.h)
- [Hardwares/CAN_Motor/PID_Controller/Src/PID_Controller.c](Hardwares/CAN_Motor/PID_Controller/Src/PID_Controller.c)
  - `PID_Controller_Init()` / `PID_Controller_Reset()`：初始化/复位 PID 状态。
  - `PID_Controller_SetTargetAngleDegrees(float m2_deg, float m4_deg)`：设置角度目标。
  - `PID_Controller_Update(void)`：主更新函数，外环用编码器累积角度计算速度目标，内环用 `GM6020_Feedback_t.speed_rpm` 计算并调用 `GM6020_SetVoltage()`。

### CAN 电机驱动

- [Hardwares/CAN_Motor/Src/can_motor.c](Hardwares/CAN_Motor/Src/can_motor.c)
  - `GM6020_CAN1_Init()`：CAN 过滤与启动。
  - `HAL_CAN_RxFifo0MsgPendingCallback()`：解析电机上报，更新 `GM6020_Feedback_t`（含 `encoder`, `speed_rpm`, `current`, `temperature`）。
  - `GM6020_SetVoltage()`：发送电压命令。

### 应用任务

- [APP/Src/task.c](APP/Src/task.c)
  - `StartTask02`（目标发布，周期 ~5ms）：读取遥控快照，解码开关和舵柄，计算 motor2/motor4 目标角度，发布到 `App_TargetAngleQueueHandle`。
  - `StartTask03`（控制主循环，周期 ~5ms）：取目标并调用 `PID_Controller_Update()` 执行两级 PID，发布 VOFA 遥测到 `App_VofaQueueHandle`。
  - `StartTask04`（遥测传输，周期 ~10ms）：通过 VOFA（JustFloat 格式）发送回传数据。

## PID 默认参数

### 角度环（外环）

| 参数 | Motor2 | Motor4 |
|------|--------|--------|
| Kp | 3.0 | 3.5 |
| Ki | 0.03 | 0.08 |
| Kd | 20.0 | 0.75 |
| integral_limit | 5000.0 | 5000.0 |
| output_limit | 4000.0 | 4000.0 |

### 速度环（内环）

| 参数 | Motor2 | Motor4 |
|------|--------|--------|
| Kp | 5.0 | 2.0 |
| Ki | 0.0 | 0.02 |
| Kd | 1.0 | 1.5 |
| integral_limit | 6000.0 | 6000.0 |
| output_limit | 25000.0 | 25000.0 |

### 全局参数

| 参数 | 值 | 说明 |
|------|----|------|
| MAX_MOTOR_VOLTAGE | 12500 | 电机电压限幅 |
| DERIVATIVE_ALPHA | 0.2 | 微分低通滤波系数 |
| MIN_EFFECTIVE_VOLTAGE | 300 | 输出死区（防微振） |

## VOFA 回传字段

`Usart_RxCallBack_SendVofaJustFloat()` 按 JustFloat 格式发送 4 个 float（帧尾 `0x00 0x00 0x80 0x7f`）：

| 序号 | 字段 | 说明 |
|------|------|------|
| 1 | motor2_actual_angle_deg | motor2 实际角度（度） |
| 2 | motor2_target_angle_deg | motor2 目标角度（度） |
| 3 | motor4_actual_angle_deg | motor4 实际角度（度） |
| 4 | motor4_target_angle_deg | motor4 目标角度（度） |

## VOFA 滑块调参（v1.2）

通过 VOFA 上位机滑块实时调整 PID 参数，无需重新编译。

### 接收

- UART4 DMA 空闲中断双缓冲接收，每行命令以 `!` 结尾。
- 协议格式：`参数名=值!`（与 JustFloat 兼容的同串口文本下行通道）。

### 命令列表

支持角度环和速度环的 Kp/Ki/Kd 三种参数，可指定单个电机或同时设置两个电机：

| 命令 | 说明 | 示例 |
|------|------|------|
| `KP_Angle=值!` | 同时设置两个电机角度环 Kp | `KP_Angle=3.5!` |
| `KI_Angle=值!` | 同时设置两个电机角度环 Ki | `KI_Angle=0.05!` |
| `KD_Angle=值!` | 同时设置两个电机角度环 Kd | `KD_Angle=15.0!` |
| `KP_Velocity=值!` | 同时设置两个电机速度环 Kp | `KP_Velocity=5.0!` |
| `KI_Velocity=值!` | 同时设置两个电机速度环 Ki | `KI_Velocity=0.01!` |
| `KD_Velocity=值!` | 同时设置两个电机速度环 Kd | `KD_Velocity=1.2!` |
| `M2_KP_Angle=值!` | 仅设置 motor2 角度环 Kp | `M2_KP_Angle=3.0!` |
| `M4_KP_Velocity=值!` | 仅设置 motor4 速度环 Kp | `M4_KP_Velocity=2.2!` |
| ... | 所有 Kp/Ki/Kd × Angle/Velocity × M2/M4 组合均支持 | |

### API

- `PID_Controller_SetAnglePID(motor_id, kp, ki, kd)` / `PID_Controller_SetSpeedPID(motor_id, kp, ki, kd)`：运行时更新 PID 参数。
- `PID_Controller_GetAnglePID(motor_id, &kp, &ki, &kd)` / `PID_Controller_GetSpeedPID(motor_id, &kp, &ki, &kd)`：查询当前参数。
- 参数变更时自动清零积分和微分历史，避免 wind-up。传 `-1.0` 保持原值不变。

## 调参建议

1. **基本原则**：先调速度环，再调角度环。
2. **速度环 PID**
   - 先把 Ki 设为 0，调 Kp 到电机能跟踪速度指令但不明显振荡。
   - 加入 Kd 抑制剩余抖动（当前默认 M2: Kd=1.0, M4: Kd=1.5）。
   - 最后小幅加入 Ki 消除稳态误差（当前默认 M2: Ki=0, M4: Ki=0.02）。
3. **角度环 PID**
   - 角度环影响较慢，主要微调 Kp 和 Kd 改善跟踪精度和减小超调。
4. **现场注意事项**
   - 确认系统编码器微分转速稳定（无跳变）后再调高 Kp。
   - 如输出抖动，检查电机输出死区 `MIN_EFFECTIVE_VOLTAGE` 是否过小。