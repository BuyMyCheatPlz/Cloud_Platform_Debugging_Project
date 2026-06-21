# Cloud Platform Debugging Project

基于 STM32F405 的**双轴云台调试项目**，使用两个 GM6020 电机分别控制 Pitch（俯仰）和 Yaw（偏航）轴，两级串级 PID + 前馈 + 死区 + 摩擦力补偿。

**最新状态：双轴调试完成。M2 偏航轴前馈+D滤波+摩擦偏置，实测跟随稳定；M4 俯仰轴重力补偿正常。**

## 硬件连接

| 硬件 | 接口 | 备注 |
|------|------|------|
| STM32F405RGT6 | - | 主控 |
| GM6020 × 2 | CAN1 | Motor 2 (Yaw 偏航), Motor 4 (Pitch 俯仰) |
| DJI DR16 接收机 | USART3 (PD9) | DBUS 遥控器 |
| 串口绘图 VOFA+ | USART1 | 调试数据输出（921600 波特率） |

## 项目结构

```
Cloud_Platform_Debugging_Project/
├── Core/                   # CubeMX 生成的核心代码
│   ├── Inc/                # HAL 配置头文件
│   └── Src/                # main.c, 中断服务函数等
├── Drivers/                # HAL 库 & CMSIS
├── Hardwares/              # 外设驱动层
│   ├── CAN_Motor/          # GM6020 电机驱动 + PID 控制器
│   │   ├── Inc/            # can_motor.h, PID_Controller.h (PID + 前馈参数)
│   │   └── Src/            # can_motor.c, PID_Controller.c (所有控制算法)
│   ├── Remote_Control/     # DJI DR16 DBUS 接收机驱动
│   └── Usart_RxCallBack/   # 串口空闲中断 + 回调框架
├── APP/                    # 应用层 (FreeRTOS 任务)
│   ├── Inc/
│   └── Src/                # task.c (目标发布/控制/遥测三任务)
├── MDK-ARM/                # Keil MDK 工程文件
├── Middlewares/            # 第三方中间件 (FreeRTOS)
└── README.md
```

## PID 控制架构

### 控制回路

每个电机采用**串级 PID + 前馈 + 死区 + 摩擦力补偿**：

```
目标角度 (deg)
    │
    ▼
┌──────────────┐
│  角度环 PID   │ ◄── 位置反馈 (encoder counts 多圈累积)
└──────┬───────┘
       │ 输出 = 速度目标 (rpm)
       ▼
┌──────────────┐
│  速度环 PID   │ ◄── 速度反馈 (rpm, GM6020 编码器微分)
└──────┬───────┘
       │ 输出 = 电压 (mV)
       ▼
┌──────────────┐
│ GM6020 电机   │
└──────────────┘
```

### 增强特性

#### 1. 积分泄放 & 防饱和 (Integral Decay & Reset)
- 误差过零时积分缩至 **30%**（`INTEGRAL_RESET_GAIN = 0.3`），防止过冲
- 每周期自然衰减 0.5%（`INTEGRAL_DECAY = 0.005`），控制积分 windup

#### 2. 前馈控制 (Feedforward)

**M2（偏航轴）：**
- 误差比例前馈：`FF_GAIN × error × soft_decay` 直接叠加到最终电压
- `FF_SOFT_START = 200 counts` 内线性衰减，接近目标时平滑交接给 PID
- 方向性摩擦偏置：正方向运动时注入 8000mV 克服静摩擦，带 2°↔0.05° 迟滞

**M4（俯仰轴）：**
- 重力前馈：|sin(angle − offset)| × 1400，上行时分级补偿（>10°: 20%, >3°: 50%, ≤3°: 100%）
- 下行跳过补偿（重力辅助），同时叠加 5000mV 偏置加速下降

#### 3. 死区 (Deadband)
- 输出 < `50mV` 时清零，滤除微小抖动
- 输出限幅在 `MAX_MOTOR_VOLTAGE = 25000mV` 内

#### 4. D 项指数滤波
- 微分项一阶低通滤波（`α = 0.18`），抑制高频噪声，防止 KD 放大编码器抖动
- 最近改进：M2_ANGLE_KD 从 205 降至 140，配合 α 加强，消除 -30° 附近极限环振荡

#### 5. M4 下行自适应 KI
- 上行（角度误差 > 0）：标准 KI = 0.02
- 下行（角度误差 < −3°）：切换到 DESCEND 模式（KI = 0.0 + 5000mV 偏置）
- 迟滞：3° 切入，1° 切出，防止抖动

#### 6. 编码器多圈累积 & 初始化保护
- 处理 GM6020 0~8191 溢出的多圈计数
- 首次数据到来前不计算增量

## 关键参数速查

### 角度环（外环）

| 参数 | M2 (Yaw) | M4 (Pitch) |
|------|----------|------------|
| KP | 45.71 | 58.99 |
| KI | 0.029 | 0.02 / 0.0 (下行) |
| KD | 140.0 | 29.84 |
| 积分限幅 | 800 | 800 / 1600 (下行) |
| 输出限幅 | 6000 | 4000 |

### 速度环（内环）

| 参数 | M2 (Yaw) | M4 (Pitch) |
|------|----------|------------|
| KP | 1.12 | 3.83 |
| KI | 0.0 | 0.0 |
| KD | 0.5 | 0.09 |
| 积分限幅 | 6000 | 6000 |
| 输出限幅 | 25000 | 25000 |

### 前馈 & 全局参数

| 参数 | 值 | 说明 |
|------|-----|------|
| M2_FF_GAIN | 18.0 | 偏航前馈增益 |
| M2_FF_SOFT_START | 200 counts | 软启动区间 (~8.8°) |
| M2_FF_OUTPUT_LIMIT | 25000 | 前馈输出限幅 |
| M2_REVERSE_BIAS_VOLTAGE | 8000 mV | 正向摩擦偏置 |
| GRAVITY_GAIN_M4 | 1400 | 俯仰重力补偿 |
| GRAVITY_OFFSET_DEG | −58.0° | 重力零位角 |
| M4_DESCEND_BIAS_VOLTAGE | 5000 mV | 下行偏置 |
| MAX_MOTOR_VOLTAGE | 25000 | 最终电压限幅 |
| MIN_EFFECTIVE_VOLTAGE | 50 | 死区阈值 |
| DERIVATIVE_ALPHA | 0.18 | D 项低通滤波系数 |
| INTEGRAL_RESET_GAIN | 0.3 | 过零积分重置系数 |
| INTEGRAL_DECAY | 0.005 | 每周期积分衰减 |

## 调试数据输出 (VOFA+)

VOFA+ 通过 USART1 以 FireWater 协议输出调试帧，包含 10 个 float 通道：

| 通道 | 名称 | 含义 |
|:----:|------|------|
| 0 | M2_Current_Angle | M2 当前角度 (°) |
| 1 | M2_Target_Angle | M2 目标角度 (°) |
| 2 | M4_Current_Angle | M4 当前角度 (°) |
| 3 | M4_Target_Angle | M4 目标角度 (°) |
| 4 | M2_Speed | M2 当前转速 (rpm) |
| 5 | M2_Target_Speed | M2 目标转速 (rpm, 角度环输出) |
| 6 | M4_Speed | M4 当前转速 (rpm) |
| 7 | M4_Target_Speed | M4 目标转速 (rpm, 角度环输出) |
| 8 | M2_Output | M2 最终输出电压 (mV) |
| 9 | M4_Output | M4 最终输出电压 (mV) |

## VOFA 滑块调参

通过 VOFA 上位机滑块实时调整 PID 参数，无需重新编译。

- **协议**：`参数名=值!`（! 结尾），UART4 DMA 空闲中断双缓冲接收
- **支持命令**：`KP_Angle`, `KI_Angle`, `KD_Angle`, `KP_Velocity`, `KI_Velocity`, `KD_Velocity`，可前缀 `M2_`/`M4_` 指定电机
- 参数变更时自动清零积分和微分历史，避免 wind-up；传 `−1.0` 保持原值不变

## 编译与烧录

1. 用 Keil MDK-ARM 打开 `MDK-ARM/Cloud_Platform_Debugging_Project.uvprojx`
2. 编译 (F7)
3. 通过 ST-Link 烧录 (F8)
4. 打开 VOFA+，配置 USART1 串口查看调试波形（波特率 921600）

## 最近改进日志

| 日期 | 改进项 | 详情 |
|------|--------|------|
| 2026-06-21 | M2 震荡修复 | KD 205→140, DERIVATIVE_ALPHA 0.25→0.18, SPEED_KD 0.6→0.5，消除 −30° 附近高频极限环振荡 |
| - | 死区收窄 | MIN_EFFECTIVE_VOLTAGE 500→50 |
| - | 积分优化 | INTEGRAL_RESET_GAIN 关闭→0.3, INTEGRAL_LIMIT 5000→800, 添加 0.005 积分衰减 |
| - | 前馈增强 | M2_FF_GAIN 8.0→18.0, FF_SOFT_START 300→200, 添加方向性摩擦偏置 |
| - | 重力补偿 | M4 重力前馈 + 下行偏置 + 自适应 KI |

## 许可证

MIT License