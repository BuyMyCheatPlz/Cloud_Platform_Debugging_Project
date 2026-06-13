**项目概述**
- 本工程为基于 STM32F4（Keil/MDK）平台的云台/平台控制项目。使用 HAL 驱动、FreeRTOS（CMSIS-RTOS2）和 GM6020 电机，通过 MPU6050 的 DMP 获得姿态与角速度，并用两级 PID（角度环+速度环）控制 motor2 与 motor4。
- **角速度参考系**：`MPU6050_DMP_Get_AngularVelocity()` 输出的 pitch/yaw 角速度已通过 DMP 四元数投影从机体坐标系转换到世界坐标系，底座倾斜或 pitch/roll 运动时不会产生交叉耦合误差。

**整体架构（概览）**
- 传感器：MPU6050（DMP）通过 I2C1 与 MCU 通信。
- 通信：电机用 CAN（GM6020），遥控和串口回传（VOFA）用于交互与遥测。
- 实时任务：按 CMSIS-RTOS2 划分为获取目标、运行 PID、发送遥测等任务。

**控制环说明**
- 级联结构：上层角度环（外环） + 下层速度环（内环）。
  - 角度环（外环）：以编码器读数为反馈，计算速度目标。
  - 速度环（内环）：以 IMU 的角速度（DMP 提供，单位 deg/s）为反馈，计算最终电压命令。
- 轴映射：`motor2` 使用 Yaw（偏航）角速度，`motor4` 使用 Pitch（俯仰）角速度。
- 缩放：软件中把角速度通过 `/6.0f` 转为与原来编码器 rpm 量纲类似的速度反馈（可调）。
- 滤波：角速度在读取层进行一阶低通滤波（`MPU6050_DMP_Get_AngularVelocity()`，α=0.20），对 GX/GY/GZ 三轴均做机体坐标系 LPF，**滤波在四元数投影之前**，初始通过首帧 seed。
- 坐标补偿：滤波后机体角速度通过 DMP 四元数投影到世界坐标系（`ω_world = q ⊗ ω_body ⊗ q⁻¹`），消除倾斜时的交叉耦合。若无四元数数据则回退到机体坐标值。

**主要文件与关键函数**
- IMU / DMP
  - [Hardwares/IMU/Src/MPU6050.c](Hardwares/IMU/Src/MPU6050.c)
    - `MPU6050_DMP_init()`：初始化 MPU/DMP、设置方向、开启 DMP、清零滤波器状态（含 `filtered_gx_rate_dps`）。
    - `MPU6050_DMP_Get_Date(float *pitch, float *roll, float *yaw)`：读取并计算四元数到姿态角（机体坐标系输出，未修改）。
    - `MPU6050_DMP_Get_AngularVelocity(float *pitch_rate_dps, float *yaw_rate_dps)`：一次 `dmp_read_fifo` 同时获取原始 `gyro[3]` 和 `quat[4]`，在机体坐标系下用一阶 LPF（α=0.20）对 GX/GY/GZ 三轴滤波，再用 DMP 四元数将滤波后机体角速度投影到世界坐标系（`ω_world = q ⊗ ω_body ⊗ q⁻¹`），输出世界坐标系的 pitch/yaw 角速度。
- InvenSense 驱动
  - [Hardwares/IMU/Src/inv_mpu.c](Hardwares/IMU/Src/inv_mpu.c)
  - [Hardwares/IMU/Src/inv_mpu_dmp_motion_driver.c](Hardwares/IMU/Src/inv_mpu_dmp_motion_driver.c)
- PID 与电机
  - [Hardwares/CAN_Motor/PID_Controller/Inc/PID_Controller.h](Hardwares/CAN_Motor/PID_Controller/Inc/PID_Controller.h)
  - [Hardwares/CAN_Motor/PID_Controller/Src/PID_Controller.c](Hardwares/CAN_Motor/PID_Controller/Src/PID_Controller.c)
    - `PID_Controller_Init()` / `PID_Controller_Reset()`：初始化/复位 PID 状态。
    - `PID_Controller_SetTargetAngleDegrees(float m2_deg, float m4_deg)`：设置角度目标。
    - `PID_Controller_Update(float m2_speed_fb_rpm, float m4_speed_fb_rpm)`：主更新函数，外环产生速度目标，内环用传入的速度反馈计算并调用 `GM6020_SetVoltage()`。
- CAN 电机驱动
  - [Hardwares/CAN_Motor/Src/can_motor.c](Hardwares/CAN_Motor/Src/can_motor.c)
    - `GM6020_CAN1_Init()`：CAN 过滤与启动。
    - `HAL_CAN_RxFifo0MsgPendingCallback()`：解析电机上报，更新 `GM6020_Feedback_t`（含 `encoder`, `speed_rpm`, `current`, `temperature`）。
    - `GM6020_SetVoltage()`：发送电压命令。
- 应用任务
  - [APP/Src/task.c](APP/Src/task.c)
    - `StartTask02`：读取遥控并发布角度目标到队列（`App_TargetAngleQueueHandle`）。
    - `StartTask03`：调用 `App_Task03_RunPidAndPublishTelemetry()`，从队列取目标、读取 IMU 角速度、计算速度反馈并调用 `PID_Controller_Update()`，生成 VOFA 数据并推送 `App_VofaQueueHandle`。
    - `StartTask04`：把 VOFA 队列数据通过串口发送出去（`Usart_RxCallBack_SendVofaJustFloat()`）。

**每个任务做了什么（简要）**
- `StartTask02`（目标发布，周期 ~5ms）
  - 读取遥控快照，解码开关和舵柄，计算 motor2/motor4 的目标角度，发布到 `App_TargetAngleQueueHandle`。
- `StartTask03`（控制主循环，周期 ~5ms）
  - 从 `App_TargetAngleQueueHandle` 取最新目标并更新角度目标。
  - 调用 `MPU6050_DMP_Get_AngularVelocity()` 获取 `pitch_rate_dps` 和 `yaw_rate_dps`。
  - 缩放角速度为速度反馈（当前实现 `/6.0f`），并传入 `PID_Controller_Update()`。
  - 若 IMU 读取失败或发生 emergency stop，则复位 PID 并切断电压命令。
  - 发布 VOFA 遥测数据到 `App_VofaQueueHandle`。
- `StartTask04`（遥测传输）
  - 从 `App_VofaQueueHandle` 取 VOFA 数据并通过串口函数发送到上位机。

**VOFA（回传）包括字段**
- `motor2_target_angle_deg`：motor2 角度目标（度）。
- `motor2_actual_angle_deg`：motor2 实际角度（由编码器换算，度）。
- `motor4_target_angle_deg`：motor4 角度目标（度）。
- `motor4_actual_angle_deg`：motor4 实际角度（度）。
- 发送函数：`Usart_RxCallBack_SendVofaJustFloat()`（见 [Hardwares/Usart_RxCallBack/Src/usart_rxcallback.c](Hardwares/Usart_RxCallBack/Src/usart_rxcallback.c)）。

**VOFA 滑块调参（v1.2 新增）**
- 通过 VOFA 上位机滑块实时调整 PID 参数，无需重新编译固件。
- 接收：UART4 使用 DMA 空闲中断（idle-line）接收上位机下发的调参命令（见 `Usart_RxCallBack_StartDmaReception()` / `Usart_RxCallBack_ProcessRxData()`）。
- 协议：VOFA JustFloat 下行格式，帧尾 `0x00 0x00 0x80 0x7f`，数据域含电机 ID、PID 选择、Kp/Ki/Kd 浮点值。解析入口 `usart_rxcallback.c` 内 `Usart_RxCallBack_ParseVofaCommand()`。
- 接口（`PID_Controller.h`）：
  - `PID_Controller_SetAnglePID(motor_id, kp, ki, kd)` / `PID_Controller_SetSpeedPID(motor_id, kp, ki, kd)`：运行时更新角度环/速度环 PID 参数。
  - `PID_Controller_GetAnglePID(motor_id, &kp, &ki, &kd)` / `PID_Controller_GetSpeedPID(motor_id, &kp, &ki, &kd)`：查询当前 PID 参数（可回传给 VOFA 做初始显示）。
- 行为：参数变更时自动清零积分和微分历史，避免 wind-up。
- 初始化：`main.c` 中 `Usart_RxCallBack_StartDmaReception()` 启动 UART4 DMA 接收，`stm32f4xx_it.c` 中 UART4 中断触发 DMA 空闲回调完成处理。

**调参建议（现场可执行步骤）**
1. 基本原则：按顺序调——先稳定传感器滤波与缩放，再调速度环，最后微调角度环。
2. IMU 滤波（在 `MPU6050_DMP_Get_AngularVelocity()`）
  - 当前 `MPU6050_ANGULAR_VELOCITY_LPF_ALPHA = 0.20`。
  - 若抖动严重，降到 `0.08~0.15`；若响应太慢，升到 `0.25~0.35`。
  - 亦可在初始化时丢弃前 N 帧再 seed 滤波（建议 N=5~20）。
3. 角速度到速度量纲缩放（在 `APP/Src/task.c`）
  - 当前 `/6.0f` 为经验值。若速度环反应过猛，改为 `/8` 或 `/10`；若太弱，改为 `/4` 或 `/5`。
4. 速度环 PID（在 `PID_Controller.h`）
  - 步骤：先把 `Ki` 设为 0 或很小；调 `Kp` 到能跟踪但不振荡；再调 `Kd` 抑制剩余抖动；最后小幅加入 `Ki` 去除稳态误差。
  - 建议初始方向：`motor2` 的 `Kp` 约 5（当前 5.0），`motor4` 的 `Kp` 约 2（当前 2.0），可按现场试验微调。
5. 角度环：先不动；若出现稳态误差或超调，再微调 `Kp/Kd`（角度环影响较慢）。
6. 现场注意事项：确认 IMU 安装方向与源码里 `gyro_orientation` 一致；若方向相反，先反转符号再调 PID。

**上电与初始化建议**
- 上电或重启时让平台静止 1~2 秒，等待 `MPU6050_DMP_init()` 自检及 DMP 稳定。
- 启动控制前丢弃前几帧 IMU 输出（建议 5~20 帧）以避免首帧冲击影响滤波 seed。

**常见问题与排查**
- PID 输出振荡：先降低速度环 `Kp`，或增强 IMU LPF（减小 `MPU6050_ANGULAR_VELOCITY_LPF_ALPHA`）；增加速度环 `Kd` 有助抑制高频振荡。
- 速度方向错误：检查 `gyro_orientation` 矩阵是否与 IMU 物理安装方向一致；四元数投影对世界 Z 轴符号无影响，如反向请调 `gyro_orientation`。
- 倾斜时 yaw 角速度不准（已修复）：v1.1 起 `MPU6050_DMP_Get_AngularVelocity()` 内置四元数投影补偿，倾斜时 yaw/pitch 角速度自动转换到世界坐标系。若仍有交叉耦合，检查 DMP 四元数是否正常（`sensors & INV_WXYZ_QUAT` 为真），或确认 LPF alpha 不过大导致投影滞后。
- 底座不水平时 PID 速度环异常：确认固件已包含四元数投影补偿（`MPU6050.c` 中 `Get_AngularVelocity` 含有 `ω_world = q ⊗ ω_body ⊗ q⁻¹` 计算），若缺少则回退到机体坐标角速度，倾斜时交叉耦合不可避免。