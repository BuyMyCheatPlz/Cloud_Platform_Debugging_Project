/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    remote_control.h
  * @brief   SBUS remote control decode interface.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __REMOTE_CONTROL_H
#define __REMOTE_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usart.h"

#define REMOTE_CONTROL_SBUS_FRAME_LENGTH 25U
#define REMOTE_CONTROL_SBUS_START_BYTE 0x0FU
#define REMOTE_CONTROL_SBUS_END_BYTE 0x00U
#define REMOTE_CONTROL_SBUS_CHANNEL_MIN 172U
#define REMOTE_CONTROL_SBUS_CHANNEL_MAX 1811U

typedef struct
{
  uint16_t channel_raw[16];
  float motor4_target_angle_deg;
  float motor2_target_angle_deg;
  uint8_t frame_valid;
  uint8_t frame_lost;
  uint8_t failsafe;
  uint32_t last_update_tick;
} Remote_Control_Data_t;

void Remote_Control_Init(void);
void Remote_Control_Reset(void);
uint8_t Remote_Control_IsFrameValid(void);
uint8_t Remote_Control_IsOnline(void);
float Remote_Control_GetMotor4TargetAngleDeg(void);
float Remote_Control_GetMotor2TargetAngleDeg(void);
void Remote_Control_GetTargetAngles(float *motor4_target_angle_deg, float *motor2_target_angle_deg);
const Remote_Control_Data_t *Remote_Control_GetData(void);

#ifdef __cplusplus
}
#endif

#endif
