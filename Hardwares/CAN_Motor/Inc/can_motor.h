/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can_motor.h
  * @brief   GM6020 CAN1 driver interface.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __CAN_MOTOR_H
#define __CAN_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "can.h"

#define GM6020_FEEDBACK_BASE_ID 0x204U
#define GM6020_MOTOR_2_ID (GM6020_FEEDBACK_BASE_ID + 2U)
#define GM6020_MOTOR_4_ID (GM6020_FEEDBACK_BASE_ID + 4U)

typedef struct
{
  uint16_t encoder;
  int16_t speed_rpm;
  int16_t current;
  uint8_t temperature;
  uint32_t last_update_tick;
  uint8_t valid;
} GM6020_Feedback_t;

void GM6020_CAN1_Init(void);
const GM6020_Feedback_t *GM6020_GetFeedback(uint8_t motor_id);
HAL_StatusTypeDef GM6020_SetVoltage(int16_t motor2_voltage, int16_t motor4_voltage);

#ifdef __cplusplus
}
#endif

#endif
