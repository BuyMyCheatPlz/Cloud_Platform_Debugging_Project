/* USER CODE BEGIN Header */
/**
	******************************************************************************
	* @file    app_task.h
	* @brief   APP-layer business logic interfaces.
	******************************************************************************
	*/
/* USER CODE END Header */

#ifndef __APP_TASK_H
#define __APP_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os2.h"

typedef struct
{
	float motor2_target_angle_deg;
	float motor4_target_angle_deg;
	uint8_t emergency_stop;
} App_TargetAngleMessage_t;

typedef struct
{
	float motor2_target_angle_deg;
	float motor2_actual_angle_deg;
	float motor4_target_angle_deg;
	float motor4_actual_angle_deg;
} App_VofaMessage_t;

extern osMessageQueueId_t App_TargetAngleQueueHandle;
extern osMessageQueueId_t App_VofaQueueHandle;

#ifdef __cplusplus
}
#endif

#endif
