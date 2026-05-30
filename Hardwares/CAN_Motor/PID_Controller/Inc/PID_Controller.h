/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    PID_Controller.h
  * @brief   Layered PID controller for GM6020 angle and speed loops.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __PID_CONTROLLER_H
#define __PID_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "can_motor.h"

#define PID_CONTROLLER_ENCODER_COUNTS_PER_REV 8192.0f
#define PID_CONTROLLER_ENCODER_HALF_REV 4096U

#define PID_CONTROLLER_DEFAULT_ANGLE_KP 4.5f
#define PID_CONTROLLER_DEFAULT_ANGLE_KI 0.0f
#define PID_CONTROLLER_DEFAULT_ANGLE_KD 1.2f
#define PID_CONTROLLER_DEFAULT_ANGLE_INTEGRAL_LIMIT 5000.0f
#define PID_CONTROLLER_DEFAULT_ANGLE_OUTPUT_LIMIT 4000.0f

#define PID_CONTROLLER_DEFAULT_SPEED_KP 5.0f
#define PID_CONTROLLER_DEFAULT_SPEED_KI 0.02f
#define PID_CONTROLLER_DEFAULT_SPEED_KD 1.0f
#define PID_CONTROLLER_DEFAULT_SPEED_INTEGRAL_LIMIT 6000.0f
#define PID_CONTROLLER_DEFAULT_SPEED_OUTPUT_LIMIT 25000.0f

/* Maximum motor output voltage magnitude (applied to final command) */
#define PID_CONTROLLER_MAX_MOTOR_VOLTAGE 2000

/* Derivative filter alpha (0..1). Smaller -> stronger smoothing. */
#define PID_CONTROLLER_DERIVATIVE_ALPHA 0.2f

/* Minimum output voltage magnitude below which output is treated as zero (deadband)
  Helps to avoid small jittering commands that do not move the motor but cause oscillation. */
#define PID_CONTROLLER_MIN_EFFECTIVE_VOLTAGE 300

typedef struct
{
  float kp;
  float ki;
  float kd;
  float derivative; /* filtered derivative */
  float integral;
  float previous_error;
  float integral_limit;
  float output_limit;
} PID_ControllerPID_t;

typedef struct
{
  PID_ControllerPID_t angle_pid;
  PID_ControllerPID_t speed_pid;
  int32_t target_angle_count;
  int32_t total_angle_count;
  uint16_t last_encoder;
  uint8_t initialized;
} PID_ControllerMotor_t;

void PID_Controller_Init(void);
void PID_Controller_Reset(void);
void PID_Controller_SetTargetAngleCounts(int32_t motor2_target_count, int32_t motor4_target_count);
void PID_Controller_SetTargetAngleDegrees(float motor2_target_degree, float motor4_target_degree);
float PID_Controller_GetMotorCurrentAngleDegrees(uint8_t motor_id);
float PID_Controller_GetMotorTargetAngleDegrees(uint8_t motor_id);
HAL_StatusTypeDef PID_Controller_Update(void);

#ifdef __cplusplus
}
#endif

#endif
