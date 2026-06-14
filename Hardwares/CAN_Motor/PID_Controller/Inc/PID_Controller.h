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

#define PID_CONTROLLER_M2_ANGLE_KP 45.71f
#define PID_CONTROLLER_M2_ANGLE_KI 0.029f
#define PID_CONTROLLER_M2_ANGLE_KD 205.0f
#define PID_CONTROLLER_M2_ANGLE_INTEGRAL_LIMIT 5000.0f
#define PID_CONTROLLER_M2_ANGLE_OUTPUT_LIMIT 4000.0f

#define PID_CONTROLLER_M4_ANGLE_KP 37.42f
#define PID_CONTROLLER_M4_ANGLE_KI 0.0f
#define PID_CONTROLLER_M4_ANGLE_KD 61.59f
#define PID_CONTROLLER_M4_ANGLE_INTEGRAL_LIMIT 5000.0f
#define PID_CONTROLLER_M4_ANGLE_OUTPUT_LIMIT 4000.0f
#define PID_CONTROLLER_M4_ANGLE_KI_DESCEND 0.117f
#define PID_CONTROLLER_M4_DESCEND_BIAS_VOLTAGE 0
#define PID_CONTROLLER_KI_DESCEND_ON_THRESHOLD_DEG  3.0f
#define PID_CONTROLLER_KI_DESCEND_OFF_THRESHOLD_DEG 0.15f

#define PID_CONTROLLER_M2_SPEED_KP 1.12f
#define PID_CONTROLLER_M2_SPEED_KI 0.0f
#define PID_CONTROLLER_M2_SPEED_KD 0.0f
#define PID_CONTROLLER_M2_SPEED_INTEGRAL_LIMIT 6000.0f
#define PID_CONTROLLER_M2_SPEED_OUTPUT_LIMIT 25000.0f

/* M2 (yaw) reverse-direction friction compensation.
   Overcomes static friction deadzone that causes ~1.4° steady-state error
   when M2 needs to rotate in reverse direction. */
#define PID_CONTROLLER_M2_REVERSE_BIAS_VOLTAGE 300
#define PID_CONTROLLER_M2_REVERSE_BIAS_ON_THRESHOLD_DEG  2.0f
#define PID_CONTROLLER_M2_REVERSE_BIAS_OFF_THRESHOLD_DEG 0.05f

#define PID_CONTROLLER_M4_SPEED_KP 1.36f
#define PID_CONTROLLER_M4_SPEED_KI 0.05f
#define PID_CONTROLLER_M4_SPEED_KD 0.0f
#define PID_CONTROLLER_M4_SPEED_INTEGRAL_LIMIT 6000.0f
#define PID_CONTROLLER_M4_SPEED_OUTPUT_LIMIT 25000.0f

/* Maximum motor output voltage magnitude (applied to final command) */
#define PID_CONTROLLER_MAX_MOTOR_VOLTAGE 12500

/* Derivative filter alpha (0..1). Smaller -> stronger smoothing. */
#define PID_CONTROLLER_DERIVATIVE_ALPHA 0.2f

/* Minimum output voltage magnitude below which output is treated as zero (deadband)
  Helps to avoid small jittering commands that do not move the motor but cause oscillation. */
#define PID_CONTROLLER_MIN_EFFECTIVE_VOLTAGE 50

/* Gravity feedforward compensation for pitch axis (M4).
   Compensates gravity torque that always acts downward regardless of direction.
   Uses |sin(angle - offset)| to always provide upward torque compensation.
   K_g is the torque-to-voltage gain at the horizontal position (|sin|=1).
   OFFSET_DEG is the angle where gravity torque is zero (vertical downward). */
#define PID_CONTROLLER_GRAVITY_GAIN_M4      800.0f
#define PID_CONTROLLER_GRAVITY_OFFSET_DEG    97.9f

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

/* Gravity feedforward compensation */
void PID_Controller_SetGravityGain(float gain);
void PID_Controller_SetGravityOffset(float offset_deg);
float PID_Controller_ComputeGravityCompensation(void);

/* VOFA PID tuning: set angle/speed PID parameters per motor at runtime */
void PID_Controller_SetAnglePID(uint8_t motor_id, float kp, float ki, float kd);
void PID_Controller_SetSpeedPID(uint8_t motor_id, float kp, float ki, float kd);
void PID_Controller_GetAnglePID(uint8_t motor_id, float *kp, float *ki, float *kd);
void PID_Controller_GetSpeedPID(uint8_t motor_id, float *kp, float *ki, float *kd);

#ifdef __cplusplus
}
#endif

#endif
