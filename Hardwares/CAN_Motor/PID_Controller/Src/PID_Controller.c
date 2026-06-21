/* Includes ------------------------------------------------------------------*/
#include "PID_Controller.h"
#include <math.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static PID_ControllerMotor_t pid_motor_2 = {0};
static PID_ControllerMotor_t pid_motor_4 = {0};
static float gravity_gain_m4 = PID_CONTROLLER_GRAVITY_GAIN_M4;
static float gravity_offset_deg = PID_CONTROLLER_GRAVITY_OFFSET_DEG;
static float m4_descend_ki_active = 0.0f;
static float m2_reverse_active = 0.0f;

/* Private function prototypes -----------------------------------------------*/
static float PID_Controller_ClampFloat(float value, float min_value, float max_value);
static void PID_Controller_InitSinglePID(PID_ControllerPID_t *pid, float kp, float ki, float kd, float integral_limit, float output_limit);
static void PID_Controller_ResetSinglePID(PID_ControllerPID_t *pid);
static int16_t PID_Controller_ComputeSinglePID(PID_ControllerPID_t *pid, float target, float feedback);
static void PID_Controller_ResetMotorState(PID_ControllerMotor_t *motor);
static int32_t PID_Controller_DegreesToCounts(float degree);
static float PID_Controller_CountsToDegrees(int32_t count);
static void PID_Controller_UpdateMotorState(PID_ControllerMotor_t *motor, const GM6020_Feedback_t *feedback);

/* Private user code ---------------------------------------------------------*/
static float PID_Controller_ClampFloat(float value, float min_value, float max_value)
{
  if (value > max_value)
  {
    return max_value;
  }

  if (value < min_value)
  {
    return min_value;
  }

  return value;
}

static void PID_Controller_InitSinglePID(PID_ControllerPID_t *pid, float kp, float ki, float kd, float integral_limit, float output_limit)
{
  if (pid == NULL)
  {
    return;
  }

  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->derivative = 0.0f;
  pid->integral = 0.0f;
  pid->previous_error = 0.0f;
  pid->integral_limit = integral_limit;
  pid->output_limit = output_limit;
}

static void PID_Controller_ResetSinglePID(PID_ControllerPID_t *pid)
{
  if (pid == NULL)
  {
    return;
  }

  pid->integral = 0.0f;
  pid->previous_error = 0.0f;
  pid->derivative = 0.0f;
}

static int16_t PID_Controller_ComputeSinglePID(PID_ControllerPID_t *pid, float target, float feedback)
{
  float error;
  float derivative;
  float output;

  if (pid == NULL)
  {
    return 0;
  }

  error = target - feedback;
  /* Integral reset on sign change: discard most accumulated integral when
     error crosses zero, preventing integrator windup from causing overshoot. */
  if (pid->previous_error * error < 0.0f)
  {
    pid->integral *= PID_CONTROLLER_INTEGRAL_RESET_GAIN;
  }
  /* Integral (simple accumulation with limit) */
  pid->integral += error;
  pid->integral = PID_Controller_ClampFloat(pid->integral, -pid->integral_limit, pid->integral_limit);

  /* Derivative: filtered difference to reduce noise sensitivity */
  derivative = error - pid->previous_error;
  pid->derivative = (pid->derivative * (1.0f - PID_CONTROLLER_DERIVATIVE_ALPHA)) + (derivative * PID_CONTROLLER_DERIVATIVE_ALPHA);
  derivative = pid->derivative;
  output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
  output = PID_Controller_ClampFloat(output, -pid->output_limit, pid->output_limit);
  pid->previous_error = error;

  if (output > 25000.0f)
  {
    return 25000;
  }

  if (output < -25000.0f)
  {
    return -25000;
  }

  return (int16_t)output;
}

static void PID_Controller_ResetMotorState(PID_ControllerMotor_t *motor)
{
  if (motor == NULL)
  {
    return;
  }

  motor->target_angle_count = 0;
  motor->total_angle_count = 0;
  motor->last_encoder = 0;
  motor->initialized = 0U;
  PID_Controller_ResetSinglePID(&motor->angle_pid);
  PID_Controller_ResetSinglePID(&motor->speed_pid);
}

static int32_t PID_Controller_DegreesToCounts(float degree)
{
  return (int32_t)((degree * PID_CONTROLLER_ENCODER_COUNTS_PER_REV) / 360.0f);
}

static float PID_Controller_CountsToDegrees(int32_t count)
{
  return ((float)count * 360.0f) / PID_CONTROLLER_ENCODER_COUNTS_PER_REV;
}

static void PID_Controller_UpdateMotorState(PID_ControllerMotor_t *motor, const GM6020_Feedback_t *feedback)
{
  int32_t delta;

  if ((motor == NULL) || (feedback == NULL))
  {
    return;
  }

  if (motor->initialized == 0U)
  {
    motor->last_encoder = feedback->encoder;
    motor->total_angle_count = 0;
    motor->initialized = 1U;
    return;
  }

  delta = (int32_t)feedback->encoder - (int32_t)motor->last_encoder;
  if (delta > (int32_t)PID_CONTROLLER_ENCODER_HALF_REV)
  {
    delta -= (int32_t)PID_CONTROLLER_ENCODER_COUNTS_PER_REV;
  }
  else if (delta < -(int32_t)PID_CONTROLLER_ENCODER_HALF_REV)
  {
    delta += (int32_t)PID_CONTROLLER_ENCODER_COUNTS_PER_REV;
  }

  motor->total_angle_count += delta;
  motor->last_encoder = feedback->encoder;
}

void PID_Controller_Init(void)
{
  PID_Controller_InitSinglePID(&pid_motor_2.angle_pid, PID_CONTROLLER_M2_ANGLE_KP, PID_CONTROLLER_M2_ANGLE_KI, PID_CONTROLLER_M2_ANGLE_KD, PID_CONTROLLER_M2_ANGLE_INTEGRAL_LIMIT, PID_CONTROLLER_M2_ANGLE_OUTPUT_LIMIT);
  PID_Controller_InitSinglePID(&pid_motor_2.speed_pid, PID_CONTROLLER_M2_SPEED_KP, PID_CONTROLLER_M2_SPEED_KI, PID_CONTROLLER_M2_SPEED_KD, PID_CONTROLLER_M2_SPEED_INTEGRAL_LIMIT, PID_CONTROLLER_M2_SPEED_OUTPUT_LIMIT);
  PID_Controller_InitSinglePID(&pid_motor_4.angle_pid, PID_CONTROLLER_M4_ANGLE_KP, PID_CONTROLLER_M4_ANGLE_KI, PID_CONTROLLER_M4_ANGLE_KD, PID_CONTROLLER_M4_ANGLE_INTEGRAL_LIMIT, PID_CONTROLLER_M4_ANGLE_OUTPUT_LIMIT);
  PID_Controller_InitSinglePID(&pid_motor_4.speed_pid, PID_CONTROLLER_M4_SPEED_KP, PID_CONTROLLER_M4_SPEED_KI, PID_CONTROLLER_M4_SPEED_KD, PID_CONTROLLER_M4_SPEED_INTEGRAL_LIMIT, PID_CONTROLLER_M4_SPEED_OUTPUT_LIMIT);
  PID_Controller_ResetMotorState(&pid_motor_2);
  PID_Controller_ResetMotorState(&pid_motor_4);
}

void PID_Controller_Reset(void)
{
  PID_Controller_ResetSinglePID(&pid_motor_2.angle_pid);
  PID_Controller_ResetSinglePID(&pid_motor_2.speed_pid);
  PID_Controller_ResetSinglePID(&pid_motor_4.angle_pid);
  PID_Controller_ResetSinglePID(&pid_motor_4.speed_pid);
}

void PID_Controller_SetTargetAngleCounts(int32_t motor2_target_count, int32_t motor4_target_count)
{
  pid_motor_2.target_angle_count = motor2_target_count;
  pid_motor_4.target_angle_count = motor4_target_count;
}

void PID_Controller_SetTargetAngleDegrees(float motor2_target_degree, float motor4_target_degree)
{
  PID_Controller_SetTargetAngleCounts(PID_Controller_DegreesToCounts(motor2_target_degree), PID_Controller_DegreesToCounts(motor4_target_degree));
}

float PID_Controller_GetMotorCurrentAngleDegrees(uint8_t motor_id)
{
  if (motor_id == 2U)
  {
    return PID_Controller_CountsToDegrees(pid_motor_2.total_angle_count);
  }

  if (motor_id == 4U)
  {
    return PID_Controller_CountsToDegrees(pid_motor_4.total_angle_count);
  }

  return 0.0f;
}

float PID_Controller_GetMotorTargetAngleDegrees(uint8_t motor_id)
{
  if (motor_id == 2U)
  {
    return PID_Controller_CountsToDegrees(pid_motor_2.target_angle_count);
  }

  if (motor_id == 4U)
  {
    return PID_Controller_CountsToDegrees(pid_motor_4.target_angle_count);
  }

  return 0.0f;
}

HAL_StatusTypeDef PID_Controller_Update(void)
{
  const GM6020_Feedback_t *motor2_feedback;
  const GM6020_Feedback_t *motor4_feedback;
  int16_t motor2_speed_target;
  int16_t motor4_speed_target;
  int16_t motor2_voltage;
  int16_t motor4_voltage;

  motor2_feedback = GM6020_GetFeedback(2U);
  motor4_feedback = GM6020_GetFeedback(4U);

  if ((motor2_feedback == NULL) || (motor4_feedback == NULL) || (motor2_feedback->valid == 0U) || (motor4_feedback->valid == 0U))
  {
    return HAL_ERROR;
  }

  PID_Controller_UpdateMotorState(&pid_motor_2, motor2_feedback);
  PID_Controller_UpdateMotorState(&pid_motor_4, motor4_feedback);

  motor2_speed_target = PID_Controller_ComputeSinglePID(&pid_motor_2.angle_pid, (float)pid_motor_2.target_angle_count, (float)pid_motor_2.total_angle_count);

  /* Direction-dependent KI for M4: descending uses higher KI to eliminate steady-state error,
     ascending uses default KI to avoid overshoot.
     Hysteresis: engage KI_DESCEND at 3° error, disengage at 1° to prevent chattering.
     KI stays set continuously (no save/restore) so integral*KI always accumulates correctly. */
  {
    float m4_angle_error = (float)pid_motor_4.target_angle_count - (float)pid_motor_4.total_angle_count;
    if (m4_angle_error < -(PID_CONTROLLER_KI_DESCEND_ON_THRESHOLD_DEG * 8192.0f / 360.0f))
    {
      m4_descend_ki_active = 1.0f;
    }
    else if (m4_angle_error > -(PID_CONTROLLER_KI_DESCEND_OFF_THRESHOLD_DEG * 8192.0f / 360.0f))
    {
      m4_descend_ki_active = 0.0f;
    }
    pid_motor_4.angle_pid.ki = (m4_descend_ki_active > 0.5f)
      ? PID_CONTROLLER_M4_ANGLE_KI_DESCEND
      : PID_CONTROLLER_M4_ANGLE_KI;
    motor4_speed_target = PID_Controller_ComputeSinglePID(&pid_motor_4.angle_pid, (float)pid_motor_4.target_angle_count, (float)pid_motor_4.total_angle_count);
  }

  motor2_voltage = PID_Controller_ComputeSinglePID(&pid_motor_2.speed_pid, (float)motor2_speed_target, (float)motor2_feedback->speed_rpm);
  motor4_voltage = PID_Controller_ComputeSinglePID(&pid_motor_4.speed_pid, (float)motor4_speed_target, (float)motor4_feedback->speed_rpm);

  /* Apply minimum effective voltage deadband to avoid small jittering commands */
  if ((motor2_voltage > 0) && (motor2_voltage < PID_CONTROLLER_MIN_EFFECTIVE_VOLTAGE))
  {
    motor2_voltage = 0;
  }
  else if ((motor2_voltage < 0) && (motor2_voltage > -PID_CONTROLLER_MIN_EFFECTIVE_VOLTAGE))
  {
    motor2_voltage = 0;
  }

  if ((motor4_voltage > 0) && (motor4_voltage < PID_CONTROLLER_MIN_EFFECTIVE_VOLTAGE))
  {
    motor4_voltage = 0;
  }
  else if ((motor4_voltage < 0) && (motor4_voltage > -PID_CONTROLLER_MIN_EFFECTIVE_VOLTAGE))
  {
    motor4_voltage = 0;
  }

  /* Clamp final outputs to allowed motor voltage range */
  if (motor2_voltage > PID_CONTROLLER_MAX_MOTOR_VOLTAGE)
  {
    motor2_voltage = PID_CONTROLLER_MAX_MOTOR_VOLTAGE;
  }
  else if (motor2_voltage < -PID_CONTROLLER_MAX_MOTOR_VOLTAGE)
  {
    motor2_voltage = -PID_CONTROLLER_MAX_MOTOR_VOLTAGE;
  }

  if (motor4_voltage > PID_CONTROLLER_MAX_MOTOR_VOLTAGE)
  {
    motor4_voltage = PID_CONTROLLER_MAX_MOTOR_VOLTAGE;
  }
  else if (motor4_voltage < -PID_CONTROLLER_MAX_MOTOR_VOLTAGE)
  {
    motor4_voltage = -PID_CONTROLLER_MAX_MOTOR_VOLTAGE;
  }

   /* Gravity feedforward compensation for pitch axis (M4).
      Compensates gravity when moving UPWARD (against gravity): ramp compensation
      up as error shrinks, because PID output also shrinks near target.
      When moving DOWNWARD (gravity assists), skip compensation so it doesn't
      counteract the negative PID output and pull the motor back up. */
  {
    float target_deg_m4   = PID_Controller_GetMotorTargetAngleDegrees(4U);
    float current_deg_m4  = PID_Controller_GetMotorCurrentAngleDegrees(4U);
    float error_deg_m4    = target_deg_m4 - current_deg_m4; /* positive = upward */
    float error_abs_deg   = fabsf(error_deg_m4);
    int   moving_upward   = (error_deg_m4 > 0.0f);

    if (moving_upward)
    {
      float gravity_scale = 1.0f;
      if (error_abs_deg > 10.0f)
      {
        gravity_scale = 0.2f;
      }
      else if (error_abs_deg > 3.0f)
      {
        gravity_scale = 0.5f;
      }
      motor4_voltage += (int16_t)(PID_Controller_ComputeGravityCompensation() * gravity_scale);
    }
    /* descending: no gravity feedforward — gravity already helps the motion */

    if (m4_descend_ki_active > 0.5f)
    {
      motor4_voltage -= (int16_t)PID_CONTROLLER_M4_DESCEND_BIAS_VOLTAGE;
    }
  }

  /* Re-clamp after gravity compensation */
  if (motor4_voltage > PID_CONTROLLER_MAX_MOTOR_VOLTAGE)
  {
    motor4_voltage = PID_CONTROLLER_MAX_MOTOR_VOLTAGE;
  }
  else if (motor4_voltage < -PID_CONTROLLER_MAX_MOTOR_VOLTAGE)
  {
    motor4_voltage = -PID_CONTROLLER_MAX_MOTOR_VOLTAGE;
  }

  /* M2 (yaw) error-proportional persistent feedforward with soft decay.
     Full strength when |error| >= SOFT_START; linearly decays to zero
     inside the soft zone, handing off smoothly to PID with no hard switch. */
  {
    int32_t m2_angle_error = pid_motor_2.target_angle_count - pid_motor_2.total_angle_count;
    if (m2_angle_error != 0)
    {
      float m2_ff_base = PID_CONTROLLER_M2_FF_GAIN * (float)m2_angle_error;
      float m2_ff_decay;
      float m2_abs_err = (m2_angle_error > 0) ? (float)m2_angle_error : -(float)m2_angle_error;
      if (m2_abs_err >= PID_CONTROLLER_M2_FF_SOFT_START_COUNT)
      {
        m2_ff_decay = 1.0f;
      }
      else
      {
        m2_ff_decay = m2_abs_err / PID_CONTROLLER_M2_FF_SOFT_START_COUNT;
      }
      float m2_ff_voltage = m2_ff_base * m2_ff_decay;
      m2_ff_voltage = PID_Controller_ClampFloat(m2_ff_voltage,
                       -PID_CONTROLLER_M2_FF_OUTPUT_LIMIT,
                        PID_CONTROLLER_M2_FF_OUTPUT_LIMIT);
      motor2_voltage += (int16_t)m2_ff_voltage;
    }
  }

  /* M2 (yaw) forward-direction friction bias.
     M2 has almost zero KI (0.002) so integral can't overcome static friction
     on its own when moving forward → ~1.4° steady-state error.
     Force-add a small voltage in the forward direction near the target.
     Hysteresis: engage at 2° error, disengage at 0.05° to prevent chattering. */
  {
    float m2_target_deg  = PID_Controller_GetMotorTargetAngleDegrees(2U);
    float m2_current_deg = PID_Controller_GetMotorCurrentAngleDegrees(2U);
    float m2_error_deg   = m2_target_deg - m2_current_deg; /* positive = forward */

    if (m2_error_deg > (PID_CONTROLLER_M2_REVERSE_BIAS_ON_THRESHOLD_DEG))
    {
      m2_reverse_active = 1.0f;
    }
    else if (m2_error_deg < (PID_CONTROLLER_M2_REVERSE_BIAS_OFF_THRESHOLD_DEG))
    {
      m2_reverse_active = 0.0f;
    }

    if (m2_reverse_active > 0.5f)
    {
      motor2_voltage += (int16_t)PID_CONTROLLER_M2_REVERSE_BIAS_VOLTAGE;
    }
  }

  /* Re-clamp M2 after feedforward + reverse bias */
  if (motor2_voltage > PID_CONTROLLER_MAX_MOTOR_VOLTAGE)
  {
    motor2_voltage = PID_CONTROLLER_MAX_MOTOR_VOLTAGE;
  }
  else if (motor2_voltage < -PID_CONTROLLER_MAX_MOTOR_VOLTAGE)
  {
    motor2_voltage = -PID_CONTROLLER_MAX_MOTOR_VOLTAGE;
  }

  return GM6020_SetVoltage(motor2_voltage, motor4_voltage);
}

/* VOFA PID tuning: runtime parameter setters */

static PID_ControllerPID_t *PID_Controller_GetAnglePidPtr(uint8_t motor_id)
{
  if (motor_id == 2U)
  {
    return &pid_motor_2.angle_pid;
  }

  if (motor_id == 4U)
  {
    return &pid_motor_4.angle_pid;
  }

  return NULL;
}

static PID_ControllerPID_t *PID_Controller_GetSpeedPidPtr(uint8_t motor_id)
{
  if (motor_id == 2U)
  {
    return &pid_motor_2.speed_pid;
  }

  if (motor_id == 4U)
  {
    return &pid_motor_4.speed_pid;
  }

  return NULL;
}

void PID_Controller_SetAnglePID(uint8_t motor_id, float kp, float ki, float kd)
{
  PID_ControllerPID_t *pid = PID_Controller_GetAnglePidPtr(motor_id);
  uint8_t changed;

  if (pid == NULL)
  {
    return;
  }

  changed = 0U;
  if (kp >= 0.0f) { pid->kp = kp; changed = 1U; }
  if (ki >= 0.0f) { pid->ki = ki; changed = 1U; }
  if (kd >= 0.0f) { pid->kd = kd; changed = 1U; }

  /* Reset integral/derivative to avoid wind-up from param jump, only if any param changed */
  if (changed != 0U)
  {
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->derivative = 0.0f;
  }
}

void PID_Controller_SetSpeedPID(uint8_t motor_id, float kp, float ki, float kd)
{
  PID_ControllerPID_t *pid = PID_Controller_GetSpeedPidPtr(motor_id);
  uint8_t changed;

  if (pid == NULL)
  {
    return;
  }

  changed = 0U;
  if (kp >= 0.0f) { pid->kp = kp; changed = 1U; }
  if (ki >= 0.0f) { pid->ki = ki; changed = 1U; }
  if (kd >= 0.0f) { pid->kd = kd; changed = 1U; }

  if (changed != 0U)
  {
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->derivative = 0.0f;
  }
}

void PID_Controller_GetAnglePID(uint8_t motor_id, float *kp, float *ki, float *kd)
{
  PID_ControllerPID_t *pid = PID_Controller_GetAnglePidPtr(motor_id);

  if (pid == NULL)
  {
    return;
  }

  if (kp != NULL) { *kp = pid->kp; }
  if (ki != NULL) { *ki = pid->ki; }
  if (kd != NULL) { *kd = pid->kd; }
}

/* Gravity feedforward compensation */

void PID_Controller_SetGravityGain(float gain)
{
  if (gain >= 0.0f)
  {
    gravity_gain_m4 = gain;
  }
}

void PID_Controller_SetGravityOffset(float offset_deg)
{
  gravity_offset_deg = offset_deg;
}

float PID_Controller_ComputeGravityCompensation(void)
{
  float angle_rel_deg;
  float angle_rel_rad;

  if (gravity_gain_m4 == 0.0f)
  {
    return 0.0f;
  }

  angle_rel_deg = PID_Controller_GetMotorCurrentAngleDegrees(4U) - gravity_offset_deg;
  angle_rel_rad = angle_rel_deg * 3.1415926535f / 180.0f;
  return gravity_gain_m4 * fabsf(sinf(angle_rel_rad));
}

void PID_Controller_GetSpeedPID(uint8_t motor_id, float *kp, float *ki, float *kd)
{
  PID_ControllerPID_t *pid = PID_Controller_GetSpeedPidPtr(motor_id);

  if (pid == NULL)
  {
    return;
  }

  if (kp != NULL) { *kp = pid->kp; }
  if (ki != NULL) { *ki = pid->ki; }
  if (kd != NULL) { *kd = pid->kd; }
}
