/* Includes ------------------------------------------------------------------*/
#include "PID_Controller.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static PID_ControllerMotor_t pid_motor_2 = {0};
static PID_ControllerMotor_t pid_motor_4 = {0};

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
  PID_Controller_InitSinglePID(&pid_motor_2.angle_pid, PID_CONTROLLER_DEFAULT_ANGLE_KP, PID_CONTROLLER_DEFAULT_ANGLE_KI, PID_CONTROLLER_DEFAULT_ANGLE_KD, PID_CONTROLLER_DEFAULT_ANGLE_INTEGRAL_LIMIT, PID_CONTROLLER_DEFAULT_ANGLE_OUTPUT_LIMIT);
  PID_Controller_InitSinglePID(&pid_motor_2.speed_pid, PID_CONTROLLER_DEFAULT_SPEED_KP, PID_CONTROLLER_DEFAULT_SPEED_KI, PID_CONTROLLER_DEFAULT_SPEED_KD, PID_CONTROLLER_DEFAULT_SPEED_INTEGRAL_LIMIT, PID_CONTROLLER_DEFAULT_SPEED_OUTPUT_LIMIT);
  PID_Controller_InitSinglePID(&pid_motor_4.angle_pid, PID_CONTROLLER_DEFAULT_ANGLE_KP, PID_CONTROLLER_DEFAULT_ANGLE_KI, PID_CONTROLLER_DEFAULT_ANGLE_KD, PID_CONTROLLER_DEFAULT_ANGLE_INTEGRAL_LIMIT, PID_CONTROLLER_DEFAULT_ANGLE_OUTPUT_LIMIT);
  PID_Controller_InitSinglePID(&pid_motor_4.speed_pid, PID_CONTROLLER_DEFAULT_SPEED_KP, PID_CONTROLLER_DEFAULT_SPEED_KI, PID_CONTROLLER_DEFAULT_SPEED_KD, PID_CONTROLLER_DEFAULT_SPEED_INTEGRAL_LIMIT, PID_CONTROLLER_DEFAULT_SPEED_OUTPUT_LIMIT);
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
  motor4_speed_target = PID_Controller_ComputeSinglePID(&pid_motor_4.angle_pid, (float)pid_motor_4.target_angle_count, (float)pid_motor_4.total_angle_count);

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

  return GM6020_SetVoltage(motor2_voltage, motor4_voltage);
}
