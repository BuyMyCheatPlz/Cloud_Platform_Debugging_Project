/* Includes ------------------------------------------------------------------*/
#include "app_task.h"

/* Private includes ----------------------------------------------------------*/
#include "remote_control.h"
#include "PID_Controller.h"
#include "usart_rxcallback.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Remote connection ramp state to avoid immediate jumps on connect */
#define APP_REMOTE_RAMP_STEPS 50U /* ~250ms with 5ms task delay */
static uint8_t remote_prev_online = 0U;
static uint8_t remote_ramp_active = 0U;
static uint16_t remote_ramp_steps_remaining = 0U;
static float remote_ramp_current_motor2 = 0.0f;
static float remote_ramp_current_motor4 = 0.0f;
static float remote_ramp_inc_motor2 = 0.0f;
static float remote_ramp_inc_motor4 = 0.0f;

/* Private function prototypes -----------------------------------------------*/
static void App_Task02_ReadRemoteSnapshot(Remote_Control_Data_t *snapshot);
static void App_Task02_PublishTargetAngles(void);
static void App_Task03_RunPidAndPublishTelemetry(void);
static void App_Task04_TransmitTelemetry(void);

/* Private user code ---------------------------------------------------------*/
static void App_Task02_ReadRemoteSnapshot(Remote_Control_Data_t *snapshot)
{
	uint32_t primask;
	const Remote_Control_Data_t *remote_data;

	if (snapshot == NULL)
	{
		return;
	}

	primask = __get_PRIMASK();
	__disable_irq();
	remote_data = Remote_Control_GetData();
	if (remote_data != NULL)
	{
		*snapshot = *remote_data;
	}
	if (primask == 0U)
	{
		__enable_irq();
	}
}

static void App_Task02_PublishTargetAngles(void)
{
	App_TargetAngleMessage_t target_message;
	Remote_Control_Data_t remote_snapshot;
	uint8_t remote_invalid;

	App_Task02_ReadRemoteSnapshot(&remote_snapshot);

	remote_invalid = (uint8_t)((Remote_Control_IsOnline() == 0U) ||
						   (remote_snapshot.frame_lost != 0U) ||
						   (remote_snapshot.failsafe != 0U));

	/* If remote is invalid, publish emergency stop and clear any ramp */
	if (remote_invalid != 0U)
	{
		target_message.motor2_target_angle_deg = 0.0f;
		target_message.motor4_target_angle_deg = 0.0f;
		target_message.emergency_stop = 1U;

		remote_ramp_active = 0U;
		remote_ramp_steps_remaining = 0U;
		remote_prev_online = 0U;
	}
	else
	{
		/* Remote is online */
		target_message.emergency_stop = 0U;

		/* Detect transition: offline -> online */
		if (remote_prev_online == 0U)
		{
			/* Start ramp from current actual angle to remote target to avoid sudden jump */
			float cur2 = PID_Controller_GetMotorCurrentAngleDegrees(2U);
			float cur4 = PID_Controller_GetMotorCurrentAngleDegrees(4U);
			float tgt2 = remote_snapshot.motor2_target_angle_deg;
			float tgt4 = remote_snapshot.motor4_target_angle_deg;

			remote_ramp_active = 1U;
			remote_ramp_steps_remaining = APP_REMOTE_RAMP_STEPS;
			remote_ramp_current_motor2 = cur2;
			remote_ramp_current_motor4 = cur4;
			remote_ramp_inc_motor2 = (tgt2 - cur2) / (float)APP_REMOTE_RAMP_STEPS;
			remote_ramp_inc_motor4 = (tgt4 - cur4) / (float)APP_REMOTE_RAMP_STEPS;

			/* Publish the first hold-at-current message */
			target_message.motor2_target_angle_deg = remote_ramp_current_motor2;
			target_message.motor4_target_angle_deg = remote_ramp_current_motor4;
		}
		else if (remote_ramp_active != 0U)
		{
			/* Continue ramping */
			target_message.motor2_target_angle_deg = remote_ramp_current_motor2;
			target_message.motor4_target_angle_deg = remote_ramp_current_motor4;

			remote_ramp_current_motor2 += remote_ramp_inc_motor2;
			remote_ramp_current_motor4 += remote_ramp_inc_motor4;

			if (remote_ramp_steps_remaining > 0U)
			{
				--remote_ramp_steps_remaining;
			}

			if (remote_ramp_steps_remaining == 0U)
			{
				/* Finish ramp: ensure final value equals remote snapshot */
				target_message.motor2_target_angle_deg = remote_snapshot.motor2_target_angle_deg;
				target_message.motor4_target_angle_deg = remote_snapshot.motor4_target_angle_deg;
				remote_ramp_active = 0U;
			}
		}
		else
		{
			/* Normal operation: publish remote target */
			target_message.motor2_target_angle_deg = remote_snapshot.motor2_target_angle_deg;
			target_message.motor4_target_angle_deg = remote_snapshot.motor4_target_angle_deg;
		}

		remote_prev_online = 1U;
	}

	(void)osMessageQueuePut(App_TargetAngleQueueHandle, &target_message, 0U, 0U);
}

static void App_Task03_RunPidAndPublishTelemetry(void)
{
	App_TargetAngleMessage_t target_message;
	App_VofaMessage_t vofa_message;
	HAL_StatusTypeDef pid_status;
	uint8_t emergency_stop_active;

	emergency_stop_active = 0U;
	while (osMessageQueueGet(App_TargetAngleQueueHandle, &target_message, NULL, 0U) == osOK)
	{
		PID_Controller_SetTargetAngleDegrees(target_message.motor2_target_angle_deg, target_message.motor4_target_angle_deg);
		if (target_message.emergency_stop != 0U)
		{
			emergency_stop_active = 1U;
			PID_Controller_Reset();
			(void)GM6020_SetVoltage(0, 0);
		}
	}

	if (emergency_stop_active != 0U)
	{
		vofa_message.motor2_target_angle_deg = 0.0f;
		vofa_message.motor2_actual_angle_deg = PID_Controller_GetMotorCurrentAngleDegrees(2U);
		vofa_message.motor4_target_angle_deg = 0.0f;
		vofa_message.motor4_actual_angle_deg = PID_Controller_GetMotorCurrentAngleDegrees(4U);
		(void)osMessageQueuePut(App_VofaQueueHandle, &vofa_message, 0U, 0U);
		return;
	}

	pid_status = PID_Controller_Update();
	if (pid_status != HAL_OK)
	{
		PID_Controller_Reset();
		(void)GM6020_SetVoltage(0, 0);
		vofa_message.motor2_target_angle_deg = 0.0f;
		vofa_message.motor2_actual_angle_deg = PID_Controller_GetMotorCurrentAngleDegrees(2U);
		vofa_message.motor4_target_angle_deg = 0.0f;
		vofa_message.motor4_actual_angle_deg = PID_Controller_GetMotorCurrentAngleDegrees(4U);
		(void)osMessageQueuePut(App_VofaQueueHandle, &vofa_message, 0U, 0U);
		return;
	}

	vofa_message.motor2_target_angle_deg = PID_Controller_GetMotorTargetAngleDegrees(2U);
	vofa_message.motor2_actual_angle_deg = PID_Controller_GetMotorCurrentAngleDegrees(2U);
	vofa_message.motor4_target_angle_deg = PID_Controller_GetMotorTargetAngleDegrees(4U);
	vofa_message.motor4_actual_angle_deg = PID_Controller_GetMotorCurrentAngleDegrees(4U);

	(void)osMessageQueuePut(App_VofaQueueHandle, &vofa_message, 0U, 0U);
}

static void App_Task04_TransmitTelemetry(void)
{
	App_VofaMessage_t vofa_message;
	Usart_RxCallBack_VofaFrame_t vofa_frame;

	if (osMessageQueueGet(App_VofaQueueHandle, &vofa_message, NULL, 0U) != osOK)
	{
		return;
	}

	vofa_frame.motor2_actual_angle_deg = vofa_message.motor2_actual_angle_deg;
	vofa_frame.motor2_target_angle_deg = vofa_message.motor2_target_angle_deg;
	vofa_frame.motor4_actual_angle_deg = vofa_message.motor4_actual_angle_deg;
	vofa_frame.motor4_target_angle_deg = vofa_message.motor4_target_angle_deg;

	(void)Usart_RxCallBack_SendVofaJustFloat(&vofa_frame);
}

void StartTask02(void *argument)
{
	(void)argument;

	for (;;)
	{
		App_Task02_PublishTargetAngles();
		osDelay(5);
	}
}

void StartTask03(void *argument)
{
	(void)argument;

	for (;;)
	{
		App_Task03_RunPidAndPublishTelemetry();
		osDelay(5);
	}
}

void StartTask04(void *argument)
{
	(void)argument;

	for (;;)
	{
		App_Task04_TransmitTelemetry();
		osDelay(10);
	}
}
