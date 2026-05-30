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
/* Remote step control settings */
#define APP_REMOTE_STEP_DEG 30.0f
#define APP_REMOTE_SWITCH_COUNT 4U
#define APP_REMOTE_SWITCH_STATE_LOW 0U
#define APP_REMOTE_SWITCH_STATE_MID 1U
#define APP_REMOTE_SWITCH_STATE_HIGH 2U

static uint8_t remote_prev_online = 0U;
static uint8_t remote_switch_initialized = 0U;
static uint8_t remote_prev_switch_states[APP_REMOTE_SWITCH_COUNT] = {0U};
static float remote_step_offset_motor2 = 0.0f;
static float remote_step_offset_motor4 = 0.0f;
static uint8_t remote_step_synced_motor2 = 0U;
static uint8_t remote_step_synced_motor4 = 0U;
/* Wait-for-user mode: keep PID targets at 0 until operator acts */
static uint8_t remote_wait_for_user_input = 1U;
/* initial raw channel snapshot for action detection */
static uint16_t remote_initial_ch0 = 0U;
static uint16_t remote_initial_ch1 = 0U;
static uint16_t remote_initial_ch6 = 0U;
static uint16_t remote_initial_ch7 = 0U;
static uint16_t remote_initial_ch8 = 0U;
static uint16_t remote_initial_ch9 = 0U;
/* raw change threshold to consider stick moved */
#define APP_REMOTE_USER_MOVE_THRESHOLD_RAW 60U
static uint8_t vofa_first_normal_frame = 1U;

/* Private function prototypes -----------------------------------------------*/
static void App_Task02_ReadRemoteSnapshot(Remote_Control_Data_t *snapshot);
static uint8_t App_Task02_DecodeBinarySwitch(uint16_t channel_raw);
static uint8_t App_Task02_DecodeThreePositionSwitch(uint16_t channel_raw);
static void App_Task02_InitializeSwitchState(const Remote_Control_Data_t *snapshot);
static void App_Task02_SyncMotor2StepOffset(float base_target_deg);
static void App_Task02_SyncMotor4StepOffset(float base_target_deg);
static void App_Task02_ApplySwitchStep(uint8_t channel_index, uint8_t current_state, float base_motor2_target_deg, float base_motor4_target_deg);
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

static uint8_t App_Task02_DecodeBinarySwitch(uint16_t channel_raw)
{
	uint16_t midpoint;

	midpoint = (uint16_t)((REMOTE_CONTROL_SBUS_CHANNEL_MIN + REMOTE_CONTROL_SBUS_CHANNEL_MAX) / 2U);
	return (uint8_t)((channel_raw > midpoint) ? 1U : 0U);
}

static uint8_t App_Task02_DecodeThreePositionSwitch(uint16_t channel_raw)
{
	uint32_t span;
	uint16_t low_threshold;
	uint16_t high_threshold;

	span = (uint32_t)(REMOTE_CONTROL_SBUS_CHANNEL_MAX - REMOTE_CONTROL_SBUS_CHANNEL_MIN);
	low_threshold = (uint16_t)(REMOTE_CONTROL_SBUS_CHANNEL_MIN + (span / 3U));
	high_threshold = (uint16_t)(REMOTE_CONTROL_SBUS_CHANNEL_MIN + ((span * 2U) / 3U));

	if (channel_raw <= low_threshold)
	{
		return APP_REMOTE_SWITCH_STATE_LOW;
	}

	if (channel_raw >= high_threshold)
	{
		return APP_REMOTE_SWITCH_STATE_HIGH;
	}

	return APP_REMOTE_SWITCH_STATE_MID;
}

static void App_Task02_InitializeSwitchState(const Remote_Control_Data_t *snapshot)
{
	remote_prev_switch_states[0U] = App_Task02_DecodeBinarySwitch(snapshot->channel_raw[6U]);
	remote_prev_switch_states[1U] = App_Task02_DecodeBinarySwitch(snapshot->channel_raw[7U]);
	remote_prev_switch_states[2U] = App_Task02_DecodeThreePositionSwitch(snapshot->channel_raw[8U]);
	remote_prev_switch_states[3U] = App_Task02_DecodeBinarySwitch(snapshot->channel_raw[9U]);

	remote_step_offset_motor2 = 0.0f;
	remote_step_offset_motor4 = 0.0f;
	remote_step_synced_motor2 = 0U;
	remote_step_synced_motor4 = 0U;

	/* store initial raw channels for user-action detection */
	remote_initial_ch0 = snapshot->channel_raw[0U];
	remote_initial_ch1 = snapshot->channel_raw[1U];
	remote_initial_ch6 = snapshot->channel_raw[6U];
	remote_initial_ch7 = snapshot->channel_raw[7U];
	remote_initial_ch8 = snapshot->channel_raw[8U];
	remote_initial_ch9 = snapshot->channel_raw[9U];

	remote_switch_initialized = 1U;
}

static uint8_t App_Task02_CheckUserAction(const Remote_Control_Data_t *snapshot)
{
	uint16_t cur0, cur1;

	if (snapshot == NULL)
	{
		return 0U;
	}

	cur0 = snapshot->channel_raw[0U];
	cur1 = snapshot->channel_raw[1U];

	if ((cur0 > (remote_initial_ch0 + APP_REMOTE_USER_MOVE_THRESHOLD_RAW)) || (cur0 + APP_REMOTE_USER_MOVE_THRESHOLD_RAW < remote_initial_ch0))
	{
		return 1U;
	}

	if ((cur1 > (remote_initial_ch1 + APP_REMOTE_USER_MOVE_THRESHOLD_RAW)) || (cur1 + APP_REMOTE_USER_MOVE_THRESHOLD_RAW < remote_initial_ch1))
	{
		return 1U;
	}

	/* any switch change (6..9) */
	if (App_Task02_DecodeBinarySwitch(snapshot->channel_raw[6U]) != App_Task02_DecodeBinarySwitch(remote_initial_ch6))
	{
		return 1U;
	}

	if (App_Task02_DecodeBinarySwitch(snapshot->channel_raw[7U]) != App_Task02_DecodeBinarySwitch(remote_initial_ch7))
	{
		return 1U;
	}

	if (App_Task02_DecodeThreePositionSwitch(snapshot->channel_raw[8U]) != App_Task02_DecodeThreePositionSwitch(remote_initial_ch8))
	{
		return 1U;
	}

	if (App_Task02_DecodeBinarySwitch(snapshot->channel_raw[9U]) != App_Task02_DecodeBinarySwitch(remote_initial_ch9))
	{
		return 1U;
	}

	return 0U;
}

static void App_Task02_SyncMotor2StepOffset(float base_target_deg)
{
	if (remote_step_synced_motor2 == 0U)
	{
		remote_step_offset_motor2 = PID_Controller_GetMotorCurrentAngleDegrees(2U) - base_target_deg;
		remote_step_synced_motor2 = 1U;
	}
}

static void App_Task02_SyncMotor4StepOffset(float base_target_deg)
{
	if (remote_step_synced_motor4 == 0U)
	{
		remote_step_offset_motor4 = PID_Controller_GetMotorCurrentAngleDegrees(4U) - base_target_deg;
		remote_step_synced_motor4 = 1U;
	}
}

static void App_Task02_ApplySwitchStep(uint8_t channel_index, uint8_t current_state, float base_motor2_target_deg, float base_motor4_target_deg)
{
	if (current_state == remote_prev_switch_states[channel_index])
	{
		return;
	}

	remote_prev_switch_states[channel_index] = current_state;

	switch (channel_index)
	{
		case 0U:
			App_Task02_SyncMotor4StepOffset(base_motor4_target_deg);
			remote_step_offset_motor4 += APP_REMOTE_STEP_DEG;
			break;

		case 1U:
			App_Task02_SyncMotor4StepOffset(base_motor4_target_deg);
			remote_step_offset_motor4 -= APP_REMOTE_STEP_DEG;
			break;

		case 2U:
			if (current_state == APP_REMOTE_SWITCH_STATE_HIGH)
			{
				App_Task02_SyncMotor2StepOffset(base_motor2_target_deg);
				remote_step_offset_motor2 += APP_REMOTE_STEP_DEG;
			}
			else if (current_state == APP_REMOTE_SWITCH_STATE_LOW)
			{
				App_Task02_SyncMotor2StepOffset(base_motor2_target_deg);
				remote_step_offset_motor2 -= APP_REMOTE_STEP_DEG;
			}
			break;

		case 3U:
			App_Task02_SyncMotor2StepOffset(base_motor2_target_deg);
			remote_step_offset_motor2 -= APP_REMOTE_STEP_DEG;
			break;

		default:
			break;
	}
}

static void App_Task02_PublishTargetAngles(void)
{
	App_TargetAngleMessage_t target_message;
	Remote_Control_Data_t remote_snapshot;
	float base_motor2_target_deg;
	float base_motor4_target_deg;
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

		remote_prev_online = 0U;
		remote_switch_initialized = 0U;
		remote_step_offset_motor2 = 0.0f;
		remote_step_offset_motor4 = 0.0f;
		remote_step_synced_motor2 = 0U;
		remote_step_synced_motor4 = 0U;
	}
	else
	{
		/* Remote is online */
		target_message.emergency_stop = 0U;
		base_motor2_target_deg = remote_snapshot.motor2_target_angle_deg;
		base_motor4_target_deg = remote_snapshot.motor4_target_angle_deg;
		remote_step_synced_motor2 = 0U;
		remote_step_synced_motor4 = 0U;

		/* If we're waiting for user action, keep targets at 0 until action detected */
		if (remote_wait_for_user_input != 0U)
		{
			if (App_Task02_CheckUserAction(&remote_snapshot) == 0U)
			{
				/* still waiting */
				target_message.motor2_target_angle_deg = 0.0f;
				target_message.motor4_target_angle_deg = 0.0f;
				remote_prev_online = 1U;
				(void)osMessageQueuePut(App_TargetAngleQueueHandle, &target_message, 0U, 0U);
				return;
			}
			else
			{
				/* user acted: exit wait mode */
				remote_wait_for_user_input = 0U;
				/* reinitialize switches offsets relative to current snapshot */
				App_Task02_InitializeSwitchState(&remote_snapshot);
			}
		}

		/* Detect transition: offline -> online */
		if ((remote_prev_online == 0U) || (remote_switch_initialized == 0U))
		{
			App_Task02_InitializeSwitchState(&remote_snapshot);
			target_message.motor2_target_angle_deg = base_motor2_target_deg;
			target_message.motor4_target_angle_deg = base_motor4_target_deg;
		}
		else
		{
			App_Task02_ApplySwitchStep(0U, App_Task02_DecodeBinarySwitch(remote_snapshot.channel_raw[6U]), base_motor2_target_deg, base_motor4_target_deg);
			App_Task02_ApplySwitchStep(1U, App_Task02_DecodeBinarySwitch(remote_snapshot.channel_raw[7U]), base_motor2_target_deg, base_motor4_target_deg);
			App_Task02_ApplySwitchStep(2U, App_Task02_DecodeThreePositionSwitch(remote_snapshot.channel_raw[8U]), base_motor2_target_deg, base_motor4_target_deg);
			App_Task02_ApplySwitchStep(3U, App_Task02_DecodeBinarySwitch(remote_snapshot.channel_raw[9U]), base_motor2_target_deg, base_motor4_target_deg);

			target_message.motor2_target_angle_deg = base_motor2_target_deg + remote_step_offset_motor2;
			target_message.motor4_target_angle_deg = base_motor4_target_deg + remote_step_offset_motor4;
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

	if (vofa_first_normal_frame != 0U)
	{
		vofa_message.motor2_target_angle_deg = 0.0f;
		vofa_message.motor4_target_angle_deg = 0.0f;
		vofa_first_normal_frame = 0U;
	}

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
