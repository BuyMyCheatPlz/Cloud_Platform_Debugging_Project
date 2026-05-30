/* Includes ------------------------------------------------------------------*/
#include "remote_control.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static Remote_Control_Data_t remote_control_data = {0};
static uint8_t remote_control_rx_byte = 0U;
static uint8_t remote_control_frame[REMOTE_CONTROL_SBUS_FRAME_LENGTH] = {0};
static uint8_t remote_control_frame_index = 0U;

/* Private function prototypes -----------------------------------------------*/
static float Remote_Control_ClampFloat(float value, float min_value, float max_value);
static float Remote_Control_MapChannelToAngle(uint16_t channel_raw);
static uint16_t Remote_Control_DecodeChannel(const uint8_t *frame, uint8_t channel_index);
static void Remote_Control_DecodeFrame(const uint8_t *frame);
static void Remote_Control_ProcessByte(uint8_t byte);

/* Private user code ---------------------------------------------------------*/
static float Remote_Control_ClampFloat(float value, float min_value, float max_value)
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

static float Remote_Control_MapChannelToAngle(uint16_t channel_raw)
{
  float normalized;

  channel_raw = (uint16_t)Remote_Control_ClampFloat((float)channel_raw, (float)REMOTE_CONTROL_SBUS_CHANNEL_MIN, (float)REMOTE_CONTROL_SBUS_CHANNEL_MAX);
  normalized = ((float)channel_raw - (float)REMOTE_CONTROL_SBUS_CHANNEL_MIN) /
               ((float)REMOTE_CONTROL_SBUS_CHANNEL_MAX - (float)REMOTE_CONTROL_SBUS_CHANNEL_MIN);

  /* Map channel to a relative angle centered at 0 (range -180..+180).
     This makes the mid-stick correspond to 0deg relative to power-on position. */
  return Remote_Control_ClampFloat((normalized * 360.0f) - 180.0f, -180.0f, 180.0f);
}

static uint16_t Remote_Control_DecodeChannel(const uint8_t *frame, uint8_t channel_index)
{
  uint32_t bit_offset;
  uint32_t byte_offset;
  uint32_t value;
  uint8_t bit_index;

  bit_offset = (uint32_t)channel_index * 11U;
  byte_offset = 1U + (bit_offset / 8U);
  bit_index = (uint8_t)(bit_offset % 8U);

  value = (uint32_t)frame[byte_offset] | ((uint32_t)frame[byte_offset + 1U] << 8) | ((uint32_t)frame[byte_offset + 2U] << 16);
  value >>= bit_index;
  value &= 0x07FFU;

  return (uint16_t)value;
}

static void Remote_Control_DecodeFrame(const uint8_t *frame)
{
  uint8_t channel_index;
  uint8_t flags;

  for (channel_index = 0U; channel_index < 16U; ++channel_index)
  {
    remote_control_data.channel_raw[channel_index] = Remote_Control_DecodeChannel(frame, channel_index);
  }

  /* Swap channel mapping: make channel 0 -> motor2, channel 1 -> motor4 */
  remote_control_data.motor2_target_angle_deg = Remote_Control_MapChannelToAngle(remote_control_data.channel_raw[0]);
  remote_control_data.motor4_target_angle_deg = Remote_Control_MapChannelToAngle(remote_control_data.channel_raw[1]);

  flags = frame[23];
  remote_control_data.frame_lost = (uint8_t)((flags >> 2) & 0x01U);
  remote_control_data.failsafe = (uint8_t)((flags >> 3) & 0x01U);
  remote_control_data.frame_valid = 1U;
  remote_control_data.last_update_tick = HAL_GetTick();
}

static void Remote_Control_ProcessByte(uint8_t byte)
{
  if (remote_control_frame_index == 0U)
  {
    if (byte != REMOTE_CONTROL_SBUS_START_BYTE)
    {
      return;
    }
  }

  remote_control_frame[remote_control_frame_index++] = byte;

  if (remote_control_frame_index < REMOTE_CONTROL_SBUS_FRAME_LENGTH)
  {
    return;
  }

  remote_control_frame_index = 0U;

  if ((remote_control_frame[0] != REMOTE_CONTROL_SBUS_START_BYTE) ||
      (remote_control_frame[24] != REMOTE_CONTROL_SBUS_END_BYTE))
  {
    return;
  }

  Remote_Control_DecodeFrame(remote_control_frame);
}

void Remote_Control_Init(void)
{
  Remote_Control_Reset();
  (void)HAL_UART_Receive_IT(&huart2, &remote_control_rx_byte, 1U);
}

void Remote_Control_Reset(void)
{
  uint8_t index;

  for (index = 0U; index < 16U; ++index)
  {
    remote_control_data.channel_raw[index] = 0U;
  }

  remote_control_data.motor4_target_angle_deg = 0.0f;
  remote_control_data.motor2_target_angle_deg = 0.0f;
  remote_control_data.frame_valid = 0U;
  remote_control_data.frame_lost = 0U;
  remote_control_data.failsafe = 0U;
  remote_control_data.last_update_tick = 0U;
  remote_control_frame_index = 0U;
}

uint8_t Remote_Control_IsFrameValid(void)
{
  return remote_control_data.frame_valid;
}

uint8_t Remote_Control_IsOnline(void)
{
  if (remote_control_data.frame_valid == 0U)
  {
    return 0U;
  }

  return (uint8_t)((HAL_GetTick() - remote_control_data.last_update_tick) < 100U);
}

float Remote_Control_GetMotor4TargetAngleDeg(void)
{
  return remote_control_data.motor4_target_angle_deg;
}

float Remote_Control_GetMotor2TargetAngleDeg(void)
{
  return remote_control_data.motor2_target_angle_deg;
}

void Remote_Control_GetTargetAngles(float *motor4_target_angle_deg, float *motor2_target_angle_deg)
{
  if (motor4_target_angle_deg != NULL)
  {
    *motor4_target_angle_deg = remote_control_data.motor4_target_angle_deg;
  }

  if (motor2_target_angle_deg != NULL)
  {
    *motor2_target_angle_deg = remote_control_data.motor2_target_angle_deg;
  }
}

const Remote_Control_Data_t *Remote_Control_GetData(void)
{
  return &remote_control_data;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((huart == NULL) || (huart->Instance != USART2))
  {
    return;
  }

  Remote_Control_ProcessByte(remote_control_rx_byte);
  (void)HAL_UART_Receive_IT(&huart2, &remote_control_rx_byte, 1U);
}
