/* Includes ------------------------------------------------------------------*/
#include "usart_rxcallback.h"

/* Private includes ----------------------------------------------------------*/
#include "PID_Controller.h"
#include <string.h>
#include <stdlib.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* Double-buffered DMA receive: buffer A for active DMA, buffer B for parsing */
static uint8_t vofa_rx_buf_a[USART_RXCALLBACK_VOFA_RX_BUF_SIZE];
static uint8_t vofa_rx_buf_b[USART_RXCALLBACK_VOFA_RX_BUF_SIZE];
static uint8_t *vofa_rx_active_buf = NULL;

/* Private function prototypes -----------------------------------------------*/
static void Usart_RxCallBack_AppendFloat(uint8_t *buffer, uint8_t *index, float value);
static void Usart_RxCallBack_ParseLine(const char *line, uint16_t line_len);
static float Usart_RxCallBack_ParseFloat(const char *str, uint16_t len);
static const char *Usart_RxCallBack_FindSubstring(const char *haystack, uint16_t haystack_len,
                                                  const char *needle, uint16_t needle_len);

/* Private user code ---------------------------------------------------------*/

/* Simple memory-based substring search (no memmem dependency) */
static const char *Usart_RxCallBack_FindSubstring(const char *haystack, uint16_t haystack_len,
                                                  const char *needle, uint16_t needle_len)
{
  uint16_t i;
  uint16_t j;

  if ((haystack == NULL) || (needle == NULL) || (haystack_len < needle_len) || (needle_len == 0U))
  {
    return NULL;
  }

  for (i = 0U; i <= (uint16_t)(haystack_len - needle_len); i++)
  {
    for (j = 0U; j < needle_len; j++)
    {
      if (haystack[i + j] != needle[j])
      {
        break;
      }
    }
    if (j == needle_len)
    {
      return &haystack[i];
    }
  }

  return NULL;
}

static void Usart_RxCallBack_AppendFloat(uint8_t *buffer, uint8_t *index, float value)
{
  union
  {
    float value;
    uint8_t bytes[4];
  } float_bytes;

  float_bytes.value = value;
  buffer[*index + 0U] = float_bytes.bytes[0];
  buffer[*index + 1U] = float_bytes.bytes[1];
  buffer[*index + 2U] = float_bytes.bytes[2];
  buffer[*index + 3U] = float_bytes.bytes[3];
  *index = (uint8_t)(*index + 4U);
}

/* Parse a float from a string fragment. Returns 0 on failure. */
static float Usart_RxCallBack_ParseFloat(const char *str, uint16_t len)
{
  char temp[32];
  float result;
  char *endptr;

  if ((str == NULL) || (len == 0U) || (len >= sizeof(temp)))
  {
    return 0.0f;
  }

  (void)memcpy(temp, str, len);
  temp[len] = '\0';

  result = strtof(temp, &endptr);
  if (endptr == temp)
  {
    return 0.0f;
  }

  return result;
}

/*
 * Supported VOFA command formats:
 *
 *   KP_Angle=10         → set angle Kp for both motors
 *   KI_Angle=10         → set angle Ki for both motors
 *   KD_Angle=10         → set angle Kd for both motors
 *   KP_Velocity=10      → set speed Kp for both motors
 *   KI_Velocity=10      → set speed Ki for both motors
 *   KD_Velocity=10      → set speed Kd for both motors
 *
 *   M2_KP_Angle=10      → set motor-2 angle Kp
 *   M4_KP_Velocity=10   → set motor-4 speed Kp
 *   ... and so on for all combinations.
 *
 *   Gravity_Gain=2000!  → set gravity compensation gain for M4
 *   Gravity_Offset=97.5! → set gravity offset angle (where gravity is zero)
 */
static void Usart_RxCallBack_ParseLine(const char *line, uint16_t line_len)
{
  const char *equal_pos;
  const char *bang_pos;
  const char *name_start;
  const char *value_start;
  uint16_t name_len;
  uint16_t value_len;
  uint8_t is_angle;
  float value;
  uint8_t motor_id;

  if ((line == NULL) || (line_len == 0U))
  {
    return;
  }

  /* Locate '=' and '!' */
  equal_pos = memchr(line, '=', line_len);
  if (equal_pos == NULL)
  {
    return;
  }

  bang_pos = memchr(line, '!', line_len);
  if (bang_pos == NULL)
  {
    return;
  }

  /* '=' must come before '!' */
  if (equal_pos >= bang_pos)
  {
    return;
  }

  name_start = line;
  name_len = (uint16_t)(equal_pos - name_start);
  value_start = equal_pos + 1U;
  value_len = (uint16_t)(bang_pos - value_start);

  if ((name_len == 0U) || (value_len == 0U))
  {
    return;
  }

  /* Parse value */
  value = Usart_RxCallBack_ParseFloat(value_start, value_len);

  /* Gravity compensation commands (checked before loop type, as they don't contain Angle/Velocity) */
  if (Usart_RxCallBack_FindSubstring(line, name_len, "Gravity_Gain", 12U) != NULL)
  {
    PID_Controller_SetGravityGain(value);
    return;
  }
  if (Usart_RxCallBack_FindSubstring(line, name_len, "Gravity_Offset", 14U) != NULL)
  {
    PID_Controller_SetGravityOffset(value);
    return;
  }

  /* Determine Angle vs Velocity */
  if (Usart_RxCallBack_FindSubstring(line, name_len, "Velocity", 8U) != NULL)
  {
    is_angle = 0U; /* speed loop */
  }
  else if (Usart_RxCallBack_FindSubstring(line, name_len, "Angle", 5U) != NULL)
  {
    is_angle = 1U; /* angle loop */
  }
  else
  {
    return; /* unknown loop */
  }

  /* Determine motor_id: check for M2_ or M4_ prefix, default to 0 = both */
  motor_id = 0U;
  if ((name_len >= 3U) && (name_start[0U] == 'M') && (name_start[2U] == '_'))
  {
    if (name_start[1U] == '2')
    {
      motor_id = 2U;
    }
    else if (name_start[1U] == '4')
    {
      motor_id = 4U;
    }
  }

  /* Determine KP / KI / KD */
  if (Usart_RxCallBack_FindSubstring(line, name_len, "KP_", 3U) != NULL)
  {
    if (is_angle != 0U)
    {
      if (motor_id == 0U || motor_id == 2U) PID_Controller_SetAnglePID(2U, value, -1.0f, -1.0f);
      if (motor_id == 0U || motor_id == 4U) PID_Controller_SetAnglePID(4U, value, -1.0f, -1.0f);
    }
    else
    {
      if (motor_id == 0U || motor_id == 2U) PID_Controller_SetSpeedPID(2U, value, -1.0f, -1.0f);
      if (motor_id == 0U || motor_id == 4U) PID_Controller_SetSpeedPID(4U, value, -1.0f, -1.0f);
    }
  }
  else if (Usart_RxCallBack_FindSubstring(line, name_len, "KI_", 3U) != NULL)
  {
    if (is_angle != 0U)
    {
      if (motor_id == 0U || motor_id == 2U) PID_Controller_SetAnglePID(2U, -1.0f, value, -1.0f);
      if (motor_id == 0U || motor_id == 4U) PID_Controller_SetAnglePID(4U, -1.0f, value, -1.0f);
    }
    else
    {
      if (motor_id == 0U || motor_id == 2U) PID_Controller_SetSpeedPID(2U, -1.0f, value, -1.0f);
      if (motor_id == 0U || motor_id == 4U) PID_Controller_SetSpeedPID(4U, -1.0f, value, -1.0f);
    }
  }
  else if (Usart_RxCallBack_FindSubstring(line, name_len, "KD_", 3U) != NULL)
  {
    if (is_angle != 0U)
    {
      if (motor_id == 0U || motor_id == 2U) PID_Controller_SetAnglePID(2U, -1.0f, -1.0f, value);
      if (motor_id == 0U || motor_id == 4U) PID_Controller_SetAnglePID(4U, -1.0f, -1.0f, value);
    }
    else
    {
      if (motor_id == 0U || motor_id == 2U) PID_Controller_SetSpeedPID(2U, -1.0f, -1.0f, value);
      if (motor_id == 0U || motor_id == 4U) PID_Controller_SetSpeedPID(4U, -1.0f, -1.0f, value);
    }
  }
}

void Usart_RxCallBack_Init(void)
{
  vofa_rx_active_buf = vofa_rx_buf_a;
  (void)memset(vofa_rx_buf_a, 0, sizeof(vofa_rx_buf_a));
  (void)memset(vofa_rx_buf_b, 0, sizeof(vofa_rx_buf_b));
}

void Usart_RxCallBack_StartDmaReception(void)
{
  (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart4, vofa_rx_active_buf, USART_RXCALLBACK_VOFA_RX_BUF_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);
}

void Usart_RxCallBack_ProcessRxData(uint16_t rx_length)
{
  uint8_t *parse_buf;
  const char *line_start;
  uint16_t remaining;
  uint16_t line_len;
  uint16_t i;

  if ((rx_length == 0U) || (vofa_rx_active_buf == NULL))
  {
    return;
  }

  /* Swap buffers: parse the just-filled buffer, restart DMA on the other */
  if (vofa_rx_active_buf == vofa_rx_buf_a)
  {
    parse_buf = vofa_rx_buf_a;
    vofa_rx_active_buf = vofa_rx_buf_b;
  }
  else
  {
    parse_buf = vofa_rx_buf_b;
    vofa_rx_active_buf = vofa_rx_buf_a;
  }

  /* Parse complete lines delimited by '!' */
  line_start = (const char *)parse_buf;
  remaining = rx_length;

  while (remaining > 0U)
  {
    line_len = 0U;
    for (i = 0U; i < remaining; i++)
    {
      if (line_start[i] == '!')
      {
        line_len = (uint16_t)(i + 1U);
        break;
      }
    }

    if ((line_len == 0U) || (line_len > USART_RXCALLBACK_VOFA_RX_BUF_SIZE))
    {
      break; /* No complete command yet, stop parsing */
    }

    Usart_RxCallBack_ParseLine(line_start, line_len);

    line_start += line_len;
    remaining = (uint16_t)(remaining - line_len);
  }

  /* Restart DMA idle-line reception */
  (void)Usart_RxCallBack_StartDmaReception();
}

HAL_StatusTypeDef Usart_RxCallBack_SendVofaJustFloat(const Usart_RxCallBack_VofaFrame_t *vofa_frame)
{
  uint8_t frame[USART_RXCALLBACK_VOFA_FLOAT_COUNT * 4U + USART_RXCALLBACK_VOFA_TAIL_SIZE] = {0};
  uint8_t index = 0U;

  if (vofa_frame == NULL)
  {
    return HAL_ERROR;
  }

  Usart_RxCallBack_AppendFloat(frame, &index, vofa_frame->motor2_actual_angle_deg);
  Usart_RxCallBack_AppendFloat(frame, &index, vofa_frame->motor2_target_angle_deg);
  Usart_RxCallBack_AppendFloat(frame, &index, vofa_frame->motor4_actual_angle_deg);
  Usart_RxCallBack_AppendFloat(frame, &index, vofa_frame->motor4_target_angle_deg);

  frame[index + 0U] = 0x00U;
  frame[index + 1U] = 0x00U;
  frame[index + 2U] = 0x80U;
  frame[index + 3U] = 0x7FU;

  return HAL_UART_Transmit(&huart4, frame, (uint16_t)sizeof(frame), 10U);
}
