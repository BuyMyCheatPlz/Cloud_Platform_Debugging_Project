/* Includes ------------------------------------------------------------------*/
#include "usart_rxcallback.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void Usart_RxCallBack_AppendFloat(uint8_t *buffer, uint8_t *index, float value);

/* Private user code ---------------------------------------------------------*/
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

void Usart_RxCallBack_Init(void)
{
  return;
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
