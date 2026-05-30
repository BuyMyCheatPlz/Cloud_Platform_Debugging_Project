/* Includes ------------------------------------------------------------------*/
#include "can_motor.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static GM6020_Feedback_t gm6020_feedback_2 = {0};
static GM6020_Feedback_t gm6020_feedback_4 = {0};

/* Private function prototypes -----------------------------------------------*/
static GM6020_Feedback_t *GM6020_GetFeedbackByStdId(uint32_t std_id);

/* Private user code ---------------------------------------------------------*/
static int16_t GM6020_ClampVoltage(int16_t voltage)
{
  if (voltage > 25000)
  {
    return 25000;
  }

  if (voltage < -25000)
  {
    return -25000;
  }

  return voltage;
}

void GM6020_CAN1_Init(void)
{
  CAN_FilterTypeDef filter_config = {0};

  filter_config.FilterBank = 0;
  filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
  filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
  filter_config.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter_config.FilterActivation = ENABLE;
  filter_config.FilterIdHigh = (uint16_t)(GM6020_MOTOR_2_ID << 5);
  filter_config.FilterIdLow = 0x0000;
  filter_config.FilterMaskIdHigh = 0xFFE0;
  filter_config.FilterMaskIdLow = 0x0000;
  filter_config.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &filter_config) != HAL_OK)
  {
    Error_Handler();
  }

  filter_config.FilterBank = 1;
  filter_config.FilterIdHigh = (uint16_t)(GM6020_MOTOR_4_ID << 5);

  if (HAL_CAN_ConfigFilter(&hcan1, &filter_config) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }
}

const GM6020_Feedback_t *GM6020_GetFeedback(uint8_t motor_id)
{
  if (motor_id == 2U)
  {
    return &gm6020_feedback_2;
  }

  if (motor_id == 4U)
  {
    return &gm6020_feedback_4;
  }

  return NULL;
}

HAL_StatusTypeDef GM6020_SetVoltage(int16_t motor2_voltage, int16_t motor4_voltage)
{
  CAN_TxHeaderTypeDef tx_header = {0};
  uint8_t tx_data[8] = {0};
  uint32_t tx_mailbox = 0;

  motor2_voltage = GM6020_ClampVoltage(motor2_voltage);
  motor4_voltage = GM6020_ClampVoltage(motor4_voltage);

  tx_header.StdId = 0x1FFU;
  tx_header.ExtId = 0x00000000U;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 8;
  tx_header.TransmitGlobalTime = DISABLE;

  tx_data[2] = (uint8_t)(((uint16_t)motor2_voltage) >> 8);
  tx_data[3] = (uint8_t)(motor2_voltage & 0xFF);
  tx_data[6] = (uint8_t)(((uint16_t)motor4_voltage) >> 8);
  tx_data[7] = (uint8_t)(motor4_voltage & 0xFF);

  return HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header = {0};
  uint8_t rx_data[8] = {0};
  GM6020_Feedback_t *feedback = NULL;

  if (hcan->Instance != CAN1)
  {
    return;
  }

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
  {
    return;
  }

  feedback = GM6020_GetFeedbackByStdId(rx_header.StdId);
  if (feedback == NULL)
  {
    return;
  }

  feedback->encoder = (uint16_t)((((uint16_t)rx_data[0]) << 8) | rx_data[1]);
  feedback->speed_rpm = (int16_t)((((uint16_t)rx_data[2]) << 8) | rx_data[3]);
  feedback->current = (int16_t)((((uint16_t)rx_data[4]) << 8) | rx_data[5]);
  feedback->temperature = rx_data[6];
  feedback->last_update_tick = HAL_GetTick();
  feedback->valid = 1U;
}

static GM6020_Feedback_t *GM6020_GetFeedbackByStdId(uint32_t std_id)
{
  if (std_id == GM6020_MOTOR_2_ID)
  {
    return &gm6020_feedback_2;
  }

  if (std_id == GM6020_MOTOR_4_ID)
  {
    return &gm6020_feedback_4;
  }

  return NULL;
}
