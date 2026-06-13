/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart_rxcallback.h
  * @brief   UART4 VOFA justfloat output interface.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __USART_RXCALLBACK_H
#define __USART_RXCALLBACK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usart.h"

#define USART_RXCALLBACK_VOFA_FLOAT_COUNT 4U
#define USART_RXCALLBACK_VOFA_TAIL_SIZE 4U

/* VOFA command receive buffer size */
#define USART_RXCALLBACK_VOFA_RX_BUF_SIZE 128U

typedef struct
{
  float motor2_actual_angle_deg;
  float motor2_target_angle_deg;
  float motor4_actual_angle_deg;
  float motor4_target_angle_deg;
} Usart_RxCallBack_VofaFrame_t;

void Usart_RxCallBack_Init(void);
HAL_StatusTypeDef Usart_RxCallBack_SendVofaJustFloat(const Usart_RxCallBack_VofaFrame_t *vofa_frame);

/* UART4 DMA idle-line reception: call from HAL_UARTEx_RxEventCallback / idle interrupt */
void Usart_RxCallBack_StartDmaReception(void);
void Usart_RxCallBack_ProcessRxData(uint16_t rx_length);

#ifdef __cplusplus
}
#endif

#endif
