/********************************** (C) COPYRIGHT *******************************
 * File Name          : ch32v20x_it.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/12/29
 * Description        : Main Interrupt Service Routines.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32v20x_it.h"
#include "apptick.h"
#include "coil.h"
#include "referee.h"

void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      NMI_Handler
 *
 * @brief   This function handles NMI exception.
 *
 * @return  none
 */
void NMI_Handler(void)
{
  while (1)
  {
  }
}

/*********************************************************************
 * @fn      HardFault_Handler
 *
 * @brief   This function handles Hard Fault exception.
 *
 * @return  none
 */
void HardFault_Handler(void)
{
  /* 复位前先把四路全桥关掉: 否则死机瞬间的占空比会一直保持到复位完成,
   * 线圈持续满电流。 */
  Coil_DisableAll();
  NVIC_SystemReset();
  while (1)
  {
  }
}

/*********************************************************************
 * @fn      TIM4_IRQHandler
 *
 * @brief   控制节拍, 只计数, 实际运算在主循环。
 *
 * @return  none
 */
void TIM4_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        Tick_ISR();
    }
}

/*********************************************************************
 * @fn      USART1_IRQHandler
 *
 * @brief   裁判系统串口收发。
 *
 * @return  none
 */
void USART1_IRQHandler(void)
{
    Referee_IRQHandler();
}
