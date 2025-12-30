/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern 	uint32_t 	hundredth;
extern 	uint32_t 	tenth_counter;
		uint32_t 	sec_counter = 0;
extern	uint32_t 	seconds;


extern 	volatile uint8_t	uart_rx_buffer[UART_RX_BUF_LEN];
extern 	volatile uint8_t	uart_rx_buffer_copy[UART_RX_BUF_LEN];
extern	volatile uint8_t	uart_rx_buffer_processed_flag;
extern 	volatile uint8_t	uart_rx_buffer_processed_flag;
extern 	volatile uint8_t	uart_tx_transfer_completed;

uint16_t caps1 [8] = {0};
uint8_t caps1_index = 0;
uint16_t caps1_mean = 0;
uint16_t tim3_cc1_caps = 0;
uint16_t tim3_cc2_caps = 0;
uint16_t tim3_update_caps = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

extern void (*ptr_isr_hundredth)(void);
extern void (*ptr_sptr_entry_lane_1)(uint16_t capture);
extern void (*ptr_sptr_exit_lane_1)(uint16_t capture);
extern void (*ptr_sptr_entry_lane_2)(uint16_t capture);
extern void (*ptr_sptr_exit_lane_2)(uint16_t capture);
extern void (*ptr_sptr_tim_overflow)(void);

extern void (*ptr_dma_man_tc)(void);
extern void (*ptr_dma_man_ht)(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */

  /* USER CODE END DMA1_Stream2_IRQn 0 */
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream4 global interrupt.
  */
void DMA1_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream4_IRQn 0 */

  /* USER CODE END DMA1_Stream4_IRQn 0 */
  /* USER CODE BEGIN DMA1_Stream4_IRQn 1 */

  /* USER CODE END DMA1_Stream4_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */
	if (LL_TIM_IsActiveFlag_UPDATE(TIM10)){
		LL_TIM_ClearFlag_UPDATE(TIM10);
		//LL_GPIO_TogglePin(OUTPUT_HUNDREDTH_TEST_GPIO_Port, OUTPUT_HUNDREDTH_TEST_Pin);
		hundredth++;
		tenth_counter++;
		sec_counter++;

		if (10 <= tenth_counter){
			tenth_counter = 0;
			LL_GPIO_TogglePin(TRACK_TICKS_IND_LED_GPIO_Port, TRACK_TICKS_IND_LED_Pin);
		}

		if (100 < sec_counter){
			sec_counter = 0;
			seconds++;
		}
	}
  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  /* USER CODE BEGIN TIM3_IRQn 1 */
	if (LL_TIM_IsActiveFlag_CC1(TIM3)){
		uint16_t capture = LL_TIM_IC_GetCaptureCH1(TIM3);
		LL_TIM_ClearFlag_CC1(TIM3);



		//TEST CODE
		caps1[caps1_index] = capture;
		caps1_index = (caps1_index+1) & 0x07;

		if (!caps1_index){
			for (uint8_t caps1_iter = 1; caps1_iter < 8; caps1_iter++)
				caps1_mean = caps1[caps1_iter]-caps1[caps1_iter-1];
			caps1_mean >>= 3;
		}
		tim3_cc1_caps++;
		//TEST CODE

		if (ptr_sptr_entry_lane_1){
			ptr_sptr_entry_lane_1(capture);
		}
	} else if (LL_TIM_IsActiveFlag_CC2(TIM3)){
		uint16_t capture = LL_TIM_IC_GetCaptureCH2(TIM3);
		LL_TIM_ClearFlag_CC2(TIM3);
		tim3_cc2_caps++;
		if (ptr_sptr_exit_lane_1){
			ptr_sptr_exit_lane_1(capture);
		}
	} else if (LL_TIM_IsActiveFlag_CC3(TIM3)){
		uint16_t capture = LL_TIM_IC_GetCaptureCH3(TIM3);
		LL_TIM_ClearFlag_CC3(TIM3);
		if (ptr_sptr_entry_lane_2){
			ptr_sptr_entry_lane_2(capture);
		}
	} else if (LL_TIM_IsActiveFlag_CC4(TIM3)){
		uint16_t capture = LL_TIM_IC_GetCaptureCH4(TIM3);
		LL_TIM_ClearFlag_CC4(TIM3);
		if (ptr_sptr_exit_lane_2){
			ptr_sptr_exit_lane_2(capture);
		}
	} else if (LL_TIM_IsActiveFlag_UPDATE(TIM3)){
		LL_TIM_ClearFlag_UPDATE(TIM3);
		tim3_update_caps++;
		if (ptr_sptr_tim_overflow)
			ptr_sptr_tim_overflow();
	}
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream7 global interrupt.
  */
void DMA1_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream7_IRQn 0 */

  /* USER CODE END DMA1_Stream7_IRQn 0 */
  /* USER CODE BEGIN DMA1_Stream7_IRQn 1 */
	if (LL_DMA_IsActiveFlag_HT7(DMA1)){
			LL_DMA_ClearFlag_HT7(DMA1);
			if (ptr_dma_man_ht) { // check if assigned
				ptr_dma_man_ht();
			}
		} else if (LL_DMA_IsActiveFlag_TC7(DMA1)){
			LL_DMA_ClearFlag_TC7(DMA1);
			if (ptr_dma_man_tc) { // check if assigned
				ptr_dma_man_tc();
			}
		}
  /* USER CODE END DMA1_Stream7_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

	// RX RX RX RX RX RX
	if (LL_DMA_IsActiveFlag_TC2(DMA2)){
		LL_DMA_ClearFlag_TC2(DMA2);
		if (uart_rx_buffer_processed_flag){
			for (uint8_t idx_rx = 0; idx_rx < UART_RX_BUF_LEN; idx_rx++)
				uart_rx_buffer_copy[idx_rx] = uart_rx_buffer[idx_rx];
			uart_rx_buffer_processed_flag = 0;
		}
	}

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream7 global interrupt.
  */
void DMA2_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream7_IRQn 0 */

  /* USER CODE END DMA2_Stream7_IRQn 0 */
  /* USER CODE BEGIN DMA2_Stream7_IRQn 1 */
	if (LL_DMA_IsActiveFlag_TC7(DMA2)){
		LL_DMA_ClearFlag_TC7(DMA2);
		uart_tx_transfer_completed = 1;
	}
  /* USER CODE END DMA2_Stream7_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
