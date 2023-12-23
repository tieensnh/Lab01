/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "math.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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

uint8_t counter = 0;
long long update = 0;
long long time_ms = 0;
int8_t colCount = -1;
uint8_t btnState = 0;

double T1 = 0.09;
double T2 = 0.18;
double K = 7;
double noise = -1;

double y[2] = {0};
double yd[2] = {0};
double ydd[2] = {0};
double x = 0;
double y0 = 0;
double yd0 = 0;
double x0 = 1;
int output1 = 0;
uint8_t outOfScreen = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

void setGraph(int output,int output1, uint8_t matrix[16][32], int8_t *colCount);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim3;
/* USER CODE BEGIN EV */

extern volatile uint8_t output;
extern uint8_t matrix[16][32];

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
  HAL_IncTick();
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
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
	counter++;
	if (HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin) && !btnState) {
		btnState = 1;
		if (update) {
			x += noise;
		}
	} else if (!HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin) && btnState){
		btnState = 0;
		if (update) {
			x -= noise;
		}
	}	
	
	if (counter >= 20) {
		if (update == 0) {
			y[1] = y0;
			yd[1] = y0;
			x = x0;
			ydd[1] = (K / T1) * x - (T2 / T1) * yd[1] - (1 / T1) * y[1];
		} else {
			y[0] = y[1];
			yd[0] = yd[1];
			ydd[0] = ydd[1];
			y[1] = y[0] + yd[0] * 0.1;
			yd[1] = yd[0] + ydd[0] * 0.1;
			ydd[1] = (K / T1) * x - (T2 / T1) * yd[1] - (1 / T1) * y[1];
		}
		
		counter = 0;
		update += 1;
		output1 = output;
		if (round(y[1]) <= -1) {
			outOfScreen = 1;
			output = 0;
		} else if (round(y[1]) >= 16) {
			outOfScreen = 1;
			output = 15;
		} else {
			output = round(y[1]);
			outOfScreen = 0;
		}
		setGraph(output, output1, matrix, &colCount);
	}
	time_ms += 5;
	
	if (btnState == 1) {
	} else {
	}
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles I2C1 event interrupt.
  */
void I2C1_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_EV_IRQn 0 */

  /* USER CODE END I2C1_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_EV_IRQn 1 */

  /* USER CODE END I2C1_EV_IRQn 1 */
}

/**
  * @brief This function handles I2C1 error interrupt.
  */
void I2C1_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_ER_IRQn 0 */

  /* USER CODE END I2C1_ER_IRQn 0 */
  HAL_I2C_ER_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_ER_IRQn 1 */

  /* USER CODE END I2C1_ER_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void setGraph(int output, int output1, uint8_t matrix[16][32], int8_t *colCount) {
	if ((*colCount) < 31) {
			(*colCount)++;
		if (*colCount == 31) {
			if (output > output1) {
				for (int i = output1; i <= output; i++) {
					matrix[i][*colCount - 1] = 1;
				}
			}
			if (output < output1) {
				for (int i = output1; i >= output; i--) {
					matrix[i][*colCount - 1] = 1;
				}
			}
			if (!outOfScreen) {
				matrix[output][*colCount] = 1;				
			}
		}			
	}
	if ((*colCount) >= 31) {
		for (uint8_t i = 0; i < 31; i++) {
			for (uint8_t j = 0; j < 16; j++) {
				matrix[j][i] = matrix[j][i + 1];
				if (i == 30) {
					matrix[j][i + 1] = 0;
				}
			}
		}
	}
	if (output > output1) {
		for (int i = output1; i <= output; i++) {
			matrix[i][*colCount - 1] = 1;
		}
	}
	if (output < output1) {
		for (int i = output1; i >= output; i--) {
			matrix[i][*colCount - 1] = 1;
		}
	}
	if (!outOfScreen) {
		matrix[output][*colCount] = 1;				
	}
}

/* USER CODE END 1 */
