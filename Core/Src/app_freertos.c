/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dac.h"
#include "tim.h"
#include "comp.h"
#include "i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for batteryTask */
osThreadId_t batteryTaskHandle;
const osThreadAttr_t batteryTask_attributes = {
  .name = "batteryTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartBatteryTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of batteryTask */
  batteryTaskHandle = osThreadNew(StartBatteryTask, NULL, &batteryTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0*312); //Spannung einstellen
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0.1*625); //Strom einstellen
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);


	HAL_COMP_Start(&hcomp1);
	HAL_COMP_Start(&hcomp2);
	HAL_TIM_PWM_Start(&htim1, HAL_TIM_ACTIVE_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, HAL_TIM_ACTIVE_CHANNEL_3);
	__HAL_TIM_SET_COMPARE(&htim1,HAL_TIM_ACTIVE_CHANNEL_2,120);
	__HAL_TIM_SET_COMPARE(&htim1,HAL_TIM_ACTIVE_CHANNEL_3,120);

	HAL_GPIO_TogglePin(GREEN_GPIO_Port, GREEN_Pin);
	//osDelay(200);

	for(float voltage = 0; voltage < 12; voltage +=0.05)
	{
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, voltage*312);
		osDelay(1);
	}

  /* Infinite loop */
  for(;;)
  {

	  osDelay(2000);

	  HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_RESET);
	  osDelay(2000);

  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartBatteryTask */
/**
* @brief Function implementing the batteryTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBatteryTask */
void StartBatteryTask(void *argument)
{
  /* USER CODE BEGIN StartBatteryTask */
	uint8_t deviceAddress = 0x6A << 1;  // Shift for 7-bit address
	uint8_t regAddr = 0x00;
	uint8_t readData[0x15] = {0};
	HAL_StatusTypeDef status;
	uint8_t writeData;

	HAL_GPIO_WritePin(CHARGE_ENABLE_GPIO_Port, CHARGE_ENABLE_Pin, GPIO_PIN_SET);

	writeData = 0b00111111; // disable ILIM pin, set max current to 3.25A
	status = HAL_I2C_Mem_Write(&hi2c1, deviceAddress, 0x00, I2C_MEMADD_SIZE_8BIT, &writeData, 1, HAL_MAX_DELAY);
	if (status == HAL_OK)
	{

	}

	writeData = 0b11100000; //Start ADC_conversion
	status = HAL_I2C_Mem_Write(&hi2c1, deviceAddress, 0x02, I2C_MEMADD_SIZE_8BIT, &writeData, 1, HAL_MAX_DELAY);
	if (status == HAL_OK)
	{

	}

	writeData = 0b00011010; //otg disable
	status = HAL_I2C_Mem_Write(&hi2c1, deviceAddress, 0x03, I2C_MEMADD_SIZE_8BIT, &writeData, 1, HAL_MAX_DELAY);
	if (status == HAL_OK)
	{

	}


	writeData = 0b01001111; //fast charge current 5A
	status = HAL_I2C_Mem_Write(&hi2c1, deviceAddress, 0x04, I2C_MEMADD_SIZE_8BIT, &writeData, 1, HAL_MAX_DELAY);
	if (status == HAL_OK)
	{

	}

	writeData = 0b10001101; //disable watchdog
	status = HAL_I2C_Mem_Write(&hi2c1, deviceAddress, 0x07, I2C_MEMADD_SIZE_8BIT, &writeData, 1, HAL_MAX_DELAY);
	if (status == HAL_OK)
	{

	}

	HAL_GPIO_WritePin(CHARGE_ENABLE_GPIO_Port, CHARGE_ENABLE_Pin, GPIO_PIN_RESET);

	for(;;)
	{
		status = HAL_I2C_Mem_Read(&hi2c1, deviceAddress, regAddr, I2C_MEMADD_SIZE_8BIT, readData, 0x15, HAL_MAX_DELAY);
		if (status == HAL_OK)
		{

		}

		enum Charging_Status
		{
			notCharge = 0,
			preCharge = 1,
			fastCharge = 2,
			terminatedCharge = 3
		};
   	  	enum Charging_Status ChargingStatus = (readData[0x0B] & 0b00011000) >> 3;



		switch(ChargingStatus)
		{
		case notCharge:
			HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_RESET); 	//LED aus
			break;
		case preCharge:
			HAL_GPIO_TogglePin(GREEN_GPIO_Port, GREEN_Pin);					//grün blinken
			break;
		case fastCharge:
			HAL_GPIO_TogglePin(GREEN_GPIO_Port, GREEN_Pin);					//grün blinken
			break;
		case terminatedCharge:
			HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_SET);	//LED ein
			break;
		default:
			break;
		}

		osDelay(500);
	}

	/* USER CODE END StartBatteryTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

