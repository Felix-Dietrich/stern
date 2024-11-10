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
#include "adc.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STAR_VOLTAGE_V 12
#define STAR_CURRENT_A 0.1
#define STAR_ON_TIME_H 5
#define STAR_TURN_ON_THRESHOLD_LUX 100
#define STAR_MINIMUM_DAY_BRIGHTNESS_LUX 500
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
enum Charging_Status
		{
			notCharge = 0,
			preCharge = 1,
			fastCharge = 2,
			terminatedCharge = 3
		};
enum Charging_Status ChargingStatus = notCharge;

uint8_t request_poweroff = 0;

float brightness_lux = 1000;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId batteryTaskHandle;
osThreadId CyclerTaskHandle;
osThreadId LuxTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartBatteryTask(void const * argument);
void StartCyclerTask(void const * argument);
void StartLuxTask(void const * argument);

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of batteryTask */
  osThreadDef(batteryTask, StartBatteryTask, osPriorityLow, 0, 128);
  batteryTaskHandle = osThreadCreate(osThread(batteryTask), NULL);

  /* definition and creation of CyclerTask */
  osThreadDef(CyclerTask, StartCyclerTask, osPriorityLow, 0, 128);
  CyclerTaskHandle = osThreadCreate(osThread(CyclerTask), NULL);

  /* definition and creation of LuxTask */
  osThreadDef(LuxTask, StartLuxTask, osPriorityLow, 0, 128);
  LuxTaskHandle = osThreadCreate(osThread(LuxTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0*312); //Spannung einstellen
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0*625); //Strom einstellen
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);


	HAL_COMP_Start(&hcomp1);
	HAL_COMP_Start(&hcomp2);
	HAL_TIM_PWM_Start(&htim1, HAL_TIM_ACTIVE_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, HAL_TIM_ACTIVE_CHANNEL_3);
	__HAL_TIM_SET_COMPARE(&htim1,HAL_TIM_ACTIVE_CHANNEL_2,11);
	__HAL_TIM_SET_COMPARE(&htim1,HAL_TIM_ACTIVE_CHANNEL_3,11);

	HAL_GPIO_TogglePin(GREEN_GPIO_Port, GREEN_Pin);
	//osDelay(200);

	/* Infinite loop */
	for(;;)
	{
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
void StartBatteryTask(void const * argument)
{
  /* USER CODE BEGIN StartBatteryTask */
	uint8_t deviceAddress = 0x6A << 1;  // Shift for 7-bit address
	uint8_t regAddr = 0x00;
	uint8_t readData[0x15] = {0};
	HAL_StatusTypeDef status;
	uint8_t writeData;
	uint8_t lowBattery = 0;

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

		if(request_poweroff)
		{
			request_poweroff = 0;
			writeData = 0b01100100;  //force batfet off without delay
			HAL_I2C_Mem_Write(&hi2c1, deviceAddress, 0x09, I2C_MEMADD_SIZE_8BIT, &writeData, 1, HAL_MAX_DELAY);
		}

		status = HAL_I2C_Mem_Read(&hi2c1, deviceAddress, regAddr, I2C_MEMADD_SIZE_8BIT, readData, 0x15, HAL_MAX_DELAY);

   	  	ChargingStatus = (readData[0x0B] & 0b00011000) >> 3;

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
		uint16_t VBUS_mV = 2600 + (readData[0x11]&0b01111111)*100;
		uint16_t ChargingCurrent_mA = readData[0x12]*50;
		uint16_t BatteryVoltage_mV = ((readData[0x0E]& 0b01111111)*20)+2304;
		if(BatteryVoltage_mV > 3700)
		{
			lowBattery = 0;

		}
		else if(BatteryVoltage_mV < 3500)
		{
			lowBattery = 1;

		}

		if(lowBattery)
		{
			HAL_GPIO_TogglePin(RED_GPIO_Port, RED_Pin); 				//rot blinken
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (STAR_CURRENT_A/4)*625);
		}
		else
		{
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, STAR_CURRENT_A*625); //Strom einstellen
			HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_RESET); 	//LED aus
		}

		osDelay(500);
	}

  /* USER CODE END StartBatteryTask */
}

/* USER CODE BEGIN Header_StartCyclerTask */
/**
* @brief Function implementing the CyclerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCyclerTask */
void StartCyclerTask(void const * argument)
{
  /* USER CODE BEGIN StartCyclerTask */
	HAL_ADC_ConfigChannel(&hadc1, ADC_CHANNEL_0_NUMBER);
	HAL_ADC_Start(&hadc1);
	float voltage = 0;

	for(; voltage < STAR_VOLTAGE_V; voltage +=0.05) //test Star on startup
	{
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, voltage*312);
		osDelay(10);
	}
	for(; voltage > 0.05; voltage -=0.05)
	{
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, voltage*312);
		osDelay(10);
	}


	/* Infinite loop */
	for(;;)
	{/*
		 *
		 * # todo turn completely off when dark for 5 days
		 * # todo detect plug in of the Star and turn then on when dark
		 * # todo turn off when brighter than 500 lux
		*/

		enum Status
		{
			ON,
			TURN_OFF,
			OFF_NIGHT,
			OFF_DAY
		};
		static enum Status status = OFF_DAY;
		static float voltage_V = 0;
		static uint32_t ontime_ms  = 0;
		static uint32_t offtime_ms = 0;

		switch(status)
		{
			case ON:
			{
				if(voltage_V < STAR_VOLTAGE_V)
				{
					voltage_V+= 0.02;
					HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (uint32_t)(voltage_V*312));
				}
				ontime_ms+=10;
				offtime_ms = 0;

				//next state logic
				//--------------------------
				if(ontime_ms >= (STAR_ON_TIME_H*60*60*1000))
				{
					status = TURN_OFF;
				}
				if(brightness_lux > STAR_MINIMUM_DAY_BRIGHTNESS_LUX)
				{
					status = TURN_OFF;
				}

				break;
			}
			case TURN_OFF:
			{
				if(voltage_V >= 0.02)
				{
					voltage_V-= 0.02;
					HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (uint32_t)(voltage_V*312));
				}
				else
				{
					//next state logic
					//--------------------------
					status = OFF_NIGHT;

				}

				break;
			}
			case OFF_NIGHT:
			{
				//next state logic
				//--------------------------
				if(brightness_lux > STAR_MINIMUM_DAY_BRIGHTNESS_LUX)
				{
					status = OFF_DAY;
				}
				break;
			}
			case OFF_DAY:
			{

				//next state logic
				//--------------------------
				if(brightness_lux < STAR_TURN_ON_THRESHOLD_LUX)
				{
					ontime_ms  = 0;
					status = ON;
				}
				break;
			}
		}
		static enum Charging_Status lastChargingStatus;
		if((ChargingStatus != notCharge) && (lastChargingStatus == notCharge)) //eingesteckt zum laden
		{
			status = TURN_OFF;
		}
		if((ChargingStatus == notCharge) && (lastChargingStatus != notCharge)) //ausgesteckt vom Laden
		{
			status = OFF_DAY;
		}
		lastChargingStatus = ChargingStatus;

		offtime_ms+=10;
		if(offtime_ms >= 1000*60*60*24*5)
		{
			offtime_ms = 0;
			request_poweroff = 1;
		}
		osDelay(10);


	}
  /* USER CODE END StartCyclerTask */
}

/* USER CODE BEGIN Header_StartLuxTask */
/**
* @brief Measures brightenss and calculates a moving average of the brightness over a minute.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLuxTask */
void StartLuxTask(void const * argument)
{
  /* USER CODE BEGIN StartLuxTask */
	/* Infinite loop */
	for(;;)
	{
		static int second = 0;
		const float gamma = 0.6;  //gamma value of ldr aout of datasheet
		const float r_10 = 14000; //ldr resistance at L0
		const float r15 = 10000; //pull up resistor used in circuit
		const float vref = 3.3; //reference voltage
		const float L_0 = 10;
		float currentBrightness_lux = 0;
		static float brightnessValues_lux[60] = {0};

		float voltage = HAL_ADC_GetValue(&hadc1)*vref/(float)(1<<12);
		if(vref - voltage > 0.005) //avoid division by zero
		{
			float r_ldr = r15 * (voltage / (vref - voltage));
			currentBrightness_lux = L_0 * pow(r_ldr/r_10,-1.0/gamma);
		}
		brightnessValues_lux[second] = currentBrightness_lux;
		second++;
		if(second >=60)
		{
			second = 0;
		}
		float brightnessSum_lux = 0;
		for(int i = 0; i < 60; i++)
		{
			brightnessSum_lux += brightnessValues_lux[i];
		}
		brightness_lux = brightnessSum_lux/60;
		osDelay(1000);
	}
  /* USER CODE END StartLuxTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

