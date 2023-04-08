/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "common.h"
#include "rosmain.h"
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
/* Definitions for MainTask */
osThreadId_t MainTaskHandle;
const osThreadAttr_t MainTask_attributes = {
  .name = "MainTask",
  .stack_size = 4000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CalcTask */
osThreadId_t CalcTaskHandle;
const osThreadAttr_t CalcTask_attributes = {
  .name = "CalcTask",
  .stack_size = 8000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ServoTask */
osThreadId_t ServoTaskHandle;
const osThreadAttr_t ServoTask_attributes = {
  .name = "ServoTask",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CalcSem */
osSemaphoreId_t CalcSemHandle;
const osSemaphoreAttr_t CalcSem_attributes = {
  .name = "CalcSem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void MainFunc(void *argument);
void CalcFunc(void *argument);
void ServoFunc(void *argument);

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

  /* Create the semaphores(s) */
  /* creation of CalcSem */
  CalcSemHandle = osSemaphoreNew(1, 1, &CalcSem_attributes);

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
  /* creation of MainTask */
  MainTaskHandle = osThreadNew(MainFunc, NULL, &MainTask_attributes);

  /* creation of CalcTask */
  CalcTaskHandle = osThreadNew(CalcFunc, NULL, &CalcTask_attributes);

  /* creation of ServoTask */
  ServoTaskHandle = osThreadNew(ServoFunc, NULL, &ServoTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_MainFunc */
/**
 * @brief  Function implementing the MainTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_MainFunc */
void MainFunc(void *argument)
{
  /* USER CODE BEGIN MainFunc */
	/* Infinite loop */
	SysInit();
	int stage = 0;
	uint32_t stage_update = HAL_GetTick();
	x_offset = 1.0;
//	setup();
	for(;;)
	{
//		ChatterLoop();
//				vTaskSuspendAll();
//				ServoDriverSetPWM(&servodriver, FLHip, 250, 0);
//				ServoDriverSetPWM(&servodriver, FLKnee, FLKneemin, 0);
//				HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, SET);
//				xTaskResumeAll();

		//		SetSmoothSpeed(0.5);
//				osDelay(2000);
//				vTaskSuspendAll();
////				ServoDriverSetPWM(&servodriver, FLHip, 500, 0);
//				ServoDriverSetPWM(&servodriver, FLKnee, FLKneemax, 0);
//				HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, RESET);
//				xTaskResumeAll();
		//		SetSmoothSpeed(0.1);
//				osDelay(2000);


		//
		//		switch(stage){
		//		case 0:
		//			height = 0.14;
		//			x_distance = 0.0;
		//			stage = 1;
		//			break;
		//		case 1:
		//			height = 0.14;
		//			x_distance = 0.02;
		//			stage = 2;
		//			break;
		//		case 2:
		//			height = 0.16;
		//			x_distance = 0.02;
		//			stage = 3;
		//			break;
		//		case 3:
		//			height = 0.16;
		//			x_distance = -0.02;
		//			stage = 4;
		//			break;
		//		case 4:
		//			height = 0.14;
		//			x_distance = -0.02;
		//			stage = 0;
		//			break;
		//		}


//		if(HAL_GetTick() - stage_update > 50){
//			if(x_offset != 0.0 || y_offset != 0.0 || yaw_offset != 0.0){
//				switch (stage){
//				case 0:
//					LegSetOffsets(&legFL, 0.0, 0.0, -step_height, 0.0, 0.0, 0.0);
////					LegSetOffsets(&legFR, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
////					LegSetOffsets(&legBL, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
////					LegSetOffsets(&legBR, 0.0, 0.0, -step_height, 0.0, 0.0, 0.0);
//					stage = 1;
//					break;
//				case 1:
//					LegSetOffsets(&legFL, step_length*x_offset, step_length*y_offset, -step_height, 0.0, 0.0, yaw_offset);
////					LegSetOffsets(&legFR, -(step_length*x_offset), -(step_length*y_offset), 0.0, 0.0, 0.0, -yaw_offset);
////					LegSetOffsets(&legBL, -(step_length*x_offset), -(step_length*y_offset), 0.0, 0.0, 0.0, -yaw_offset);
////					LegSetOffsets(&legBR, step_length*x_offset, step_length*y_offset, -step_height, 0.0, 0.0, yaw_offset);
//					stage = 2;
//					break;
//				case 2:
//					LegSetOffsets(&legFL, step_length*x_offset, step_length*y_offset, 0.0, 0.0, 0.0, yaw_offset);
////					LegSetOffsets(&legFR, -(step_length*x_offset), -(step_length*y_offset), 0.0, 0.0, 0.0, -yaw_offset);
////					LegSetOffsets(&legBL, -(step_length*x_offset), -(step_length*y_offset), 0.0, 0.0, 0.0, -yaw_offset);
////					LegSetOffsets(&legBR, step_length*x_offset, step_length*y_offset, 0.0, 0.0, 0.0, yaw_offset);
//					stage = 3;
//					break;
//				case 3:
//					LegSetOffsets(&legFL, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
////					LegSetOffsets(&legFR, 0.0, 0.0, -step_height, 0.0, 0.0, 0.0);
////					LegSetOffsets(&legBL, 0.0, 0.0, -step_height, 0.0, 0.0, 0.0);
////					LegSetOffsets(&legBR, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
//					stage = 4;
//					break;
//				case 4:
//					LegSetOffsets(&legFL, -(step_length*x_offset), -(step_length*y_offset), 0.0, 0.0, 0.0, -yaw_offset);
////					LegSetOffsets(&legFR, (step_length*x_offset), (step_length*y_offset), -step_height, 0.0, 0.0, yaw_offset);
////					LegSetOffsets(&legBL, (step_length*x_offset), (step_length*y_offset), -step_height, 0.0, 0.0, yaw_offset);
////					LegSetOffsets(&legBR, -(step_length*x_offset), -(step_length*y_offset), 0.0, 0.0, 0.0, -yaw_offset);
//					stage = 5;
//					break;
//				case 5:
//					LegSetOffsets(&legFL, -(step_length*x_offset), -(step_length*y_offset), 0.0, 0.0, 0.0, -yaw_offset);
////					LegSetOffsets(&legFR, (step_length*x_offset), (step_length*y_offset), 0.0, 0.0, 0.0, yaw_offset);
////					LegSetOffsets(&legBL, (step_length*x_offset), (step_length*y_offset), 0.0, 0.0, 0.0, yaw_offset);
////					LegSetOffsets(&legBR, -(step_length*x_offset), -(step_length*y_offset), 0.0, 0.0, 0.0, -yaw_offset);
//					stage = 0;
//					break;
//				}
//			}else{
//				LegSetOffsets(&legFL, 0.0, 0.0, -step_height, 0.0, 0.0, 0.0);
//				LegSetOffsets(&legFR, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
//				LegSetOffsets(&legBL, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
//				LegSetOffsets(&legBR, 0.0, 0.0, -step_height, 0.0, 0.0, 0.0);
//				stage = 0;
//			}
//			stage_update = HAL_GetTick();
////			HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
//
//		}

//		sprintf(data,"value 1: %d \r\n",ADCValue[0]);
//		HAL_UART_Transmit(&huart3, data, strlen(data), 100);
//		LegSetOffsets(&legFL, 0.0, 0.0, -step_height, 0.0, 0.0, 0.0);
		osDelay(50);
//		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
//		LegSetOffsets(&legFL, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
//		osDelay(20);



	}
  /* USER CODE END MainFunc */
}

/* USER CODE BEGIN Header_CalcFunc */
/**
 * @brief Function implementing the CalcTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_CalcFunc */
void CalcFunc(void *argument)
{
  /* USER CODE BEGIN CalcFunc */
	/* Infinite loop */
	for(;;)
	{
		osSemaphoreAcquire(CalcSemHandle, osWaitForever);
//		vTaskSuspendAll();
		ControlLoop();
		LegComply(&legFL,ADCValue[0]);
		CompPitchRoll(&imu);
//		xTaskResumeAll();
		HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
//		HAL_Delay(5);

	}
  /* USER CODE END CalcFunc */
}

/* USER CODE BEGIN Header_ServoFunc */
/**
 * @brief Function implementing the ServoSmoother thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_ServoFunc */
void ServoFunc(void *argument)
{
  /* USER CODE BEGIN ServoFunc */
	/* Infinite loop */
	for(;;)
	{
		LegStepLoop(&legFL);
		//		LegStepLoop(&legFR);
		//		LegStepLoop(&legBL);
		//		LegStepLoop(&legBR);
		//
		vTaskSuspendAll();
		ServoDriverSetPWM(&servodriver, FLHip, MAP_Angle2Pulse(legFL.hip_angle - legFL.comply_angle,144.0,0.0,FLHipmin,FLHipmax),0);
		ServoDriverSetPWM(&servodriver, FLKnee, MAP_Angle2Pulse(legFL.knee_angle - legFL.comply_angle,23.0,97.0,FLKneemin,FLKneemax),0);
		xTaskResumeAll();
		osDelay(smoothdelay);
//		HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);


	}
  /* USER CODE END ServoFunc */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if(htim->Instance == TIM6){
		osSemaphoreRelease(CalcSemHandle);

	}else if (htim->Instance == TIM7) {
		HAL_IncTick();
	}
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	HAL_UART_Receive_IT(&huart3, &control, 1);
//}

void HAL_I2C_MasterRxCpltCallback (I2C_HandleTypeDef * hi2c)
{
	if(hi2c->Instance == I2C2){
		MPUHandlebuff(&imu);
//		QMCGetYaw(&qmc,  imu->pitch, imu->roll);
		MPUReqAccGyro(&imu);
	}
}

/* USER CODE END Application */

