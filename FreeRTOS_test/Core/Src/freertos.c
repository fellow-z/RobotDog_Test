/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stdio.h"
#include "string.h"
#include "stdlib.h"

#include "sys.h"
#include "robotcmd.h"
#include "body_task.h"
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
/* Definitions for RobotCmd */
osThreadId_t RobotCmdHandle;
const osThreadAttr_t RobotCmd_attributes = {
  .name = "RobotCmd",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for Imu */
osThreadId_t ImuHandle;
const osThreadAttr_t Imu_attributes = {
  .name = "Imu",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for Body */
osThreadId_t BodyHandle;
const osThreadAttr_t Body_attributes = {
  .name = "Body",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4
};
/* Definitions for Remote */
osThreadId_t RemoteHandle;
const osThreadAttr_t Remote_attributes = {
  .name = "Remote",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for SendData */
osThreadId_t SendDataHandle;
const osThreadAttr_t SendData_attributes = {
  .name = "SendData",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for led */
osTimerId_t ledHandle;
const osTimerAttr_t led_attributes = {
  .name = "led"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void robotcmd_task(void *argument);
void imu_task(void *argument);
void body_task(void *argument);
void remote_task(void *argument);
void SendDataTask(void *argument);
void led_timer(void *argument);

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

  /* Create the timer(s) */
  /* creation of led */
  ledHandle = osTimerNew(led_timer, osTimerPeriodic, NULL, &led_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
	osTimerStart(ledHandle,500);
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of RobotCmd */
  RobotCmdHandle = osThreadNew(robotcmd_task, NULL, &RobotCmd_attributes);

  /* creation of Imu */
  ImuHandle = osThreadNew(imu_task, NULL, &Imu_attributes);

  /* creation of Body */
  BodyHandle = osThreadNew(body_task, NULL, &Body_attributes);

  /* creation of Remote */
  RemoteHandle = osThreadNew(remote_task, NULL, &Remote_attributes);

  /* creation of SendData */
  SendDataHandle = osThreadNew(SendDataTask, NULL, &SendData_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_robotcmd_task */
/**
  * @brief  Function implementing the RobotCmd thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_robotcmd_task */
void robotcmd_task(void *argument)
{
  /* USER CODE BEGIN robotcmd_task */

  /* Infinite loop */
	for(;;)
	{
		LedStateChange();
		BodyParamChange();
		osDelay(1);
	}
  /* USER CODE END robotcmd_task */
}

/* USER CODE BEGIN Header_imu_task */
/**
* @brief Function implementing the Imu thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_imu_task */
void imu_task(void *argument)
{
  /* USER CODE BEGIN imu_task */
	DMP_Init();
  /* Infinite loop */
  for(;;)
  {
	  Read_DMP();
	  osDelay(1);
  }
  /* USER CODE END imu_task */
}

/* USER CODE BEGIN Header_body_task */
/**
* @brief Function implementing the Body thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_body_task */
void body_task(void *argument)
{
  /* USER CODE BEGIN body_task */
	BodyInit();
  /* Infinite loop */
  for(;;)
  {
	BodyChange();
	LegChange();
	ServoSendData();
    osDelay(2);
  }
  /* USER CODE END body_task */
}

/* USER CODE BEGIN Header_remote_task */
/**
* @brief Function implementing the Remote thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_remote_task */
void remote_task(void *argument)
{
  /* USER CODE BEGIN remote_task */
	
  /* Infinite loop */
  for(;;)
  {
	remote.last_value = remote.value;
	remote.value.left_x = 0;
	remote.value.right_x = 0;
	remote.value.right_y = 255;
	remote.value.key_1 = 0;
	remote.value.key_2 = 0;
	remote.value.key_3 = 0;
	remote.value.key_4 = 0;
	remote.value.key_5 = 0;
	remote.value.key_6 = 1;
	
	if(remote.value.left_x != 0 || remote.value.right_x != 0 || remote.value.right_y != 0)
	{
		remote.state = 1;
	}
	else
	{
		remote.state = 0;
	}
	osDelay(1);
  }
  /* USER CODE END remote_task */
}

/* USER CODE BEGIN Header_SendDataTask */
/**
* @brief Function implementing the SendData thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SendDataTask */
void SendDataTask(void *argument)
{
  /* USER CODE BEGIN SendDataTask */
	osDelay(2000);
  /* Infinite loop */
  for(;;)
  {
	printf("Yaw:%f\t Pitch:%f\t Roll:%f\n",Yaw,Pitch,Roll);
	printf("%d\t\t %d\t\t %d\n",FR_Leg.position[0],FR_Leg.position[1],FR_Leg.position[2]);
	printf("\n\n\n\n\n\n\n\n\n\n\n\n\n");
	osDelay(100);
  }	
  /* USER CODE END SendDataTask */
}

/* led_timer function */
void led_timer(void *argument)
{
  /* USER CODE BEGIN led_timer */
	HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
  /* USER CODE END led_timer */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
