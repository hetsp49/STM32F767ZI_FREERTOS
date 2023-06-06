/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "queue.h"
#include <stdio.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* Define an enumerated type used to identify the source of the data. */
typedef enum
{
eSender1,
eSender2
} DataSource_t;
/* Define the structure type that will be passed on the queue. */
typedef struct
{
uint8_t ucValue;
DataSource_t eDataSource;
} Data_t;
/* Declare two variables of type Data_t that will be passed on the queue. */
static const Data_t xStructsToSend[ 2 ] =
{
{ 100, eSender1 }, /* Used by Sender1. */
{ 200, eSender2 } /* Used by Sender2. */
};
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

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);

/* USER CODE BEGIN PFP */

QueueHandle_t xQueue;
//int data = 0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	 xQueue = xQueueCreate(5,sizeof(int));
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  if(xQueue != NULL){
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, &xStructsToSend[0], &myTask02_attributes);

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(StartTask03, &xStructsToSend[1], &myTask03_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();
  }
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, gled_Pin|rled_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : gled_Pin rled_Pin */
  GPIO_InitStruct.Pin = gled_Pin|rled_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */

  /* Infinite loop */
//  while(1)
//  {
//	  xQueueSend(xQueue, &data, 0);
//	  printf("Sender Task: Sent data %d\r\n", data);
//	  data++;
//	  HAL_GPIO_TogglePin(gled_GPIO_Port, gled_Pin);
//	  HAL_Delay(1000);
//	  vTaskDelay(pdMS_TO_TICKS(1000));
//  }

	// RECEIVER TASK :
//    int receivedData;
//
//    while (1) {
//        // Receive data from the queue
//        if (xQueueReceive(xQueue, &receivedData, portMAX_DELAY) == pdTRUE) {
//            printf("Receiver Task: Received data %d \r\n", receivedData);
//        }
//    }

	Data_t xReceivedStructure;
	BaseType_t xStatus;
	/* This task is also defined within an infinite loop. */
	for( ;; )
	{
	/* Because it has the lowest priority this task will only run when the
	sending tasks are in the Blocked state. The sending tasks will only enter
	the Blocked state when the queue is full so this task always expects the
	number of items in the queue to be equal to the queue length, which is 3 in
	this case. */
	if( uxQueueMessagesWaiting( xQueue ) != 3 )
	{
	printf( "Queue should have been full!\r\n" );
	}
	/* Receive from the queue.
	The second parameter is the buffer into which the received data will be
	placed. In this case the buffer is simply the address of a variable that
	has the required size to hold the received structure.
	The last parameter is the block time - the maximum amount of time that the
	task will remain in the Blocked state to wait for data to be available
	if the queue is already empty. In this case a block time is not necessary
	because this task will only run when the queue is full. */
	xStatus = xQueueReceive( xQueue, &xReceivedStructure, 100 );
	if( xStatus == pdPASS )
	{
	/* Data was successfully received from the queue, print out the received
	value and the source of the value. */
	if( xReceivedStructure.eDataSource == eSender1 )
	{
	printf( "From Sender 1 = %d \r\n ", xReceivedStructure.ucValue );
	}
	else
	{
	printf( "From Sender 2 = %d \r\n ", xReceivedStructure.ucValue );
	}
	}
	else
	{
	/* Nothing was received from the queue. This must be an error as this
	task should only run when the queue is full. */
	printf( "Could not receive from the queue.\r\n" );
	}
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
	//int receivedData;
  /* Infinite loop */
//  while(1)
//  {
//	  if (xQueueReceive(xQueue, &data, portMAX_DELAY) == pdTRUE)
//	  {
//	              printf("Receiver Task: Received data %d\n", data);
//	  }
////	  HAL_GPIO_TogglePin(rled_GPIO_Port, rled_Pin);
////	  //vTaskDelay(100);
////	  HAL_Delay(100);
//  }

	// SENDER TASK :
//	 int data = 1;
//
//	    while (1) {
//	        // Send data to the queue
//	    	if(xQueueSend(xQueue, &data, 0) == pdTRUE )
//	    	{
//	        printf("Sender Task 1: Sent data %d \r\n", data);
//
//	        // Increment the data value
//	        data++;
//	    	}
//	        // Delay before sending the next data
//	        vTaskDelay(1000);
//	    }

	BaseType_t xStatus;
	const TickType_t xTicksToWait = pdMS_TO_TICKS( 100 );
	/* As per most tasks, this task is implemented within an infinite loop. */
	for( ;; )
	{
	/* Send to the queue.
	The second parameter is the address of the structure being sent. The
	address is passed in as the task parameter so pvParameters is used
	directly.
	The third parameter is the Block time - the time the task should be kept
	in the Blocked state to wait for space to become available on the queue
	if the queue is already full. A block time is specified because the
	sending tasks have a higher priority than the receiving task so the queue
	is expected to become full. The receiving task will remove items from
	the queue when both sending tasks are in the Blocked state. */
	xStatus = xQueueSendToBack( xQueue, argument, xTicksToWait );
	if( xStatus != pdPASS )
	{
	/* The send operation could not complete, even after waiting for 100ms.
	This must be an error as the receiving task should make space in the
	queue as soon as both sending tasks are in the Blocked state. */
	printf( "Could not send to the queue.\r\n" );
	}
	}

  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{

  /* USER CODE BEGIN StartTask03
   Infinite loop
  for(;;)
  {
    osDelay(1);
  }
   USER CODE END StartTask03 */
	// SENDER TASK :
//	int data = 100;
//
//	    while (1) {
//	        // Send data to the queue
//	        if(xQueueSend(xQueue, &data, 0)==pdTRUE){
//	        printf("Sender Task 2: Sent data %d \r\n", data);
//
//	        // Increment the data value
//	        data += 10;
//	        }
//	        // Delay before sending the next data
//	        vTaskDelay(1000);
//	    }
	BaseType_t xStatus;
		const TickType_t xTicksToWait = pdMS_TO_TICKS( 100 );
		/* As per most tasks, this task is implemented within an infinite loop. */
		for( ;; )
		{
		/* Send to the queue.
		The second parameter is the address of the structure being sent. The
		address is passed in as the task parameter so pvParameters is used
		directly.
		The third parameter is the Block time - the time the task should be kept
		in the Blocked state to wait for space to become available on the queue
		if the queue is already full. A block time is specified because the
		sending tasks have a higher priority than the receiving task so the queue
		is expected to become full. The receiving task will remove items from
		the queue when both sending tasks are in the Blocked state. */
		xStatus = xQueueSendToBack( xQueue, argument, xTicksToWait );
		if( xStatus != pdPASS )
		{
		/* The send operation could not complete, even after waiting for 100ms.
		This must be an error as the receiving task should make space in the
		queue as soon as both sending tasks are in the Blocked state. */
		printf( "Could not send to the queue.\r\n" );
		}
		}
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
