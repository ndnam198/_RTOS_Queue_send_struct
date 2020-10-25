/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
	Sử dụng queue để truyền đi struct với trường eDataSource_t thể hiện nguồn của data đến từ các luồng khác nhau.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "queue.h"
#include "timers.h"
/* USER CODE END Includes */
#include "myLib.h"
#include "myRTOSaddons.h"
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  eChargePoint_1 = 0x00U,
  eChargePoint_2,
  eChargePoint_3,
  eChargePoint_4,
  eChargePoint_5,
  eChargePoint_6,
  eGateway
} eDataSource_t;

typedef struct
{
  eDataSource_t eDataSource;
  uint32_t ulValue;
} DataFrame_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define QUEUE_SIZE (7UL)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
char main_string[100];
/* Definitions for defaultTask */

/* Definitions for blinkLed1 */
osThreadId_t blinkLed1Handle;
const osThreadAttr_t blinkLed1_attributes = {
    .name = "blinkLed1",
    .priority = (osPriority_t)osPriorityLow3,
    .stack_size = 32 * 4};

/* Definitions Sender Task*/
osThreadId_t SenderHandle1;
osThreadId_t SenderHandle2;
osThreadId_t SenderHandle3;
osThreadId_t SenderHandle4;
osThreadId_t SenderHandle5;
osThreadId_t SenderHandle6;
osThreadId_t GatewayHandle;
const osThreadAttr_t Sender_attributes = {
    .name = "Sender",
    .priority = (osPriority_t)osPriorityLow5, /* Senser has the higher priority so Queue's always full of data */
    .stack_size = 128 * 4};

/* Definitions Receiver Task*/
osThreadId_t ReceiverHandle;
const osThreadAttr_t Receiver_attributes = {
    .name = "Receiver",
    .priority = (osPriority_t)osPriorityLow4,
    .stack_size = 128 * 4};

/* USER CODE BEGIN PV */
osMessageQueueId_t queue_handle = NULL;

uint32_t error_count = 0;

/**
 * @brief Create an array with DataFrame_t type
 * 
 */
DataFrame_t DataStructToSend[7] = {
    {eChargePoint_1, 1},
    {eChargePoint_2, 2},
    {eChargePoint_3, 3},
    {eChargePoint_4, 4},
    {eChargePoint_5, 5},
    {eChargePoint_6, 6},
    {eGateway, 15},
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void blinkTask(void *argument);
void senderTask(void *argument);
void receiverTask(void *argument);
/* USER CODE BEGIN PFP */

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);
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
  queue_handle = osMessageQueueNew(QUEUE_SIZE, sizeof(DataFrame_t), NULL);
  if (queue_handle == NULL)
  {
    error_count++;
    PRINTF("Failed to create new queue\r\n");
  }
  /* USER CODE END RTOS_QUEUES */
  /* creation of blinkLed1 */
  blinkLed1Handle = osThreadNew(blinkTask, NULL, &blinkLed1_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* creation of Sender task*/

  SenderHandle1 = osThreadNew(senderTask, (void *)DataStructToSend, &Sender_attributes);
  SenderHandle2 = osThreadNew(senderTask, (void *)(DataStructToSend + 1), &Sender_attributes);
  SenderHandle3 = osThreadNew(senderTask, (void *)(DataStructToSend + 2), &Sender_attributes);
  SenderHandle4 = osThreadNew(senderTask, (void *)(DataStructToSend + 3), &Sender_attributes);
  SenderHandle5 = osThreadNew(senderTask, (void *)(DataStructToSend + 4), &Sender_attributes);
  SenderHandle6 = osThreadNew(senderTask, (void *)(DataStructToSend + 5), &Sender_attributes);
  GatewayHandle = osThreadNew(senderTask, (void *)(DataStructToSend + 6), &Sender_attributes);

  //Check if all tasks are created successful or not
  if ((SenderHandle1 == NULL) || (SenderHandle2 == NULL) || (SenderHandle3 == NULL) || (SenderHandle4 == NULL) || (SenderHandle5 == NULL) || (SenderHandle6 == NULL) || (GatewayHandle == NULL))
  {
    PRINTF("Failed to create senderTask\r\n");
    error_count++;
  }

  /* creation of Receiver task */
  ReceiverHandle = osThreadNew(receiverTask, NULL, &Receiver_attributes);
  if (ReceiverHandle == NULL)
  {
    PRINTF("Failed to create receiverTask");
    error_count++;
  }
  /* add threads, ... */

  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  //If any error is counted, Kernel will not start
  if (error_count != 0)
  {
    PRINTF("Failed to start Kernel\r\n");
    PRINT_VAR(error_count);
  }
  else
  {
    PRINTF("Start RTOS_Kernel\r\n");
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 | GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA5 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

void senderTask(void *argument)
{
  BaseType_t status;
  TickType_t send_freq = pdMS_TO_TICKS(500);
  TickType_t block_time = pdMS_TO_TICKS(3000);
  DataFrame_t *dtDataToSend = (DataFrame_t *)argument;

  /* Infinite loop */
  for (;;)
  {
    newline;
    PRINT_VAR_IN_TASK(send_freq);
    send_freq = RAND_U32(300, 3000);
    sprintf(main_string, "ChargePoint number %d starts transfering data\r\n", dtDataToSend->eDataSource);
    PRINT_IN_TASK(main_string);
    status = xQueueSendToBack(queue_handle, (void *)dtDataToSend, block_time); /* If the queue is full, senderTask will be blocked for maximum block_time before return fail */
    if (status != pdPASS)
    {
      sprintf(main_string, "ChargePoint number %d failed to transfer data\r\n", dtDataToSend->eDataSource);
      PRINT_IN_TASK(main_string);
    }

    osDelay(send_freq);
  }
}

/*receiverTask */
/**

*/
void receiverTask(void *argument)
{
  DataFrame_t dtReceivedDataFrame; /* Create a variable to store data received from queue */
  BaseType_t status;
  TickType_t receive_freq = pdMS_TO_TICKS(500);
  uint8_t nb_data_on_queue = 0;
  /* Infinite loop */
  for (;;)
  {
    newline;
    //Check number of data already on queue
    nb_data_on_queue = (uint8_t)uxQueueMessagesWaiting(queue_handle);
    if (nb_data_on_queue != 7)
    {
      PRINT_IN_TASK("Queue is not full\r\n");
      newline;
    }

    status = xQueueReceive(queue_handle, &dtReceivedDataFrame, 0);
    if (status != pdPASS)
    {
      PRINT_IN_TASK("Receive data failed!\r\n");
    }
    else /* When receive data successfully */
    {
      switch (dtReceivedDataFrame.eDataSource)
      {
      case (eChargePoint_1):
        sprintf((main_string), "Received data from ChargePoint %d - Data = %d\n\n\r", dtReceivedDataFrame.eDataSource + 1, (int)dtReceivedDataFrame.ulValue);
        break;
      case (eChargePoint_2):
        sprintf((main_string), "Received data from ChargePoint %d - Data = %d\n\n\r", dtReceivedDataFrame.eDataSource + 1, (int)dtReceivedDataFrame.ulValue);
        break;
      case (eChargePoint_3):
        sprintf((main_string), "Received data from ChargePoint %d - Data = %d\n\n\r", dtReceivedDataFrame.eDataSource + 1, (int)dtReceivedDataFrame.ulValue);
        break;
      case (eChargePoint_4):
        sprintf((main_string), "Received data from ChargePoint %d - Data = %d\n\n\r", dtReceivedDataFrame.eDataSource + 1, (int)dtReceivedDataFrame.ulValue);
        break;
      case (eChargePoint_5):
        sprintf((main_string), "Received data from ChargePoint %d - Data = %d\n\n\r", dtReceivedDataFrame.eDataSource + 1, (int)dtReceivedDataFrame.ulValue);
        break;
      case (eChargePoint_6):
        sprintf((main_string), "Received data from ChargePoint %d - Data = %d\n\n\r", dtReceivedDataFrame.eDataSource + 1, (int)dtReceivedDataFrame.ulValue);
        break;
      case (eGateway):
        sprintf((main_string), "Received data from GateWay - Data = %d\n\n\r", (int)dtReceivedDataFrame.ulValue);
        break;
      default:
        sprintf((main_string), "Unknown data %d from %d\n\n\r", (int)dtReceivedDataFrame.ulValue, dtReceivedDataFrame.eDataSource);
        break;
      }
      PRINT_IN_TASK(main_string);
    }
    osDelay(receive_freq);
  }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_blinkTask */
/**
* @brief Function implementing the blinkLed2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_blinkTask */
void blinkTask(void *argument)
{
  /* USER CODE BEGIN blinkTask */
  /* Infinite loop */
  for (;;)
  {
    toggleLed4;
    osDelay(1000);
  }
  /* USER CODE END blinkTask */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
