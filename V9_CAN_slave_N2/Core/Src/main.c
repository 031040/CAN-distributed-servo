/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "ssd1306.h"
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
CAN_HandleTypeDef hcan;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* Definitions for my_Task */
osThreadId_t my_TaskHandle;
const osThreadAttr_t my_Task_attributes = {
  .name = "my_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
volatile int16_t g_enc = 0;
int16_t g_target_speed = 0;        // ?????(????30,???0)
int16_t g_last_enc = 0;
volatile int8_t g_pwm = 0;
volatile int16_t g_speed = 0;     // measured speed, read by DiagTask / CommTask
float g_integral = 0.0f;
int16_t g_last_error = 0;
volatile float Kp = 2.0f;                   // ??? P
volatile float Ki = 0.1f;                   // ??? I
volatile float Kd = 0.0f;                   // ??? D
volatile uint8_t g_stall = 0;      // stall latch: set by DiagTask, cleared by new "pos" command

//=== ????? ===
volatile int16_t g_target_position = 200;    // ????(??????,??)
volatile float Kp_pos = 0.2f;               // ??? P ??

#define PID_PERIOD_MS  20
#define PWM_MAX        49
#define INTEGRAL_LIMIT 500
#define MAX_SPEED       80          // ??????
#define DECEL_ZONE      150         // ?????(?????????)
#define DEAD_BAND       3           // ????(�3??????)
//=== Serial command channel: ISR -> double buffer -> semaphore -> CommTask ===
uint8_t rx_byte = 0;              // byte buffer for HAL_UART_Receive_IT (ISR context only)
char cmd_buf[2][64];              // double buffer: ISR fills, CommTask consumes
volatile uint8_t  cmd_idx   = 0;  // buffer index the ISR is currently filling
volatile uint16_t cmd_len[2] = {0, 0};
volatile uint8_t  cmd_ready[2] = {0, 0};  // 1 = a complete line is waiting in this buffer
SemaphoreHandle_t g_comm_sem = NULL;      // wakes CommTask (20ms timeout = telemetry cadence)

//=== CAN bus (stage 3, byte-layer test) ===
// 3-node build: this firmware is SLAVE #2 (board C, motor #1). Same source as
// the slave_pid firmware, recompiled with NODE_ID=2 -> it filters 0x102, sends
// feedback to 0x202, and is driven by the master's 0x102 target frame. Board B
// keeps NODE_ID=1 (V9_CAN_slave_pid, unchanged).
#define NODE_ID   2        // compile-time role: 0 = master, 1 = slave #1 (board B), 2 = slave #2 (board C)
#define NODE_COUNT 3       // protocol: 0x100+id master->node, 0x200+id node->master
volatile uint32_t g_can_rx_cnt = 0;   // frames received (count freezing => link down)
volatile uint32_t g_can_tx_cnt = 0;   // frames queued & ACKed (growing => link up)
volatile int16_t  g_remote_pos = 0;   // slave: latest target received from master
uint32_t g_can_mailbox = 0;           // mailbox slot returned by AddTxMessage
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG_Init(void);
static void MX_CAN_Init(void);
void StartmyTask(void *argument);

/* USER CODE BEGIN PFP */
void ControlTask(void *argument);
void CommTask(void *argument);
void DiagTask(void *argument);
void DisplayTask(void *argument);
void CANTask(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void uart_send_str(const char *s)
{
    while (*s)
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)s, 1, 10);
        s++;
    }
}
// ????,????
void uart_print_int(int16_t val)
{
    char buf[12];
    int i = 0;
    int neg = 0;

    if (val < 0)
    {
        neg = 1;
        val = -val;
    }

    if (val == 0)
    {
        buf[i++] = '0';
    }
    else
    {
        while (val > 0)
        {
            buf[i++] = '0' + (val % 10);
            val = val / 10;
        }
    }


    if (neg)
        buf[i++] = '-';

    // ??
    int start = 0, end = i - 1;
    while (start < end)
    {
        char tmp = buf[start];
        buf[start] = buf[end];
        buf[end] = tmp;
        start++;
        end--;
    }

    buf[i] = '\0';
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, i, 10);
}
void uart_print_enc(int16_t val)
{
    char buf[12];
    int i = 0;
    int neg = 0;

    if (val < 0)
    {
        neg = 1;
        val = -val;
    }

    if (val == 0)
    {
        buf[i++] = '0';
    }
    else
    {
        while (val > 0)
        {
            buf[i++] = '0' + (val % 10);
            val = val / 10;
        }
    }

    if (neg)
        buf[i++] = '-';

    // ??????
    int start = 0, end = i - 1;
    while (start < end)
    {
        char tmp = buf[start];
        buf[start] = buf[end];
        buf[end] = tmp;
        start++;
        end--;
    }

    buf[i++] = '\r';
    buf[i++] = '\n';
    buf[i] = '\0';

    HAL_UART_Transmit(&huart1, (uint8_t *)buf, i, 10);
}

//==== Serial command handling (USART1 RX interrupt) ====

// Parse and execute one received command line.
// Runs in CommTask context (a task, not an ISR). Returns 1 if recognized.
// Sets control variables only; all UART TX happens in CommTask.
static int process_cmd(char *cmd)
{
    char *p = cmd;
    while (*p == ' ') p++;   // skip leading spaces

    if (strncmp(p, "Kp_pos", 6) == 0)      // position-loop P gain, check BEFORE "Kp"
    {
        float v = atof(p + 6);
        if (v > 0.001f) { Kp_pos = v; g_integral = 0; g_last_error = 0; }
        return 1;
    }
    else if (strncmp(p, "Kp", 2) == 0)     // speed-loop P gain
    {
        float v = atof(p + 2);
        if (v >= 0.0f) { Kp = v; g_integral = 0; g_last_error = 0; }
        return 1;
    }
    else if (strncmp(p, "Ki", 2) == 0)     // speed-loop I gain
    {
        float v = atof(p + 2);
        if (v >= 0.0f) { Ki = v; g_integral = 0; g_last_error = 0; }
        return 1;
    }
    else if (strncmp(p, "Kd", 2) == 0)     // speed-loop D gain
    {
        float v = atof(p + 2);
        if (v >= 0.0f) { Kd = v; }
        return 1;
    }
    else if (strncmp(p, "pos", 3) == 0)    // target position
    {
        int v = atoi(p + 3);
        g_target_position = (int16_t)v;
        g_integral = 0; g_last_error = 0;  // clean start at the new target
        g_stall = 0;                       // a fresh command releases the stall latch
        return 1;
    }
    return 0;   // unrecognized line
}

// Called by HAL_UART_IRQHandler for each received byte.
// ISR context: accumulate into the active buffer, then wake CommTask.
// No parsing, no UART TX here - just fill the buffer and signal the task.
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1)
        return;

    if (rx_byte == '\r' || rx_byte == '\n')   // end of a command line
    {
        if (cmd_len[cmd_idx] > 0)
        {
            cmd_buf[cmd_idx][cmd_len[cmd_idx]] = '\0';
            cmd_ready[cmd_idx] = 1;                     // a complete line is pending
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(g_comm_sem, &xHigherPriorityTaskWoken);
            cmd_idx ^= 1;                               // next bytes go to the other buffer
            cmd_len[cmd_idx] = 0;
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
    else if (cmd_len[cmd_idx] < (sizeof(cmd_buf[0]) - 1))
    {
        cmd_buf[cmd_idx][cmd_len[cmd_idx]++] = rx_byte;
    }
    else
    {
        cmd_len[cmd_idx] = 0;   // buffer overflow, discard the line
    }

    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);   // re-arm for the next byte
}

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
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_IWDG_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

// ??:????
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);

// ?????
__HAL_TIM_SET_COUNTER(&htim2, 0);
HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

g_last_enc = 0;
g_target_position = 200;           // ? ????

// RX is armed by CommTask (scheduler must be running before a byte can
// trigger xSemaphoreGiveFromISR). USART1 NVIC priority is configured by
// CubeMX in stm32f1xx_hal_msp.c (preemption priority 5).

uart_send_str("Position Loop Start\r\n");
uart_send_str("CMDS: pos <n>, Kp <f>, Kp_pos <f>, Ki <f>, Kd <f>\r\n");

// OLED: software I2C on PB6(SCL)/PB7(SDA), powered from 3V3. Init here before
// the scheduler starts (one-shot, <200ms << 1s IWDG timeout). From now on
// DisplayTask is the only owner of the I2C bus.
OLED_Init();

// Report the OLED init result as a banner line. This is the observable
// self-diagnostic for a dead display: a disconnected OLED has no power to
// show on-screen text, so we surface the NACK flag over the UART instead.
// Free-text banner (no commas, <5 fields) - main_plot.py skips such lines,
// so the 5-value CSV protocol is unchanged.
if (g_oled_nack)
    uart_send_str("OLED: FAIL (check VCC/GND/SCL/SDA wiring)\r\n");
else
    uart_send_str("OLED: OK\r\n");

// PC13 = onboard blue-pill LED, plain output. Board B (bare, no OLED) blinks
// it per received frame as the visible CAN indicator; harmless on board A
// where the LED is simply held off.
__HAL_RCC_GPIOC_CLK_ENABLE();
GPIO_InitTypeDef pc13 = {0};
pc13.Pin = GPIO_PIN_13;
pc13.Mode = GPIO_MODE_OUTPUT_PP;
pc13.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOC, &pc13);
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);   // active-low LED: start off

// Power-on self-test: 3 short blinks so a fresh flash is unmistakable
// (new firmware running + LED wiring good) before the per-frame RX blink
// takes over. ~0.25s total, inside the pre-scheduler 1s IWDG budget.
for (int i = 0; i < 3; i++)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);      // on (active low)
    for (volatile uint32_t d = 0; d < 60000; d++) {}
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);        // off
    for (volatile uint32_t d = 0; d < 60000; d++) {}
}

// CAN1 startup. CubeMX only generated the FIFO1 RX interrupt (CAN1_RX1_IRQn),
// so the FIFO0 one we actually use is enabled here in USER CODE - it survives
// regeneration. Error notification is deliberately NOT enabled: while the bus
// is still empty the retry storm would flood the SCE ISR and could starve
// DiagTask out of its IWDG feed; link health is inferred from the counters.
HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

CAN_FilterTypeDef can_f = {0};
can_f.FilterBank = 0;
can_f.FilterMode = CAN_FILTERMODE_IDMASK;
can_f.FilterScale = CAN_FILTERSCALE_32BIT;
can_f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
can_f.FilterActivation = CAN_FILTER_ENABLE;   // gotcha: {0} init leaves it DISABLE -> filter inactive -> RX never accepted
#if NODE_ID == 0
    // master: accept everything (0x200+id feedback from every node)
    can_f.FilterIdHigh = 0; can_f.FilterIdLow = 0;
    can_f.FilterMaskIdHigh = 0xFFFF; can_f.FilterMaskIdLow = 0xFFFF;
#else
    // slave: accept only its own target ID (0x100+node). STM32 mask semantics:
    // mask bit 1 = don't care, 0 = must match - the classic filter gotcha.
    can_f.FilterIdHigh = (uint16_t)(((0x100 + NODE_ID) << 5) & 0xFFFF);
    can_f.FilterIdLow  = 0;
    can_f.FilterMaskIdHigh = 0; can_f.FilterMaskIdLow = 0;
#endif
HAL_CAN_ConfigFilter(&hcan, &can_f);

if (HAL_CAN_Start(&hcan) == HAL_OK
    && HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK)
    uart_send_str("CAN: OK\r\n");
else
    uart_send_str("CAN: FAIL\r\n");

// Create the application tasks before the scheduler starts (osKernelStart()
// runs below). ControlTask runs the 20ms PID, CommTask the serial channel,
// DiagTask the heartbeat + stall protection, DisplayTask the OLED.
g_comm_sem = xSemaphoreCreateBinary();
xTaskCreate(ControlTask, "Control", 256, NULL, 3, NULL);
xTaskCreate(CommTask,    "Comm",    256, NULL, 2, NULL);
xTaskCreate(DiagTask,    "Diag",    128, NULL, 1, NULL);
xTaskCreate(DisplayTask, "Disp",    256, NULL, 1, NULL);
xTaskCreate(CANTask,     "CAN",     256, NULL, 2, NULL);
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
  /* creation of my_Task */
  my_TaskHandle = osThreadNew(StartmyTask, NULL, &my_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // The FreeRTOS scheduler now owns the CPU (osKernelStart() never returns).
    // All application work lives in ControlTask / CommTask / DiagTask.
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;              // auto-recover bus-off (test: board A may run alone)
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;      // 120R termination present: retry is safe now
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 624;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 49;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);

  /*Configure GPIO pins : PA4 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//===========================================================================
// FreeRTOS task 1: 20ms position + speed PID loop (highest priority, no UART)
//===========================================================================
void ControlTask(void *argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(PID_PERIOD_MS));

        // Stall latch: honor the protection - clear output and skip the PID.
        // Without this guard the PID would re-assert PWM every 20ms and fight
        // DiagTask's protection.
        if (g_stall)
        {
            g_target_speed = 0; g_integral = 0; g_pwm = 0;
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
            continue;
        }

        //---- read encoder ----
        g_enc = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);

        //---- position loop (outer) ----
        int16_t pos_error = g_target_position - g_enc;

        // inside dead band: hold position, clear integral
        if (pos_error > -DEAD_BAND && pos_error < DEAD_BAND)
        {
            g_target_speed = 0;
            g_integral = 0;
        }
        else
        {
            // P control: error -> target speed
            g_target_speed = (int16_t)(Kp_pos * pos_error);

            // limit speed, and slow down when approaching the target
            int16_t abs_err = (pos_error > 0) ? pos_error : -pos_error;
            int16_t speed_limit = MAX_SPEED;
            if (abs_err < DECEL_ZONE)
            {
                speed_limit = MAX_SPEED * abs_err / DECEL_ZONE;
                if (speed_limit < 5) speed_limit = 5;   // minimum creep speed
            }

            if (g_target_speed >  speed_limit) g_target_speed =  speed_limit;
            if (g_target_speed < -speed_limit) g_target_speed = -speed_limit;
        }

        //---- speed loop (inner) ----
        g_speed = (int16_t)(g_enc - g_last_enc);   // global: stall detection + telemetry
        g_last_enc = g_enc;

        int16_t error = g_target_speed - g_speed;

        g_integral += error;
        if (g_integral >  INTEGRAL_LIMIT) g_integral =  INTEGRAL_LIMIT;
        if (g_integral < -INTEGRAL_LIMIT) g_integral = -INTEGRAL_LIMIT;

        int16_t derivative = error - g_last_error;
        g_last_error = error;

        float output = Kp * error + Ki * g_integral + Kd * derivative;

        if (output >  PWM_MAX) output =  PWM_MAX;
        if (output < -PWM_MAX) output = -PWM_MAX;

        g_pwm = (int8_t)output;

        //---- apply PWM + direction ----
        if (g_pwm >= 0)
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, g_pwm);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, -g_pwm);
        }
    }
}

//===========================================================================
// FreeRTOS task 2: serial commands + periodic 5-value telemetry line
//===========================================================================
void CommTask(void *argument)
{
    // Arm RX only now: the scheduler is running, so a byte can safely
    // trigger xSemaphoreGiveFromISR (illegal before osKernelStart).
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

    for (;;)
    {
        // Block until a command line is ready, or ~20ms tick elapsed.
        // Return value ignored: the cmd_ready[] flags are the source of truth
        // (a binary semaphore merges consecutive gives, so its count must not
        // be trusted). Draining on timeout is what makes a 2nd command that
        // arrived within the same 20ms window still get processed.
        xSemaphoreTake(g_comm_sem, pdMS_TO_TICKS(20));

        uint8_t b = (uint8_t)(cmd_idx ^ 1);      // buffer the ISR just finished
        char local[sizeof(cmd_buf[0])];
        int handled = 0;

        taskENTER_CRITICAL();                    // masks USART1 ISR (prio 5): safe copy
        if (cmd_ready[b])
        {
            memcpy(local, cmd_buf[b], sizeof(cmd_buf[0]));
            cmd_ready[b] = 0;
            handled = 1;
        }
        taskEXIT_CRITICAL();

        if (handled)
        {
            if (process_cmd(local))
                uart_send_str("OK\r\n");
            continue;                            // no telemetry this cycle
        }

        // 5-value CSV telemetry line: enc,pos,speed,tspeed,pwm
        uart_print_int(g_enc);            uart_send_str(",");
        uart_print_int(g_target_position); uart_send_str(",");
        uart_print_int(g_speed);          uart_send_str(",");
        uart_print_int(g_target_speed);   uart_send_str(",");
        uart_print_enc(g_pwm);            // value + \r\n
    }
}

//===========================================================================
// FreeRTOS task 3: LED heartbeat + stall protection (lowest priority)
//===========================================================================
void DiagTask(void *argument)
{
    uint32_t led_cnt = 0;
    uint32_t stall_cnt = 0;

    for (;;)
    {
        HAL_IWDG_Refresh(&hiwdg);   // feed the 1s IWDG watchdog - DiagTask is the ONLY feeder
        vTaskDelay(pdMS_TO_TICKS(50));

        if (++led_cnt >= 10) { led_cnt = 0; HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4); }  // 500ms heartbeat

        // Stall: we command drive (PWM != 0) but the shaft barely moves for ~1s.
        if (g_pwm != 0 && g_speed > -2 && g_speed < 2)
        {
            if (++stall_cnt >= 20)   // 20 * 50ms = 1s
            {
                g_stall = 1; g_pwm = 0;
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
            }
        }
        else
        {
            stall_cnt = 0;
        }
    }
}

//===========================================================================
// FreeRTOS task 4: OLED display (software I2C on PB6/PB7, lowest priority).
// Stage A: static screen - proves wiring + driver before live data is wired.
//===========================================================================
static void fmt_signed(char *out, int16_t v, uint8_t width)
{
    // Right-justified signed integer into a fixed 'width' cell (space padded).
    // Avoids snprintf, which would drag the whole printf family into flash.
    char tmp[8];
    uint8_t i = 0;
    uint8_t neg = (v < 0);
    if (neg) v = (int16_t)-v;
    do {
        tmp[i++] = (char)('0' + (v % 10));
        v = (int16_t)(v / 10);
    } while (v > 0 && i < 7);
    if (neg) tmp[i++] = '-';
    while (i < width) tmp[i++] = ' ';
    for (uint8_t j = 0; j < width; j++)
        out[j] = tmp[width - 1 - j];
    out[width] = '\0';
}

static void fmt_uint(char *out, uint32_t v, uint8_t width)
{
    // Right-justified unsigned integer into a fixed 'width' cell (space padded).
    // Companion to fmt_signed for counters; also keeps snprintf out of flash.
    char tmp[8];
    uint8_t i = 0;
    do { tmp[i++] = (char)('0' + (v % 10)); v /= 10; } while (v > 0 && i < 7);
    while (i < width) tmp[i++] = ' ';
    for (uint8_t j = 0; j < width; j++) out[j] = tmp[width - 1 - j];
    out[width] = '\0';
}

void DisplayTask(void *argument)
{
    char cache[5][8] = {0};     // last rendered value strings (for change detection)
    char b[8];
    uint8_t bar_old = 0xFF;     // last PWM bar length; 0xFF forces first redraw

    // CAN link line (page 6): shows "CAN OK <n>" while frames keep arriving
    // and flips back to idle once the RX counter freezes for ~5s.
    char can_cache[12] = {0};
    uint32_t rx_last = 0;
    uint8_t rx_freeze = 0;
    uint8_t can_alive = 0;

    // static layout (drawn once; values below are re-drawn on change only)
    OLED_DrawString(0, 0,  "CAN SERVO");
    OLED_DrawString(1, 0,  "POS");
    OLED_DrawString(2, 0,  "TGT");
    OLED_DrawString(3, 0,  "SPD");
    OLED_DrawString(4, 0,  "PWM");
    OLED_DrawString(5, 0,  "STA");
    OLED_DrawString(6, 0,  "CAN  --");
    OLED_Flush();

    uint8_t blink = 0;
    uint8_t cnt = 0;

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(200));     // 5 Hz refresh

        // --- 5 live lines: re-draw only when the rendered text changed ---
        fmt_signed(b, g_enc, 6);
        if (strcmp(b, cache[0]) != 0) { strcpy(cache[0], b); OLED_DrawString(1, 24, b); }

        fmt_signed(b, g_target_position, 6);
        if (strcmp(b, cache[1]) != 0) { strcpy(cache[1], b); OLED_DrawString(2, 24, b); }

        fmt_signed(b, g_speed, 6);
        if (strcmp(b, cache[2]) != 0) { strcpy(cache[2], b); OLED_DrawString(3, 24, b); }

        fmt_signed(b, g_pwm, 6);
        if (strcmp(b, cache[3]) != 0) { strcpy(cache[3], b); OLED_DrawString(4, 24, b); }

        // status: ERR (I2C NACK = wiring/device problem) > STALL > RUN
        if (g_oled_nack)      { strcpy(b, "ERR   "); }
        else if (g_stall)     { strcpy(b, "STALL "); }
        else                  { strcpy(b, "RUN   "); }
        if (strcmp(b, cache[4]) != 0) { strcpy(cache[4], b); OLED_DrawString(5, 24, b); }

        // --- CAN link status on page 6 (counter-freeze link monitor) ---
        {
            uint32_t rx = g_can_rx_cnt;
            if (rx > 0)
            {
                if (rx != rx_last) { rx_last = rx; rx_freeze = 0; can_alive = 1; }
                else if (++rx_freeze >= 25) can_alive = 0;   // ~5s of silence
            }
            char cbuf[12];
            if (can_alive)
            {
                strcpy(cbuf, "CAN ");
                cbuf[4] = 'O'; cbuf[5] = 'K'; cbuf[6] = ' ';
                fmt_uint(cbuf + 7, rx, 4);              // digits + '\0' at [11]
            }
            else
            {
                strcpy(cbuf, "CAN  --");
            }
            if (strcmp(cbuf, can_cache) != 0) { strcpy(can_cache, cbuf); OLED_DrawString(6, 0, cbuf); }
        }

        // --- PWM bar on page 7 (|pwm| proportional length; redraw on change) ---
        int16_t p = g_pwm; if (p < 0) p = (int16_t)-p;
        uint8_t len = (uint8_t)(p * OLED_COLS / PWM_MAX);
        if (len != bar_old)
        {
            bar_old = len;
            OLED_ClearPage(7);
            if (len) OLED_DrawHBar(7, 0, len);
        }

        // --- heartbeat every ~1s ---
        if (++cnt >= 5)
        {
            cnt = 0;
            blink = (uint8_t)!blink;
            OLED_DrawString(0, 120, blink ? "*" : " ");
        }

        OLED_Flush();           // push only the pages that changed
    }
}

//===========================================================================
// FreeRTOS task 5: CAN link (prio 2, 100ms tick). Master broadcasts the
// target frame; the slave replies with its feedback frame. The wire is
// verified with the TX/RX counters, not error interrupts (see the startup
// comment in USER CODE 2). Protocol: 0x100+id -> node, 0x200+id <- node.
//===========================================================================
void CANTask(void *argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    CAN_TxHeaderTypeDef tx = {0};
    uint8_t tx_data[8] = {0};

    tx.IDE = CAN_ID_STD;
    tx.RTR = CAN_RTR_DATA;
    tx.DLC = 3;                          // protocol: 2 data bytes + 1 flag byte

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));   // 10 Hz link tick

        if (NODE_ID == 0)
        {
            // Master -> slave 1: target position (int16 little-endian) + flags.
            tx.StdId = 0x100 + 1;
            int16_t t = g_target_position;
            tx_data[0] = (uint8_t)(t & 0xFF);
            tx_data[1] = (uint8_t)((t >> 8) & 0xFF);
            tx_data[2] = 0x01;                   // flags: bit0 = enable
        }
        else
        {
            // Slave -> master: encoder feedback (int16 LE) + status.
            tx.StdId = 0x200 + NODE_ID;
            int16_t e = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
            tx_data[0] = (uint8_t)(e & 0xFF);
            tx_data[1] = (uint8_t)((e >> 8) & 0xFF);
            tx_data[2] = (uint8_t)(g_stall ? 0x01 : 0x00);   // status: bit0 = stall
        }

        // Auto-retransmission is ON: while no other node ACKs, the mailbox
        // stays busy retrying and this call returns non-OK, so g_can_tx_cnt
        // only grows once the link is actually alive (both ends + termination).
        if (HAL_CAN_AddTxMessage(&hcan, &tx, tx_data, &g_can_mailbox) == HAL_OK)
            g_can_tx_cnt++;

        // DIAGNOSTIC: PC13 = RX blink (ISR, one per 0x101) + TX alarm. The old
        // FIFO poll is gone (ISR blinks on its own now). Every 5s, if
        // g_can_tx_cnt has NOT grown, our 0x201 is not being ACKed (board A's RX
        // path or our TX path is broken) -> 5 rapid blinks as an alarm. Auto-
        // retransmit ON means a mailbox only frees on a real ACK, so a stalled
        // counter is a reliable "nobody hears us" signal.
        if (NODE_ID != 0)
        {
            static uint32_t last_tx = 0;
            static uint32_t last_check = 0;
            if (g_can_tx_cnt == 0) last_tx = 0;           // never sent yet
            if ((xTaskGetTickCount() - last_check) >= pdMS_TO_TICKS(5000))
            {
                uint32_t grew = g_can_tx_cnt - last_tx;
                last_check = xTaskGetTickCount();
                last_tx = g_can_tx_cnt;
                if (grew == 0)
                {
                    for (int i = 0; i < 5; i++)           // TX alarm: 5 rapid blinks
                    {
                        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
                        for (volatile uint32_t d = 0; d < 25000; d++) {}
                        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
                        for (volatile uint32_t d = 0; d < 25000; d++) {}
                    }
                }
            }
        }
    }
}

//===========================================================================
// CAN RX ISR (FIFO0 message pending). Copy the frame and count it; on the
// slave side also latch the master's target and blink PC13 per frame (board B
// has no OLED, so the LED is its only receive indicator).
//===========================================================================
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcanp)
{
    static CAN_RxHeaderTypeDef rx_hdr;
    static uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(hcanp, CAN_RX_FIFO0, &rx_hdr, rx_data) != HAL_OK)
        return;

    g_can_rx_cnt++;

    if (NODE_ID != 0 && rx_hdr.IDE == CAN_ID_STD && rx_hdr.StdId == (0x100 + NODE_ID))
    {
        g_remote_pos = (int16_t)(rx_data[0] | (rx_data[1] << 8));
        // 5.3 step 2: the master's 0x100+NODE_ID target now drives the position
        // loop. Before, it was only latched into g_remote_pos while ControlTask
        // kept using the boot-default g_target_position, so CAN commands had no
        // effect. (Slave #1 = 0x101, slave #2 = 0x102.)
        g_target_position = g_remote_pos;
        // 5.3 step 4: protocol bit1 = master clears the stall latch over CAN,
        // so a stalled slave recovers on the next "pos" command - no reboot.
        if (rx_data[2] & 0x02) { g_stall = 0; }
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartmyTask */
/**
  * @brief  Function implementing the my_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartmyTask */
void StartmyTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
