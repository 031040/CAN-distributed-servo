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

UART_HandleTypeDef huart1;

/* Definitions for my_Task */
osThreadId_t my_TaskHandle;
const osThreadAttr_t my_Task_attributes = {
  .name = "my_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
// Pure commander has NO local axis, so there is no encoder/speed/PWM/PID state
// at all - those globals live only in the slave firmware. The commander keeps
// one control variable: the target position it broadcasts to every slave.
volatile int16_t g_target_position = 200;   // command target, sent to 0x100+1..NODE_COUNT-1
//=== Serial command channel: ISR -> double buffer -> semaphore -> CommTask ===
uint8_t rx_byte = 0;              // byte buffer for HAL_UART_Receive_IT (ISR context only)
char cmd_buf[2][64];              // double buffer: ISR fills, CommTask consumes
volatile uint8_t  cmd_idx   = 0;  // buffer index the ISR is currently filling
volatile uint16_t cmd_len[2] = {0, 0};
volatile uint8_t  cmd_ready[2] = {0, 0};  // 1 = a complete line is waiting in this buffer
SemaphoreHandle_t g_comm_sem = NULL;      // wakes CommTask (20ms timeout = telemetry cadence)

//=== CAN bus (3-node distributed servo) ===
// Pure commander (board D): NO local motor / encoder / PID. It broadcasts the
// target frame to BOTH slave nodes (0x100+1, 0x100+2) and latches both feedback
// frames (0x201, 0x202). Board B = slave #1 (motor #2, NODE_ID=1), board C =
// slave #2 (motor #1, NODE_ID=2). Adding a slave = raise NODE_COUNT, add a
// filter bank for its 0x200+id, an array slot, and an OLED row.
#define NODE_ID   0        // compile-time role: 0 = master, 1/2 = slaves
#define NODE_COUNT 3       // protocol: 0x100+id master->node, 0x200+id node->master
volatile uint32_t g_can_rx_cnt = 0;   // frames received (count freezing => link down)
volatile uint32_t g_can_tx_cnt = 0;   // frames queued & ACKed (growing => link up)
uint32_t g_can_mailbox = 0;           // mailbox slot returned by AddTxMessage
volatile int16_t  g_slave_pos[NODE_COUNT]   = {0};  // master: each slave's feedback position (0x200+id)
volatile uint8_t  g_slave_stall[NODE_COUNT] = {0};  // master: each slave's stall status bit (0x200+id data[2])
volatile uint8_t  g_clr_stall_cnt = 0; // master: ticks left sending flags bit1 (clear-stall)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG_Init(void);
static void MX_CAN_Init(void);
void StartmyTask(void *argument);

/* USER CODE BEGIN PFP */
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

//==== Serial command handling (USART1 RX interrupt) ====

// Parse and execute one received command line.
// Runs in CommTask context (a task, not an ISR). Returns 1 if recognized.
// Sets control variables only; all UART TX happens in CommTask.
static int process_cmd(char *cmd)
{
    char *p = cmd;
    while (*p == ' ') p++;   // skip leading spaces

    // Pure commander console: "pos" is the only command - it broadcasts a new
    // target position to every slave. The PID gain commands (Kp/Ki/Kd/Kp_pos)
    // are gone because there is no local loop to tune; each slave is tuned on
    // its OWN serial port, not over CAN (the protocol has no gain channel).
    if (strncmp(p, "pos", 3) == 0)         // broadcast a new target position
    {
        int v = atoi(p + 3);
        g_target_position = (int16_t)v;
        g_clr_stall_cnt = 10;   // piggyback flags bit1 for ~1s: wakes a stalled slave over CAN
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
  MX_USART1_UART_Init();
  MX_IWDG_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
// No motor / encoder / PID on this board: it is a PURE COMMANDER that only
// broadcasts target positions and watches the two slave feedbacks. TIM2/TIM3
// and their GPIO are intentionally not initialized, so no Axis code lives here.
g_target_position = 200;           // boot default: broadcast to both slaves

// RX is armed by CommTask (scheduler must be running before a byte can
// trigger xSemaphoreGiveFromISR). USART1 NVIC priority is configured by
// CubeMX in stm32f1xx_hal_msp.c (preemption priority 5).

uart_send_str("CAN 3-Node Commander Start\r\n");
uart_send_str("CMDS: pos <n>   (broadcast to slaves #1..#2)\r\n");

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

// PC13 = onboard blue-pill LED, plain output. On the pure commander it blinks
// per received slave-feedback frame (in the RX ISR) as the visible CAN
// indicator - the bare boards have no OLED, so the LED is their only one.
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

// master (pure commander): one filter bank per slave feedback ID (0x200+id).
// mask=0 = exact match on genuine STM32 (bit 1 = don't-care); on GD32-style
// clones the mask-bit meaning is inverted, so mask=0 degrades to accept-all
// and still passes every feedback - the same config is therefore safe on BOTH
// silicon families and cannot false-negative a working board. The RX ISR then
// dispatches each accepted frame to its slave slot by StdId. This mirrors the
// slave's proven exact-match setup.
for (uint8_t n = 1; n < NODE_COUNT; n++)
{
    CAN_FilterTypeDef can_f = {0};
    can_f.FilterBank = n - 1;
    can_f.FilterMode = CAN_FILTERMODE_IDMASK;
    can_f.FilterScale = CAN_FILTERSCALE_32BIT;
    can_f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    can_f.FilterActivation = CAN_FILTER_ENABLE;   // gotcha: {0} init leaves it DISABLE -> filter inactive -> RX never accepted
    can_f.FilterIdHigh = (uint16_t)(((0x200 + n) << 5) & 0xFFFF);
    can_f.FilterIdLow  = 0;
    can_f.FilterMaskIdHigh = 0; can_f.FilterMaskIdLow = 0;
    HAL_CAN_ConfigFilter(&hcan, &can_f);
}

if (HAL_CAN_Start(&hcan) == HAL_OK
    && HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK)
    uart_send_str("CAN: OK\r\n");
else
    uart_send_str("CAN: FAIL\r\n");

// Pure commander task set: no ControlTask (there is no local axis/PID to run).
// CommTask is the serial console ("pos" only), DiagTask the heartbeat + IWDG
// feed, CANTask broadcasts the target and prints the link-health line,
// DisplayTask shows both slave positions on the OLED.
g_comm_sem = xSemaphoreCreateBinary();
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
    // All application work lives in CommTask / DiagTask / CANTask / DisplayTask.
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

// NOTE: TIM2 (encoder) and TIM3 (PWM) initializers were removed - this is a
// pure commander with no local motor axis, so those peripherals stay off and
// their GPIO (PA0/PA1/PA6) are left floating. stm32f1xx_hal_msp.c still
// carries the TIM MSP callbacks but they only run when HAL_TIM_*_Init is
// called, so leaving them in place is harmless and CubeMX-regeneration-safe.

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
// FreeRTOS task 1: serial console (the pure commander has NO periodic CSV -
// it has no local axis to report, so the console stays clean for typing).
// Link liveness is instead the "CAN DBG rx=/tx=" line printed by CANTask.
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
        // be trusted). Two lines can complete inside one window (the ISR
        // alternates buffers), so both buffers are drained oldest-first below.
        xSemaphoreTake(g_comm_sem, pdMS_TO_TICKS(20));

        for (uint8_t pass = 0; pass < 2; pass++)
        {
            uint8_t b = (uint8_t)(cmd_idx ^ pass);  // pass0 = older of the two, pass1 = newest
            char local[sizeof(cmd_buf[0])];
            int handled = 0;

            taskENTER_CRITICAL();                   // masks USART1 ISR (prio 5): safe copy
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
                // unrecognized line: ignored (same as the 2-node master)
            }
        }
    }
}

//===========================================================================
// FreeRTOS task 2: LED heartbeat + IWDG feed (lowest priority)
//===========================================================================
void DiagTask(void *argument)
{
    uint32_t led_cnt = 0;

    for (;;)
    {
        HAL_IWDG_Refresh(&hiwdg);   // feed the 1s IWDG watchdog - DiagTask is the ONLY feeder
        vTaskDelay(pdMS_TO_TICKS(50));

        if (++led_cnt >= 10) { led_cnt = 0; HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4); }  // 500ms heartbeat

        // No local stall protection here: the pure commander has no axis to
        // stall. Each slave latches and clears its own stall; the master only
        // carries the clear-stall flag bit (see CANTask / process_cmd).
    }
}

//===========================================================================
// FreeRTOS task 3: OLED display (software I2C on PB6/PB7, lowest priority).
// Pure-commander screen: one row per slave (its feedback position, replaced
// by "STALL" while that slave has latched a stall) + the broadcast target
// + a CAN link line. Font is 6px/char; values live in a 6-char cell starting
// at pixel 24, so every label here is <= 4 chars (4*6 = 24px) and can never
// collide with a value.
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
    char cache[3][8] = {0};     // rendered value strings: [0]=S1 [1]=S2 [2]=TGT
    char b[8];

    // CAN link line (row 5): "CAN OK <n>" while frames keep arriving, flips
    // back to idle once the RX counter freezes for ~5s (dead bus / bus-off).
    char can_cache[12] = {0};
    uint32_t rx_last = 0;
    uint8_t rx_freeze = 0;
    uint8_t can_alive = 0;

    // static layout (drawn once; values below are re-drawn on change only)
    OLED_DrawString(0, 0,  "CAN SERVO");
    OLED_DrawString(1, 0,  "S1");     // slave #1 = 0x201 (board B, motor #2)
    OLED_DrawString(2, 0,  "S2");     // slave #2 = 0x202 (board C, motor #1)
    OLED_DrawString(3, 0,  "TGT");    // broadcast target (0x100+1..2 payload)
    OLED_DrawString(5, 0,  "CAN  --");
    OLED_Flush();

    uint8_t blink = 0;
    uint8_t cnt = 0;

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(200));     // 5 Hz refresh

        // slave #1/#2 feedback positions (rows 1/2); a latched stall on that
        // slave replaces the frozen number so the fault is visible at a glance.
        for (uint8_t s = 1; s < NODE_COUNT; s++)
        {
            if (g_slave_stall[s]) { strcpy(b, "STALL "); }
            else                  { fmt_signed(b, g_slave_pos[s], 6); }
            if (strcmp(b, cache[s - 1]) != 0)
            {
                strcpy(cache[s - 1], b);
                OLED_DrawString(s, 24, b);
            }
        }

        // broadcast target (row 3)
        fmt_signed(b, g_target_position, 6);
        if (strcmp(b, cache[2]) != 0) { strcpy(cache[2], b); OLED_DrawString(3, 24, b); }

        // --- CAN link status on row 5 (counter-freeze link monitor) ---
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
                strcpy(cbuf, "CAN OK ");        // 7 glyphs, digits follow at [7]
                fmt_uint(cbuf + 7, rx, 4);      // digits + '\0' at [11]
            }
            else
            {
                strcpy(cbuf, "CAN  --");
            }
            if (strcmp(cbuf, can_cache) != 0) { strcpy(can_cache, cbuf); OLED_DrawString(5, 0, cbuf); }
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
// FreeRTOS task 4: CAN link (prio 2, 100ms tick). The pure commander
// broadcasts the SAME target frame to every slave (0x100+1, 0x100+2); each
// slave filters only its own ID. The wire is verified with the TX/RX counters,
// not error interrupts (see the startup comment in USER CODE 2).
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

        // One frame per slave per tick. Both carry the same command so every
        // node tracks the same target (that is what a 2-motor synchronized
        // move is here). Auto-retransmission is ON: while no slave ACKs, the
        // mailbox stays busy retrying and AddTxMessage returns non-OK, so
        // g_can_tx_cnt only grows once the link is actually alive.
        for (uint8_t n = 1; n < NODE_COUNT; n++)
        {
            tx.StdId = 0x100 + n;
            int16_t t = g_target_position;
            tx_data[0] = (uint8_t)(t & 0xFF);
            tx_data[1] = (uint8_t)((t >> 8) & 0xFF);
            tx_data[2] = 0x01;                   // flags: bit0 = enable
            if (g_clr_stall_cnt)                 // bit1 = clear-stall: set for a
                tx_data[2] |= 0x02;              // full tick right after a fresh
            if (HAL_CAN_AddTxMessage(&hcan, &tx, tx_data, &g_can_mailbox) == HAL_OK)
                g_can_tx_cnt++;                  // "pos", so a stalled slave
        }                                        // recovers without a power cycle
        if (g_clr_stall_cnt) g_clr_stall_cnt--;  // counted down once per tick

        // Periodic serial link-health line (free text, no commas - main_plot.py
        // skips it, so the CSV protocol is unchanged). rx growing => at least
        // one slave's feedback is arriving; tx growing => our target frames are
        // being ACKed on the bus. This is the serial-visible half of the link
        // check for a console that no longer prints per-tick CSV.
        {
            static uint32_t dbg_tick = 0;
            if (++dbg_tick >= 20)            // 20 * 100ms = 2s
            {
                dbg_tick = 0;
                char n[12];
                uart_send_str("CAN DBG rx=");
                fmt_uint(n, g_can_rx_cnt, 6); uart_send_str(n);
                uart_send_str(" tx=");
                fmt_uint(n, g_can_tx_cnt, 6); uart_send_str(n);
                uart_send_str("\r\n");
            }
        }
    }
}

//===========================================================================
// CAN RX ISR (FIFO0 message pending). Copy the frame and count it. The two
// per-slave filter banks (0x201/0x202) both land in FIFO0, so the StdId tells
// us which slave spoke; dispatch it to its array slot and blink PC13 per
// frame - the same visible receive indicator the bare slave boards have.
//===========================================================================
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcanp)
{
    static CAN_RxHeaderTypeDef rx_hdr;
    static uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(hcanp, CAN_RX_FIFO0, &rx_hdr, rx_data) != HAL_OK)
        return;

    g_can_rx_cnt++;

    uint32_t id = rx_hdr.StdId - 0x200;   // 1..NODE_COUNT-1 (guard: belt & braces)
    if (id >= 1 && id < NODE_COUNT)
    {
        // payload: [0:1] slave encoder pos (int16 LE), [2] status bit0 = stall
        g_slave_pos[id]   = (int16_t)(rx_data[0] | (rx_data[1] << 8));
        g_slave_stall[id] = rx_data[2] & 0x01;
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
