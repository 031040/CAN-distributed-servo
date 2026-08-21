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
typedef struct { uint8_t x, y; } SnakePoint;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GRID_W      16          /* game field: 16 x 7 cells (top 8px row = score) */
#define GRID_H      7
#define SNAKE_MAX   (GRID_W * GRID_H)
#define TURN_THRESH  3          /* encoder counts per turn decision */
#define TURN_LOCK_MS 200        /* debounce: ignore re-turns for this long */
#define START_SPEED_MS 200      /* ms per step, speeds up as score grows */
#define MIN_SPEED_MS    60
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
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
//=== Snake game state (SnakeControl writes, DisplayTask reads) ===
SnakePoint g_snake[SNAKE_MAX];
uint8_t    g_snake_len;
uint8_t    g_dir;             // 0=右 1=下 2=左 3=上
SnakePoint g_food;
uint16_t   g_score;
uint16_t   g_speed_ms;        // ms per step, 200 start -> 60 min
uint8_t    g_state;           // 0=PLAYING 1=GAME_OVER
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG_Init(void);
void StartmyTask(void *argument);

/* USER CODE BEGIN PFP */
void SnakeControlTask(void *argument);
void CommTask(void *argument);
void DiagTask(void *argument);
void DisplayTask(void *argument);
void snake_init(void);
void snake_step(void);
void spawn_food(void);
static uint8_t food_on_snake(void);
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
// send a signed int16 as ASCII digits
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

    // reverse digit order
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

//==== Snake game logic ========================================================
void snake_init(void)
{
    g_snake_len = 3;
    g_snake[0].x = 8; g_snake[0].y = 3;
    g_snake[1].x = 7; g_snake[1].y = 3;
    g_snake[2].x = 6; g_snake[2].y = 3;
    g_dir       = 0;              // 向右
    g_score     = 0;
    g_speed_ms  = START_SPEED_MS;
    g_state     = 0;              // PLAYING
    spawn_food();
}

// 伪随机生成食物，落在空格里（FreeRTOS tick 做熵源 + LCG）
void spawn_food(void)
{
    uint32_t seed = xTaskGetTickCount();
    for (uint16_t tries = 0; tries < 200; tries++)
    {
        seed = seed * 1103515245u + 12345u;    // LCG
        g_food.x = (uint8_t)((seed >> 8)  % GRID_W);
        g_food.y = (uint8_t)((seed >> 16) % GRID_H);
        if (!food_on_snake()) return;
    }
    // 兜底：200 次都撞上蛇的概率极低，扫第一个空格
    for (uint8_t y = 0; y < GRID_H; y++)
        for (uint8_t x = 0; x < GRID_W; x++)
        {
            g_food.x = x; g_food.y = y;
            if (!food_on_snake()) return;
        }
}

static uint8_t food_on_snake(void)
{
    for (uint8_t i = 0; i < g_snake_len; i++)
        if (g_snake[i].x == g_food.x && g_snake[i].y == g_food.y) return 1;
    return 0;
}

// 前进一格。撞墙/撞自己 -> GAME_OVER；吃到食物 -> 长+1、分+1、加速、换食物
void snake_step(void)
{
    int16_t nx = g_snake[0].x, ny = g_snake[0].y;
    switch (g_dir)
    {
        case 0: nx++; break;    // 右
        case 1: ny++; break;    // 下
        case 2: nx--; break;    // 左
        case 3: ny--; break;    // 上
    }
    if (nx < 0 || nx >= GRID_W || ny < 0 || ny >= GRID_H) { g_state = 1; return; }

    uint8_t eat = (nx == g_food.x && ny == g_food.y);
    // 撞自己：吃到食物时尾巴保留，查全身；否则尾巴这步会移走，不查它
    uint8_t body = eat ? g_snake_len : (uint8_t)(g_snake_len - 1);
    for (uint8_t i = 0; i < body; i++)
        if (g_snake[i].x == nx && g_snake[i].y == ny) { g_state = 1; return; }

    // 头插入，尾丢弃（吃到时长度+1，尾巴保留）。蛇最长 SNAKE_MAX 格，
    // 满格时移位会写 g_snake[SNAKE_MAX]（越界1），已满则视为胜利结束。
    if (g_snake_len >= SNAKE_MAX) { g_state = 1; return; }
    for (uint8_t i = g_snake_len; i > 0; i--) g_snake[i] = g_snake[i - 1];
    g_snake[0].x = (uint8_t)nx;
    g_snake[0].y = (uint8_t)ny;

    if (eat)
    {
        g_snake_len++;
        g_score++;
        if (g_speed_ms > MIN_SPEED_MS) g_speed_ms -= 10;
        spawn_food();
    }
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
  /* USER CODE BEGIN 2 */
// 编码器清零并启动（TIM2 硬件编码器模式，PA0=A 相 PA1=B 相，来自 JGA25-370）
__HAL_TIM_SET_COUNTER(&htim2, 0);
HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

// OLED: software I2C on PB6(SCL)/PB7(SDA), powered from 3V3. Init here before
// the scheduler starts (one-shot, <200ms << 1s IWDG timeout). From now on
// DisplayTask is the only owner of the I2C bus.
OLED_Init();

// Report the OLED init result as a banner line (observable self-diagnostic for
// a dead display: a disconnected OLED has no power to show text).
if (g_oled_nack)
    uart_send_str("OLED: FAIL (check VCC/GND/SCL/SDA wiring)\r\n");
else
    uart_send_str("OLED: OK\r\n");

// 初始化贪吃蛇，打启动横幅（串口日志，57600 8N1）
snake_init();
uart_send_str("Snake Start. Rotate motor shaft: CW = turn right, CCW = turn left\r\n");

// Create the application tasks before the scheduler starts (osKernelStart()
// runs below). SnakeControlTask: 20ms 编码器判向 + 游戏状态机; CommTask: 串口
// 事件日志; DiagTask: 心跳 + 喂狗; DisplayTask: OLED 绘制（I2C 唯一主人）。
xTaskCreate(SnakeControlTask, "SnakeCtrl", 256, NULL, 3, NULL);
xTaskCreate(CommTask,         "Comm",      256, NULL, 2, NULL);
xTaskCreate(DiagTask,         "Diag",      128, NULL, 1, NULL);
xTaskCreate(DisplayTask,      "Disp",      256, NULL, 1, NULL);
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
    // All application work lives in SnakeControlTask / CommTask / DiagTask /
    // DisplayTask.
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
// FreeRTOS task 1: 20ms - read encoder knob, steer the snake, advance it
// at g_speed_ms cadence. Owns all game-state writes (single-writer rule).
//===========================================================================
void SnakeControlTask(void *argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    int16_t prev_enc = 0;
    TickType_t last_turn = 0;       // debounce: when the last turn was applied
    TickType_t last_step = 0;       // when the snake last moved one cell

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));

        //---- read encoder delta (signed int16 handles counter wrap) ----
        int16_t cur = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
        int16_t delta = cur - prev_enc;
        prev_enc = cur;

        //---- GAME_OVER: any real knob turn restarts ----
        if (g_state == 1)
        {
            if (delta >= TURN_THRESH || delta <= -TURN_THRESH)
            {
                snake_init();
                uart_send_str("[Snake] restart\r\n");
            }
            continue;
        }

        //---- steering: CW -> turn right, CCW -> turn left, debounced ----
        if ((delta >= TURN_THRESH || delta <= -TURN_THRESH) &&
            (xTaskGetTickCount() - last_turn) >= TURN_LOCK_MS)
        {
            if (delta > 0)
                g_dir = (uint8_t)((g_dir + 1) % 4);   // CW: 右转
            else
                g_dir = (uint8_t)((g_dir + 3) % 4);   // CCW: 左转
            last_turn = xTaskGetTickCount();
        }

        //---- advance one cell at g_speed_ms cadence ----
        if ((xTaskGetTickCount() - last_step) >= g_speed_ms)
        {
            last_step = xTaskGetTickCount();
            snake_step();
            if (g_state == 1)
                uart_send_str("[Snake] game over\r\n");
        }
    }
}

//===========================================================================
// FreeRTOS task 2: serial log of game events (game over / score milestones)
//===========================================================================
void CommTask(void *argument)
{
    uint8_t last_state = g_state;
    uint16_t last_score = g_score;

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(100));

        // score increased -> an "eat" happened (report new score/length)
        if (g_state == 0 && g_score != last_score)
        {
            last_score = g_score;
            uart_send_str("[Snake] eat score="); uart_print_int(g_score);
            uart_send_str(" len="); uart_print_int(g_snake_len);
            uart_send_str(" speed="); uart_print_int(g_speed_ms);
            uart_send_str("ms\r\n");
        }
        if (g_state != last_state)
        {
            last_state = g_state;
            last_score = g_score;
        }
    }
}

//===========================================================================
// FreeRTOS task 3: LED heartbeat + IWDG feed (lowest priority, only feeder)
//===========================================================================
void DiagTask(void *argument)
{
    uint32_t led_cnt = 0;

    for (;;)
    {
        HAL_IWDG_Refresh(&hiwdg);   // feed the 1s IWDG watchdog - DiagTask is the ONLY feeder
        vTaskDelay(pdMS_TO_TICKS(50));

        if (++led_cnt >= 10) { led_cnt = 0; HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4); }  // 500ms heartbeat
    }
}

//===========================================================================
// FreeRTOS task 4: OLED display (software I2C on PB6/PB7, lowest priority).
// 128x64 screen: top row (page 0) = score bar, below = 16x7 cell field.
// Every 100ms redraw the whole field into the framebuffer and Flush only
// the pages that changed (dirty-page mechanism in the driver).
//===========================================================================
static void fmt_uint(char *out, uint16_t v, uint8_t width)
{
    // Right-justified unsigned integer into a fixed 'width' cell (space padded).
    // Avoids snprintf, which would drag the whole printf family into flash.
    char tmp[8];
    uint8_t i = 0;
    do {
        tmp[i++] = (char)('0' + (v % 10));
        v = (uint16_t)(v / 10);
    } while (v > 0 && i < 7);
    while (i < width) tmp[i++] = ' ';
    for (uint8_t j = 0; j < width; j++)
        out[j] = tmp[width - 1 - j];
    out[width] = '\0';
}

// draw one 8x8 grid cell as a 6x6 solid square (1px margin for a clean grid)
static void draw_cell(uint8_t gx, uint8_t gy, uint8_t on)
{
    if (gx >= GRID_W || gy >= GRID_H) return;
    uint8_t x0 = (uint8_t)(gx * 8 + 1);
    uint8_t y0 = (uint8_t)(8 + gy * 8 + 1);   // page 0 is the score bar
    OLED_FillRect(x0, y0, (uint8_t)(x0 + 5), (uint8_t)(y0 + 5), on);
}

void DisplayTask(void *argument)
{
    char b[8];
    uint8_t blink = 0;
    uint8_t cnt = 0;

    OLED_Clear();
    OLED_DrawString(0, 0, "SCORE ");
    OLED_Flush();

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(100));     // 10 Hz refresh

        if (g_state == 1)
        {
            // game over screen: keep it simple, redraw every cycle
            OLED_Clear();
            OLED_DrawString(2, 32, "GAME OVER");
            OLED_DrawString(4, 16, "SCORE ");
            fmt_uint(b, g_score, 4);
            OLED_DrawString(4, 64, b);
            OLED_DrawString(6, 8, "TURN TO RESTART");
            OLED_Flush();
            continue;
        }

        // ---- playing: redraw whole field each cycle ----
        OLED_Clear();
        OLED_DrawString(0, 0, "SCORE ");
        fmt_uint(b, g_score, 4);
        OLED_DrawString(0, 48, b);

        // snake body
        for (uint8_t i = 0; i < g_snake_len; i++)
            draw_cell(g_snake[i].x, g_snake[i].y, 1);
        // food: a hollow 8x8 outline so it reads differently from the snake
        if (g_food.x < GRID_W && g_food.y < GRID_H)
        {
            uint8_t x0 = (uint8_t)(g_food.x * 8);
            uint8_t y0 = (uint8_t)(8 + g_food.y * 8);
            OLED_FillRect(x0, y0, (uint8_t)(x0 + 7), (uint8_t)(y0 + 7), 1);
        }

        // blinking dot on the score row = task alive
        if (++cnt >= 5)
        {
            cnt = 0;
            blink = (uint8_t)!blink;
            OLED_SetPixel(126, 0, blink);
        }

        OLED_Flush();           // push only the pages that changed
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
