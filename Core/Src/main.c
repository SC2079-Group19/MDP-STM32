/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ICM20948.h"
#include "oled.h"
#include "stdlib.h"
#include "stdio.h"
#include "math.h"

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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* Definitions for encoderTask */
osThreadId_t encoderTaskHandle;
const osThreadAttr_t encoderTask_attributes = {
    .name = "encoderTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for OledTask */
osThreadId_t OledTaskHandle;
const osThreadAttr_t OledTask_attributes = {
    .name = "OledTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for FWTask */
osThreadId_t FWTaskHandle;
const osThreadAttr_t FWTask_attributes = {
    .name = "FWTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for BWTask */
osThreadId_t BWTaskHandle;
const osThreadAttr_t BWTask_attributes = {
    .name = "BWTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for FLTask */
osThreadId_t FLTaskHandle;
const osThreadAttr_t FLTask_attributes = {
    .name = "FLTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for FRTask */
osThreadId_t FRTaskHandle;
const osThreadAttr_t FRTask_attributes = {
    .name = "FRTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for BLTask */
osThreadId_t BLTaskHandle;
const osThreadAttr_t BLTask_attributes = {
    .name = "BLTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for BRTask */
osThreadId_t BRTaskHandle;
const osThreadAttr_t BRTask_attributes = {
    .name = "BRTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for ADCTask */
osThreadId_t ADCTaskHandle;
const osThreadAttr_t ADCTask_attributes = {
    .name = "ADCTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for moveDistTask */
osThreadId_t moveDistTaskHandle;
const osThreadAttr_t moveDistTask_attributes = {
    .name = "moveDistTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for cmdTask */
osThreadId_t cmdTaskHandle;
const osThreadAttr_t cmdTask_attributes = {
    .name = "cmdTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* USER CODE BEGIN PV */

uint8_t RX_BUFFER_SIZE = 5;
uint8_t aRxBuffer[10];

typedef struct _command
{
  uint8_t index;
  uint16_t val;
} Command;

uint8_t CMD_BUFFER_SIZE = 12;
typedef struct _commandQueue
{
  uint8_t head;
  uint8_t tail;
  uint8_t size;
  Command buffer[12];
} CommandQueue;

CommandQueue cQueue;

Command curCmd;
uint8_t rxMsg[16];
char ch[16];

uint8_t manualMode = 0;

// imu
float targetAngle = 0;
float angleNow = 0;
float angleTemp = 0;
uint8_t readGyroZData[2];
int16_t gyroZ = 0;
int16_t readGyroData[3];

int16_t readAccData[3];
int16_t accX = 0;
int16_t accY = 0;

// motor
uint16_t newDutyL, newDutyR;
uint32_t last_curTask_tick = 0;

// diatance
float targetDist = 0;
uint16_t curDistTick = 0;
uint16_t targetDistTick = 0;
uint16_t dist_dL = 0;
uint16_t lastDistTick_L = 0;

typedef struct _pidConfig
{
  float Kp;
  float Ki;
  float Kd;
  float ek1;
  float ekSum;
} PIDConfig;

PIDConfig pidSlow, pidTSlow, pidFast;

typedef struct _commandConfig
{
  uint16_t leftDuty;
  uint16_t rightDuty;
  float servoTurnVal;
  float targetAngle;
  uint8_t direction;
} CmdConfig;

// command value used for Command struct
#define CONFIG_FL00 7
#define CONFIG_FR00 8
#define CONFIG_BL00 9
#define CONFIG_BR00 10

#define CONFIG_FL20 11
#define CONFIG_FR20 12
#define CONFIG_BL20 13
#define CONFIG_BR20 14

#define CONFIG_FL30 15
#define CONFIG_FR30 16
#define CONFIG_BL30 17
#define CONFIG_BR30 18

CmdConfig cfgs[19] = {
    {0, 0, SERVO_CENTER, 0, DIR_FORWARD},        // STOP
    {1200, 1200, SERVO_CENTER, 0, DIR_FORWARD},  // FW00
    {1200, 1200, SERVO_CENTER, 0, DIR_BACKWARD}, // BW00

    {800, 1200, 50, 0, DIR_FORWARD},   // FL--
    {1200, 800, 115, 0, DIR_FORWARD},  // FR--
    {800, 1200, 50, 0, DIR_BACKWARD},  // BL--
    {1200, 800, 115, 0, DIR_BACKWARD}, // BR--

    {700, 1800, 50, 89, DIR_FORWARD},   // FL00
    {1800, 400, 115, -87, DIR_FORWARD}, // FR00
    {500, 1700, 50, -88, DIR_BACKWARD}, // BL00
    {1800, 500, 115, 89, DIR_BACKWARD}, // BR00,

    {800, 1800, 51.85, 89, DIR_FORWARD}, // FL20
    {1800, 900, 115, -87, DIR_FORWARD},  // FR20
    {700, 1800, 50, -89, DIR_BACKWARD},  // BL20
    {1800, 700, 115, 89, DIR_BACKWARD},  // BR20,

    {1500, 1500, 53, 87.5, DIR_FORWARD},   // FL30
    {1500, 1500, 108, -86.5, DIR_FORWARD}, // FR30
    {1500, 1500, 51, -87.5, DIR_BACKWARD}, // BL30
    {1500, 1100, 115, 88, DIR_BACKWARD},   // BR30
};

enum TASK_TYPE
{
  TASK_MOVE_FOREWARD,
  TASK_MOVE_BACKWARD,
  TASK_FL,
  TASK_FR,
  TASK_BL,
  TASK_BR,
  // TASK_ADC,
  // TASK_MOVE_OBS,
  // TASK_FASTESTPATH,
  // TASK_BUZZER,
  TASK_NONE
};
enum TASK_TYPE curTask = TASK_NONE, prevTask = TASK_NONE;

enum MOVE_MODE
{
  SLOW,
  FAST
};
enum MOVE_MODE moveMode = FAST;

// ADC
uint16_t obsTick_Ultrasonic = 0;

float obsDist_IR_L = 0, obsDist_IR_LR = 0, obsDist_US = 0; // left/right IR and front ultrasonic
// float IR_data_raw_acc = 0, dataPoint = 0;
uint16_t dataPoint = 0;
uint32_t IR_data_raw_acc_L = 0;
uint32_t IR_data_raw_acc_R = 0;

float speedScale = 1;
uint16_t pwmVal = 1000;

// battery
float batteryVal;

// fastest path variable
float obs_a, x, angle_left, angle_right;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
void runEncoder(void *argument);
void runOledTask(void *argument);
void runFWTask(void *argument);
void runBWTask(void *argument);
void runFLTask(void *argument);
void runFRTask(void *argument);
void runBLTask(void *argument);
void runBRTask(void *argument);
void runADCTask(void *argument);
void runMoveDistTask(void *argument);
void runCmdTask(void *argument);

/* USER CODE BEGIN PFP */
void PIDConfigInit(PIDConfig *cfg, const float Kp, const float Ki, const float Kd);
void PIDConfigReset(PIDConfig *cfg);

void StraightLineMove(const uint8_t speedMode);
void StraightLineMoveDist(const uint8_t speedMode);
void StraightLineMoveSpeedScale(const uint8_t speedMode, float *speedScale);
void RobotMoveDist(float *targetDist, const uint8_t dir, const uint8_t speedMode);
void RobotMoveDistObstacle(float *targetDist, const uint8_t speedMode);

void RobotTurn(float *targetAngle);
void RobotTurnFastest(float *targetAngle);

// update angleNow
void getAngle();

uint32_t IC_Val1 = 0, IC_Val2 = 0;
uint8_t Is_First_Captured = 0;

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
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  // oled
  OLED_Init();

  // gyro
  ICM20948_init(&hi2c1, 0, GYRO_FULL_SCALE_2000DPS, ACCEL_FULL_SCALE_2G);

  // accelerometer

  // servo

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  // motor
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

  // encoder
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  __RESET_SERVO_TURN(&htim1);

  // TODO:pid controller

  // command queue initialization
  curCmd.index = 100;
  curCmd.val = 10;

  cQueue.head = 0;
  cQueue.tail = 0;
  cQueue.size = CMD_BUFFER_SIZE;
  for (int i = 0; i < CMD_BUFFER_SIZE; i++)
  {
    Command cmd;
    cmd.index = 100;
    cmd.val = 0;
    cQueue.buffer[i] = cmd;
  }

  // overwrite curCmd for debugging individual task
  curCmd.index = 1;
  curCmd.val = 00;

  // UART Rx
  HAL_UART_Receive_IT(&huart3, aRxBuffer, RX_BUFFER_SIZE);

  // adjust steering
  __RESET_SERVO_TURN(&htim1);

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
  /* creation of encoderTask */
  encoderTaskHandle = osThreadNew(runEncoder, NULL, &encoderTask_attributes);

  /* creation of OledTask */
  OledTaskHandle = osThreadNew(runOledTask, NULL, &OledTask_attributes);

  /* creation of FWTask */
  FWTaskHandle = osThreadNew(runFWTask, NULL, &FWTask_attributes);

  /* creation of BWTask */
  BWTaskHandle = osThreadNew(runBWTask, NULL, &BWTask_attributes);

  /* creation of FLTask */
  FLTaskHandle = osThreadNew(runFLTask, NULL, &FLTask_attributes);

  /* creation of FRTask */
  FRTaskHandle = osThreadNew(runFRTask, NULL, &FRTask_attributes);

  /* creation of BLTask */
  BLTaskHandle = osThreadNew(runBLTask, NULL, &BLTask_attributes);

  /* creation of BRTask */
  BRTaskHandle = osThreadNew(runBRTask, NULL, &BRTask_attributes);

  /* creation of ADCTask */
  ADCTaskHandle = osThreadNew(runADCTask, NULL, &ADCTask_attributes);

  /* creation of moveDistTask */
  moveDistTaskHandle = osThreadNew(runMoveDistTask, NULL, &moveDistTask_attributes);

  /* creation of cmdTask */
  cmdTaskHandle = osThreadNew(runCmdTask, NULL, &cmdTask_attributes);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 160;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
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
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
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
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 7199;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin | OLED_SDA_Pin | OLED_RST_Pin | OLED_DC_Pin | LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin | AIN1_Pin | BIN1_Pin | BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RST_Pin OLED_DC_Pin
                           LED3_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin | OLED_SDA_Pin | OLED_RST_Pin | OLED_DC_Pin | LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN2_Pin AIN1_Pin */
  GPIO_InitStruct.Pin = AIN2_Pin | AIN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BIN1_Pin BIN2_Pin */
  GPIO_InitStruct.Pin = BIN1_Pin | BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  UNUSED(huart);
  HAL_UART_Transmit(&huart3, (uint8_t *)aRxBuffer, 10, 0xffff);
}

// pid
void PIDConfigInit(PIDConfig *cfg, const float Kp, const float Ki, const float Kd)
{
  cfg->Kp = Kp;
  cfg->Ki = Ki;
  cfg->Kd = Kd;
  cfg->ek1 = 0;
  cfg->ekSum = 0;
}

void PIDConfigReset(PIDConfig *cfg)
{
  cfg->ek1 = 0;
  cfg->ekSum = 0;
}

int8_t dir = 1;
int correction = 0;

void StraightLineMove(const uint8_t speedMode)
{
  __Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ); // polling
  dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);
  angleNow += ((gyroZ >= -4 && gyroZ <= 11) ? 0 : gyroZ); // / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;s

  if (speedMode == SPEED_MODE_T)
    __PID_SPEED_T(pidTSlow, angleNow, correction, dir, newDutyL, newDutyR);
  else if (speedMode == SPEED_MODE_2)
    __PID_SPEED_2(pidFast, angleNow, correction, dir, newDutyL, newDutyR);
  else if (speedMode == SPEED_MODE_1)
    __PID_SPEED_1(pidSlow, angleNow, correction, dir, newDutyL, newDutyR);

  __SET_MOTOR_DUTY(&htim8, newDutyL, newDutyR);
}
void StraightLineMoveDist(const uint8_t speedMode)
{
  int dist = 10; // for testing, distance = 10 cm
  angleNow = 0;
  gyroZ = 0;
  __SET_MOTOR_DUTY(&htim8, newDutyL, newDutyR);
}

void RobotMoveDist(float *targetDist, const uint8_t dir, const uint8_t speedMode)
{
  angleNow = 0;
  gyroZ = 0; // reset angle for PID
  PIDConfigReset(&pidTSlow);
  PIDConfigReset(&pidSlow);
  PIDConfigReset(&pidFast);
  curDistTick = 0;

  __GET_TARGETTICK(*targetDist, targetDistTick);

  last_curTask_tick = HAL_GetTick();
  __SET_MOTOR_DIRECTION(dir);
  __SET_ENCODER_LAST_TICK(&htim2, lastDistTick_L);
  do
  {
    __GET_ENCODER_TICK_DELTA(&htim2, lastDistTick_L, dist_dL);
    curDistTick += dist_dL;

    if (curDistTick >= targetDistTick)
      break;

    if (HAL_GetTick() - last_curTask_tick >= 10)
    {
      if (speedMode == SPEED_MODE_T)
      {
        StraightLineMove(SPEED_MODE_T);
      }
      else
      {
        speedScale = abs(curDistTick - targetDistTick) / 990; // start to slow down at last 990 ticks (15cm)
        if (speedMode == SPEED_MODE_1)
          speedScale = speedScale > 1 ? 1 : (speedScale < 0.75 ? 0.75 : speedScale);
        else if (speedMode == SPEED_MODE_2)
          speedScale = speedScale > 1 ? 1 : (speedScale < 0.4 ? 0.4 : speedScale);
        StraightLineMoveSpeedScale(speedMode, &speedScale);
      }

      last_curTask_tick = HAL_GetTick();
    }
  } while (1);
  __SET_MOTOR_DUTY(&htim8, 0, 0);
}

void StraightLineMoveSpeedScale(const uint8_t speedMode, float *speedScale)
{
  __Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ);            // polling
  dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2) ? 1 : -1;  // use only one of the wheel to determine car direction
  angleNow += ((gyroZ >= -4 && gyroZ <= 11) ? 0 : gyroZ); // / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;
  if (speedMode == SPEED_MODE_1)
    __PID_SPEED_1(pidSlow, angleNow, correction, dir, newDutyL, newDutyR);
  else if (speedMode == SPEED_MODE_2)
    __PID_SPEED_2(pidFast, angleNow, correction, dir, newDutyL, newDutyR);

  __SET_MOTOR_DUTY(&htim8, newDutyL * (*speedScale), newDutyR * (*speedScale));
}

void getAngle()
{
  // __Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ);
  // gyroZ = gyroZ / GRYO_SENSITIVITY_SCALE_FACTOR_250DPS;
  ICM20948_readGyroscope_allAxises(&hi2c1, 0, GYRO_FULL_SCALE_250DPS, readGyroData);
  ICM20948_readAccelerometer_allAxises(&hi2c1, 0, ACCEL_FULL_SCALE_2G, readAccData);

  float yaw_accel, yaw_gyro, millisNow, millisOld, yaw_CF, dt;
  yaw_accel = yaw_gyro = yaw_CF = 0;
  millisNow = millisOld = 0;

  float a = 0.95;

  yaw_accel = atan2(readAccData[1], readAccData[0]);
  yaw_accel = yaw_accel / 3.14159265 * 180;

  millisNow = HAL_GetTick();
  dt = millisNow - millisOld;
  millisOld = millisNow;

  yaw_gyro = yaw_gyro + readGyroData[2] * dt * 0.001;

  angleNow = (1 - a) * yaw_accel + a * (angleNow + readGyroData[2] * dt * 0.001);
}

// For tesing:
uint16_t newDutyL = 500;
uint16_t newDutyR = 500;
uint16_t period = 4000;
uint16_t startTick;
uint16_t curTick;

uint8_t curTestingTask = 00;

/* USER CODE END 4 */

/* USER CODE BEGIN Header_runEncoder */
/**
 * @brief  Function implementing the encoderTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_runEncoder */
void runEncoder(void *argument)
{
  /* USER CODE BEGIN 5 */
  // HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  int cnt1 = 0, cnt2 = 0, diff = 0;

  uint32_t tick = 0;

  cnt1 = __HAL_TIM_GET_COUNTER(&htim2);
  tick = HAL_GetTick();

  // uint8_t encoderBuffer[20];
  uint8_t speedBuffer[20];
  uint8_t directionBuffer[10];
  dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);

  /* Infinite loop */

  for (;;)
  {
    // HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_10);
    if (HAL_GetTick() - tick > 1000L)
    {
      cnt2 = __HAL_TIM_GET_COUNTER(&htim2);
      if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2))
      {
        if (cnt2 < cnt1)
        {
          diff = cnt1 - cnt2;
        }
        else
          diff = (65535 - cnt2) + cnt1;
      }
      else
      {
        if (cnt2 > cnt1)
        {
          diff = cnt2 - cnt1;
        }
        else
          diff = (65535 - cnt1) + cnt2;
      }

      // display on oled
      sprintf(speedBuffer, "Speed:%5d\0", diff);
      OLED_ShowString(0, 0, speedBuffer);
      sprintf(directionBuffer, "Dir:%5d\0", dir);
      OLED_ShowString(0, 15, directionBuffer);

      // OLED_Refresh_Gram();
      cnt1 = __HAL_TIM_GET_COUNTER(&htim2);
      tick = HAL_GetTick();
    }

    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_runOledTask */
/**
 * @brief Function implementing the OledTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_runOledTask */
void runOledTask(void *argument)
{
  /* USER CODE BEGIN runOledTask */
  /* Infinite loop */

  for (;;)
  {

    // __Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ);
    angleTemp = angleNow / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;
    // angleNow += ((gyroZ >= -4 && gyroZ <= 11) ? 0 : gyroZ);
    snprintf(ch, sizeof(ch), "angle:%-4d", (int)angleTemp);
    OLED_ShowString(0, 40, (char *)ch);

    OLED_Refresh_Gram();
    osDelay(10);
  }
  /* USER CODE END runOledTask */
}

/* USER CODE BEGIN Header_runFWTask */
/**
 * @brief Function implementing the FWTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_runFWTask */
void runFWTask(void *argument)
{
  /* USER CODE BEGIN runFWTask */

  /* Infinite loop */
  for (;;)
  {
    newDutyL = 500;
    newDutyR = 500;
    if (curTask != TASK_MOVE_FOREWARD)
      osDelay(100);
    else
    {
      OLED_ShowString(0, 25, (uint8_t *)("FW\0"));
      __SET_MOTOR_DIRECTION(DIR_FORWARD);
      __RESET_SERVO_TURN(&htim1);
      OLED_ShowNumber(0, 50, (&htim1)->Instance->CCR4, 5, 12);
      startTick = HAL_GetTick();
      while (HAL_GetTick() - startTick < period)
      {
        __SET_MOTOR_DUTY(&htim8, newDutyL, newDutyR);
      }
      // RobotMoveDist(&targetDist, curTask == TASK_MOVE_FOREWARD ? DIR_FORWARD : DIR_BACKWARD, SPEED_MODE_1);
      // targetDist = (float)curCmd.val;
      // if (targetDist <= 15)
      //   moveMode = SLOW;

      // if (moveMode == SLOW)
      // {
      //   RobotMoveDist(&targetDist, curTask == TASK_MOVE_FOREWARD ? DIR_FORWARD : DIR_BACKWARD, SPEED_MODE_1);
      // }
      // else
      // {
      //   RobotMoveDist(&targetDist, curTask == TASK_MOVE_FOREWARD ? DIR_FORWARD : DIR_BACKWARD, SPEED_MODE_2);
      // }

      osDelay(100);
    }
    osDelay(1);
  }

  /* USER CODE END runFWTask */
}

/* USER CODE BEGIN Header_runBWTask */
/**
 * @brief Function implementing the BWTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_runBWTask */
void runBWTask(void *argument)
{
  /* USER CODE BEGIN runBWTask */

  /* Infinite loop */
  for (;;)
  {
    if (curTask != TASK_MOVE_BACKWARD)
      osDelay(100);

    else
    {
      OLED_ShowString(0, 30, (uint8_t *)("FW\0"));
      __SET_MOTOR_DIRECTION(DIR_BACKWARD);
      __RESET_SERVO_TURN(&htim1);
      OLED_ShowNumber(0, 50, (&htim1)->Instance->CCR4, 5, 12);
      startTick = HAL_GetTick();
      while (HAL_GetTick() - startTick < period)
      {
        __SET_MOTOR_DUTY(&htim8, newDutyL, newDutyR);
      }
    }
  }
  /* USER CODE END runBWTask */
}

/* USER CODE BEGIN Header_runFLTask */
/**
 * @brief Function implementing the FLTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_runFLTask */
void runFLTask(void *argument)
{
  /* USER CODE BEGIN runFLTask */

  /* Infinite loop */
  for (;;)
  {
    if (curTask != TASK_FL)
      osDelay(100);
    else
    {
      OLED_ShowString(0, 30, (uint8_t *)("FW\0"));
      __SET_MOTOR_DIRECTION(DIR_FORWARD);
      __SET_SERVO_TURN_MAX(&htim1, 0);
      OLED_ShowNumber(0, 50, (&htim1)->Instance->CCR4, 5, 12);
      startTick = HAL_GetTick();
      while (HAL_GetTick() - startTick < period)
      {
        __SET_MOTOR_DUTY(&htim8, newDutyL, newDutyR);
      }
    }
  }
  /* USER CODE END runFLTask */
}

/* USER CODE BEGIN Header_runFRTask */
/**
 * @brief Function implementing the FRTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_runFRTask */
void runFRTask(void *argument)
{
  /* USER CODE BEGIN runFRTask */

  /* Infinite loop */
  for (;;)
  {
    if (curTask != TASK_FR)
      osDelay(100);
    else
    {
      OLED_ShowString(0, 30, (uint8_t *)("FW\0"));
      __SET_MOTOR_DIRECTION(DIR_FORWARD);
      __SET_SERVO_TURN_MAX(&htim1, 1);
      OLED_ShowNumber(0, 50, (&htim1)->Instance->CCR4, 5, 12);
      startTick = HAL_GetTick();
      while (HAL_GetTick() - startTick < period)
      {
        __SET_MOTOR_DUTY(&htim8, newDutyL, newDutyR);
      }
    }
    /* USER CODE END runFRTask */
  }
}

/* USER CODE BEGIN Header_runBLTask */
/**
 * @brief Function implementing the BLTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_runBLTask */
void runBLTask(void *argument)
{
  /* USER CODE BEGIN runBLTask */

  /* Infinite loop */
  for (;;)
  {
    if (curTask != TASK_BL)
      osDelay(100);
    else
    {
      OLED_ShowString(0, 30, (uint8_t *)("FW\0"));
      __SET_MOTOR_DIRECTION(DIR_BACKWARD);
      __SET_SERVO_TURN_MAX(&htim1, 0);
      OLED_ShowNumber(0, 50, (&htim1)->Instance->CCR4, 5, 12);
      startTick = HAL_GetTick();
      while (HAL_GetTick() - startTick < period)
      {
        __SET_MOTOR_DUTY(&htim8, newDutyL, newDutyR);
      }
    }
  }
  /* USER CODE END runBLTask */
}

/* USER CODE BEGIN Header_runBRTask */
/**
 * @brief Function implementing the BRTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_runBRTask */
void runBRTask(void *argument)
{
  /* USER CODE BEGIN runBRTask */

  /* Infinite loop */
  for (;;)
  {
    if (curTask != TASK_BR)
      osDelay(100);
    else
    {
      OLED_ShowString(0, 30, (uint8_t *)("FW\0"));
      __SET_MOTOR_DIRECTION(DIR_BACKWARD);
      __SET_SERVO_TURN_MAX(&htim1, 1);
      OLED_ShowNumber(0, 50, (&htim1)->Instance->CCR4, 5, 12);
      startTick = HAL_GetTick();
      while (HAL_GetTick() - startTick < period)
      {
        __SET_MOTOR_DUTY(&htim8, newDutyL, newDutyR);
      }
    }
  }
  /* USER CODE END runBRTask */
}

/* USER CODE BEGIN Header_runADCTask */
/**
 * @brief Function implementing the ADCTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_runADCTask */
void runADCTask(void *argument)
{
  /* USER CODE BEGIN runADCTask */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END runADCTask */
}

/* USER CODE BEGIN Header_runMoveDistTask */
/**
 * @brief Function implementing the moveDistTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_runMoveDistTask */
void runMoveDistTask(void *argument)
{
  /* USER CODE BEGIN runMoveDistTask */
  /* Infinite loop */
  for (;;)
  {
    //   if (curTask != TASK_MOVE_FOREWARD && curTask != TASK_MOVE_BACKWARD)
    //     osDelay(1000);
    //   else
    //   {
    //     OLED_ShowString(0, 30, (uint8_t *)("FW\0"));
    //     // __SET_MOTOR_DIRECTION(DIR_FORWARD);
    //     __RESET_SERVO_TURN(&htim1);
    //     OLED_ShowNumber(0, 50, (&htim1)->Instance->CCR4, 5, 12);
    //     // startTick = HAL_GetTick();
    //     // while (HAL_GetTick() - startTick < period)
    //     // {
    //     //   // __SET_MOTOR_DUTY(&htim8, newDutyL, newDutyR);
    //     // }
    //     targetDist = (float)curCmd.val;
    //     // for target distance lesser than 15, move mode must be forced to SLOW
    //     if (targetDist <= 15)
    //       moveMode = SLOW;

    //     if (moveMode == SLOW)
    //     {
    //       RobotMoveDist(&targetDist, curTask == TASK_MOVE_FOREWARD ? DIR_FORWARD : DIR_BACKWARD, SPEED_MODE_1);
    //     }
    //     else
    //     {
    //       RobotMoveDist(&targetDist, curTask == TASK_MOVE_FOREWARD ? DIR_FORWARD : DIR_BACKWARD, SPEED_MODE_2);
    //     }

    //     // __ON_TASK_END(&htim8, prevTask, curTask);
    //     // // clickOnce = 0;

    //     // if (__COMMAND_QUEUE_IS_EMPTY(cQueue))
    //     // {
    //     //   __CLEAR_CURCMD(curCmd);
    //     //   __ACK_TASK_DONE(&huart3, rxMsg);
    //     // }
    //     // else
    //     //   __READ_COMMAND(cQueue, curCmd, rxMsg);
    //   }

    //   osDelay(100);
    osDelay(1);
  }

  /* USER CODE END runMoveDistTask */
}

/* USER CODE BEGIN Header_runCmdTask */
/**
 * @brief Function implementing the cmdTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_runCmdTask */
void runCmdTask(void *argument)
{
  /* USER CODE BEGIN runCmdTask */
  /* Infinite loop */
  for (;;)
  {
    switch (curCmd.index)
    {
      //	  	 case 0: // STOP handled in UART IRQ directly
      //	  	  	  break;
    case 1: // FW
    case 2: // BW
      curTask = curCmd.index == 1 ? TASK_MOVE_FOREWARD : TASK_MOVE_BACKWARD;
      __PEND_CURCMD(curCmd);
      break;
    case 3: // FL manual
    case 4: // FR manual
    case 5: // BL manual
    case 6: // BR manual
      __SET_CMD_CONFIG(cfgs[curCmd.index], &htim8, &htim1, targetAngle);
      if (__COMMAND_QUEUE_IS_EMPTY(cQueue))
      {
        __CLEAR_CURCMD(curCmd);
        __ACK_TASK_DONE(&huart3, rxMsg);
      }
      else
        __READ_COMMAND(cQueue, curCmd, rxMsg);
      __PEND_CURCMD(curCmd);
      break;
    case 7: // FL
      curTask = TASK_FL;
      __PEND_CURCMD(curCmd);
      break;
    case 8: // FR
      curTask = TASK_FR;
      __PEND_CURCMD(curCmd);
      break;
    case 9: // BL
      curTask = TASK_BL;
      __PEND_CURCMD(curCmd);
      break;
    case 10: // BR
      curTask = TASK_BR;
      __PEND_CURCMD(curCmd);
      break;
    case 11: // TL
    case 12: // TR
      __SET_SERVO_TURN_MAX(&htim1, curCmd.index - 11 ? 1 : 0);
      __CLEAR_CURCMD(curCmd);
      __ACK_TASK_DONE(&huart3, rxMsg);
      break;
    // case 13: // debug IR sensor
    //   curTask = TASK_ADC;
    //   break;
    // case 14: // DT move until specified distance from obstacle
    //   curTask = TASK_MOVE_OBS;
    //   __PEND_CURCMD(curCmd);
    //   break;
    // case 15:
    //   curTask = TASK_BUZZER;
    //   __PEND_CURCMD(curCmd);
    //   break;
    // case 16:
    //   curTask = TASK_FASTESTPATH;
    //   __PEND_CURCMD(curCmd);
    //   break;
    // case 17:
    //   curTask = TASK_FASTESTPATH_V2;
    //   __PEND_CURCMD(curCmd);
    //   break;
    case 88: // Axxx, rotate left by xxx degree
    case 89: // Cxxx, rotate right by xxx degree
      __SET_SERVO_TURN_MAX(&htim1, curCmd.index - 88);
      __SET_MOTOR_DIRECTION(DIR_FORWARD);
      if (curCmd.index == 88)
      {
        targetAngle = curCmd.val;
        __SET_MOTOR_DUTY(&htim8, 800, 1200);
      }
      else
      {
        targetAngle = -curCmd.val;
        __SET_MOTOR_DUTY(&htim8, 1200, 800);
      }
      __PEND_CURCMD(curCmd);
      // RobotTurn(&targetAngle);
      break;
    case 99:
      break;
    case 100:
      break;
    default:
      //		 curCmd.index = 99;
      break;
    }
    osDelay(1);
  }
  /* USER CODE END runCmdTask */
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
     ex: printf("Wrong parameters value: file %s on line %d\r\0", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
