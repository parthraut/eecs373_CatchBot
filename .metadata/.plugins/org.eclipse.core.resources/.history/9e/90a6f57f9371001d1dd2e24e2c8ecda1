/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef hlpuart1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include "stdio.h"

// ============================
//
// Start: Code originated from Pixy Documentation
//
// ============================

#define PIXY_ARRAYSIZE              100
#define PIXY_START_WORD             0xaa55
#define PIXY_START_WORD_CC          0xaa56
#define PIXY_START_WORDX            0x55aa

#define PIXY_SYNC_BYTE              0x5a
#define PIXY_SYNC_BYTE_DATA         0x5b

// the routines
void pixy_init();
int getStart(void);
uint16_t getBlocks(uint16_t maxBlocks);
static uint16_t getWord(void);
uint8_t getByte(uint8_t out);

typedef enum
{
  NORMAL_BLOCK,
  CC_BLOCK // color code block
} BlockType;

typedef struct
{
  uint16_t signature;
  uint16_t x;
  uint16_t y;
  uint16_t width;
  uint16_t height;
  uint16_t angle; // angle is only available for color coded blocks
} Block;

static BlockType g_blockType; // use this to remember the next object block type between function calls
static int g_skipStart = 0;
static Block *g_blocks;

int getStart(void)
{
  uint16_t w, lastw;

  lastw = 0xffff; // some inconsequential initial value

  while(1)
  {
    w = getWord();
    //printf("%x\n", w);
    if (w==0 && lastw==0){

      return 0; // in I2C and SPI modes this means no data, so return immediately
    }
    else if (w==PIXY_START_WORD && lastw==PIXY_START_WORD)
    {
      g_blockType = NORMAL_BLOCK; // remember block type
      return 1; // code found!
    }
    else if (w==PIXY_START_WORD_CC && lastw==PIXY_START_WORD)
    {
      g_blockType = CC_BLOCK; // found color code block
      return 1;
    }
    else if (w==PIXY_START_WORDX) // this is important, we might be juxtaposed
      getByte(0); // we're out of sync! (backwards)
    lastw = w; // save
  }
}

uint16_t getBlocks(uint16_t maxBlocks)
{
  uint8_t i;
  uint16_t w, blockCount, checksum, sum;
  Block *block;

  if (!g_skipStart)
  {
    if (getStart()==0)
      return 0;
  }
  else
    g_skipStart = 0;

  for(blockCount=0; blockCount<maxBlocks && blockCount<PIXY_ARRAYSIZE;)
  {
    checksum = getWord();
    if (checksum==PIXY_START_WORD) // we've reached the beginning of the next frame
    {
      g_skipStart = 1;
      g_blockType = NORMAL_BLOCK;
      return blockCount;
    }
    else if (checksum==PIXY_START_WORD_CC)
    {
      g_skipStart = 1;
      g_blockType = CC_BLOCK;
      return blockCount;
    }
    else if (checksum==0)
      return blockCount;

    block = g_blocks + blockCount;

    for (i=0, sum=0; i<sizeof(Block)/sizeof(uint16_t); i++)
    {
      if (g_blockType==NORMAL_BLOCK && i>=5) // no angle for normal block
      {
        block->angle = 0;
        break;
      }
      w = getWord();
      sum += w;
      *((uint16_t *)block + i) = w;
    }

    // check checksum
    if (checksum==sum)
      blockCount++;
    else
      printf("checksum error!\n");

    w = getWord();
    if (w==PIXY_START_WORD)
      g_blockType = NORMAL_BLOCK;
    else if (w==PIXY_START_WORD_CC)
      g_blockType = CC_BLOCK;
    else
      return blockCount;
  }
}

void pixy_init()
{
  g_blocks = (Block *)malloc(sizeof(Block)*PIXY_ARRAYSIZE);
}

// ============================
//
// End: Code originated from Pixy Documentation
//
// ============================

// ============================
//
// Start: Code to handle PWM and State
//
// ============================

// Sets a timer's PWM to a value
// timerIndex = which timer (eg TIM2 -> pass 2, TIM3 -> pass 3)
// pwmVal = value to set that timer's CCR to; these should be defined below
void SetPWM(uint16_t timerIndex, uint16_t pwmVal)
{
	TIM_TypeDef* timer;

	switch (timerIndex)
	{
	case 2:
		timer = TIM2;
		break;
	case 3:
		timer = TIM3;
		break;
	case 4:
		timer = TIM4;
		break;
	default:
		return; // just in case.
	}

	timer->CCR3 = pwmVal;
}

// States the machine will be in
#define STATE_FIND_BALL		0
#define STATE_GRAB_BALL		1
#define STATE_RETURN_BALL	2

uint8_t currentState = STATE_FIND_BALL;

// Updates the current state
void UpdateState(uint8_t nextState)
{
	currentState = nextState;
	switch(currentState)
	{
	case STATE_FIND_BALL:
		ResetAverage();
		break;
	case STATE_GRAB_BALL:
		break;
	case STATE_RETURN_BALL:
		ResetAverage();
		break;
	}
}

// ============================
//
// Start: Code to handle data from the Pixy cam
//
// ============================

// Structures to get data from the Pixy cam
uint8_t rx_data[2];
uint8_t tx_data[2] = { PIXY_SYNC_BYTE, 0 };

// Gets one byte from the Pixy cam
uint8_t getByte(uint8_t output)
{
	HAL_StatusTypeDef hal_status;
	uint8_t data = 0;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	hal_status = HAL_SPI_TransmitReceive(&hspi1, &output, &data, 1, 0xFFFFFFFF);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

	if (hal_status != HAL_OK)
	{
		printf("not ok\n");
	}

	return data;
}

// Gets a word (2 bytes) from the Pixy cam
uint16_t getWord(void)
{
	uint16_t w;

	rx_data[0] = getByte(PIXY_SYNC_BYTE_DATA);
	rx_data[1] = getByte(0);

	w = rx_data[0];
	w <<= 8;
	w |= rx_data[1];

	return w;
}

// Variables for handling the running average
// We're just keeping track of the X value of whatever we're tracking
uint16_t xVals[4] = { 0, 0, 0, 0 };
uint16_t xValsIndex = 0;
float runningAverage = 0.0f;

// Resets the running average and array
void ResetAverage()
{
	xVals[0] = xVals[1] = xVals[2] = xVals[3] = 0;
	runningAverage = 0.0f;
}

// Gets the index of whatever signature we're looking for in the block data from the Pixy cam
// I'm basically assuming whatever we're looking for will be the first piece of data in the blocks.
uint16_t GetSignatureIndex(uint32_t num_blocks, uint8_t sigVal)
{
	int i = 0;
	while (i < num_blocks)
	{
		if (g_blocks[i].signature == sigVal)
			break;
		i++;
	}
	return i;
}

// Signatures of the items we're looking for
// Will need to make sure these match the actual signatures in the Pixy cam
#define PIXY_SIG_BALL				1
#define PIXY_SIG_RETURN				2

// temporary, idk if it'll even work
#define HEIGHT	185

// Gets the X value of the signature items and updates the running average
// sig = one of the two signatures defined above
void GetNextX(uint8_t sig)
{
	uint32_t num_blocks = getBlocks(PIXY_ARRAYSIZE);
	if (!num_blocks)
		return;

	uint16_t index = GetSignatureIndex(num_blocks, sig);
	uint16_t newX = 0;
	if (index != num_blocks)
	{
		newX = g_blocks[index].x;
	}

	//printf("width: %d\n", g_blocks[index].height);
	//printf("y val: %d\n", g_blocks[index].y);
	// x vals don't go above 320
	int16_t diff = newX - xVals[xValsIndex];
	runningAverage += diff * 0.25f;

	xVals[xValsIndex] = newX;
	xValsIndex = (xValsIndex + 1) % 4;

	// I have no idea if this will even work, I just wanted to make sure there was a way to get to each state
	if (g_blocks[index].y >= HEIGHT && currentState == STATE_FIND_BALL)
		UpdateState(STATE_GRAB_BALL);
}


// Updates the Pixy cam
void UpdatePixyCam()
{
	switch (currentState)
	{
	case STATE_FIND_BALL:
		GetNextX(PIXY_SIG_BALL);
		break;
	case STATE_GRAB_BALL:
		return; // pixy cam doesn't need to do anything here
	case STATE_RETURN_BALL:
		GetNextX(PIXY_SIG_RETURN);
		break;
	}
}

// ============================
//
// Start: Code to handle wheel control
//
// ============================

// Defines to make code more readable.
#define WHEEL_LEFT			2
#define WHEEL_RIGHT			3
#define WHEEL_SPEED_ZERO	139
#define WHEEL_SPEED_FULL	80
#define WHEEL_SPEED_HALF	120
#define MAX_BALL_X			220
#define MIN_BALL_X			100
#define MAX_RETURN_X		190
#define MIN_RETURN_X		130

// Sets the wheel speed
// speed = PWM value for the wheel
// wheel = which wheel to use
// Use the defines above.
void SetWheelSpeed(uint8_t wheel, uint8_t speed)
{
	// still need to determine which timer is controlling which wheel
	// speed = PWM value
	SetPWM(wheel, speed);
}

// Updates the wheel speed
// avgMax = max value the running average can be
// avgMin = min value the running average can be
// speed = speed to set the wheels
// This will turn off one of the wheels depending on where the ball is
void UpdateWheelSpeed(uint16_t avgMax, uint16_t avgMin, uint8_t speed)
{
	uint8_t wheelLeftSpeed = speed;
	uint8_t wheelRightSpeed = speed;
	if (runningAverage < avgMin)
		wheelLeftSpeed = WHEEL_SPEED_ZERO;
	if (runningAverage > avgMax)
		wheelRightSpeed = WHEEL_SPEED_ZERO;

	SetWheelSpeed(WHEEL_LEFT, wheelLeftSpeed);
	SetWheelSpeed(WHEEL_RIGHT, wheelRightSpeed);
}

// Updates the wheels
void UpdateWheels()
{
	uint8_t speed = WHEEL_SPEED_FULL;
	switch (currentState)
	{
	case STATE_FIND_BALL:
		UpdateWheelSpeed(MAX_BALL_X, MIN_BALL_X, speed);
		break;
	case STATE_GRAB_BALL:
		SetWheelSpeed(WHEEL_LEFT, WHEEL_SPEED_HALF);
		SetWheelSpeed(WHEEL_RIGHT, WHEEL_SPEED_HALF);
		break;
	case STATE_RETURN_BALL:
		UpdateWheelSpeed(MAX_RETURN_X, MIN_RETURN_X, speed);
		break;
	}
}

// ============================
//
// Start: Code to handle IR Sensor
//
// ============================

#define DIST_TO_BALL	11.0f

// Checks the value reported by the IR sensor
uint8_t CheckIR()
{
	uint16_t adcVal = 0;
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 0xFFFFFFFF);
	adcVal = HAL_ADC_GetValue(&hadc1);
	float distVal = exp((log(adcVal) - 10.66) / (-1.123));
	//printf("distVal: %f\n", distVal);
	if (distVal <= DIST_TO_BALL)
		return 1;
	return 0;
}

// ============================
//
// Start: Code to handle claw control
//
// ============================

// Defines to make code more readable
#define CLAW_OPEN		110
#define CLAW_CLOSED		170
#define PUSH_TIME		80000
#define WAIT_TIME		4000000
#define CLAW_WAIT_TIME	4000000

// Closes the claw
void CloseClaw()
{
	SetPWM(4, CLAW_CLOSED);
}

// Opens the claw
void OpenClaw()
{
	SetPWM(4, CLAW_OPEN);
}

void WaitForClaw()
{
	SetWheelSpeed(WHEEL_LEFT, WHEEL_SPEED_ZERO);
	SetWheelSpeed(WHEEL_RIGHT, WHEEL_SPEED_ZERO);
	for (int i = 0; i < CLAW_WAIT_TIME; i++);
}

// Routine for returning the ball
void ReturnBall()
{
	uint8_t speed = WHEEL_SPEED_HALF;
	//speed /= 2;

	// roll the ball forward
	SetWheelSpeed(WHEEL_LEFT, speed);
	SetWheelSpeed(WHEEL_RIGHT, speed);
	OpenClaw();
	for (int i = 0; i < PUSH_TIME; i++);

	// stop for a little bit
	SetWheelSpeed(WHEEL_LEFT, WHEEL_SPEED_ZERO);
	SetWheelSpeed(WHEEL_RIGHT, WHEEL_SPEED_ZERO);
	for (int i = 0; i < WAIT_TIME; i++);

	// turn a bit so the camera doesn't immediately find the ball again
	SetWheelSpeed(WHEEL_LEFT, speed);
	SetWheelSpeed(WHEEL_RIGHT, WHEEL_SPEED_ZERO);
	for (int i = 0; i < PUSH_TIME; i++);
}

// Update the claw
void UpdateClaw()
{
	switch (currentState)
	{
	case STATE_FIND_BALL:
		return; // Claw shouldn't need to do anything here
	case STATE_GRAB_BALL:
    // if IR sensor detects ball
		if (CheckIR())
		{
			CloseClaw();
			WaitForClaw();
			UpdateState(STATE_RETURN_BALL);
		}
		break;
	case STATE_RETURN_BALL:
		if (runningAverage >= MIN_RETURN_X &&
			runningAverage <= MAX_RETURN_X)
		{
			ReturnBall();
			UpdateState(STATE_FIND_BALL);
		}
		break;
	}
}

// Initializes the timers for the wheels
void WheelsInit()
{
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	SetWheelSpeed(WHEEL_LEFT, WHEEL_SPEED_ZERO);
	SetWheelSpeed(WHEEL_RIGHT, WHEEL_SPEED_ZERO);
}

// Initializes the timer for the claw
void ClawInit()
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	OpenClaw();
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
	pixy_init();
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_SPI1_Init();
	MX_LPUART1_UART_Init();
	MX_ADC1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */
	WheelsInit();
	ClawInit();

	// I don't think we're using these?
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		//CheckIR();
		printf("Current state: %d\n", currentState);
		UpdatePixyCam();
		UpdateWheels();
		UpdateClaw();

		/*
		//Used to test the Claw PMW, Timer is on 2.5ms period and .1MHz resolution
		//https://www.gie.com.my/shop.php?action=robotics/motors/ldx335

		// I'm leaving all this old code here for reference.
		tim4PWMCCR = clawTest;
		SetPWM(4, tim4PWMCCR);
		for (int i = 0; i < 100000; i++);
		clawTest += add;
		if (clawTest <= 100)
			add = 2;
		if (clawTest >= 160)
			add = -2;

		// runningAverage = avg of x vals
		tim2PWMCCR = 139;
		tim3PWMCCR = tim2PWMCCR;
		SetPWM(2, tim2PWMCCR);
		SetPWM(3, tim3PWMCCR);

		int ADC_VAL = 0;
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 0xFFFFFFFF);
		ADC_VAL = HAL_ADC_GetValue(&hadc1);
		float DIST_VAL = exp((log(ADC_VAL) - 10.66) / (-1.123));
		//printf("Distance: %f cm\n\r", DIST_VAL);

		while (DIST_VAL > 9)
		{
			// Get updates from the pixy cam
			GetNextBallX();

			SetPWM(2, tim2PWMCCR);
			SetPWM(3, tim3PWMCCR);
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 0xFFFFFFFF);
			ADC_VAL = HAL_ADC_GetValue(&hadc1);
			float DIST_VAL = exp((log(ADC_VAL) - 10.66) / (-1.123));
			//printf("Distance: %f cm\n", DIST_VAL);
			//*
			while (DIST_VAL < 9)
			{
				tim2PWMCCR = 139;
				tim3PWMCCR = 139;
				SetPWM(2, tim2PWMCCR);
				SetPWM(3, tim3PWMCCR);
				HAL_ADC_Start(&hadc1);
				HAL_ADC_PollForConversion(&hadc1, 0xFFFFFFFF);
				ADC_VAL = HAL_ADC_GetValue(&hadc1);
				DIST_VAL = exp((log(ADC_VAL) - 10.66) / (-1.123));
				printf("Distance: %f cm\n", DIST_VAL);
				for (int i = 0; i < 100000; i++);
			}
		}
		//*/
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV32;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 159;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 79;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 159;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.Pulse = 79;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 39;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 249;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 79;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)&ch, 1, 0xFFFF);
	return ch;
}
/* USER CODE END 4 */

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
