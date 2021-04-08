/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 .
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file STMicroelectronicsexcept in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "MotorControl.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NormalRun 1

#define ADC1_DR_Address ((uint32_t)0x4001244C)
#define ADC_Sample_Times 1000000 // So Lan doc ADC Lay nguong
#define NumberOfSensor 8

#define DiThang 0
#define LechPhai 1
#define LechTrai -1

#define HalfLeft -1
#define HalfRight 1

#define CuaTrai -1
#define CuaPhai 1
#define BTN_Servo_Step 0.6;

#define MaximumSpeed 6900
#define SignalSpeed 2000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define sbi(Reg, Bit) (Reg |= (1 << Bit))
#define cbi(Reg, Bit) (Reg &= ~(1 << Bit))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// Sensor Related Variable
volatile uint16_t Sensor_ADC_Value[8];
//uint16_t Sensor_Threshold[] = { 1500, 1500, 1200, 1800, 1200, 1500, 1200, 1500 }; // Morning
uint16_t Sensor_Threshold[] = { 1500, 2000, 2000, 2500, 1600, 2000, 1200, 1500 }; // Morning

uint8_t GetThreshold_Flag = 0;
//int8_t MaxAngle = 70;
float ServoAngle = 0.00;

// Motor Related Variable
uint16_t MaxSpeed = 7200;
uint8_t LineDetect = 0;
int8_t CarState = 0;

int8_t ChuyenLaneFlag = 0;
int8_t HalfWhiteFlag = 0;
int8_t HalfWhiteFlag_Raw = 0;
int8_t CuaFlag = 0;
uint8_t FullWhiteFlag = 0;
uint8_t MatLineFlag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

void GetThreshold();
void Sensor_Convert_A2D();//Cost Time: 6 us with full Black . 8 uS with full white
void Sensor_Print_Thres();
void Sensor_PrintValue();
void Sensor_Print_LineDetect();
void BTN_Process();
void Car_DiThang_Process();
void Car_BamLine_Process();
void Car_MatLine_Process();
void Car_ChuyenLanePhai_Process();
void Car_ChuyenLaneTrai_Process();
void Car_CuaPhai_Process();
void Car_CuaTrai_Process();
void Car_Avoid_Process();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
PUTCHAR_PROTOTYPE {
	while (LL_USART_IsActiveFlag_TC(USART1) == 0) {
	}
	LL_USART_TransmitData8(USART1, (uint8_t) ch);

	return ch;
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

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  NVIC_SetPriority(MemoryManagement_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  /* BusFault_IRQn interrupt configuration */
  NVIC_SetPriority(BusFault_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  /* UsageFault_IRQn interrupt configuration */
  NVIC_SetPriority(UsageFault_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  /* SVCall_IRQn interrupt configuration */
  NVIC_SetPriority(SVCall_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  /* DebugMonitor_IRQn interrupt configuration */
  NVIC_SetPriority(DebugMonitor_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  /* PendSV_IRQn interrupt configuration */
  NVIC_SetPriority(PendSV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));

  /** NOJTAG: JTAG-DP Disabled and SW-DP Enabled
  */
  LL_GPIO_AF_Remap_SWJ_NOJTAG();

  /* USER CODE BEGIN Init */
	setvbuf(stdin, NULL, _IONBF, 0);
	setvbuf(stdout, NULL, _IONBF, 0);
	setvbuf(stderr, NULL, _IONBF, 0);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	MotorL_EnablePWM();
	MotorR_EnablePWM();
	MotorL_SetPWM(0);
	MotorR_SetPWM(0);
	Servo_SetAngle(0);
	//  OC2_IT_Setmillis(2.5);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	//  uint32_t Count = LL_TIM_GetCounter(TIM2);
	uint8_t DistanceState = 0;
	while (1) {
//		LL_mDelay(1);
	Sensor_Convert_A2D();
//    Sensor_PrintValue();


#if NormalRun == 0
		BTN_Process();
		//	  Sensor_Print_Thres();
		Sensor_PrintValue();
//    	  LL_mDelay(200);
//	  Sensor_Print_LineDetect();
#endif

#if NormalRun == 1

    DistanceState = LL_GPIO_IsInputPinSet(Distance_GPIO_Port, Distance_Pin);

    if(GetThreshold_Flag == 1)
    {
    	while(LL_GPIO_IsInputPinSet(BTN1_GPIO_Port,BTN1_Pin) == 0);
    	GetThreshold_Flag = 0;
    	GetThreshold();
    }
    if(BTN3_Flag == 0){
    	MotorL_SetPWM(0);
    	MotorR_SetPWM(0);
    	Servo_SetAngle(0);
    	CarState = 0;
    	FullWhiteFlag = 0;
    	HalfWhiteFlag = 0;
    	HalfWhiteFlag_Raw = 0;
    	MatLineFlag = 0;
    	CuaFlag = 0;
    	LineDetect = 0;
    	continue;
    }
    if (HalfWhiteFlag != 0 || FullWhiteFlag != 0 ||
    	HalfWhiteFlag_Raw != 0 || ChuyenLaneFlag != 0)
    {
      MaxSpeed = SignalSpeed;
    }
    else
    {
      MaxSpeed = MaximumSpeed;
    }
    if (CarState == DiThang)
        {
          if (LineDetect == 0b10000000 || LineDetect == 0b11000000 ||
              LineDetect == 0b11100000 || LineDetect == 0b01110000 ||
              LineDetect == 0b00110000 || LineDetect == 0b00010000)
          {
            CarState = LechPhai;// 25-30us
          }
          else if (LineDetect == 0b00000001 || LineDetect == 0b00000011 ||
                   LineDetect == 0b00000111 || LineDetect == 0b00001110 ||
                   LineDetect == 0b00001100 || LineDetect == 0b00001000)
          {
            CarState = LechTrai;// 25-30us
          }
        }
    if(FullWhiteFlag == 1 && DistanceState == 0)
    {
    	Car_Avoid_Process();
    	FullWhiteFlag = 0;

		continue;
    }
    if (LineDetect == 0b00011000 || LineDetect == 0b00011100 || LineDetect == 0b00111000) // 18.8 us
    {
      if (HalfWhiteFlag_Raw == HalfLeft)
      {
        HalfWhiteFlag_Raw = 0;
        HalfWhiteFlag = HalfLeft;
      }
      else if (HalfWhiteFlag_Raw == HalfRight)
      {
        HalfWhiteFlag_Raw = 0;
        HalfWhiteFlag = HalfRight;
      }

      MatLineFlag = 0;
      ChuyenLaneFlag = 0;
      CarState = DiThang;

      Car_DiThang_Process();
      continue;
    }
    else if (LineDetect == 0b01111111 || LineDetect == 0b00111111 || LineDetect == 0b00011111)
    {
    	if(FullWhiteFlag == 1)
    	{
    		FullWhiteFlag = 0;
    		while(LineDetect != 0)
    			Sensor_Convert_A2D();
    		Car_CuaPhai_Process();
    		HalfWhiteFlag = 0;
    		continue;
    	} else if(ChuyenLaneFlag == 0)
    	{
    		HalfWhiteFlag_Raw = HalfRight;
        	while(LineDetect == 0b01111111 || LineDetect == 0b00111111 || LineDetect == 0b00011111 || LineDetect == 0b00001111)
        	{
        		Sensor_Convert_A2D();
        	}
    	};

    }
    else if (LineDetect == 0b11111000 || LineDetect == 0b11111100 || LineDetect == 0b11111110)
    {
    	if(FullWhiteFlag == 1)
    	{
    		FullWhiteFlag = 0;
    		while(LineDetect != 0)
    			Sensor_Convert_A2D();
    		Car_CuaTrai_Process();
    		HalfWhiteFlag = 0;
    		continue;
    	} else if(ChuyenLaneFlag == 0)
    	{
    		HalfWhiteFlag_Raw = HalfLeft;
        	while(LineDetect == 0b11111000 || LineDetect == 0b11111100 || LineDetect == 0b11111110)
        	{
        		Sensor_Convert_A2D();
        	}
    	}
    }
    else if (LineDetect == 0xff)
    {

    	if(HalfWhiteFlag == HalfLeft)
    	{
    		while(LineDetect != 0)
    			Sensor_Convert_A2D();
    		Car_CuaTrai_Process();
    		CuaFlag = 0;
    		HalfWhiteFlag = 0;
    		HalfWhiteFlag_Raw = 0;
    		continue;
    	} else if(HalfWhiteFlag == HalfRight)
    	{
    		while(LineDetect != 0)
    		{
    			Sensor_Convert_A2D();
    		}
    		Car_CuaPhai_Process();
    		CuaFlag = 0;
    		HalfWhiteFlag = 0;
    		HalfWhiteFlag_Raw = 0;
    		continue;
    	}else
    	{
    		FullWhiteFlag = 1;
    		HalfWhiteFlag_Raw = 0;
    	}
    	while(!(LineDetect == 0b00011000 || LineDetect == 0b00011100 || LineDetect == 0b00111000||
    			LineDetect == 0b11100000 || LineDetect == 0b01110000 ||
    			LineDetect == 0b00110000 || LineDetect == 0b00010000 ||
				LineDetect == 0b00000111 || LineDetect == 0b00001110 ||
				LineDetect == 0b00001100 || LineDetect == 0b00001000 ||
				LineDetect == 0))
    	    			Sensor_Convert_A2D();
    	HalfWhiteFlag_Raw = 0;
    }
    else if (LineDetect == 0)
    {
    	if (HalfWhiteFlag == HalfRight)
    	{
    		HalfWhiteFlag = 0;
    		HalfWhiteFlag_Raw = 0;
    		Car_ChuyenLanePhai_Process();
    		continue;
    	} else if(HalfWhiteFlag == HalfLeft)
    	{
    		HalfWhiteFlag = 0;
    		HalfWhiteFlag_Raw = 0;
    		Car_ChuyenLaneTrai_Process();
    		continue;
    	} else if(FullWhiteFlag == 1)
    	{
    		FullWhiteFlag = 0 ;
    		HalfWhiteFlag_Raw = 0;
    		Servo_SetAngle(-10);
    		MotorL_SetPWM(MaxSpeed * 1.2);
    		MotorR_SetPWM(MaxSpeed * 1.5);
    		while(!(LineDetect == 0b00011000 || LineDetect == 0b00011100 || LineDetect == 0b00111000))
    		{
    			Sensor_Convert_A2D();
    			Car_MatLine_Process();
    		}
    		continue;
    	}
    };
      Car_BamLine_Process();
#endif
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(72000000);
  LL_SetSystemCoreClock(72000000);
  LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSRC_PCLK2_DIV_6);
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

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**ADC1 GPIO Configuration
  PA0-WKUP   ------> ADC1_IN0
  PA1   ------> ADC1_IN1
  PA2   ------> ADC1_IN2
  PA3   ------> ADC1_IN3
  PA4   ------> ADC1_IN4
  PA5   ------> ADC1_IN5
  PA6   ------> ADC1_IN6
  PA7   ------> ADC1_IN7
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1|LL_GPIO_PIN_2|LL_GPIO_PIN_3
                          |LL_GPIO_PIN_4|LL_GPIO_PIN_5|LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* ADC1 DMA Init */

  /* ADC1 Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_HIGH);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);

  /* USER CODE BEGIN ADC1_Init 1 */
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 8);
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1,
			(uint32_t) &Sensor_ADC_Value);
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, ADC1_DR_Address);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_ENABLE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_8RANKS;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_0);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_0, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_1);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_2);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_2, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_4, LL_ADC_CHANNEL_3);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_3, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_5, LL_ADC_CHANNEL_4);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_4, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_6, LL_ADC_CHANNEL_5);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_5, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_7, LL_ADC_CHANNEL_6);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_6, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_8, LL_ADC_CHANNEL_7);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_7, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  /* USER CODE BEGIN ADC1_Init 2 */

	LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
	/* Khoi dong bo ADC */
	LL_ADC_Enable(ADC1);
	LL_ADC_StartCalibration(ADC1);

	/* Cho trang thai cablib duoc bat *
	 *
	 */
	while (LL_ADC_IsCalibrationOnGoing(ADC1))
		;

	/* Bat dau chuyen doi ADC */
	LL_ADC_REG_StartConversionSWStart(ADC1);
  /* USER CODE END ADC1_Init 2 */

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

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**I2C1 GPIO Configuration
  PB8   ------> I2C1_SCL
  PB9   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8|LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  LL_GPIO_AF_EnableRemap_I2C1();

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  /** I2C Initialization
  */
  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.ClockSpeed = 100000;
  I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_SetOwnAddress2(I2C1, 0);
  /* USER CODE BEGIN I2C1_Init 2 */
	LL_I2C_Enable(I2C1);

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

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 7199;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM1);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH3);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM1);
  TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
  TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
  TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
  TIM_BDTRInitStruct.DeadTime = 0;
  TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
  TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
  TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
  LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);
  /* USER CODE BEGIN TIM1_Init 2 */
	LL_TIM_EnableIT_UPDATE(TIM1);
	LL_TIM_SetCounter(TIM1, 0);
	LL_TIM_EnableAllOutputs(TIM1);
	LL_TIM_EnableCounter(TIM1);

  /* USER CODE END TIM1_Init 2 */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**TIM1 GPIO Configuration
  PA8   ------> TIM1_CH1
  PA10   ------> TIM1_CH3
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8|LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**TIM2 GPIO Configuration
  PA15   ------> TIM2_CH1
  PB3   ------> TIM2_CH2
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* TIM2 interrupt Init */
  NVIC_SetPriority(TIM2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),2, 0));
  NVIC_EnableIRQ(TIM2_IRQn);

  /* USER CODE BEGIN TIM2_Init 1 */
	LL_GPIO_AF_EnableRemap_TIM2();
  /* USER CODE END TIM2_Init 1 */
  LL_TIM_SetEncoderMode(TIM2, LL_TIM_ENCODERMODE_X4_TI12);
  LL_TIM_IC_SetActiveInput(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPrescaler(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1);
  LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
  LL_TIM_IC_SetActiveInput(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPrescaler(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV1);
  LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM2, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM2);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM2);
  /* USER CODE BEGIN TIM2_Init 2 */
	//  LL_TIM_CC_EnableChannel(TIM2,LL_TIM_CHANNEL_CH1);
	//  LL_TIM_CC_EnableChannel(TIM2,LL_TIM_CHANNEL_CH2);
	LL_TIM_SetCounter(TIM2, 0);
	LL_TIM_ClearFlag_UPDATE(TIM2);
	LL_TIM_EnableIT_UPDATE(TIM2);
	LL_TIM_EnableCounter(TIM2);
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

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**TIM3 GPIO Configuration
  PB4   ------> TIM3_CH1
  PB5   ------> TIM3_CH2
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4|LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* TIM3 interrupt Init */
  NVIC_SetPriority(TIM3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),2, 0));
  NVIC_EnableIRQ(TIM3_IRQn);

  /* USER CODE BEGIN TIM3_Init 1 */
	LL_GPIO_AF_RemapPartial_TIM3();
  /* USER CODE END TIM3_Init 1 */
  LL_TIM_SetEncoderMode(TIM3, LL_TIM_ENCODERMODE_X4_TI12);
  LL_TIM_IC_SetActiveInput(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPrescaler(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1);
  LL_TIM_IC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
  LL_TIM_IC_SetActiveInput(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPrescaler(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV1);
  LL_TIM_IC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM3);
  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM3);
  /* USER CODE BEGIN TIM3_Init 2 */
	LL_TIM_SetCounter(TIM3, 0);
	LL_TIM_EnableCounter(TIM3);
	LL_TIM_ClearFlag_UPDATE(TIM3);
	LL_TIM_EnableIT_UPDATE(TIM3);
  /* USER CODE END TIM3_Init 2 */

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

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

  /* TIM4 interrupt Init */
  NVIC_SetPriority(TIM4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM4_IRQn);

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  TIM_InitStruct.Prescaler = 23;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 59999;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM4, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM4);
  LL_TIM_SetClockSource(TIM4, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 4500;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM4, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_FROZEN;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM4, LL_TIM_CHANNEL_CH2);
  LL_TIM_SetTriggerOutput(TIM4, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM4);
  LL_TIM_OC_DisablePreload(TIM4, LL_TIM_CHANNEL_CH1);
  /* USER CODE BEGIN TIM4_Init 2 */

	LL_TIM_EnableIT_UPDATE(TIM4);
	LL_TIM_ClearFlag_UPDATE(TIM4);

	LL_TIM_EnableIT_CC1(TIM4);
	LL_TIM_ClearFlag_CC1(TIM4);

//  LL_TIM_EnableIT_CC2(TIM4);
//  LL_TIM_ClearFlag_CC2(TIM4);

	LL_TIM_SetCounter(TIM4, 0);
	LL_TIM_EnableCounter(TIM4);

  /* USER CODE END TIM4_Init 2 */

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

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**USART1 GPIO Configuration
  PB6   ------> USART1_TX
  PB7   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  LL_GPIO_AF_EnableRemap_USART1();

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(GPIOC, Debug_Led_Pin|Debug_GPIO_Pin);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_15);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_9|LL_GPIO_PIN_11);

  /**/
  GPIO_InitStruct.Pin = Debug_Led_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(Debug_Led_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Debug_GPIO_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(Debug_GPIO_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Distance_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(Distance_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTB, LL_GPIO_AF_EXTI_LINE12);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTB, LL_GPIO_AF_EXTI_LINE13);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTB, LL_GPIO_AF_EXTI_LINE14);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_12;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_13;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_14;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  LL_GPIO_SetPinMode(BTN1_GPIO_Port, BTN1_Pin, LL_GPIO_MODE_FLOATING);

  /**/
  LL_GPIO_SetPinMode(BTN2_GPIO_Port, BTN2_Pin, LL_GPIO_MODE_FLOATING);

  /**/
  LL_GPIO_SetPinMode(BTN3_GPIO_Port, BTN3_Pin, LL_GPIO_MODE_FLOATING);

  /* EXTI interrupt init*/
  NVIC_SetPriority(EXTI15_10_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),2, 0));
  NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void GetThreshold() {
	printf("Getting White Line \n");
	while(BTN2_Flag == 0);
//	while(LL_GPIO_IsInputPinSet(BTN2_GPIO_Port,BTN2_Pin) == 0);
	BTN2_Flag = 0;
	MotorL_SetPWM(3600);
	MotorR_SetPWM(3600);
	uint16_t WhiteValue[] = { 0, 0, 0, 0, 0, 0, 0, 0 };

	for (int i = 0; i < ADC_Sample_Times; ++i) {
		for (int i = 0; i < NumberOfSensor; ++i) {
			if (Sensor_ADC_Value[i] > WhiteValue[i]) {
				WhiteValue[i] = Sensor_ADC_Value[i];
			}
		}
	}
	printf("WhiteValue:\t");
	for (int i = 0; i < 8; ++i) {
			printf("%d \t",WhiteValue[i] );
		}
	printf("\r\n");
	MotorL_SetPWM(0);
	MotorR_SetPWM(0);
	BTN2_Flag = 0;
	while(BTN2_Flag == 0);
//	while(LL_GPIO_IsInputPinSet(BTN2_GPIO_Port,BTN2_Pin) == 0);
	BTN2_Flag = 0;
	printf("Getting Black Line\n");
	MotorL_SetPWM(-3600);
	MotorR_SetPWM(-3600);
	uint16_t BlackValue[] = { 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095 };
	for (int i = 0; i < ADC_Sample_Times; ++i) {
		for (int i = 0; i < NumberOfSensor; ++i) {
			if (Sensor_ADC_Value[i] < BlackValue[i]) {
				BlackValue[i] = Sensor_ADC_Value[i];
			}
		}
	}
	printf("BlackValue:\t");
	for (int i = 0; i < 8; ++i) {
			printf("%d \t",BlackValue[i] );
		}
	printf("\r\n");
	MotorL_SetPWM(0);
	MotorR_SetPWM(0);
	printf("Done:\t");
	for (int i = 0; i < 8; ++i) {
		Sensor_Threshold[i] = (BlackValue[i] + WhiteValue[i]) / 2;
		printf("%d \t",Sensor_Threshold[i] );
	}
	printf("\r\n");

}
void Sensor_Convert_A2D() {
	LineDetect = 0;
	for (int i = 0; i < 8; ++i) {
		if (Sensor_ADC_Value[i] < Sensor_Threshold[i]) {
			sbi(LineDetect, (7 - i));
			//			  printf("1 ");
		}
	};
	//	printf("\n");
	//	LL_mDelay(500);
}

void Sensor_Print_Thres() {
	printf("Threshold Val:\t");
	for (int i = 0; i < 8; ++i) {
		printf("%u \t", Sensor_Threshold[i]);
	}
	printf("\n");
}
void Sensor_PrintValue() {
	printf("Sensor Val:\t");
	for (int i = 0; i < 8; ++i) {
		printf("%u \t", Sensor_ADC_Value[i]);
	};
	printf("\n");
}

void Sensor_Print_LineDetect() {
	//	if(PrevLine != LineDetect)
	//	{
	char buffer[8];
	itoa(LineDetect, buffer, 2);
	printf("binary: %s\n", buffer);
	//	}
}
void BTN_Process() {
	if (GetThreshold_Flag == 1) {
		GetThreshold_Flag = 0;
		Sensor_Print_LineDetect();
	}

	if (BTN2_Flag == 1) {
		BTN2_Flag = 0;
		ServoAngle = ServoAngle - BTN_Servo_Step;
		Servo_SetAngle(ServoAngle);
		printf("Servo Angle: %g  \n", ServoAngle);
	}

	if (BTN3_Flag == 1) {
		BTN3_Flag = 0;
		ServoAngle = ServoAngle + BTN_Servo_Step;
		Servo_SetAngle(ServoAngle);
		printf("Servo Angle: %g \n", ServoAngle);
	}
}

void Car_DiThang_Process() {
	MotorL_SetPWM(MaxSpeed);
	MotorR_SetPWM(MaxSpeed*0.95);
	Servo_SetAngle(0);

}
void Car_BamLine_Process() {
	if (CarState == LechTrai) {
		switch (LineDetect) {
		case 0b11000000: // 1
			MotorR_SetPWM(MaxSpeed * 0.60);
			MotorL_SetPWM(MaxSpeed * 0.80);
			Servo_SetAngle(59); // 73
			break;
		case 0b10000000: // 2
			MotorR_SetPWM(MaxSpeed * 0.60);
			MotorL_SetPWM(MaxSpeed * 0.85);
			Servo_SetAngle(52.5); // 64
			break;
			//    case 0b10000001: // 3
			//      MotorR_SetPWM(MaxSpeed * 0.8);
			//      MotorL_SetPWM(MaxSpeed * 0.9);
			//      Servo_SetAngle(59);
			//      break;
		case 0b00000000: // 4
			MotorR_SetPWM(MaxSpeed * 0.65);
			MotorL_SetPWM(MaxSpeed * 0.85);
			Servo_SetAngle(47); // 59
			break;
		case 0b00000001: // 5
			MotorR_SetPWM(MaxSpeed * 0.70);
			MotorL_SetPWM(MaxSpeed * 0.9);
			Servo_SetAngle(31); // 45
			break;
		case 0b00000011: // 6
			MotorR_SetPWM(MaxSpeed * 0.85);
			MotorL_SetPWM(MaxSpeed * 0.95);
			Servo_SetAngle(33); //33
			break;
		case 0b00000111: // 7
			MotorR_SetPWM(MaxSpeed * 0.70);
			MotorL_SetPWM(MaxSpeed * 0.95);
			Servo_SetAngle(25); // 25
			break;
		case 0b00000110:
			MotorR_SetPWM(MaxSpeed * 0.75);
			MotorL_SetPWM(MaxSpeed * 0.95);
			Servo_SetAngle(19); //-24
			break;
		case 0b00001110: // 8
			MotorR_SetPWM(MaxSpeed * 0.93);
			MotorL_SetPWM(MaxSpeed * 1);
			Servo_SetAngle(18); //20
			break;
		case 0b00001100: //9
			MotorR_SetPWM(MaxSpeed * 0.90);
			MotorL_SetPWM(MaxSpeed * 1);
			Servo_SetAngle(10); //10
			break;
//    case 0b00001000: // 10
//      MotorL_SetPWM(MaxSpeed * 0.98);
//      MotorR_SetPWM(MaxSpeed * 1);
//      Servo_SetAngle(6);
//      break;
			//			  case 0b00011100:
			//				  MotorR_SetPWM(MaxSpeed * 0.95);
			//				  MotorL_SetPWM(MaxSpeed * 1);
			//				  Servo_SetAngle(4);
		}
		return;
	};
	if (CarState == LechPhai) {
		switch (LineDetect) {
		case 0b00000011: // 1
			MotorR_SetPWM(MaxSpeed * 0.60);
			MotorL_SetPWM(MaxSpeed * 0.80);
			Servo_SetAngle(-45); // -67
			break;
		case 0b00000001: // 2
			MotorL_SetPWM(MaxSpeed * 0.60);
			MotorR_SetPWM(MaxSpeed * 0.85);
			Servo_SetAngle(-40); // -62
			break;
//			    case 0b10000001: // 3
//			      MotorR_SetPWM(MaxSpeed * 0.80);
//			      MotorL_SetPWM(MaxSpeed * 0.90);
//			      Servo_SetAngle(-59);
//			      break;
		case 0b00000000: // 4
			MotorL_SetPWM(MaxSpeed * 0.65);
			MotorR_SetPWM(MaxSpeed * 0.85);
			Servo_SetAngle(-36); //-52
			break;
		case 0b10000000: // 5
			MotorL_SetPWM(MaxSpeed * 0.70);
			MotorR_SetPWM(MaxSpeed * 0.90);
			Servo_SetAngle(-31); //-43
			break;
		case 0b11000000: // 6
			MotorL_SetPWM(MaxSpeed * 0.87);
			MotorR_SetPWM(MaxSpeed * 0.95);
			Servo_SetAngle(-26); //-35
			break;
		case 0b11100000: // 7
			MotorL_SetPWM(MaxSpeed * 0.70);
			MotorR_SetPWM(MaxSpeed * 0.95);
			Servo_SetAngle(-22); //-27
			break;
		case 0b01100000:
			MotorL_SetPWM(MaxSpeed * 0.75);
			MotorR_SetPWM(MaxSpeed * 0.95);
			Servo_SetAngle(-21); //-24
			break;
		case 0b01110000: // 8
			MotorL_SetPWM(MaxSpeed * 0.93);
			MotorR_SetPWM(MaxSpeed * 1);
			Servo_SetAngle(-16); //-22
			break;
		case 0b00110000: //9
			MotorL_SetPWM(MaxSpeed * 0.95);
			MotorR_SetPWM(MaxSpeed * 1);
			Servo_SetAngle(-10); //-13
			break;
//    case 0b00010000: // 10
//      MotorL_SetPWM(MaxSpeed * 0.98);
//      MotorR_SetPWM(MaxSpeed * 1);
//      Servo_SetAngle(-5);
//      break;
			//			  case 0b00111000:
			//				  MotorL_SetPWM(MaxSpeed * 0.95);
			//				  MotorR_SetPWM(MaxSpeed * 1);
			//				  Servo_SetAngle(-0);
		}
		return;
	}
}

void Car_MatLine_Process() {
//	LL_GPIO_SetOutputPin(Debug_Led_GPIO_Port, Debug_Led_Pin);

	switch (LineDetect) {
	case 0b10000000:
		MotorR_SetPWM(MaxSpeed * 0.9);
		MotorL_SetPWM(MaxSpeed * 1.3);
		Servo_SetAngle(15);
		break;
	case 0b11000000:
		MotorR_SetPWM(MaxSpeed * 0.8);
		MotorL_SetPWM(MaxSpeed * 1.3);
		Servo_SetAngle(19);
		break;
	}
}
void Car_ChuyenLanePhai_Process() {
	ChuyenLaneFlag = 1;
//	LL_GPIO_SetOutputPin(Debug_Led_GPIO_Port, Debug_Led_Pin);
	MotorL_SetPWM(MaxSpeed * 2);      //0.7
	MotorR_SetPWM(MaxSpeed * 1.7);      //0.5
	Servo_SetAngle(54);
	CarState = LechTrai;
	while (!(LineDetect == 0b00011000 || LineDetect == 0b00011100
			|| LineDetect == 0b00111000))
		Sensor_Convert_A2D();
	MatLineFlag = 0;
	FullWhiteFlag = 0;
	HalfWhiteFlag = 0;
	HalfWhiteFlag_Raw = 0;
}
void Car_ChuyenLaneTrai_Process() {
	ChuyenLaneFlag = 1;
	MotorR_SetPWM(MaxSpeed * 2);      //0.7
	MotorL_SetPWM(MaxSpeed * 1.7);      //0.5
	Servo_SetAngle(-30);
	CarState = LechPhai;
	while (!(LineDetect == 0b00011000 || LineDetect == 0b00011100
			|| LineDetect == 0b00111000))
		Sensor_Convert_A2D();
	MatLineFlag = 0;
	FullWhiteFlag = 0;
	HalfWhiteFlag = 0;
	HalfWhiteFlag_Raw = 0;
}
void Car_CuaPhai_Process() {
	Servo_SetAngle(85);
	MotorR_SetPWM(MaxSpeed * 1.0);
	MotorL_SetPWM(MaxSpeed * 1.6);
	while (!(LineDetect == 0b00011000 || LineDetect == 0b00011100
			|| LineDetect == 0b00111000)) {
		Sensor_Convert_A2D();
	}
	MatLineFlag = 0;
	FullWhiteFlag = 0;
	HalfWhiteFlag = 0;
	HalfWhiteFlag_Raw = 0;
}
void Car_CuaTrai_Process() {
	Servo_SetAngle(-75);
	MotorL_SetPWM(MaxSpeed * 1.4);
	MotorR_SetPWM(MaxSpeed * 1.7);
	while (!(LineDetect == 0b00011000 || LineDetect == 0b00011100
			|| LineDetect == 0b00111000)) {
		Sensor_Convert_A2D();
	}
	MatLineFlag = 0;
	FullWhiteFlag = 0;
	HalfWhiteFlag = 0;
	HalfWhiteFlag_Raw = 0;
}
void Car_Avoid_Process()
{
	Servo_SetAngle(65);
	MotorL_SetPWM(MaxSpeed * 1.5);
	MotorR_SetPWM(MaxSpeed * 1.9);
	while(!(LineDetect == 0b00000001 || LineDetect == 0b00000011) )
		Sensor_Convert_A2D();
	Servo_SetAngle(-45);
	MotorR_SetPWM(MaxSpeed * 1.2);
	MotorL_SetPWM(MaxSpeed * 1.5);
	while(!(LineDetect == 0b10000000 || LineDetect == 0b11000000))
		Sensor_Convert_A2D();
	Servo_SetAngle(15);
	MotorR_SetPWM(MaxSpeed * 1.2);
	MotorL_SetPWM(MaxSpeed * 1.7);
	while(!(LineDetect == 0b00011000 || LineDetect == 0b00011100 || LineDetect == 0b00111000))
		Sensor_Convert_A2D();
	MatLineFlag = 0;
	FullWhiteFlag = 0;
	HalfWhiteFlag = 0;
	HalfWhiteFlag_Raw = 0;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
