/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * MORE ROAR or something like this - very beta UAL 1/5/24 JR
 *
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "math.h"
#define NS       256 // number of samples in wavetable 0 - 255
#define MAX_CTR       1 // number of programs


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

uint16_t i = 0, AD_RES[2];
ADC_ChannelConfTypeDef ADC_CH_Cfg = {0};
uint32_t ADC_Channels[2] = {ADC_CHANNEL_16, ADC_CHANNEL_17};


uint32_t DstAddress = (uint32_t) &(TIM1->CCR1);

/* tables --------------------------------------------------------------------*/

// noise
uint32_t noise[NS];

// sine wave 8 bit resolution scaled to 94% max val
uint32_t sine[NS] = { 120, 123, 126, 129, 132, 134, 137, 140, 143, 146, 149,
		152, 155, 157, 160, 163, 165, 168, 171, 174, 177, 179, 181, 184, 186,
		189, 191, 194, 196, 198, 200, 202, 205, 207, 209, 211, 212, 214, 216,
		218, 220, 221, 223, 224, 226, 227, 228, 229, 230, 231, 233, 234, 235,
		235, 236, 237, 238, 238, 239, 239, 239, 240, 240, 240, 240, 240, 240,
		240, 239, 239, 239, 238, 238, 237, 236, 235, 235, 234, 233, 231, 230,
		229, 228, 227, 226, 224, 223, 221, 220, 218, 216, 214, 212, 211, 209,
		207, 205, 202, 200, 198, 196, 194, 191, 189, 186, 184, 181, 179, 177,
		174, 171, 168, 165, 163, 160, 157, 155, 152, 149, 146, 143, 140, 137,
		134, 132, 129, 126, 123, 120, 117, 114, 111, 108, 105, 102, 100, 97, 94,
		91, 87, 85, 83, 80, 77, 74, 71, 69, 66, 63, 61, 58, 55, 54, 51, 49, 46,
		44, 41, 39, 38, 35, 33, 31, 29, 27, 25, 24, 22, 20, 19, 17, 16, 14, 13,
		11, 10, 9, 8, 7, 6, 5, 5, 4, 3, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1,
		1, 1, 2, 2, 3, 4, 5, 5, 6, 7, 8, 9, 10, 11, 13, 14, 16, 17, 19, 20, 22,
		24, 25, 27, 29, 31, 33, 35, 38, 39, 41, 44, 46, 49, 51, 54, 55, 58, 61,
		63, 66, 69, 71, 74, 77, 80, 83, 85, 87, 91, 94, 97, 100, 102, 105, 108,
		111, 114, 117 };

// triangle wave 8 bit resolution scaled to 94% max val
uint32_t triangle[NS] = { 120, 121, 123, 125, 127, 129, 131, 133, 134, 136, 138,
		140, 142, 144, 146, 148, 149, 151, 153, 155, 157, 159, 161, 163, 165,
		166, 168, 170, 172, 174, 176, 178, 180, 181, 183, 185, 187, 189, 191,
		193, 195, 196, 198, 200, 202, 204, 206, 208, 210, 212, 213, 215, 217,
		219, 221, 223, 225, 227, 228, 230, 232, 234, 236, 238, 240, 238, 236,
		234, 232, 230, 228, 227, 225, 223, 221, 219, 217, 215, 213, 212, 210,
		208, 206, 204, 202, 200, 198, 196, 195, 193, 191, 189, 187, 185, 183,
		181, 180, 178, 176, 174, 172, 170, 168, 166, 165, 163, 161, 159, 157,
		155, 153, 151, 149, 148, 146, 144, 142, 140, 138, 136, 134, 133, 131,
		129, 127, 125, 123, 121, 120, 118, 117, 115, 113, 111, 109, 107, 105,
		103, 102, 100, 98, 96, 94, 92, 90, 88, 86, 85, 83, 81, 79, 77, 75, 73,
		71, 70, 68, 66, 64, 62, 60, 58, 56, 55, 53, 51, 49, 47, 45, 43, 41, 39,
		38, 36, 34, 32, 30, 28, 26, 24, 23, 21, 19, 17, 15, 13, 11, 9, 8, 6, 4,
		2, 0, 2, 4, 6, 8, 9, 11, 13, 15, 17, 19, 21, 23, 24, 26, 28, 30, 32, 34,
		36, 38, 39, 41, 43, 45, 47, 49, 51, 53, 55, 56, 58, 60, 62, 64, 66, 68,
		70, 71, 73, 75, 77, 79, 81, 83, 85, 86, 88, 90, 92, 94, 96, 98, 100,
		102, 103, 105, 107, 109, 111, 113, 115, 117, 118 };

// saw_xmax wave tilted 0.96 - 8 bit resolution scaled to 94% max val
uint32_t saw_xmax[NS] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 12, 13, 14,
		15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32,
		33, 34, 35, 36, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
		50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 61, 62, 63, 64, 65, 66,
		67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84,
		85, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100,
		101, 102, 103, 104, 105, 106, 107, 108, 109, 109, 110, 111, 112, 113,
		114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127,
		128, 129, 130, 131, 132, 133, 133, 134, 135, 136, 137, 138, 139, 140,
		141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154,
		155, 156, 157, 158, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167,
		168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181,
		182, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194,
		195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 206, 207,
		208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221,
		222, 223, 224, 225, 226, 227, 228, 229, 230, 230, 231, 232, 233, 234,
		235, 236, 237, 238, 239, 235, 188, 141, 94, 47, 0 };

// weierstrass cosine wave 8 bit resolution scaled to 94% max val
uint32_t weierstrass[NS] = { 236, 234, 229, 223, 217, 213, 212, 213, 216, 217,
		216, 211, 203, 194, 186, 179, 176, 176, 177, 179, 178, 175, 169, 162,
		155, 150, 149, 151, 156, 161, 164, 165, 163, 159, 156, 153, 154, 158,
		165, 172, 177, 179, 179, 175, 171, 167, 166, 168, 172, 176, 179, 179,
		175, 169, 160, 153, 147, 144, 144, 144, 144, 141, 135, 126, 115, 105,
		98, 93, 92, 92, 92, 91, 87, 80, 72, 64, 58, 56, 58, 62, 66, 69, 70, 67,
		63, 59, 57, 57, 61, 68, 75, 80, 83, 82, 79, 75, 71, 71, 73, 78, 82, 86,
		87, 84, 78, 70, 64, 59, 57, 58, 60, 60, 59, 54, 46, 37, 28, 22, 19, 19,
		21, 23, 24, 21, 16, 10, 4, 0, 0, 4, 10, 16, 21, 24, 23, 21, 19, 19, 22,
		28, 37, 46, 54, 59, 60, 60, 58, 57, 59, 64, 70, 78, 84, 87, 86, 82, 78,
		73, 71, 71, 75, 79, 82, 83, 80, 75, 68, 61, 57, 57, 59, 63, 67, 70, 69,
		66, 62, 58, 56, 58, 64, 72, 80, 87, 91, 92, 92, 92, 93, 98, 105, 115,
		126, 135, 141, 144, 144, 144, 144, 147, 153, 160, 169, 175, 179, 179,
		176, 172, 168, 166, 167, 171, 175, 179, 179, 177, 172, 165, 158, 154,
		153, 156, 159, 163, 165, 164, 161, 156, 151, 149, 150, 155, 162, 169,
		175, 178, 179, 177, 176, 176, 179, 186, 194, 203, 211, 216, 217, 216,
		213, 212, 213, 217, 223, 229, 234, 236 };

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch1;

/* USER CODE BEGIN PV */
uint8_t ctr = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// EXTI Line9 External Interrupt ISR Handler CallBackFun
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {

	//number of programs here!
	ctr = ctr + 1;
	if (ctr > MAX_CTR) // if counter reaches n, + 1 more than programs
		ctr = 1;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */

int main(void) {

	/* USER CODE BEGIN 1 */
  // Map function
	uint32_t MAP(uint32_t au32_IN, uint32_t au32_INmin, uint32_t au32_INmax,
			uint32_t au32_OUTmin, uint32_t au32_OUTmax) {
		return ((((au32_IN - au32_INmin) * (au32_OUTmax - au32_OUTmin))
				/ (au32_INmax - au32_INmin)) + au32_OUTmin);
	}
	// Noise function
	void noise_func(uint32_t array[], uint32_t length, int max) {
		for (int i = 0; i < length; i++)
			array[i] = (rand() % max) + 1;
	}


	uint32_t take_log(uint32_t num) {
		double num_log = sqrt(num);
		num_log = num_log*10;
		uint32_t int_log = (int) num_log;
		return int_log;
	}


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
	MX_DMA_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_ADC1_Init();
	/* USER CODE BEGIN 2 */

////PWM test
//	TIM1->CCR1 = 128;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);
	HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t) sine, DstAddress, NS);
	__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);
	// Calibrate The ADC On Power-Up For Better Accuracy
//	HAL_ADCEx_Calibration_Start(&hadc1);

	ADC_CH_Cfg.Rank =  ADC_REGULAR_RANK_1;
	ADC_CH_Cfg.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;


	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

	    for(i=0; i<2; i++)
	    {
	        ADC_CH_Cfg.Channel = ADC_Channels[i];         // Select The ADC Channel [i]
	        HAL_ADC_ConfigChannel(&hadc1, &ADC_CH_Cfg);   // Configure The Selected ADC Channel
	        HAL_ADC_Start(&hadc1);                        // Start ADC Conversion @ Selected Channel
	        HAL_ADC_PollForConversion(&hadc1, 1);         // Poll The ADC Channel With TimeOut = 1mSec
	        AD_RES[i] = HAL_ADC_GetValue(&hadc1);         // Read The ADC Conversion Result
	    }

		// sine
		if (ctr == 1) {
			DMA1_Channel1->CMAR = (uint32_t) sine; // SrcAddress
			TIM2->ARR = ( ( AD_RES[1] >> 1) + (AD_RES[0] >> 4)); // ADC
		}


		// AD_RES[0] >> 3
		// AD_RES[1]

//		// triangle
//		if (ctr == 2) {
//			DMA1_Channel1->CMAR = (uint32_t) sine; // SrcAddress
//			TIM2->ARR = (AD_RES[1] >> 4); // ADC
//		}
//
//
//		if (ctr == 3) {
//			DMA1_Channel1->CMAR = (uint32_t) sine; // SrcAddress
//			TIM2->ARR = ((AD_RES[1] >> 3) * (AD_RES[0] >> 3) >> 3); // ADC
//		}
//
//		if (ctr == 4) {
//			DMA1_Channel1->CMAR = (uint32_t) sine; // SrcAddress
//			TIM2->ARR = (AD_RES[1] >> 2) + (AD_RES[0] >> 3); // ADC
//		}
//
//		if (ctr == 5) {
//			DMA1_Channel1->CMAR = (uint32_t) sine; // SrcAddress
//			TIM2->ARR = (AD_RES[1] >> 3) + (AD_RES[0] >> 2) + 4; // ADC
//		}

//		// saw_xmax
//		if (ctr == 3) {
//			DMA1_Channel1->CMAR = (uint32_t) saw_xmax; // SrcAddress
//			TIM2->ARR = (AD_RES >> 3) + 10; // ADC
//		}
//
//		// weierstrass
//		if (ctr == 4) {
//			DMA1_Channel1->CMAR = (uint32_t) weierstrass; // SrcAddress
//			TIM2->ARR = (AD_RES >> 3) + 10; // ADC
//		}
//
////		// noise
////		if (ctr == 5) {
////			noise_func(noise, NS, 240);
////			DMA1_Channel1->CMAR = (uint32_t) noise; // SrcAddress
////			TIM2->ARR = (AD_RES >> 3) + 10; // ADC
////		}
//
//		// more noise
//		if (ctr == 5) {
//			DMA1_Channel1->CMAR = (uint32_t) sine; // SrcAddress
//			TIM2->ARR = (AD_RES >> 3) + 10; // ADC
//			AD_RES = AD_RES * rand () % 23 + 1; // Counter Period (ARR) random length
//		}
//
//		// God awful noise
//		if (ctr == 6) {
//			DMA1_Channel1->CMAR = (uint32_t) sine; // SrcAddress
//			TIM2->ARR = (AD_RES << 1) + 10; // ADC
//			AD_RES = AD_RES * rand() % 2;
//		}

// ADC DMA
// Pass (ADC handle, Destination Buffer address, Number of data to ADC peripheral to memory)
//		HAL_ADC_Start_DMA(&hadc1, &AD_RES, 1);

//		// See Map function
//		TIM2->ARR = MAP(AD_RES, 0, 8191, 5, 3500);

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
	RCC_OscInitStruct.PLL.PLLN = 16;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_SEQ_FIXED;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.LowPowerAutoPowerOff = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
	hadc1.Init.OversamplingMode = ENABLE;
	hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_256;
	hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_4;
	hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
	hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_16;
	sConfig.Rank = 1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	sConfig.Channel = ADC_CHANNEL_17;
	sConfig.Rank = 2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 256 - 1;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
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
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1000;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_OC_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	/* DMA1_Channel2_3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin : PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// DMA/or Interrupt
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
// Conversion complete & DMA transfer/or Interrupt complete
// AD_RES is now updated & move to TIM2->ARR (see while)
// Update ARR with latest ADC conversion result
//	AD_RES = HAL_ADC_GetValue(&hadc1);

    for(i=0; i<4; i++)
    {
        ADC_CH_Cfg.Channel = ADC_Channels[i];         // Select The ADC Channel [i]
        HAL_ADC_ConfigChannel(&hadc1, &ADC_CH_Cfg);   // Configure The Selected ADC Channel
        HAL_ADC_Start(&hadc1);                        // Start ADC Conversion @ Selected Channel
        HAL_ADC_PollForConversion(&hadc1, 1);         // Poll The ADC Channel With TimeOut = 1mSec
        AD_RES[i] = HAL_ADC_GetValue(&hadc1);         // Read The ADC Conversion Result
    }
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
