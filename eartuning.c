/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define SAMPLE_RATE 3000.0f  // Sample rate in Hz
//#define FREQUENCY 65.0f      // Target frequency for sine wave in Hz
//#define AMPLITUDE 127.5f     // Half of 8-bit max value for scaling
//#define OFFSET 127.5f        // Offset to move sine wave to 0-255 rang
#define NUM_SAMPLES (SAMPLE_RATE / WAVE_FREQUENCY)  // Number of samples for one sine wave cycle
#define WAVE_FREQUENCY 1000  // Desired frequency of the sine wave (e.g., 1 kHz)
#define SAMPLE_RATE 48000  // Sampling rate (44.1 kHz)
#define PI 3.14159265358979323846
#define FREQ1 82.41
#define FREQ2 110.0
#define FREQ3 146.83
#define FREQ4 196.0
#define FREQ5 246.94
#define FREQ6 329.63



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
uint32_t dac_value;
uint32_t sine_wave[NUM_SAMPLES];
volatile int sample_index = 0;
float angle_step;
volatile int wave_choice = 0;

uint32_t sine_wave_freq1[(uint32_t)(SAMPLE_RATE / FREQ1)];
uint32_t sine_wave_freq2[(uint32_t)(SAMPLE_RATE / FREQ2)];
uint32_t sine_wave_freq3[(uint32_t)(SAMPLE_RATE / FREQ3)];
uint32_t sine_wave_freq4[(uint32_t)(SAMPLE_RATE / FREQ4)];
uint32_t sine_wave_freq5[(uint32_t)(SAMPLE_RATE / FREQ5)];
uint32_t sine_wave_freq6[(uint32_t)(SAMPLE_RATE / FREQ6)];
uint32_t current_wave_length;

uint32_t *current_wave = sine_wave_freq1;
uint32_t current_wave_length;             // Length of the current wave buffer
int frequency_case = 1;                   // Frequency selection state




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void switch_frequency(void) {
    switch (frequency_case) {
        case 1:
            current_wave = sine_wave_freq1;
            current_wave_length = SAMPLE_RATE / FREQ1;
            break;
        case 2:
            current_wave = sine_wave_freq2;
            current_wave_length = SAMPLE_RATE / FREQ2;
            break;
        case 3:
            current_wave = sine_wave_freq3;
            current_wave_length = SAMPLE_RATE / FREQ3;
            break;
        case 4:
            current_wave = sine_wave_freq4;
            current_wave_length = SAMPLE_RATE / FREQ4;
            break;
        case 5:
            current_wave = sine_wave_freq5;
            current_wave_length = SAMPLE_RATE / FREQ5;
            break;
        case 6:
            current_wave = sine_wave_freq6;
            current_wave_length = SAMPLE_RATE / FREQ6;
            break;
        default:
            current_wave = sine_wave_freq1;
            current_wave_length = SAMPLE_RATE / FREQ1;
    }

    // Restart DMA with the new wave buffer
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, current_wave, current_wave_length, DAC_ALIGN_8B_R);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_13) {  // Assuming button is connected to Pin 13
        frequency_case++;
        if (frequency_case > 6) {
            frequency_case = 1;  // Wrap around to the first frequency
        }

        switch_frequency();
    }
}

//
//void generate_triangle_wave() {
//    for (uint16_t i = 0; i < 7; i++) {  // Rising part
//    	triangle = (i * 256) / 7;
//        HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, triangle);
//        HAL_Delay(0.3);  // Delay to control the speed of change
//    }
//    for (uint16_t i = 7; i > 0; i--) {  // Falling part
//    	triangle = (i * 255) / 7;
//    	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, triangle);
//        HAL_Delay(0.3);
//    }
//}
//
//void generate_sawtooth_wave() {
//    for (uint16_t i = 0; i < 14; i++) {  // Gradual increase
//    	saw = (i * 255) / 14;  // Store value in global variable
//        HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_8B_R, saw);
//        HAL_Delay(0.3);
//
//    }
//    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_8B_R, 0);
//}


//void generate_sine_wave() {
//    float angle_step = ((2.0f * PI * FREQUENCY) / SAMPLE_RATE)*2;  // Step size per sample
//    float angle = 0.0f;  // Initial angle
//
//    while (1) {
//        // Generate sine wave value and scale to 8-bit range (0-255)
//        float sine_value = arm_sin_f32(angle);
//        dac_value = (uint8_t)(AMPLITUDE * sine_value + OFFSET);  // Scale to 0-255
//
//        // Send scaled value to DAC
//        HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, dac_value);
//
//        // Update angle
//        angle += angle_step;
//        if (angle >= 2.0f * PI) {
//            angle -= 2.0f * PI;  // Wrap around when reaching full cycle
//        }
//
//        // Control frequency with delay to maintain 65 Hz (approx.)
//        HAL_Delay(1);  // Adjust delay as needed based on sample rate
//    }
//}


//void generate_sine_wave() {
//	// Step size for the sine wave (one complete cycle in NUM_SAMPLES points)
//	    float angle_step = 2 * PI / NUM_SAMPLES;
//
//	    // Generate the sine wave and scale it to 0-255
//	    for (int i = 0; i < NUM_SAMPLES; i++) {
//	        float sine_value = arm_sin_f32(i * angle_step);  // Generate sine value (range -1.0 to 1.0)
//	        sine_wave[i] = (uint32_t)((sine_value + 1.0f) * 127.5f);  // Scale sine wave to 0-255
//}
//}

void generate_sine_wave(uint32_t *wave_array, float frequency) {
    int num_samples = SAMPLE_RATE / frequency;  // Calculate samples per cycle
    float angle_step = 2 * PI / num_samples;   // Angle increment for each sample
    for (int i = 0; i < num_samples; i++) {
        float sine_value = sinf(i * angle_step);  // Generate sine value (-1.0 to 1.0)
        wave_array[i] = (uint32_t)((sine_value + 1.0f) * 127.5f);  // Scale to 0-255
    }
}

void initialize_sine_waves() {
    generate_sine_wave(sine_wave_freq1, FREQ1);
    generate_sine_wave(sine_wave_freq2, FREQ2);
    generate_sine_wave(sine_wave_freq3, FREQ3);
    generate_sine_wave(sine_wave_freq4, FREQ4);
    generate_sine_wave(sine_wave_freq5, FREQ5);
    generate_sine_wave(sine_wave_freq6, FREQ6);

    // Set default wave
       current_wave = sine_wave_freq1;
       current_wave_length = SAMPLE_RATE / FREQ1;
}



void select_frequency(uint32_t *wave_array, int wave_frequency) {
    int num_samples = SAMPLE_RATE / wave_frequency;
    generate_sine_wave(wave_array, num_samples);
    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, wave_array, num_samples, DAC_ALIGN_8B_R);
}




// Timer interrupt callback function
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {  // Check if the interrupt is from TIM2
        // Send the current sine wave value to the DAC

    	dac_value = (uint16_t)(current_wave[sample_index]);  // Scale to 0-255
    	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, dac_value);

        // Move to the next sample
        sample_index++;
        if (sample_index >= NUM_SAMPLES) {
            sample_index = 0;  // Wrap around when reaching the end of the array
        }
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
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  //HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
  initialize_sine_waves();
//  generate_sine_wave();
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, current_wave, current_wave_length, DAC_ALIGN_8B_R);
  HAL_TIM_Base_Start(&htim2);





  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  //generate_triangle_wave();  // Generate triangle wave on Channel 1
	  //generate_sawtooth_wave();  // Generate sawtooth wave on Channel 2


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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_ABOVE_80MHZ;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : interrupts_Pin */
  GPIO_InitStruct.Pin = interrupts_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(interrupts_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
