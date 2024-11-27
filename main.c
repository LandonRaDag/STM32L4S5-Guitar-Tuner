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
#include <stdio.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_RATE 48000       // Sampling rate in Hz
#define MIC_BUFFER_SIZE 2048    // Buffer size for microphone data
#define ADC_BUFFER_SIZE 2048    // Buffer size for input jack data
#define THRESHOLD 0.1f          // Threshold for the YIN algorithm
#define HIGH_PASS_FREQ 50       // High-pass filter frequency
#define LOW_PASS_FREQ 2000      // Low-pass filter frequency

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int32_t mic_buffer[MIC_BUFFER_SIZE];  // DFSDM microphone buffer (PCM)
uint16_t adc_buffer[ADC_BUFFER_SIZE]; // ADC input jack buffer (16-bit ADC values)
float32_t test_buffer[MIC_BUFFER_SIZE];
float32_t frequency_buffer = 0.0f;    // Detected frequency storage
volatile uint8_t buffer_full_flag = 0; // Flag indicating buffer is full
uint8_t current_mode = 0;  // 0: Microphone, 1: Input Jack


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void processSignalMic(void);
void processSignalADC(void);
void detectFrequency(const int32_t *buffer, uint32_t length, float32_t *frequency);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float32_t yin_detect_frequency(const float32_t *buffer, uint32_t length, float32_t sample_rate) {
    float32_t min_value = 1.0f;
    uint32_t min_index = 0;
    float32_t cumulative_diff[length];
    float32_t threshold = THRESHOLD;

    // Step 1: Calculate difference function
    for (uint32_t lag = 1; lag < length / 2; lag++) {
        float32_t sum = 0.0f;
        for (uint32_t i = 0; i < length - lag; i++) {
            float32_t diff = buffer[i] - buffer[i + lag];
            sum += diff * diff;
        }
        cumulative_diff[lag] = sum / (float32_t)(length - lag);

        // Step 2: Check if cumulative difference crosses the threshold
        if (cumulative_diff[lag] < threshold && lag > 1) {
            if (cumulative_diff[lag] < min_value) {
                min_value = cumulative_diff[lag];
                min_index = lag;
            }
        }
    }

    if (min_index > 0) {
            return sample_rate / (float32_t)min_index;
        } else {
            return 0.0f;  // No valid frequency found
        }
}

void HAL_DMA_ConvCpltCallback(DMA_HandleTypeDef *hdma) {
    buffer_full_flag = 1;  // Buffer is full
}

void generateSineWave(float32_t *buffer, uint32_t length, float32_t frequency) {
    for (uint32_t i = 0; i < length; i++) {
        buffer[i] = arm_sin_f32(2 * M_PI * frequency * i / SAMPLE_RATE);
    }
}

void generateComplexSignal(float32_t *buffer, uint32_t length) {
    for (uint32_t i = 0; i < length; i++) {
        buffer[i] = arm_sin_f32(2 * M_PI * 440.0 * i / SAMPLE_RATE) +  // Base frequency
                    0.5f * arm_sin_f32(2 * M_PI * 880.0 * i / SAMPLE_RATE) +  // 1st harmonic
                    0.25f * arm_sin_f32(2 * M_PI * 1760.0 * i / SAMPLE_RATE); // 2nd harmonic
    }
}

void generateRandomNoise(float32_t *buffer, uint32_t length) {
    for (uint32_t i = 0; i < length; i++) {
        buffer[i] = ((float32_t)rand() / (float32_t)RAND_MAX) * 2.0f - 1.0f;  // Values between -1 and 1
    }
}




void applyHannWindow(float32_t *signal, float32_t *windowedSignal, uint32_t length) {
    for (uint32_t i = 0; i < length; i++) {
        float32_t hannValue = 0.5 * (1 - cosf((2 * M_PI * i) / (length - 1)));
        windowedSignal[i] = signal[i] * hannValue;
    }
}

void processSignal(void) {
    float32_t detected_frequency = 0.0f;

    // Simulate processing based on the current mode
    if (current_mode == 0) { // Microphone mode
        // Use test_buffer as mic_buffer
        detected_frequency = yin_detect_frequency(test_buffer, MIC_BUFFER_SIZE, SAMPLE_RATE);
    } else if (current_mode == 1) { // Input jack mode
        // Use test_buffer as adc_buffer
        detected_frequency = yin_detect_frequency(test_buffer, ADC_BUFFER_SIZE, SAMPLE_RATE);
    }

    // Store detected frequency
    frequency_buffer = detected_frequency;

    // Print the result for debugging
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
//  DFSDM_Init();  // Initialize DFSDM for microphone
//  ADC_Init();    // Initialize ADC for input jack
//  DMA_Init();    // Initialize DMA for both peripherals
//  HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, mic_buffer, MIC_BUFFER_SIZE);
//  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, ADC_BUFFER_SIZE);
//  generateSineWave(test_buffer, MIC_BUFFER_SIZE, 440.0);


  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
  processSignal();

  /* USER CODE END 2 */

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
