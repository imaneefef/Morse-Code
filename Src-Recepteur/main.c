/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_SAMPLES 5  // 5 échantillons pour 1 ms
#define ADC_THRESHOLD 2000 // Ajuster selon le capteur
#define DOT_DURATION 150       // Durée max d'un point (ms)
#define DASH_DURATION 400      // Durée min d'un tiret (ms)
#define CHAR_GAP 400 // Durée min entre caractères (ms)
#define WORD_GAP 1000 // Durée min entre mots (ms)
#define MAX_MESSAGE_LENGTH 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

typedef struct{
	volatile bool is_signal;			// Signal ON/OFF
	volatile bool first_signal;
	char morse_symbol[7];				// Symbol morse ex (".-")
	int morse_index;					// Index de morse_symbol
}RecepteurAudio;


bool is_recording = false;				// Bouton ON/OFF
volatile uint32_t last_time = 0;		// Dernier instant recu
volatile RecepteurAudio ra = {0};		// Recepteur Audio
char message[MAX_MESSAGE_LENGTH] = {0};	// message a transmettre
int message_index = 0;					// Index de message
volatile uint8_t sample_count = 0;		//index de l'échantillon
volatile bool seuil_atteint = false;	// Indicateur de dépassement du seuil

// tableau référencant les symboles morses : Lettre de A-Z
const char* morse_letter[] = {
		".-",		//A
		"-...",		//B
		"-.-.",		//C
		"-..",		//D
		".",		//E
		"..-.",		//F
		"--.",		//G
		"....",		//H
		"..",		//I
		".---",		//J
		"-.-",		//K
		".-..",		//L
		"--",		//M
		"-.",		//N
		"---",		//O
		".--.",		//P
		"--.-",		//Q
		".-.",		//R
		"...",		//S
		"-",		//T
		"..-",		//U
		"...-",		//V
		".--",		//W
		"-..-",		//X
		"-.--",		//Y
		"--.."		//Z
};

// tableau référencant les symboles morses : Chiffre 0 à 9
const char* morse_digit[] = {
		"-----",	//0
		".----",	//1
		"..---",	//2
		"...--",	//3
		"....-",	//4
		".....",	//5
		"-....",	//6
		"--...",	//7
		"---..",	//8
		"----."		//9
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
int __io_putchar(int ch);
void init_audio(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void process_signal_duration(uint32_t duration);
void process_silence_duration(uint32_t duration);
char decode_morse(const volatile char* symbol);
void add_to_message(char c);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int __io_putchar(int ch){
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}

void init_audio(void){
	is_recording = true;
	last_time = HAL_GetTick();
	ra.is_signal = false;
	ra.first_signal = true;
	ra.morse_index = 0;
	message_index = 0;
	memset((void*)ra.morse_symbol, 0, sizeof(ra.morse_symbol));
	memset(message, 0, sizeof(message));
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_ADC_Start(&hadc1);
}

void end_audio(void){
	is_recording = false;
	HAL_TIM_Base_Stop_IT(&htim3);
	HAL_ADC_Stop(&hadc1);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_13){
		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET){
			printf("Bouton appuye\r\n");  // Debug log
			init_audio();
		}else{
			end_audio();
			printf("Bouton relache\r\n"); // Debug log
			//décoder le dernier symbol
			if(ra.morse_index > 0){
				char c = decode_morse(ra.morse_symbol);
				add_to_message(c);
			}

			printf("message final : %s\r\n", message);
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        if (is_recording) {
            // Démarrer une conversion ADC
            HAL_ADC_Start(&hadc1);
            if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
                uint16_t adc_value = HAL_ADC_GetValue(&hadc1);

                // Vérification du seuil immédiatement après l'acquisition
                if (adc_value > ADC_THRESHOLD) {
                    seuil_atteint = true; // Un son a été détecté !
                }

                sample_count++;
            }

            // Toutes les 1 ms (après NUM_SAMPLES échantillons)
            if (sample_count >= NUM_SAMPLES) {
                uint32_t now = HAL_GetTick(); // On récupère HAL_GetTick() uniquement ici

                if (seuil_atteint) { // Si on a détecté un son au moins une fois dans la période
                    if (!ra.is_signal) { // Si on n'était pas déjà en train de détecter un son
                        uint32_t silence_duration = now - last_time;
                        last_time = now;
                        if (!ra.first_signal) {
                            process_silence_duration(silence_duration);
                        } else {
                            ra.first_signal = false;
                        }
                        ra.is_signal = true;
                    }
                } else { // Aucun son détecté dans la période
                    if (ra.is_signal) { // Si on était en train de détecter un son
                        uint32_t signal_duration = now - last_time;
                        last_time = now;
                        process_signal_duration(signal_duration);
                        ra.is_signal = false;
                    }
                }

                // Réinitialisation pour la prochaine période
                sample_count = 0;
                seuil_atteint = false;
            }
        }
    }
}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//    if (htim->Instance == TIM3) {
//        if (is_recording) {
//            // Démarrer une conversion ADC
//            HAL_ADC_Start(&hadc1);
//            if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
//                uint16_t adc_value = HAL_ADC_GetValue(&hadc1);
//
//                // Vérification immédiate du seuil
//                if (adc_value > ADC_THRESHOLD) {
//                    seuil_atteint = true; // On a détecté un son !
//                }
//
//                sample_count++;
//            }
//
//            // Si on a accumulé 4 échantillons (1 ms)
//            if (sample_count >= NUM_SAMPLES) {
//                if (seuil_atteint) {
//                    printf("vrai \r\n"); // Détection de son
//                } else {
//                    printf("faux \r\n"); // Pas de son détecté
//                }
//
//                // Réinitialisation pour la prochaine période
//                sample_count = 0;
//                seuil_atteint = false;
//            }
//        }
//    }
//}
//
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//  if (htim->Instance == TIM3) { // Vérifie que c'est TIM3 qui a déclenché l'interruption
//	  if(is_recording){
//		  //Démarrer une nouvelle convertion
//		  HAL_ADC_Start(&hadc1);
//
//		  if(HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK){
//			  uint16_t adc_val = HAL_ADC_GetValue(&hadc1);
//			  uint32_t now = HAL_GetTick();
//			  if(adc_val > ADC_THRESHOLD){
//				  if(!(ra.is_signal)){
//					  uint32_t silence_duration = now - last_time;
//					  last_time = now;
//					  if(!(ra.first_signal)){
//						  process_silence_duration(silence_duration);
//			          }else {
//			        	  ra.first_signal = false;
//			          }
//			          ra.is_signal = true;
//			       }
//			   }else {
//				   if(ra.is_signal){
//					   uint32_t signal_duration = now - last_time;
//					   last_time = now;
//					   process_signal_duration(signal_duration);
//					   ra.is_signal = false;
//				   }
//			   }
//		  }
//	  }
//  }
//}

/* Traitement des durées */

/*
 * @brief traitement d'un signal sonore selon sa durée afin de déterminer s'il s'agit d'un '.' ou d'un '-'
 */
void process_signal_duration(uint32_t duration){
	if((duration >= DOT_DURATION - 50) && (duration < DOT_DURATION + 50)){
		ra.morse_symbol[ra.morse_index] = '.';
		ra.morse_index++;
	}else if((duration >= DASH_DURATION - 50) && (duration < DASH_DURATION + 50)){
		ra.morse_symbol[ra.morse_index] = '-';
		ra.morse_index++;
	}
	printf("morse : %s\r\n", ra.morse_symbol);
}

/*
 * @brief traitement d'un silence et décodage du dernier symbole morse
 * Selon la durée du silence on vérifie s'il s'agit d'un espace entre 2 mots ou d'un espace entre 2 symboles
 * morses s'il s'agit de l'un ou l'autre on décode le dernier symbole morse
 */
void process_silence_duration(uint32_t duration){
	if((duration >= WORD_GAP - 100) && (duration < WORD_GAP + 100)){
		if(ra.morse_index > 0){
			printf("word gap : %d \r\n", duration);
			char c = decode_morse(ra.morse_symbol);
			add_to_message(c);
			ra.morse_index = 0;
			memset((void*)ra.morse_symbol, 0, sizeof(ra.morse_symbol));
		}
		add_to_message(' ');
	}
	if((duration >= CHAR_GAP - 50) && (duration < CHAR_GAP + 50)){
		if(ra.morse_index > 0){
			printf("char gap : %d \r\n", duration);
			char c = decode_morse(ra.morse_symbol);
			add_to_message(c);
			ra.morse_index = 0;
			memset((void*)ra.morse_symbol, 0, sizeof(ra.morse_symbol));
		}
	}
	printf("other gap : %d \r\n",duration);
}

char decode_morse(const volatile char* symbol){
	char local_symbol[7];
	strncpy(local_symbol, (const char*)symbol, 6);
	local_symbol[6] = '\0';

	for(int i = 0; i<26; i++){
		if(strcmp(local_symbol, morse_letter[i]) == 0){
			return 'A' + i;
		}
	}
	for(int i = 0; i<10; i++){
		if(strcmp(local_symbol, morse_digit[i]) == 0){
			return '0' + i;
		}
	}
	return '?';
}

void add_to_message(char c){
	if(message_index >= MAX_MESSAGE_LENGTH - 1){
		return;
	}
	message[message_index] = c;
	message_index++;
	message[message_index] = '\0';
	printf("decode : %s\r\n", message);
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
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start(&hadc1);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  /* USER CODE END ADC1_Init 2 */

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

  /* USER CODE BEGIN TIM3_Init 1 */
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 200-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  /* USER CODE END TIM3_Init 2 */

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
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
