#include "main.h"
#include "usb_device.h"
#include "string.h"
#include "stdbool.h"
#include "math.h"

#define ELECTROMAGNET GPIOB, GPIO_PIN_1
#define STOP_DETECTION GPIOB, GPIO_PIN_11
#define ON 1
#define OFF 0
#define HIGH 1
#define LOW 0
#define TRUE 1
#define FALSE 0
#define BUFFER_LEN 120
#define FORWARD_DC 100
#define TURN_DC 45
#define REVERSE_DC 70
#define ADC_THRESHOLD 600
#define ONE_SECOND 10000

ADC_HandleTypeDef handleADCOne;
ADC_HandleTypeDef handleADCTwo;
TIM_HandleTypeDef handleTimerOne;
TIM_HandleTypeDef handleTimerTwo;
TIM_HandleTypeDef handleTimerThree;
TIM_HandleTypeDef handleTimerFour;
UART_HandleTypeDef handleUART;

void System_Clock_Configuration(void);
static void Initialize_GPIO(void);
static void Initialize_ADC_One(void);
static void Initialize_ADC_Two(void);
static void Initialize_Timer_One(void);
static void Initialize_Timer_Two(void);
static void Initialize_Timer_Three(void);
static void Initialize_Timer_Four(void);
static void Initialize_UART(void);
uint8_t CDC_Transmit_FS(uint8_t *buffer, uint16_t length);

uint8_t coinCount = 0;
uint8_t iterator;
uint8_t bufferRX[BUFFER_LEN] = {0};
uint8_t bufferTX[BUFFER_LEN] = {0};
int16_t ADCValues[2];
int duty;
bool magneticInterference = false;
volatile bool pickupCoin = true;
volatile bool receivedCoinFlag = false;

int _write(int file, char *pointer, int length) {
	CDC_Transmit_FS((uint8_t *)pointer, length);
	return length;
}

void Forward(uint8_t dutyCycle) {
	__HAL_TIM_SET_COMPARE(&handleTimerTwo, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&handleTimerThree, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&handleTimerTwo, TIM_CHANNEL_1, dutyCycle);
	__HAL_TIM_SET_COMPARE(&handleTimerThree, TIM_CHANNEL_1, dutyCycle);
}

void Reverse(uint8_t dutyCycle) {
	__HAL_TIM_SET_COMPARE(&handleTimerTwo, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&handleTimerThree, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&handleTimerTwo, TIM_CHANNEL_2, dutyCycle);
	__HAL_TIM_SET_COMPARE(&handleTimerThree, TIM_CHANNEL_2, dutyCycle);
}

void Right(uint8_t dutyCycle) {
	__HAL_TIM_SET_COMPARE(&handleTimerTwo, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&handleTimerThree, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&handleTimerThree, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&handleTimerTwo, TIM_CHANNEL_1, dutyCycle);
}

void Left(uint8_t dutyCycle) {
	__HAL_TIM_SET_COMPARE(&handleTimerTwo, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&handleTimerThree, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&handleTimerTwo, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&handleTimerThree, TIM_CHANNEL_1, dutyCycle);
}

void Turn_Off() {
	__HAL_TIM_SET_COMPARE(&handleTimerTwo, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&handleTimerThree, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&handleTimerTwo, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&handleTimerThree, TIM_CHANNEL_1, 0);
}

void Avoid_Perimeter() {
	uint8_t random;
	HAL_GPIO_WritePin(STOP_DETECTION, HIGH);
	Reverse(REVERSE_DC);
	HAL_Delay(1000);
	Turn_Off();
	Right(TURN_DC);
	random = (rand() % 4) + 1;
	HAL_Delay(random * 1000);
	Forward(FORWARD_DC);
	HAL_GPIO_WritePin(STOP_DETECTION, LOW);
}

void Arm_Sequence() {
	HAL_Delay(200);
	handleTimerFour.Instance->CCR2 = 79;
	HAL_Delay(1000);
	HAL_GPIO_WritePin(ELECTROMAGNET, HIGH);
	handleTimerFour.Instance->CCR1 = 126;
	HAL_Delay(1500);

	for(iterator = 100; iterator > 59; iterator =- 3) {
		handleTimerFour.Instance->CCR2 = iterator;
		HAL_Delay(55);
	}

	handleTimerFour.Instance->CCR2 = 79;

	for(iterator = 126; iterator > 60; iterator =- 3) {
		handleTimerFour.Instance->CCR1 = iterator;
		HAL_Delay(55);
	}

	for(iterator = 79; iterator > 46; iterator =- 3) {
		handleTimerFour.Instance->CCR2 = iterator;
		HAL_Delay(55);
	}

	HAL_Delay(500);
	HAL_GPIO_WritePin(ELECTROMAGNET, LOW);
	coinCount++;
}

void Pickup_Coin() {
	HAL_GPIO_WritePin(STOP_DETECTION, HIGH);
	Turn_Off();
	HAL_Delay(20);
	Reverse(REVERSE_DC);
	HAL_Delay(275);
	Turn_Off();
	Arm_Sequence();
	HAL_GPIO_WritePin(STOP_DETECTION, LOW);
}

void Initialize() {
	HAL_GPIO_WritePin(STOP_DETECTION, LOW);
	HAL_GPIO_WritePin(ELECTROMAGNET, LOW);
	handleTimerFour.Instance->CCR1 = 60;
	HAL_Delay(500);
	handleTimerFour.Instance->CCR2 = 59;
	HAL_Delay(500);
    Turn_Off();
	HAL_Delay(40);
	Forward(FORWARD_DC);
}

int main(void) {
	HAL_Init();

	System_Clock_Configuration();

	Initialize_GPIO();
	Initialize_ADC_One();
	Initialize_Timer_Three();
	Initialize_Timer_Two();
	Initialize_UART();
	Initialize_Timer_Four();
	Initialize_ADC_Two();
	Initialize_Timer_One();

	HAL_UART_Receive_IT(&handleUART, bufferRX, BUFFER_LEN);
	HAL_TIM_PWM_Start(&handleTimerTwo, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&handleTimerTwo, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&handleTimerThree, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&handleTimerThree, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&handleTimerFour, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&handleTimerFour, TIM_CHANNEL_2);
	HAL_TIM_Base_Start(&handleTimerOne);
	HAL_ADC_Start(&handleADCOne);
	HAL_ADC_Start(&handleADCTwo);
	HAL_Delay(1000);

	Initialize();

	while (TRUE) {
		if(coinCount == 20) {
			Turn_Off();
			HAL_GPIO_WritePin(ELECTROMAGNET, OFF);
			return 0;
		}

		if(receivedCoinFlag == true) {
			magneticInterference = true;
			Pickup_Coin();
			Forward(FORWARD_DC);
			receivedCoinFlag = false;
			handleTimerOne.Instance->CNT = 0;
			HAL_TIM_Base_Start(&handleTimerOne);
		}

		HAL_ADC_PollForConversion(&handleADCOne, 1000);
		ADCValues[0] = HAL_ADC_GetValue(&handleADCOne);
		HAL_ADC_PollForConversion(&handleADCTwo, 1000);
		ADCValues[1] = HAL_ADC_GetValue(&handleADCTwo);

		if(__HAL_TIM_GET_COUNTER(&handleTimerOne) < ONE_SECOND) {
			magneticInterference = true;
			ADCValues[0] = 0;
			ADCValues[1] = 0;
		}
		else {
			HAL_TIM_Base_Stop(&handleTimerOne);
			handleTimerOne.Instance->CNT = ONE_SECOND;
			magneticInterference = false;
		}

		if(((ADCValues[0] > ADC_THRESHOLD) || (ADCValues[1] > ADC_THRESHOLD)) && (!magneticInterference)) {
			HAL_GPIO_WritePin(STOP_DETECTION, HIGH);
			Avoid_Perimeter();
			HAL_GPIO_WritePin(STOP_DETECTION, LOW);
		}
	}
}

void System_Clock_Configuration(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC | RCC_PERIPHCLK_USB;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;

    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }
}

static void Initialize_ADC_One(void) {
    ADC_ChannelConfTypeDef sConfig = {0};

    handleADCOne.Instance = ADC1;
    handleADCOne.Init.ScanConvMode = ADC_SCAN_DISABLE;
    handleADCOne.Init.ContinuousConvMode = ENABLE;
    handleADCOne.Init.DiscontinuousConvMode = DISABLE;
    handleADCOne.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    handleADCOne.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    handleADCOne.Init.NbrOfConversion = 1;

    if (HAL_ADC_Init(&handleADCOne) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;

    if (HAL_ADC_ConfigChannel(&handleADCOne, &sConfig) != HAL_OK) {
        Error_Handler();
    }
}

static void Initialize_ADC_Two(void) {
    ADC_ChannelConfTypeDef sConfig = {0};

    handleADCTwo.Instance = ADC2;
    handleADCTwo.Init.ScanConvMode = ADC_SCAN_DISABLE;
    handleADCTwo.Init.ContinuousConvMode = ENABLE;
    handleADCTwo.Init.DiscontinuousConvMode = DISABLE;
    handleADCTwo.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    handleADCTwo.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    handleADCTwo.Init.NbrOfConversion = 1;

    if (HAL_ADC_Init(&handleADCTwo) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;

    if (HAL_ADC_ConfigChannel(&handleADCTwo, &sConfig) != HAL_OK) {
        Error_Handler();
    }
}

static void Initialize_Timer_One(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    handleTimerOne.Instance = TIM1;
    handleTimerOne.Init.Prescaler = 7200-1;
    handleTimerOne.Init.CounterMode = TIM_COUNTERMODE_UP;
    handleTimerOne.Init.Period = 65535;
    handleTimerOne.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    handleTimerOne.Init.RepetitionCounter = 0;
    handleTimerOne.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&handleTimerOne) != HAL_OK) {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

    if (HAL_TIM_ConfigClockSource(&handleTimerOne, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_TIMEx_MasterConfigSynchronization(&handleTimerOne, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
}

static void Initialize_Timer_Two(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    handleTimerTwo.Instance = TIM2;
    handleTimerTwo.Init.Prescaler = 1440-1;
    handleTimerTwo.Init.CounterMode = TIM_COUNTERMODE_UP;
    handleTimerTwo.Init.Period = 100-1;
    handleTimerTwo.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    handleTimerTwo.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&handleTimerTwo) != HAL_OK) {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

    if (HAL_TIM_ConfigClockSource(&handleTimerTwo, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Init(&handleTimerTwo) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_TIMEx_MasterConfigSynchronization(&handleTimerTwo, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 50-1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&handleTimerTwo, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_PWM_ConfigChannel(&handleTimerTwo, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }

    HAL_TIM_MspPostInit(&handleTimerTwo);
}

static void Initialize_Timer_Three(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    handleTimerThree.Instance = TIM3;
    handleTimerThree.Init.Prescaler = 1440-1;
    handleTimerThree.Init.CounterMode = TIM_COUNTERMODE_UP;
    handleTimerThree.Init.Period = 100-1;
    handleTimerThree.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    handleTimerThree.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&handleTimerThree) != HAL_OK) {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

    if (HAL_TIM_ConfigClockSource(&handleTimerThree, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Init(&handleTimerThree) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_TIMEx_MasterConfigSynchronization(&handleTimerThree, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 50-1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&handleTimerThree, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_PWM_ConfigChannel(&handleTimerThree, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }

    HAL_TIM_MspPostInit(&handleTimerThree);
}

static void Initialize_Timer_Four(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    handleTimerFour.Instance = TIM4;
    handleTimerFour.Init.Prescaler = 1440-1;
    handleTimerFour.Init.CounterMode = TIM_COUNTERMODE_UP;
    handleTimerFour.Init.Period = 1440-1;
    handleTimerFour.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    handleTimerFour.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&handleTimerFour) != HAL_OK) {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

    if (HAL_TIM_ConfigClockSource(&handleTimerFour, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Init(&handleTimerFour) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_TIMEx_MasterConfigSynchronization(&handleTimerFour, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&handleTimerFour, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_PWM_ConfigChannel(&handleTimerFour, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }

    HAL_TIM_MspPostInit(&handleTimerFour);
}

static void Initialize_UART(void) {
    handleUART.Instance = USART1;
    handleUART.Init.BaudRate = 9600;
    handleUART.Init.WordLength = UART_WORDLENGTH_8B;
    handleUART.Init.StopBits = UART_STOPBITS_1;
    handleUART.Init.Parity = UART_PARITY_NONE;
    handleUART.Init.Mode = UART_MODE_TX_RX;
    handleUART.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    handleUART.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&handleUART) != HAL_OK) {
        Error_Handler();
    }
}

static void Initialize_GPIO(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 | GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);

    HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);

    HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if((GPIO_Pin == GPIO_PIN_3) && pickupCoin) {
        receivedCoinFlag = true;
    }
}

void Error_Handler(void) {
    __disable_irq();
    while (1);
}
