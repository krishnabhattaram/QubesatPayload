/* USER CODE BEGIN Header */

/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
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

#include <stdarg.h>
#include <stdio.h>

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

ADC_HandleTypeDef hadc;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void MX_USART2_UART_Init(void);

static void MX_SPI1_Init(void);

static void MX_ADC_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */

int main(void) {
    /* USER CODE BEGIN 1 */

    char spi_buf[20];
    uint16_t freq;
    uint16_t addr;

    /* USER CODE END 1 */

    /* MCU
     * Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the
     * Systick. */

    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */

    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */

    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_SPI1_Init();
    MX_ADC_Init();

    /* USER CODE BEGIN 2 */

    // CS pin should default high

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

    printf_to_uart("Starting SPI Test\r\n");

    HAL_Delay(500);  // Waiting for default register initialization of RF
                     // generator (synchronous turn on)

    Initialize_Microwave_Generator();

    // Maximum gain

    sendBytes(0x4DB8, 0b00101000);

    // Set divider to 1

    sendBytes(0x90, 0b00101000);

    // Ref Divider Register-Default value = 1h (Rdiv=1).

    sendBytes(0x1, 0b00010000);

    /// Make sure to remove!!

    // Set_VCO_Out_Divider(20);

    /* USER CODE END 2 */

    int num_cycles = 1;
    int samples_per_freq = 100;
    int freq_steps = 100;
    int start_frequency = 2700;
    int end_frequency = 3000;
    int step_size = (end_frequency - start_frequency) / freq_steps;
    int set_freq;

    int contrast_arr_len = freq_steps * sizeof(uint16_t);
    uint16_t *contrast_arr;
    double cur_sum;
    /* Infinite loop */

    /* USER CODE BEGIN WHILE */

    // int ms1 = HAL_GetTick();
    for (unsigned i = 0; i < num_cycles; i++) {
        set_freq = start_frequency;
        contrast_arr = (uint16_t *)malloc(contrast_arr_len);
        for (int j = 0; set_freq <= end_frequency; j++) {
            set_freq += step_size;
            cur_sum = 0;
            for (unsigned sample_count = 0; sample_count < samples_per_freq;
                 sample_count++) {
                cur_sum +=
                    measure_at_frequency(set_freq) / measure_at_frequency(1500);
            }
            contrast_arr[j] = (uint16_t)(cur_sum / samples_per_freq);
        }
        // HAL_UART_Transmit(&huart2, (uint8_t *)contrast_arr, contrast_arr_len,
        //   HAL_MAX_DELAY); // for production
        print_data_to_uart(contrast_arr, contrast_arr_len);  // for testing
    }

    // uint16_t *data_array = (uint16_t*) malloc(3000 * sizeof(uint16_t));
    // HAL_UART_Transmit(&huart2, (uint8_t*)data_array, 6000, HAL_MAX_DELAY);

    // int ms2 = HAL_GetTick();

    // uart_buf_len = sprintf(uart_buf, "%d \r\n", ms2 - ms1);
    // HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, uart_buf_len,
    // HAL_MAX_DELAY);

    Set_VCO_Frequency(1500);

    /* USER CODE END 3 */
}

double measure_at_frequency(int frequency) {
    Set_VCO_Frequency(frequency);

    HAL_ADC_Start(&hadc);
    HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
    return (double)HAL_ADC_GetValue(&hadc);

    // // Convert to string and print
    // printf_to_uart("%hu %d %d\r\n", photodiode_in, frequency,
    //                frequency == 1500);
}

void printf_to_uart(char *format, ...) {
    char print_buf[100];  // allocate a larger buffer
    char uart_buf[50];
    int print_len;

    va_list args;
    va_start(args, format);
    print_len = vsnprintf(print_buf, sizeof(print_buf), format, args);
    va_end(args);

    // copy the relevant portion of the print buffer to uart buffer
    int uart_buf_len = (print_len > 50) ? 50 : print_len;
    memcpy(uart_buf, print_buf, uart_buf_len);

    HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len,
                      HAL_MAX_DELAY);
}

/* Prints 16-bit int array to UART with newlines in between. */
void print_data_to_uart(uint16_t *data, int len) {
    unsigned MAX_INTS_PER_TRANSMIT = 30; // pulled this out my ass ngl

    char fmt[50];   // Create a format string buffer
    char buf[500];  // Create a buffer for the formatted string

    for (unsigned printed = 0; printed < len; printed += MAX_INTS_PER_TRANSMIT) {
        int l = 0;  // Keep track of the length of the formatted string
        int n;  // Keep track of the number of characters added to the formatted
                // string

        // Build the format string with %d and delimiter
        snprintf(fmt, sizeof(fmt), "%%d%s", "\r\n");

        // Add each array element to the formatted string
        unsigned to_transmit = MAX_INTS_PER_TRANSMIT;
        if (len - printed < MAX_INTS_PER_TRANSMIT) {
            to_transmit = len - printed;
        }
        for (unsigned i = 0; i < to_transmit; i++) {
            n = snprintf(buf + l, sizeof(buf) - l, fmt, data[i]);
            if (n < 0 || l + n >= sizeof(buf)) {
                // Error handling: buffer overflow or snprintf error
                return;
            }
            l += n;
        }

        // Print the formatted string
        printf_to_uart("%s", buf);
    }
}

/**

  * @brief System Clock Configuration

  * @retval None

  */

void SystemClock_Config(void)

{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Configure the main internal regulator output voltage

    */

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters

    * in the RCC_OscInitTypeDef structure.

    */

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_3;
    RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks

    */

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
        Error_Handler();
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;

    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief ADC Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC_Init(void) {
    /* USER CODE BEGIN ADC_Init 0 */

    /* USER CODE END ADC_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC_Init 1 */

    /* USER CODE END ADC_Init 1 */

    /** Configure the global features of the ADC (Clock, Resolution, Data
       Alignment and number of conversion)

    */

    hadc.Instance = ADC1;

    hadc.Init.OversamplingMode = DISABLE;

    hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;

    hadc.Init.Resolution = ADC_RESOLUTION_12B;

    hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;

    hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;

    hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;

    hadc.Init.ContinuousConvMode = DISABLE;

    hadc.Init.DiscontinuousConvMode = DISABLE;

    hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;

    hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;

    hadc.Init.DMAContinuousRequests = DISABLE;

    hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

    hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;

    hadc.Init.LowPowerAutoWait = DISABLE;

    hadc.Init.LowPowerFrequencyMode = DISABLE;

    hadc.Init.LowPowerAutoPowerOff = DISABLE;

    if (HAL_ADC_Init(&hadc) != HAL_OK)

    {
        Error_Handler();
    }

    /** Configure for the selected ADC regular channel to be converted.

    */

    sConfig.Channel = ADC_CHANNEL_0;

    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;

    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)

    {
        Error_Handler();
    }

    /* USER CODE BEGIN ADC_Init 2 */

    /* USER CODE END ADC_Init 2 */
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

    hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;

    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;

    hspi1.Init.NSS = SPI_NSS_SOFT;

    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;

    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;

    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;

    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;

    hspi1.Init.CRCPolynomial = 7;

    if (HAL_SPI_Init(&hspi1) != HAL_OK)

    {
        Error_Handler();
    }

    /* USER CODE BEGIN SPI1_Init 2 */

    /* USER CODE END SPI1_Init 2 */
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

    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;

    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

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

    /* GPIO Ports Clock Enable */

    __HAL_RCC_GPIOC_CLK_ENABLE();

    __HAL_RCC_GPIOH_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

    /*Configure GPIO pin : B1_Pin */

    GPIO_InitStruct.Pin = B1_Pin;

    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;

    GPIO_InitStruct.Pull = GPIO_NOPULL;

    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PA10 */

    GPIO_InitStruct.Pin = GPIO_PIN_10;

    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;

    GPIO_InitStruct.Pull = GPIO_NOPULL;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : PA12 */

    GPIO_InitStruct.Pin = GPIO_PIN_12;

    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;

    GPIO_InitStruct.Pull = GPIO_NOPULL;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : PB6 */

    GPIO_InitStruct.Pin = GPIO_PIN_6;

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull = GPIO_NOPULL;

    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Set_VCO_Out_Divider(int divisor) {
    uint16_t reg_5_bottom_bits = 0b0010000;

    sendBytes(divisor << 7 | reg_5_bottom_bits, 0b00101000);

    sendBytes(0x0, 0b00101000);
}

/* USER CODE BEGIN 4 */

void Set_VCO_Frequency(int freq) {
    int xtal = 50;

    int integer_data = freq / xtal;

    int frac_mod = freq % xtal;

    double frac_value = 1.0 * freq / xtal - integer_data;

    int frac_data = frac_value * pow(2, 24);

    uint8_t reg_5 = 0b00101000;

    // REG 3 1C

    uint8_t reg_3 = 0b00011000;

    sendBytes(integer_data, reg_3);

    // REG 5 0

    sendBytes(0, reg_5);

    // REG 4 0

    uint8_t reg_4 = 0b00100000;

    sendBytes(frac_data, reg_4);
}

void sendBytes(int data, uint8_t addressByte) {
    uint8_t firstIntDataByte = data & 0xff;

    data = data >> 8;

    uint8_t secondIntDataByte = data & 0xff;

    data = data >> 8;

    uint8_t thirdIntDataByte = data & 0xff;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);  // SEN Deassert

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&thirdIntDataByte, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&secondIntDataByte, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&firstIntDataByte, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&addressByte, 1, 100);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);  // SEN Assert
}

void Initialize_Microwave_Generator(void) {
    uint8_t VarZero = 0b00000000;

    uint8_t VarOne = 0b00100000;

    uint8_t VarTwo = 0b00000010;

    uint8_t VarThree = 0b00001000;

    uint8_t VarFour = 0b00000001;

    uint8_t VarFive = 0b00010000;

    uint8_t VarSix = 0b10010000;

    uint8_t VarSeven = 0b00101000;

    uint8_t VarEight = 0b00001111;

    uint8_t VarNine = 0b10011000;

    uint8_t VarOneZero = 0b01001011;

    uint8_t VarOneOne = 0b00111000;

    uint8_t VarOneTwo = 0b01001010;

    uint8_t VarOneThree = 0b00110000;

    uint8_t VarOneFour = 0b01001101;

    uint8_t VarOneFive = 0b11000001;

    uint8_t VarOneSix = 0b10111110;

    uint8_t VarOneSeven = 0b11111111;

    uint8_t VarOneEight = 0b01000000;

    uint8_t VarOneNine = 0b00111111;

    uint8_t VarTwoZero = 0b11111110;

    uint8_t VarTwoOne = 0b11111101;

    uint8_t VarTwoTwo = 0b01001000;

    uint8_t VarTwoThree = 0b01000110;

    uint8_t VarTwoFour = 0b01010000;

    uint8_t VarTwoFive = 0b01001111;

    uint8_t VarTwoSix = 0b10000000;

    uint8_t VarTwoSeven = 0b01100001;

    uint8_t VarTwoEight = 0b01011000;

    uint8_t VarTwoNine = 0b01100000;

    uint8_t VarThreeZero = 0b10000001;

    uint8_t VarThreeOne = 0b01111000;

    uint8_t VarThreeTwo = 0b00101010;

    uint8_t VarThreeThree = 0b00011000;

    uint8_t VarThreeFour = 0b01100110;

    // REG 0 20

    // RESET command.

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);  // SEN Deassert

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarZero, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarZero, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarOne, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarZero, 1, 100);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);  // SEN Assert

    HAL_Delay(50);

    // REG 1 2

    // Default = 2.  This value assigns PLL Chip Enable control to the SPI Reg 1
    // [1], 1 enabled, 0 disabled.  To assign PLL CE control to CE pin, write
    // Reg 1[0]=1.

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);  // SEN Deassert

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarZero, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarZero, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarTwo, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarThree, 1, 100);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);  // SEN Assert

    HAL_Delay(50);

    // REG 2 1

    // Ref Divider Register-Default value = 1h (Rdiv=1). Program as needed.

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);  // SEN Deassert

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarZero, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarZero, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarFour, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarFive, 1, 100);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);  // SEN Assert

    HAL_Delay(50);

    // REG 5 90

    // Reg02 = 000000001 = fo (VCO output Div-by-1);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);  // SEN Deassert

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarZero, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarZero, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarSix, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarSeven, 1, 100);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);  // SEN Assert

    HAL_Delay(50);

    // REG 5 F98

    // Reg03 = 000011111 = Hi Perf, RF_P & RF_N enabled, 5dB RL,

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);  // SEN Deassert

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarZero, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarEight, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarNine, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarSeven, 1, 100);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);  // SEN Assert

    HAL_Delay(50);

    // REG 5 4B38

    // Reg07 = 010010110 = o/p -6dB.  For maximum o/p power program 4DB8h.

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);  // SEN Deassert

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarZero, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarOneZero, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarOneOne, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarSeven, 1, 100);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);  // SEN Assert

    HAL_Delay(50);

    // REG 5 0

    // Close out VCO register programming by writing Reg 5 = 0.

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);  // SEN Deassert

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarZero, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarZero, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarZero, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarSeven, 1, 100);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);  // SEN Assert

    HAL_Delay(50);

    // REG 6 F4A

    // Delta-Sigma Modulator Configuration Register. Program this value for Frac
    // Mode.

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);  // SEN Deassert

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarZero, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarEight, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarOneTwo, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarOneThree, 1, 100);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);  // SEN Assert

    HAL_Delay(50);

    // REG 7 14D

    // 14Dh is the default value for LD programming (correct for 50MHz
    // comparison). For different configurations, especially higher PFD rates,
    // this may need to change.

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);  // SEN Deassert

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarZero, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarFour, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarOneFour, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarOneOne, 1, 100);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);  // SEN Assert

    HAL_Delay(50);

    // REG 8 C1BEFF

    // Default value = C1BEFFh.  No need to program.

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);  // SEN Deassert

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarOneFive, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarOneSix, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarOneSeven, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarOneEight, 1, 100);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);  // SEN Assert

    HAL_Delay(50);

    // REG 9 3FFEFD

    // CP Register-Program as needed. 3FFEFDh = 2.54mA CP current with 635uA Up
    // CP Offset current.

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);  // SEN Deassert

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarOneNine, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarTwoZero, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarTwoOne, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarTwoTwo, 1, 100);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);  // SEN Assert

    HAL_Delay(50);

    // REG A 2046

    // VCO Tuning Configuration Register-Program this value.

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);  // SEN Deassert

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarZero, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarOne, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarTwoThree, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarTwoFour, 1, 100);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);  // SEN Assert

    HAL_Delay(50);

    // REG B 4F8061

    // PFD/CP Control Register.  Default value = F8061h.  4F8061h sets LD/SDO
    // output level to 3.3V from 1.8V default (bit[22]=1 sets 3.3V output
    // level).

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);  // SEN Deassert

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarTwoFive, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarTwoSix, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarTwoSeven, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarTwoEight, 1, 100);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);  // SEN Assert

    HAL_Delay(50);

    // REG C 0

    // Exact Frequency register.  Default value =0h.  No need to program if not
    // using Exact Frequency Mode.

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);  // SEN Deassert

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarZero, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarZero, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarZero, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarTwoNine, 1, 100);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);  // SEN Assert

    HAL_Delay(50);

    // REG F 81

    // Default vaue =1. 81h configures LD/SDO pin to output LD status always,
    // except during SPI reads when the pin is automatically mux'ed to output
    // the serial data.

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);  // SEN Deassert

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarZero, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarZero, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarThreeZero, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarThreeOne, 1, 100);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);  // SEN Assert

    HAL_Delay(50);

    // REG 3 2A

    // Integer VCO Divider Register-Progarm as needed to set frequency.

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);  // SEN Deassert

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarZero, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarZero, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarThreeTwo, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarThreeThree, 1, 100);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);  // SEN Assert

    HAL_Delay(50);

    // REG 4 666666

    // Fractional VCO Divider Register-Program as needed to set frequency. When
    // this register is written, a VCO auto-cal is initiated.

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);  // SEN Deassert

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarThreeFour, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarThreeFour, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarThreeFour, 1, 100);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&VarOne, 1, 100);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);  // SEN Assert

    HAL_Delay(50);
}

/* USER CODE END 4 */

/**

  * @brief  This function is executed in case of error occurrence.

  * @retval None

  */

void Error_Handler(void)

{
    /* USER CODE BEGIN Error_Handler_Debug */

    /* User can add his own implementation to report the HAL error return state
     */

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

    /* User can add his own implementation to report the file name and line
       number,

       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line)
     */

    /* USER CODE END 6 */
}

#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
