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
#include "bmp280_secondary.h"
#include "bmp280.h"
#include "mpu6050.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
float altitude = 0.00f;
float gx = 0.00f;
float gy = 0.00f;
float gz = 0.00f;
float ax = 0.00f;
float ay = 0.00f;
float az = 0.00f;
float cc = 0.00f;
float cv = 0.00f;
float fused_vz = 0.00f;
float roll_deg = 0.00f;
float pitch_deg = 0.00f;
float prev_altitude = 0.00f;
float vz_avg = 0.00f;
int debug = 0;
float fused_vz_bias = 0.0f;
float pitch_bias = 0.0f;

int sensor_task_running = 0;
int display_task_running = 0;
int failure_monitor_running = 0;
float pitch_calibrated = 0.0f;


int bmp_primary_failure_state = 0;
int bmp_recovery_flag = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart2;

/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
uint32_t SensorTaskBuffer[ 2024 ];
osStaticThreadDef_t SensorTaskControlBlock;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .cb_mem = &SensorTaskControlBlock,
  .cb_size = sizeof(SensorTaskControlBlock),
  .stack_mem = &SensorTaskBuffer[0],
  .stack_size = sizeof(SensorTaskBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for DisplayTask */
osThreadId_t DisplayTaskHandle;
uint32_t DisplayTaskBuffer[ 5024 ];
osStaticThreadDef_t DisplayTaskControlBlock;
const osThreadAttr_t DisplayTask_attributes = {
  .name = "DisplayTask",
  .cb_mem = &DisplayTaskControlBlock,
  .cb_size = sizeof(DisplayTaskControlBlock),
  .stack_mem = &DisplayTaskBuffer[0],
  .stack_size = sizeof(DisplayTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for FailureMonitor */
osThreadId_t FailureMonitorHandle;
uint32_t FailureMonitorBuffer[ 2024 ];
osStaticThreadDef_t FailureMonitorControlBlock;
const osThreadAttr_t FailureMonitor_attributes = {
  .name = "FailureMonitor",
  .cb_mem = &FailureMonitorControlBlock,
  .cb_size = sizeof(FailureMonitorControlBlock),
  .stack_mem = &FailureMonitorBuffer[0],
  .stack_size = sizeof(FailureMonitorBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART2_UART_Init(void);
void sensor_aq_task(void *argument);
void display_refresh(void *argument);
void failiure_detector(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

PUTCHAR_PROTOTYPE {
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART1 and Loop until the end of transmission */
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 300);

	return ch;
}

float vertical_velocity_baro(float altitude, float dt) {
	static float prev_alt = 0.0f;
	static uint8_t initialized = 0;
	static float vz = 0.0f;

	if (!initialized) {
		prev_alt = altitude;
		initialized = 1;
		return 0.0f;
	}

	float raw_vz = (altitude - prev_alt) / dt;
	const float alpha = 0.85f;
	vz = alpha * vz + (1.0f - alpha) * raw_vz;

	prev_alt = altitude;

	return vz;
}

float vertical_velocity_imu(float ax, float ay, float az, float roll_deg,
		float pitch_deg, float dt) {
	static float vz = 0.0f;
	static float bias = 0.0f;
	static uint8_t initialized = 0;

	const float g = 9.80665f;

	float roll = roll_deg * 0.0174533f;
	float pitch = pitch_deg * 0.0174533f;

	float accZ_inertial = -sinf(pitch) * ax + cosf(pitch) * sinf(roll) * ay
			+ cosf(pitch) * cosf(roll) * az;

	float az_true = (accZ_inertial - 1.0f) * g;

	if (!initialized) {
		bias = az_true;
		initialized = 1;
		return 0.0f;
	}

	bias = 0.995f * bias + 0.005f * az_true;
	az_true -= bias;
	vz += az_true * dt;
	vz *= 0.999f;

	return vz;
}

float average_altitude(int steps) {
	float sum = 0.0f;

	for (int i = 0; i < steps; i++) {
		sum += altitude_calc();
		HAL_Delay(10);
	}

	return sum / steps;
}

float vertical_velocity_fused(float ax, float ay, float az, float roll_deg,
		float pitch_deg, float dt) {
	static float vz = 0.0f;
	static float prev_alt = 0.0f;
	static float vz_baro = 0.0f;

	const float g = 9.80665f;

	float roll = roll_deg * 0.0174533f;
	float pitch = pitch_deg * 0.0174533f;

	float accZ_inertial = -sinf(pitch) * ax + cosf(pitch) * sinf(roll) * ay
			+ cosf(pitch) * cosf(roll) * az;

	float az_true = (accZ_inertial - 1.0f) * g;
	float altitude = altitude_calc();
	float raw_vz_baro = (altitude - prev_alt) / dt;

	const float alpha_baro = 0.91f;
	vz_baro = alpha_baro * vz_baro + (1.0f - alpha_baro) * raw_vz_baro;
	prev_alt = altitude;

	const float k_baro = 0.05f; //earlier 0,05

	vz += az_true * dt;
	vz += (vz_baro - vz) * k_baro;

	return vz;
}

float display_refresh_averaged(float input, int count) {
	for (int i = 0; i <= count; i++) {
		input += input;
	}

	input = input / count;

	return input;
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	mpu_init();
	bmp_i2c_setup();
	bmp_i2c_setup_secondary();

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0); // Inverted LED logic
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);

	HAL_Delay(100);

	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);



	float sum = 0.0f;
	float pitch_sum = 0.0f;

	debug = 1;

	for (int i = 0; i < 100; i++) {
		gx = mpu_gyro_read(0);
		gy = mpu_gyro_read(1);
		gz = mpu_gyro_read(2);

		ax = mpu_accel_read(0);
		ay = mpu_accel_read(1);
		az = mpu_accel_read(2);

		mpu_get_kalman_angles(&roll_deg, &pitch_deg);

		pitch_sum += pitch_deg;

		altitude = altitude_calc();

		float temp = vertical_velocity_fused(ax, ay, az, roll_deg, pitch_deg,
				0.02f);

		sum += temp;

		HAL_Delay(2);
	}

	pitch_bias = pitch_sum / 100.0f;

	fused_vz_bias = sum / 100.0f;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0); //Inverted LED logic
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);

	debug = 100;

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
  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(sensor_aq_task, NULL, &SensorTask_attributes);

  /* creation of DisplayTask */
  DisplayTaskHandle = osThreadNew(display_refresh, NULL, &DisplayTask_attributes);

  /* creation of FailureMonitor */
  FailureMonitorHandle = osThreadNew(failiure_detector, NULL, &FailureMonitor_attributes);

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
	while (1) {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_sensor_aq_task */
/**
 * @brief  Function implementing the SensorTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_sensor_aq_task */
void sensor_aq_task(void *argument)
{
  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {

		//sensor_task_running = 1;

		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // Toggle LED

		gx = mpu_gyro_read(0);
		gy = mpu_gyro_read(1);
		gz = mpu_gyro_read(2);
		ax = mpu_accel_read(0);
		ay = mpu_accel_read(1);
		az = mpu_accel_read(2);

		mpu_get_kalman_angles(&roll_deg, &pitch_deg);

		pitch_calibrated = pitch_deg - pitch_bias;

		if (bmp_recovery_flag == 1) {

			altitude = altitude_calc_secondary();
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
		} else {
			altitude = altitude_calc();
		}
		cv = vertical_velocity_baro(altitude, 0.01f);
		cc = vertical_velocity_imu(ax, ay, az, roll_deg, pitch_deg, 0.01f);
		fused_vz = vertical_velocity_fused(ax, ay, az, roll_deg, pitch_deg,
				0.01f);
		fused_vz -= fused_vz_bias;


		uint32_t err = HAL_I2C_GetError(&hi2c1);

		if (bmp_recovery_flag == 0) {
			if (err == HAL_I2C_ERROR_NONE) {
				bmp_primary_failure_state = 0;
			} else if (err & HAL_I2C_ERROR_AF) {
				bmp_primary_failure_state++;
			} else if (err & HAL_I2C_ERROR_BERR) {
				bmp_primary_failure_state++;
			} else if (err & HAL_I2C_ERROR_TIMEOUT) {
				bmp_primary_failure_state++;
			} else {
				bmp_primary_failure_state++;
			}
		}

		osDelay(1);
		//sensor_task_running = 0;

	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_display_refresh */
/**
 * @brief Function implementing the DisplayTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_display_refresh */
void display_refresh(void *argument)
{
  /* USER CODE BEGIN display_refresh */
	/* Infinite loop */
	for (;;) {
		//display_task_running = 1;
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13); // Toggle LED
		printf(
				"t1.txt=\"%.2f m/s\"\xFF\xFF\xFFt2.txt=\"%.2f m\"\xFF\xFF\xFFt3.txt=\"%.2f deg\"\xFF\xFF\xFF",
				fused_vz, altitude, pitch_calibrated);
		osDelay(1);
		//display_task_running = 0;
	}
  /* USER CODE END display_refresh */
}

/* USER CODE BEGIN Header_failiure_detector */
/**
 * @brief Function implementing the FailureMonitor thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_failiure_detector */
void failiure_detector(void *argument)
{
  /* USER CODE BEGIN failiure_detector */
	/* Infinite loop */
	for (;;) {
		//failure_monitor_running = 1;

		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); // Toggle LED
		if (bmp_primary_failure_state >= 5) {
			bmp_recovery_flag = 1;
			//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
		}
		osDelay(200);
		//failure_monitor_running = 0;
	}
  /* USER CODE END failiure_detector */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11)
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
	while (1) {
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
