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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "TJ_MPU6050.h"
#include "stdio.h"
#include "micros.h"
#include "stdlib.h"
#include "packet.h"
#include "string.h"
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

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);

/* USER CODE BEGIN PFP */
#define MAX_SAMPLE_ADC 10

#define WAIT_FOR_DIRECTING 100000

#define MAX_BUFFER_SIZE 30

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 200);
  return ch;
}

typedef struct
{
		uint8_t flag;
		uint8_t buff[MAX_BUFFER_SIZE];
		uint8_t index;
} Uart_Typedef;

static uint8_t        data_rx;
Uart_Typedef huart_data;


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
        if(huart->Instance == huart3.Instance)
        {
			if(data_rx == 'Z')
			{
					huart_data.flag = 1;
			}
			else
			{
				huart_data.buff[huart_data.index++] = data_rx;
				if(huart_data.index > MAX_BUFFER_SIZE)
				{
					huart_data.index = 0;
				}
			}
			HAL_UART_Receive_IT(&huart3, &data_rx, 1);
        }
}
// pwm
int testPWM = 0;

// mpu
RawData_Def myAccelRaw, myGyroRaw, myMagRaw;
Offset offset;
double pitchA, pitchG;
int status;
float SelfTest[6]; // -> new
long sampling_timer;                          // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0;         // used to calculate integration interval
uint32_t Now = 0;
float axg, ayg, azg, gxrs, gyrs, gzrs;
ScaledData_Def myAccelScaled, myGyroScaled;
updateQuater upQua;
q_volatile qVol;
rawpitchyaw rpy;

// pid
float sampleFreq = 0.0f;
PID_param pid;
float last_sampleFreq;
float LF_1_wing;
float RF_2_wing;
float RB_3_wing;
float LB_4_wing;

float desired_roll_angle = 0;
float desired_pitch_angle = 0;
float desired_yaw_angle = 0;
float temp_count_roll = 0;
float temp_count_pitch = 0;

float temp = 1200;
float last_yaw;
//For fly status
uint8_t is_already_to_fly = 0;
uint8_t is_stop_flying = 0;
uint8_t is_init = 0;
uint16_t Init_CCR = 1160;

//GPS_t GPS;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	MPU_ConfigTypeDef myMpuConfig;
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
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_UART_Receive_IT(&huart3, &data_rx, 1);

  DWT_Init();
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  // mpu raw
  MPU6050_Init(&hi2c1);
  myMpuConfig.Accel_Full_Scale = AFS_SEL_8g;
  myMpuConfig.ClockSource = Internal_8MHz;
  myMpuConfig.CONFIG_DLPF = DLPF_260A_256G_Hz;
  myMpuConfig.Gyro_Full_Scale = FS_SEL_2000;
  myMpuConfig.Sleep_Mode_Bit = 0; // 1: sleep mode; 0: normal mode
  MPU6050_Config(&myMpuConfig);
  Calibration(&offset, &status); // 5000ms

  //Calibration done
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  HAL_Delay(300);
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  HAL_Delay(300);
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  HAL_Delay(300);
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  HAL_Delay(300);
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  HAL_Delay(300);
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  HAL_Delay(300);

  //GPS_Init();

  // pid
  sampling_timer = micros();
  qVol.q0 =  1.0f;
  qVol.q1 =  0.0f;
  qVol.q2 =  0.0f;
  qVol.q3 =  0.0f;
  pid.pid_d = 0;
  pid.pid_i = 0;
  pid.pid_p = 0;
//  pid.Kp_roll = 3.501;
//  pid.Ki_roll = 0.001;
//  pid.Kd_roll = 0.4;
//  pid.Kp_pitch = 3.501;
//  pid.Ki_pitch = 0.001;
//  pid.Kd_pitch = 0.4;

// 25/04: PID01 Balance P only
  pid.Kp_roll = 2.55;
  pid.Ki_roll = 0.008;
  pid.Kd_roll = 0.045;

//Best for now
//  pid.Kp_roll = 2.5;
//  pid.Ki_roll = 0.0005;
//  pid.Kd_roll = 0.115;

  //dung day can bang PID quay demo
//  pid.Kp_roll = 2.3;
//  pid.Ki_roll = 0.0001;
//  pid.Kd_roll = 0.13;

//  pid.Kp_roll = 3.0;
//  pid.Ki_roll = 0.000001;
//  pid.Kd_roll = 0.15;

//  ROL P 00.901 I 00.001 D 00.201 Z -> pid 1 truc
//  ROL P 01.250 I 00.001 D 00.221 Z -> P manh hon nhung dao dong hon.
//  ROL P 01.950 I 00.001 D 00.271 Z -> PID 2 truc
  pid.Kp_pitch = 3.6;//pid.Kp_roll / 2.0;
  pid.Ki_pitch = 0.0005;//pid.Ki_roll / 2.0;
  pid.Kd_pitch = 0.075;//pid.Kd_roll / 2.0;

  pid.Kp_yaw = 0.65;
  pid.Ki_yaw = 0.000001;
  pid.Kd_yaw = 0.002;

  pid.time = HAL_GetTick(); //   time = millis(); on arduino

//  HAL_Delay(5000);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  /* USER CODE END 2 */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, StartTask02, osPriorityNormal, 0, 128);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, StartTask03, osPriorityNormal, 0, 128);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  htim3.Init.Prescaler = 1439;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
		  MPU6050_Get_Accel_RawData(&myAccelRaw);
		  MPU6050_Get_Gyro_RawData(&myGyroRaw);
		  //MPU6050_Get_Mag_RawData(&myMagRaw);
		  updateQuaternion(myAccelRaw, myGyroRaw, myMagRaw,offset, &upQua);

		  Now = micros();
		  if (Now < lastUpdate)
			  {
		  //	      sampleFreq = 1582;
			  sampleFreq = last_sampleFreq;
			  }
		  else
			  sampleFreq = (1000000.0f / (Now - lastUpdate)); // set integration time by time elapsed since last filter update

		  lastUpdate = Now;//	  if (Now == 25000000)
		  last_sampleFreq = sampleFreq;

		  //MahonyAHRSupdateIMU(upQua, sampleFreq, &qVol);
		  //sampleFreq
		  MahonyAHRSupdate(upQua, sampleFreq, &qVol);

		  //last_yaw = rpy.yaw;
		  mpu6050_getRollPitchYaw(qVol, &rpy);
//		  if(abs(rpy.yaw - last_yaw) < 1)
//			  rpy.yaw = rpy.yaw - (rpy.yaw - last_yaw);

		  PID(&rpy, &pid, 1, Init_CCR, desired_roll_angle, desired_pitch_angle, desired_yaw_angle); // roll

		  LF_1_wing = (pid.pwm_LF_1)/20;
		  RF_2_wing = (pid.pwm_RF_2)/20;
		  RB_3_wing = (pid.pwm_RB_3)/20;
		  LB_4_wing = (pid.pwm_LB_4)/20;
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  char roll_buffer[5];
  char pitch_buffer[5];

  for(;;)
  {
	gcvt(rpy.roll, 4, roll_buffer);
	gcvt(rpy.pitch, 4, pitch_buffer);

	//printf("Roll_Angle: %s, Pitch_Angle: %s, CCR: %d,LF: %d, RF: %d, RB: %d, LB: %d\n", roll_angle_buffer, pitch_angle_buffer, (int)Init_CCR, (int)pid.pwm_LF_1, (int)pid.pwm_RF_2, (int)pid.pwm_RB_3, (int)pid.pwm_LB_4);
	printf("%s,%s,%d,\n", roll_buffer, pitch_buffer, Init_CCR);
	//printf("%s,%s,%d,\n", lat_buffer, long_buffer, Init_CCR);
	//printf("%d, %d, %d, %d \n", (int)pid.pwm_LF_1, (int)pid.pwm_RF_2, (int)pid.pwm_RB_3, (int)pid.pwm_LB_4);
	//printf("Hello\n");
	//printf("input_voltage_percentage: %d\n", (int)myADC.input_voltage_percentage);
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	//printf("%f,%f,%d\n", GPS.dec_latitude, GPS.dec_longitude, Init_CCR);
	osDelay(1000);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
//	char PID_Roll_Kp[6];
//	char PID_Roll_Ki[6];
//	char PID_Roll_Kd[6];
//	char PID_Pitch_Kp[6];
//	char PID_Pitch_Ki[6];
//	char PID_Pitch_Kd[6];
//	char Total_string[45] = "PID:";
//	char PID_Pitch_Kp[6];
//	char PID_Pitch_Ki[6];
//	char PID_Pitch_Kd[6];
  for(;;)
  {
	  if(huart_data.flag == 1){
		    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);	//Received command
	  		//ROL P 00.123 I 00.124 D 00.123 Z
	  		if(huart_data.buff[0] == 82 && huart_data.buff[1] == 79 && huart_data.buff[2] == 76){
	  			  pid.Kp_roll = Ascii_to_Int(huart_data.buff[6])*10
	  					  	  + Ascii_to_Int(huart_data.buff[7])*1
	  			  	  	  	  + Ascii_to_Int(huart_data.buff[9])*0.1
	  						  + Ascii_to_Int(huart_data.buff[10])*0.01
	  						  + Ascii_to_Int(huart_data.buff[11])*0.001;
	  			  pid.Ki_roll = Ascii_to_Int(huart_data.buff[15])*10
	  						  + Ascii_to_Int(huart_data.buff[16])*1
	  						  + Ascii_to_Int(huart_data.buff[18])*0.1
	  						  + Ascii_to_Int(huart_data.buff[19])*0.01
	  						  + Ascii_to_Int(huart_data.buff[20])*0.001;
	  			  pid.Kd_roll = Ascii_to_Int(huart_data.buff[24])*10
	  						  + Ascii_to_Int(huart_data.buff[25])*1
	  						  + Ascii_to_Int(huart_data.buff[27])*0.1
	  						  + Ascii_to_Int(huart_data.buff[28])*0.01
	  						  + Ascii_to_Int(huart_data.buff[29])*0.001;
	  		}
	  		//PIT P 00.123 I 00.124 D 00.123 Z
	  		else if(huart_data.buff[0] == 80 && huart_data.buff[1] == 73 && huart_data.buff[2] == 84){
	  			  pid.Kp_pitch = Ascii_to_Int(huart_data.buff[6])*10
	  					  	  + Ascii_to_Int(huart_data.buff[7])*1
	  			  	  	  	  + Ascii_to_Int(huart_data.buff[9])*0.1
	  						  + Ascii_to_Int(huart_data.buff[10])*0.01
	  						  + Ascii_to_Int(huart_data.buff[11])*0.001;
	  			  pid.Ki_pitch = Ascii_to_Int(huart_data.buff[15])*10
	  						  + Ascii_to_Int(huart_data.buff[16])*1
	  						  + Ascii_to_Int(huart_data.buff[18])*0.1
	  						  + Ascii_to_Int(huart_data.buff[19])*0.01
	  						  + Ascii_to_Int(huart_data.buff[20])*0.001;
	  			  pid.Kd_pitch = Ascii_to_Int(huart_data.buff[24])*10
	  						  + Ascii_to_Int(huart_data.buff[25])*1
	  						  + Ascii_to_Int(huart_data.buff[27])*0.1
	  						  + Ascii_to_Int(huart_data.buff[28])*0.01
	  						  + Ascii_to_Int(huart_data.buff[29])*0.001;
	  		}
	  		//cmd = "CCR 1600 Z"
	  		else if(huart_data.buff[0] == 67 && huart_data.buff[1] == 67 && huart_data.buff[2] == 82){
	  				Init_CCR = Ascii_to_Int(huart_data.buff[4])*1000 +
	  						Ascii_to_Int(huart_data.buff[5])*100 +
	  						Ascii_to_Int(huart_data.buff[6])*10 +
	  						Ascii_to_Int(huart_data.buff[7])*1;
	  				is_stop_flying = 0;
	  				is_already_to_fly = 1;
	  		}
	  		//cmd = "ICRR Z"
	  		else if(huart_data.buff[0] == 73 && huart_data.buff[1] == 67 && huart_data.buff[2] == 67 && huart_data.buff[3] == 82){
	  				Init_CCR +=2;
	  		}
	  		//cmd = "DCCR Z"
	  		else if(huart_data.buff[0] == 68 && huart_data.buff[1] == 67 && huart_data.buff[2] == 67 && huart_data.buff[3] == 82){
	  				Init_CCR -=5;
	  		}
	  		//cmd = "INIT Z"
	  		else if(huart_data.buff[0] == 73 && huart_data.buff[1] == 78 && huart_data.buff[2] == 73 && huart_data.buff[3] == 84){
	  				is_init = 1;
	  		}
	  		//cmd = STOP Z
	  		else if(huart_data.buff[0] == 83 && huart_data.buff[1] == 84 && huart_data.buff[2] == 79 && huart_data.buff[3] == 80){
	  				is_stop_flying = 1;
	  		}
	  		//cmd = LEFT Z
	  		else if(huart_data.buff[0] == 76 && huart_data.buff[1] == 69 && huart_data.buff[2] == 70 && huart_data.buff[3] == 84){
	  				desired_roll_angle = -8;
	  				temp_count_roll = WAIT_FOR_DIRECTING;
	  		}
	  		//cmd = RIGHT Z
	  		else if(huart_data.buff[0] == 82 && huart_data.buff[1] == 73 && huart_data.buff[2] == 71 && huart_data.buff[3] == 72 && huart_data.buff[4] == 84){
	  				desired_roll_angle = 8;
	  				temp_count_roll = WAIT_FOR_DIRECTING;
	  		}
	  		//cmd = FORWARD Z
	  		else if(huart_data.buff[0] == 70 && huart_data.buff[1] == 79 && huart_data.buff[2] == 82 && huart_data.buff[3] == 87 && huart_data.buff[4] == 65 && huart_data.buff[5] == 82 && huart_data.buff[6] == 68){
	  				desired_pitch_angle = -8;
	  				temp_count_pitch = WAIT_FOR_DIRECTING;
	  		}
	  		//cmd = BACKWARD Z
	  		else if(huart_data.buff[0] == 66 && huart_data.buff[1] == 65 && huart_data.buff[2] == 67 && huart_data.buff[3] == 75 && huart_data.buff[4] == 87 && huart_data.buff[5] == 65 && huart_data.buff[6] == 82 && huart_data.buff[7] == 68){
	  				desired_pitch_angle = 8;
	  				temp_count_pitch = WAIT_FOR_DIRECTING;
	  		}
			memset(huart_data.buff, 0, MAX_BUFFER_SIZE);
			huart_data.index = 0;
			huart_data.flag = 0;
	  }
	  	  //direction

	  if(temp_count_roll > 0)
	  {
		  temp_count_roll = temp_count_roll - 1;
	  }
	  else if(temp_count_roll == 0)
	  {
		  desired_roll_angle = 0;
	  }

	  if(temp_count_pitch > 0)
	  {
		  temp_count_pitch = temp_count_pitch - 1;
	  }
	  else if(temp_count_pitch == 0)
	  {
		  desired_pitch_angle = 0;
	  }

	  if(is_init == 1){
		  for(int i=50; i<=60; i++){
			  //debug
			  htim3.Instance->CCR2 = i;
			  htim3.Instance->CCR3 = i;
			  htim3.Instance->CCR1 = i;
			  htim3.Instance->CCR4 = i;
			  HAL_Delay(200);
		  }

		  for(int i = 60; i>=50; i--){
			  //debug9
			  htim3.Instance->CCR2 = i;
			  htim3.Instance->CCR3 = i;
			  htim3.Instance->CCR1 = i;
			  htim3.Instance->CCR4 = i;
			  HAL_Delay(200);
		  }
		  is_init = 0;
	  }

	  if(is_stop_flying == 1)
	  {
		  htim3.Instance->CCR2 = 50;
		  htim3.Instance->CCR3 = 50;
		  htim3.Instance->CCR1 = 50;
		  htim3.Instance->CCR4 = 50;
		  is_already_to_fly = 0;
	  }

	  if(is_already_to_fly == 1)
	  {
		  htim3.Instance->CCR2 = RF_2_wing;
		  htim3.Instance->CCR3 = RB_3_wing;
		  htim3.Instance->CCR1 = LF_1_wing;
		  htim3.Instance->CCR4 = LB_4_wing;
	  }
  }
  /* USER CODE END StartTask03 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
