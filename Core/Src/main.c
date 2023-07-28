/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define STEP3 600
#define STEP1 1250
#define STEP2 1400
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */

int step1;
int step2;
int step3;
int step4;
int step5;

int step_number_1;
int step_number_2;
int step_number_3;
int step_number_4;
int step_number_5;

int step=0,durum=0,sayac=0;
int step_2 =0,durum_2 = 0, sayac_2=0;
int step_3 =0,durum_3 = 0, sayac_3=0;
int step_4 =0,durum_4 = 0, sayac_4=0;
int step_5 =0,durum_5 = 0, sayac_5=0;

int second_stage;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
	second_stage = 1;
}
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2) // eger kesme kaynagi timer1 den gelmis ise
	{
																// step fonksiyonunun içindeki while döngüsünden çikilmasi için
																							// durum degiskeni sifirlandi
	if(htim ->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		sayac++;								// her kesmede sayaci arttir.
		if(sayac == step)				// eger sayac istenilen adim sayisi kadar artmis ise
		{
			HAL_TIM_PWM_Stop_IT(&htim2,TIM_CHANNEL_1);	// PWM'i durdur.
			sayac=0;																// bir sonraki komutta sayac sifirdan baslamali
			durum=0;

		}
	}

	if(htim ->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
		sayac_2++;								// her kesmede sayaci arttir.
		if(sayac_2 == step_2)				// eger sayac istenilen adim sayisi kadar artmis ise
		{
			HAL_TIM_PWM_Stop_IT(&htim2,TIM_CHANNEL_2);	// PWM'i durdur.
			sayac_2=0;																// bir sonraki komutta sayac sifirdan baslamali
			durum_2=0;
		}

	}
	}
	else if(htim ->Instance == TIM3)
	{
		sayac_3++;								// her kesmede sayaci arttir.
		if(sayac_3 == step_3)				// eger sayac istenilen adim sayisi kadar artmis ise
		{
			HAL_TIM_PWM_Stop_IT(&htim3,TIM_CHANNEL_2);	// PWM'i durdur.
			sayac_3=0;																// bir sonraki komutta sayac sifirdan baslamali
			durum_3=0;
		}
	}
	else if(htim ->Instance == TIM4)
	{
		sayac_4++;								// her kesmede sayaci arttir.
		if(sayac_4 == step_4)				// eger sayac istenilen adim sayisi kadar artmis ise
		{
			HAL_TIM_PWM_Stop_IT(&htim4,TIM_CHANNEL_1);	// PWM'i durdur.
			sayac_4=0;																// bir sonraki komutta sayac sifirdan baslamali
			durum_4=0;
		}

	}
	else if(htim ->Instance == TIM5)
	{
		sayac_5++;								// her kesmede sayaci arttir.
		if(sayac_5 == step_5)				// eger sayac istenilen adim sayisi kadar artmis ise
		{
			HAL_TIM_PWM_Stop_IT(&htim5,TIM_CHANNEL_1);	// PWM'i durdur.
			sayac_5=0;																// bir sonraki komutta sayac sifirdan baslamali
			durum_5=0;
		}

	}
	}




void Step(int adim ,int yon)  // step motor fonksiyonu. iki adet parametre alicak. Pals ve yön parametreleri
{
	// Baştan sona 4000 adım
	// 1 içeri
	// 0 dışarı
	step=adim;
	if(yon==0)
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);	// eger yön bilgisi sifir ise DIR pini lojik 0 yapildi.
	else
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);	// sifirdan farkli bir deger ise DIR pini lojik 1 yapildi.
	durum=1;
	HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_1);	// PWM'i baslatildi.
	while(1) // step motorun istenilen konuma gitmesi için belli bir süre gececek
	{						//	bu süre boyunca programin tekrar step fonksiyonunun içine girmesini önlemek için sonsuz döngü olusturuldu.
		if(durum==0)					// step motor istenilen konuma gittiginde durum degiskeni sifir olacak ve döngüden çikilacak.
			break;
		HAL_Delay(1);

	}

}


void Step_2(int adim_2 ,int yon_2)  // step motor fonksiyonu. iki adet parametre alicak. Pals ve yön parametreleri
{
	// Baştan sona 4500 step
	// 1 içeri
	// 0 dışarı
	step_2=adim_2;
	if(yon_2==0)
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);	// eger yön bilgisi sifir ise DIR pini lojik 0 yapildi.
	else
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);	// sifirdan farkli bir deger ise DIR pini lojik 1 yapildi.
	durum_2=1;
	HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_2);	// PWM'i baslatildi.
	while(1) // step motorun istenilen konuma gitmesi için belli bir süre gececek
	{						//	bu süre boyunca programin tekrar step fonksiyonunun içine girmesini önlemek için sonsuz döngü olusturuldu.
		if(durum_2==0)					// step motor istenilen konuma gittiginde durum degiskeni sifir olacak ve döngüden çikilacak.
			break;
		HAL_Delay(1);

	}

}



void Step_3(int adim_3 ,int yon_3)  // step motor fonksiyonu. iki adet parametre alicak. Pals ve yön parametreleri
{
	step_3=adim_3;
	if(yon_3==0)
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);	// eger yön bilgisi sifir ise DIR pini lojik 0 yapildi.
	else
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);	// sifirdan farkli bir deger ise DIR pini lojik 1 yapildi.
	durum_3=1;
	HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_2);	// PWM'i baslatildi.
	while(1) // step motorun istenilen konuma gitmesi için belli bir süre gececek
	{						//	bu süre boyunca programin tekrar step fonksiyonunun içine girmesini önlemek için sonsuz döngü olusturuldu.
		if(durum_3==0)					// step motor istenilen konuma gittiginde durum degiskeni sifir olacak ve döngüden çikilacak.
			break;
		HAL_Delay(1);

	}

}

void Step_4(int adim_4 ,int yon_4)  // step motor fonksiyonu. iki adet parametre alicak. Pals ve yön parametreleri
{
	step_4=adim_4;
	if(yon_4==0)
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);	// eger yön bilgisi sifir ise DIR pini lojik 0 yapildi.
	else
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);	// sifirdan farkli bir deger ise DIR pini lojik 1 yapildi.
	durum_4=1;
	HAL_TIM_PWM_Start_IT(&htim4,TIM_CHANNEL_1);	// PWM'i baslatildi.
	while(1) // step motorun istenilen konuma gitmesi için belli bir süre gececek
	{						//	bu süre boyunca programin tekrar step fonksiyonunun içine girmesini önlemek için sonsuz döngü olusturuldu.
		if(durum_4==0)					// step motor istenilen konuma gittiginde durum degiskeni sifir olacak ve döngüden çikilacak.
			break;
		HAL_Delay(1);

	}

}

void Step_5(int adim_5 ,int yon_5)  // step motor fonksiyonu. iki adet parametre alicak. Pals ve yön parametreleri
{
	step_5=adim_5;
	if(yon_5==0)
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);	// eger yön bilgisi sifir ise DIR pini lojik 0 yapildi.
	else
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);	// sifirdan farkli bir deger ise DIR pini lojik 1 yapildi.
	durum_5=1;
	HAL_TIM_PWM_Start_IT(&htim5,TIM_CHANNEL_1);	// PWM'i baslatildi.
	while(1) // step motorun istenilen konuma gitmesi için belli bir süre gececek
	{						//	bu süre boyunca programin tekrar step fonksiyonunun içine girmesini önlemek için sonsuz döngü olusturuldu.
		if(durum_5==0)					// step motor istenilen konuma gittiginde durum degiskeni sifir olacak ve döngüden çikilacak.
			break;
		HAL_Delay(1);

	}

}





void Determine_direction(int motor,int direction) {
	/*
	* Used to determine the direction of the motors
	*/
	if (motor == 1) {
		if (direction) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
		}
		else {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
		}
	}
	else if (motor == 2) {
		if (direction) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
			}
		else {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
			}
	}
	else if (motor == 3) {
			if (direction) {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
				}
			else {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
				}
		}
	else if (motor == 4) {
			if (direction) {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
				}
			else {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
				}
		}
	else if (motor == 5) {
			if (direction) {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
				}
			else {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
				}
		}
}

void Multiple_Rotate(int motor,int step, int direction) {
	/*
	 * Used to determine the number of steps the motor will take
	 * motor is used to determine which motor will rotate
	 * Integer number step represents the number of steps for the motor
	 * direction is used to change the directions of the motors (0 for clockwise, something else represents counterclockwise
	 * direction)
	 * If the step value is given as 0, the motor rotates continuously
	*/
	if (motor == 1){
	step1 = 0;
	Determine_direction(motor, direction);
	step_number_1 = step;
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
	}
	else if (motor == 2) {
	step2 = 0;
	Determine_direction(motor, direction);
	step_number_2 = step;
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);
	}
	else if (motor == 3){
	step3 = 0;
	Determine_direction(motor, direction);
	step_number_3 = step;
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_2);
	}
	else if (motor == 4) {
	step4 = 0;
	Determine_direction(motor, direction);
	step_number_4 = step;
	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_1);
	}
	else if (motor == 5) {
	step5 = 0;
	Determine_direction(motor, direction);
	step_number_5 = step;
	HAL_TIM_PWM_Start_IT(&htim5, TIM_CHANNEL_1);
	}
}
/*
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
			step1++;
			if (!step_number_1);
			else if (step1 == step_number_1) {
				HAL_TIM_PWM_Stop_IT(htim, TIM_CHANNEL_1);
			}
		}
		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
			step2++;
			if (!step_number_2);
			else if (step2 == step_number_2) {
				HAL_TIM_PWM_Stop_IT(htim, TIM_CHANNEL_2);
		}
		}
	}
	else if (htim->Instance == TIM3) {
		step3++;
		if (!step_number_3);
		else if (step3 == step_number_3) {
			HAL_TIM_PWM_Stop_IT(htim, TIM_CHANNEL_2);
		}
}
	else if (htim->Instance == TIM4) {
		step4++;
		if (!step_number_4);
		else if (step4 == step_number_4) {
			HAL_TIM_PWM_Stop_IT(htim, TIM_CHANNEL_1);
		}
	}
	else if (htim->Instance == TIM5) {
		step5++;
		if (!step_number_5);
		else if (step5 == step_number_5) {
			HAL_TIM_PWM_Stop_IT(htim, TIM_CHANNEL_1);
		}
	}
}
*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
    HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_OC_Start(&htim8, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,10000); // MAKARA 1
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,10000); // MAKARA 2
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,10000); //  SIKISTIRMA
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,10000); //  DONDURME
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1,10000); //  İLERLEME

  /*
  Multiple_Rotate(1,600, 1); //MAKARA 1
  Multiple_Rotate(2,600, 0); //MAKARA 2
  */
  //Multiple_Rotate(3,100, 1); // SIKISTIRMA

  //Multiple_Rotate(3,10000, 0); // DONDURME // 1 aç, 0 sıkıştır

  //Multiple_Rotate(5,600, 0);

  /*Step(400, 1); // içe
  Step(400, 0); // dışa
  Step_2(400, 1);
  Step_2(400, 0);
  Step_3(100, 1); // sıkıştır
  Step_3(100, 0); // gevşet
  Step_4(400, 1);
  Step_4(400, 0);
  Step_5(400, 1);
  Step_5(400, 0);
  */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (second_stage) {

	  Step_3(STEP3, 1); // sıkıştırma
	  Step(STEP1, 1); // sağ makara
	  Step_2(STEP2, 0); // sol makara
	  Step_3(STEP3, 1);
	  Step(STEP1, 1);
	  Step_2(STEP2, 0);
	  Step_3(STEP3, 1);
	  Step(STEP1, 1);
	  Step_2(STEP2, 0);

	  Step_3(STEP3, 1);
	  Step(STEP1, 0);
	  Step_2(STEP2, 1);
	  Step_3(STEP3, 1);
	  Step(STEP1, 0);
	  Step_2(STEP2, 1);
	  Step_3(STEP3, 1);
	  Step(STEP1, 0);
	  Step_2(STEP2, 1);

	  Step_3(STEP3, 0); // gevşetme
	  Step_4(200, 0); // döndürme

	  }
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
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
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 2;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 2;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 2;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 0;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

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
  HAL_GPIO_WritePin(GPIOA, dondurme_dir_Pin|sikistirma_dir_Pin|sol_makara_dir_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ilerletme_dir_GPIO_Port, ilerletme_dir_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(sag_makara_dir_GPIO_Port, sag_makara_dir_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : dondurme_dir_Pin sikistirma_dir_Pin sol_makara_dir_Pin */
  GPIO_InitStruct.Pin = dondurme_dir_Pin|sikistirma_dir_Pin|sol_makara_dir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ilerletme_dir_Pin */
  GPIO_InitStruct.Pin = ilerletme_dir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ilerletme_dir_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : sag_makara_dir_Pin */
  GPIO_InitStruct.Pin = sag_makara_dir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(sag_makara_dir_GPIO_Port, &GPIO_InitStruct);

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
