
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "xprintf.h"
#include "control_operations.h"
#include "basic_operations.h"
#include "odometry.h"
#include "body.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM7_Init(void);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

GPIO_PinState state1,state2,state3,state4;

TIM_MasterConfigTypeDef sMasterConfig;
TIM_OC_InitTypeDef sConfigOC;

void uart_putc(uint8_t c)
{
	char buf[1];
	buf[0] = c;
	HAL_UART_Transmit(&huart2, (uint8_t *)buf, sizeof(buf), 0xFFFF);
}

void uart_puts(char *str)
{
	while (*str) {
		uart_putc(*str++);
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  xdev_out(uart_putc);

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM12_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  if(HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1)!= HAL_OK){Error_Handler();}
  if(HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2)!= HAL_OK){Error_Handler();}
  if(HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3)!= HAL_OK){Error_Handler();}
  if(HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4)!= HAL_OK){Error_Handler();}

  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);//right-wheel
  HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);//left-wheel
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);//throwing arm

  HAL_TIM_Base_Start_IT(&htim7);

  /* Non loop */
  const double high_speed = 2.0;
  const double low_speed = 0.5;
  const double middle_speed = (high_speed+low_speed)/2.0;
  const double safe_speed = 0.8;

  //set odometry
  int field_dir = -1;
  odometry_set_position(field_dir*0.615,-0.5, (double)(1-field_dir)/2.0*PI);

  //wait
  while(remote_sw_getState() == 0){HAL_Delay(10);}

  //set encoder
  enc_arm_setAngle_rad(PI/2.0);
  control_av_throwingArm_start (PI/8.0, 0);

  vec4 coes_x, coes_y;

  //-15cm
  //start moving
  coes_x = cubicCurve_bezier(field_dir*0.615, field_dir*1.79, field_dir*1.79, field_dir*1.79);
  coes_y = cubicCurve_bezier(-0.5, -0.5, -1.25, -2.0);
  control_follow_cubicCurve_start(coes_x, coes_y, low_speed, middle_speed, middle_speed, middle_speed, middle_speed, 0);

  coes_x = cubicCurve_bezier(field_dir*1.79, field_dir*1.79, field_dir*0.66, field_dir*0.66);
  coes_y = cubicCurve_bezier(-2.0, -2.75, -2.75, -3.5);
  while (isUnderControl(2)){HAL_Delay(10);}
  control_follow_cubicCurve_start(coes_x, coes_y, middle_speed, middle_speed, middle_speed, middle_speed, middle_speed, 0);

  coes_x = cubicCurve_bezier(field_dir*0.66, field_dir*0.66, field_dir*1.79, field_dir*1.79);
  coes_y = cubicCurve_bezier(-3.5, -4.25, -4.25, -5.0);
  while (isUnderControl(2)){HAL_Delay(10);}
  control_follow_cubicCurve_start(coes_x, coes_y, middle_speed, middle_speed, middle_speed, middle_speed, middle_speed, 0);

  coes_x = cubicCurve_bezier(field_dir*1.79, field_dir*1.79, field_dir*1.33, field_dir*1.33);
  coes_y = cubicCurve_bezier(-5.0, -5.75, -5.75, -6.5);
  while (isUnderControl(2)){HAL_Delay(10);}
  control_follow_cubicCurve_start(coes_x, coes_y, middle_speed, middle_speed, middle_speed, middle_speed, middle_speed, 0);


  /*
  coes_x = cubicCurve_bezier(field_dir*1.79, field_dir*1.79, field_dir*1.225, field_dir*1.225);
  coes_y = cubicCurve_bezier(-5.0, -5.75, -5.75, -6.5);
  while (isUnderControl(2)){HAL_Delay(10);}
  control_follow_cubicCurve_start(coes_x, coes_y, middle_speed, middle_speed, middle_speed, middle_speed, middle_speed);
  */

  //bridge

  coes_x = cubicCurve_bezier(field_dir*1.33, field_dir*1.33, field_dir*1.33, field_dir*1.33);
  coes_y = cubicCurve_bezier(-6.5, -7.0, -7.4, -7.9);
  while (isUnderControl(2)){HAL_Delay(10);}
  control_follow_cubicCurve_start(coes_x, coes_y, middle_speed, middle_speed, middle_speed, middle_speed, middle_speed, 0);

  while (isUnderControl(2)){HAL_Delay(10);}
  odometry_set_position(field_dir*1.225, -8.0, -PI/2.0);


  /*
  odometry_set_position(field_dir*1.225, -6.5, -PI/2.0);

  coes_x = cubicCurve_bezier(field_dir*1.225, field_dir*1.225, field_dir*1.225, field_dir*1.225);
  coes_y = cubicCurve_bezier(-6.5, -7.0, -7.5, -8.0);
  while (isUnderControl(2)){HAL_Delay(10);}
  control_follow_cubicCurve_start(coes_x, coes_y, low_speed, middle_speed, middle_speed, middle_speed, middle_speed);
  */

  //meadow

  coes_x = cubicCurve_bezier(field_dir*1.225, field_dir*1.225, field_dir*3.46, field_dir*4.5);
  coes_y = cubicCurve_bezier(-8.0, -9.0, -9.0, -9.0);
  while (isUnderControl(2)){HAL_Delay(10);}
  control_follow_cubicCurve_start(coes_x, coes_y, middle_speed, middle_speed, high_speed, high_speed, middle_speed, 0);

  coes_x = cubicCurve_bezier(field_dir*4.5, field_dir*4.9675, field_dir*5.435, field_dir*5.435);
  coes_y = cubicCurve_bezier(-9.0, -9.0, -9.0, -8.3);
  while (isUnderControl(2)){HAL_Delay(10);}
  control_follow_cubicCurve_start(coes_x, coes_y, middle_speed, middle_speed, middle_speed, low_speed, low_speed, 0);

  while (isUnderControl(2)){HAL_Delay(10);}
  control_av_wheel_start(0.0, 0.0, 0, 0);

  //pass
  HAL_Delay(200);
  solenoid_toggle();

  HAL_Delay(1000);
  solenoid_toggle();


//HAL_Delay(1000);
//odometry_set_position(field_dir*5.435, -8.3, PI/2.0);
  //pick shagai
  control_av_throwingArm_end();
  control_angle_throwingArm_start(0.0, 0);
  while (enc_arm_getAngle_rad() > 0.15){HAL_Delay(10);}
  control_angle_throwingArm_end();

  control_follow_cubicCurve_invert();

  coes_x = cubicCurve_bezier(field_dir*5.435, field_dir*5.435, field_dir*5.435, field_dir*5.435);
  coes_y = cubicCurve_bezier(-8.3, -8.5, -8.7, -10+L_GP_THROWING_ARM);
  //while (isUnderControl(2)){HAL_Delay(10);}
  control_follow_cubicCurve_start(coes_x, coes_y, low_speed, safe_speed, safe_speed, safe_speed, low_speed, 0);

  while (isUnderControl(2)){HAL_Delay(10);}

  control_av_wheel_start(0.0, 0.0, 0.0, 1);
  control_angle_throwingArm_start(PI/15.0, 1);

  HAL_Delay(1800);

  control_follow_cubicCurve_invert();

  odometry_set_position(field_dir*5.435, -10+L_GP_THROWING_ARM, PI/2.0);

  if (field_dir == 1){
	  control_av_wheel_start(2.0*PI, -2.0*PI, 0.0, 1);
      while(odometry_get_angle() < PI-0.1){HAL_Delay(10);}
  }else{
  	  control_av_wheel_start(-2.0*PI, 2.0*PI, 0.0, 1);
      while(odometry_get_angle() > 0.1){HAL_Delay(10);}
  }
  control_av_wheel_start(0.0, 0.0, 0.0, 1);

  coes_x = cubicCurve_bezier(field_dir*5.435, field_dir*4.9, field_dir*4.4, field_dir*3.91);
  coes_y = cubicCurve_bezier(odometry_get_y(), odometry_get_y(), -10+L_GP_THROWING_ARM, -10+L_GP_THROWING_ARM);
  control_follow_cubicCurve_start(coes_x, coes_y, low_speed, safe_speed, safe_speed, safe_speed, low_speed, 1);
  while (isUnderControl(2)){HAL_Delay(10);}

  HAL_Delay(1000);

  if (field_dir == 1){
	  control_av_wheel_start(-2.0*PI, 2.0*PI, 0.0, 1);
      while(odometry_get_angle() > PI/2.0+0.1 || odometry_get_angle() < 0.0){HAL_Delay(10);}
  }else{
  	  control_av_wheel_start(2.0*PI, -2.0*PI, 0.0, 1);
      while(odometry_get_angle() < PI/2.0-0.1){HAL_Delay(10);}
  }
  control_av_wheel_start(0.0, 0.0, 0.0, 1);

  //wait
  led2_write(1);
  while(remote_sw_getState() == 0){HAL_Delay(10);}
  led2_write(0);

  //move to throwing position
  coes_x = cubicCurve_bezier(odometry_get_x(), odometry_get_x(), field_dir*3.5, field_dir*3.5);
  coes_y = cubicCurve_bezier(-10+L_GP_THROWING_ARM, -7.3, -6.6, -5.96);
  control_follow_cubicCurve_start(coes_x, coes_y, low_speed, high_speed, high_speed, high_speed, middle_speed, 1);
  while (isUnderControl(2)){HAL_Delay(10);}

  coes_x = cubicCurve_bezier(field_dir*3.5, field_dir*3.5, field_dir*3.5, field_dir*4.0);
  coes_y = cubicCurve_bezier(-5.96, -5.5, -5.0, -4.5);
  control_follow_cubicCurve_start(coes_x, coes_y, middle_speed, middle_speed, middle_speed, middle_speed, low_speed, 1);
  while (isUnderControl(2)){HAL_Delay(10);}

  //throw
  control_angle_throwingArm_end();
  control_av_throwingArm_start(PI*1.5, 1);
  control_av_wheel_start(0.0, 0.0, 0.0, 1);

  HAL_Delay(1500);

  //2nd and 3rd shagai
  for (int i = 0; i < 2; i++){
	  control_follow_cubicCurve_invert();

	  coes_x = cubicCurve_bezier(field_dir*4.0, field_dir*3.5, field_dir*3.5, field_dir*3.5);
	  coes_y = cubicCurve_bezier(-4.5, -5.0, -5.5, -5.96);
	  control_follow_cubicCurve_start(coes_x, coes_y, low_speed, middle_speed, high_speed, high_speed, middle_speed, 1);
	  {
		  HAL_Delay(1000);
		  control_av_throwingArm_end();
		  control_angle_throwingArm_start(0.0, 0);
		  while (enc_arm_getAngle_rad() > 0.15){HAL_Delay(10);}
		  control_angle_throwingArm_end();
	  }
	  while (isUnderControl(2)){HAL_Delay(10);}

	  coes_x = cubicCurve_bezier(field_dir*3.5, field_dir*3.5, field_dir*(3.5-i*0.45-0.1), field_dir*(3.5-i*0.45-0.1));
	  coes_y = cubicCurve_bezier(-5.96, -6.6, -7.3, -10+L_GP_THROWING_ARM);
	  control_follow_cubicCurve_start(coes_x, coes_y, middle_speed, high_speed, middle_speed, middle_speed, low_speed, 1);
	  while (isUnderControl(2)){HAL_Delay(10);}

	  control_av_wheel_start(0.0, 0.0, 0.0, 1);
	  control_angle_throwingArm_start(PI/15.0, 1);

	  HAL_Delay(1800);

	  control_follow_cubicCurve_invert();

	  coes_x = cubicCurve_bezier(field_dir*(3.5-i*0.45-0.1), field_dir*(3.5-i*0.45-0.1), field_dir*3.5, field_dir*3.5);
	  coes_y = cubicCurve_bezier(-10+L_GP_THROWING_ARM, -7.3, -6.6, -5.96);
	  control_follow_cubicCurve_start(coes_x, coes_y, low_speed, high_speed, high_speed, high_speed, middle_speed, 1);
	  while (isUnderControl(2)){HAL_Delay(10);}

	  coes_x = cubicCurve_bezier(field_dir*3.51, field_dir*3.51, field_dir*3.51, field_dir*4.01);
	  coes_y = cubicCurve_bezier(-5.96, -5.5, -5.0, -4.5);
      control_follow_cubicCurve_start(coes_x, coes_y, middle_speed, middle_speed, middle_speed, middle_speed, low_speed, 1);
	  while (isUnderControl(2)){HAL_Delay(10);}

	  control_angle_throwingArm_end();
	  control_av_throwingArm_start(PI*1.5, 1);
	  control_av_wheel_start(0.0, 0.0, 0.0, 1);

	  HAL_Delay(1500);
  }

  //end
  while (isUnderControl(2)){HAL_Delay(10);}
  control_av_wheel_start(0.0, 0.0, 0.0, 1);
  led2_write(1);
  while(1);

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLRCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 168;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM7 init function */
static void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 10000-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 84;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM12 init function */
static void MX_TIM12_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 0;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 0;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim12);

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SOLENOID_OUT_Pin|LED1_Pin|LED_THREE_2_Pin|LED_THREE_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : B1_Pin PR_EVENT_Pin */
  GPIO_InitStruct.Pin = B1_Pin|PR_EVENT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : REMOTE_SWITCH_Pin */
  GPIO_InitStruct.Pin = REMOTE_SWITCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(REMOTE_SWITCH_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SOLENOID_OUT_Pin LED1_Pin LED_THREE_2_Pin LED_THREE_1_Pin */
  GPIO_InitStruct.Pin = SOLENOID_OUT_Pin|LED1_Pin|LED_THREE_2_Pin|LED_THREE_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SWITCH2_Pin SWITCH_Pin SWITCH3_Pin */
  GPIO_InitStruct.Pin = SWITCH2_Pin|SWITCH_Pin|SWITCH3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
