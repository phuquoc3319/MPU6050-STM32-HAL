
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
  * COPYRIGHT(c) 2018 STMicroelectronics
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

#include "KALMAN.h"


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


#define   SMPLRT_DIV      0x19   //0x07(125Hz)
#define   CONFIG          0x1A   //0x06(5Hz)
#define   GYRO_CONFIG     0x1B   //0x18(不自检，2000deg/s)
#define   ACCEL_CONFIG    0x1C   //0x01(不自检，2G，5Hz)
#define   ACCEL_XOUT_H    0x3B
#define   ACCEL_XOUT_L    0x3C
#define   ACCEL_YOUT_H    0x3D
#define   ACCEL_YOUT_L    0x3E
#define   ACCEL_ZOUT_H    0x3F
#define   ACCEL_ZOUT_L    0x40
#define   TEMP_OUT_H      0x41
#define   TEMP_OUT_L      0x42
#define   GYRO_XOUT_H     0x43
#define   GYRO_XOUT_L     0x44   
#define   GYRO_YOUT_H     0x45
#define   GYRO_YOUT_L     0x46
#define   GYRO_ZOUT_H     0x47
#define   GYRO_ZOUT_L     0x48
#define   USER_CTRL       0x6A
#define   PWR_MGMT_1      0x6B
#define   WHO_AM_I        0x75

#define mpu6050 0xD0  //Dia chi cua cam bien mpu6050 co 7 bit doi sang trai 1 bit de set read or write


uint8_t data[6]={0};
uint8_t data1[6]={0};
uint8_t tmp1;
uint8_t tmp2;
int16_t x=0;
int16_t y=0;
int16_t z=0;

int16_t x1=0;
int16_t y1=0;
int16_t z1=0;

float x_kalman;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


void mpu6050_init(void)
{
	HAL_I2C_Mem_Write(&hi2c1,mpu6050,PWR_MGMT_1,1,(uint8_t *)0x80,1,100);
	HAL_Delay(5);
	HAL_I2C_Mem_Write(&hi2c1,mpu6050,PWR_MGMT_1,1,(uint8_t *)0x00,1,100);
	HAL_I2C_Mem_Write(&hi2c1,mpu6050,SMPLRT_DIV,1,(uint8_t *)0x07,1,100);
	HAL_I2C_Mem_Write(&hi2c1,mpu6050,CONFIG,1,(uint8_t *)0x00,1,100);
	HAL_I2C_Mem_Write(&hi2c1,mpu6050,GYRO_CONFIG,1,(uint8_t *)0x00,1,100);
	HAL_I2C_Mem_Write(&hi2c1,mpu6050,ACCEL_CONFIG,1,(uint8_t *)0x00,1,100);
	HAL_I2C_Mem_Write(&hi2c1,mpu6050,USER_CTRL,1,(uint8_t *)0x00,1,100);
	HAL_I2C_Mem_Write(&hi2c1,mpu6050,PWR_MGMT_1,1,(uint8_t *)0x01,1,100);
	HAL_Delay(10);
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	
	
	mpu6050_init();
	
	kalman Kalman_Param;
	Kalman_Param.Q_angle = 0.001;
	Kalman_Param.Q_bias = 0.003;
	Kalman_Param.Q_measure = 0.03;
	Kalman_Param.bias = 0;
	Kalman_Param.p[0][0] =0;
	Kalman_Param.p[0][1] = 0;
	Kalman_Param.p[1][0] =0;
	Kalman_Param.p[1][1] =0;
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		HAL_I2C_Mem_Write(&hi2c1,mpu6050,ACCEL_XOUT_H,1,(uint8_t *)(mpu6050|1),1,100);
		HAL_I2C_Mem_Read(&hi2c1,mpu6050,ACCEL_XOUT_H,1,data,6,100);
	
		x = (data[0]<<8)|data[1];
		y = (data[2]<<8)|data[3];
		z = (data[4]<<8)|data[5];
		
		HAL_I2C_Mem_Read(&hi2c1,mpu6050,GYRO_XOUT_H,1,data1,6,100);
		x1 = (data1[0]<<8)|data1[1];
		y1 = (data1[2]<<8)|data1[3];
		z1 = (data1[4]<<8)|data1[5];
		
		x_kalman = getAngle(x,15,100,&Kalman_Param);
		HAL_Delay(1);

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
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
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

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
