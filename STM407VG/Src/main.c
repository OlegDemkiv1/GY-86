/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//Registers MPU6050
uint16_t addres_devise_MPU6050=0x68;
uint16_t WHO_AM_I_MPU6050=0x68;
uint16_t WHO_AM_I_ADDRES_MPU6050=0x75;  
int16_t ACCEL_X=0;
int16_t ACCEL_Y=0;
int16_t ACCEL_Z=0;
int16_t GIRO_X=0;
int16_t GIRO_Y=0;
int16_t GIRO_Z=0;
//
// variables for HMC5883L
uint16_t addres_devise_HMC5883L=0x1E;
uint16_t WHO_AM_I_HMC5883L_A=0x10;
uint16_t WHO_AM_I_HMC5883L_B=0x11;
uint16_t WHO_AM_I_HMC5883L_C=0x12;
uint16_t identification_register_A=0x48;
uint16_t identification_register_B=0x34;
uint16_t identification_register_C=0x33;
uint8_t Continuous_Measurement_Moden=0x01;
uint8_t Mode_Register=0x02;
uint16_t Data_Output_X_MSB_Register=0x03;
int16_t MAG_X=0;
int16_t MAG_Y=0;
int16_t MAG_Z=0;
	  
//

void I2C_scaner(void);
void init_MPU6050(void);
void read_data_from_HMC5883L(void);
void read_data_from_MPU6050(void);


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

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
  MX_USART2_UART_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
  I2C_scaner();
	init_MPU6050();
	
	HAL_TIM_Base_Start(&htim2);    		 //Start Timer1
	HAL_TIM_Base_Start_IT(&htim2);
	
	//init_HMC5883L();
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
			HAL_Delay(50);

			char str3[100]={0};
		  uint8_t size=0;
			HAL_TIM_Base_Stop_IT(&htim2);    // Stop interrupt
			sprintf(str3,"MAG| X: %d, Y: %d, Z: %d|\r\nACCEL| X: %d, Y: %d, Z: %d|\r\nGIRO| X: %d, Y: %d, Z: %d|\r\n",MAG_X, MAG_Y, MAG_Z,ACCEL_X, ACCEL_X, ACCEL_X,GIRO_X,GIRO_Y,GIRO_Z);      // convert   in  str 
			size=sizeof(str3);
			HAL_UART_Transmit(&huart2 , (uint8_t *)str3, size, 0xFFFF);
			HAL_TIM_Base_Start_IT(&htim2);    // Start interrupt
		
		
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 200;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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
     PC3   ------> I2S2_SD
     PA4   ------> I2S3_WS
     PA5   ------> SPI1_SCK
     PA6   ------> SPI1_MISO
     PA7   ------> SPI1_MOSI
     PB10   ------> I2S2_CK
     PC7   ------> I2S3_MCK
     PA9   ------> USB_OTG_FS_VBUS
     PA10   ------> USB_OTG_FS_ID
     PA11   ------> USB_OTG_FS_DM
     PA12   ------> USB_OTG_FS_DP
     PC10   ------> I2S3_CK
     PC12   ------> I2S3_SD
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5 
                           PE6 PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin 
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

///////////////////////////////////////////////////////////////////////////////////////////////
void I2C_scaner(void)
{
	/*Description function
	This function search devise connected to I2C in this case -hi2c1.
	After thet function print in console information about what to connect to I2C. 
	*/
	#define DEVICE_FOUND  0
	uint8_t addres_devise=0x00;;      //ADRESS_MPU6050=0x68; -> return 0  ,   DRESS_MS5611=0x77;-> return 0 		 
	uint8_t addr=0;
	uint16_t sizebuf=1;								// size how many data we receive from devise			
	uint8_t buff=0;										// data for receive
	uint32_t timeout=1000;						// timeout for receive
  uint16_t STATUS=0;								// Status connect to device (if STATUS==0 - device if found, if STATUS==1 - device if not found)
	uint8_t number_of_device=0;				// How many device controller is found
	
	uint8_t size=0;
	char str3[35]={0};
	uint8_t size_mas=sizeof(str3);
	uint8_t i=0;
	
	sprintf(str3,"SEARCH DEVISES... \r\n");      										// convert   in  str 
	size=sizeof(str3);
	HAL_UART_Transmit(&huart2 , (uint8_t *)str3, size, 0xFFFF);
	HAL_Delay(500);
	for(addres_devise=0x00;addres_devise<0x7F;addres_devise++)
	{
			HAL_Delay(10);
			STATUS=HAL_I2C_Mem_Read(&hi2c1, (uint16_t)addres_devise<<1,(uint16_t)addr, (uint16_t) sizebuf, &buff, (uint16_t) sizebuf,(uint32_t) timeout);
			if(STATUS==DEVICE_FOUND)																		// if devsice is found
			{	
				  for(i=0;i<size_mas;i++)    															// Delete data in str3[]
					{
						str3[i]=0;
					}
					sprintf(str3,"Device address-0x%x - found \r\n",addres_devise);      // convert   in  str 
					size=sizeof(str3);
					HAL_UART_Transmit(&huart2 , (uint8_t *)str3, size, 0xFFFF);
					number_of_device++;
			}
	}
	if(number_of_device==0)  																				// If devices nofound
	{	
			for(i=0;i<size_mas;i++)				 															// Delete data in str3[]
			{
				str3[i]=0;
			}
			sprintf(str3,"Devices no found!!!\r\n");      							// convert   in  str 
			size=sizeof(str3);
			HAL_UART_Transmit(&huart2 , (uint8_t *)str3, size, 0xFFFF);
	}
	HAL_Delay(500);
	for(i=0;i<size_mas;i++)																					// Delete data in str3[]
	{
		str3[i]=0;
	}
	sprintf(str3,"DONE\r\n");      																	// convert   in  str 
	size=sizeof(str3);
	HAL_UART_Transmit(&huart2 , (uint8_t *)str3, size, 0xFFFF);	
}

///////////////////////////////////////////////////////////////////////////////////////////////
void init_MPU6050(void)
{
	uint8_t Device_found=0;
	uint8_t STATUS=1;	
	uint16_t sizebuf=2;
	uint8_t buff=0x00;
	uint32_t timeout=0xFFF;
	// variables for print data in com port
	uint8_t size=0;
	char str3[35]={0};
	//
	
	// Who I am for MPU6050. Gyroscope, acceleromert
	STATUS=HAL_I2C_Mem_Read(&hi2c1, (uint16_t)addres_devise_MPU6050<<1,(uint16_t)WHO_AM_I_ADDRES_MPU6050, (uint16_t) 1, &buff, (uint16_t) sizebuf,(uint32_t) timeout);
	if(((buff!=WHO_AM_I_MPU6050)|(STATUS!=Device_found)))
	{
		  sprintf(str3,"ERROR: MPU6050 not detected!\r\n");      // convert   in  str 
			size=sizeof(str3);
			HAL_UART_Transmit(&huart2 , (uint8_t *)str3, size, 0xFFFF);
	}
	else
	{  
			sprintf(str3,"MPU6050 conected.Status - OK!\r\n");      // convert   in  str 
			size=sizeof(str3);
			HAL_UART_Transmit(&huart2 , (uint8_t *)str3, size, 0xFFFF);
		  // Init MPU6050
		  unsigned char buffer[2];					     	// Bufer  
			uint16_t sizebuf=2;
			uint8_t addr=0x6B;											// It  is  pointer  on  resistor  in  slave      
			uint8_t data=0x00;											// It  data  for  slave														// exit sleep
			HAL_I2C_Mem_Write(&hi2c1, (uint16_t) addres_devise_MPU6050<<1, addr, (uint16_t) 1, &data, (uint16_t) 1, (uint32_t) 1000);
			HAL_Delay(20);
			addr=0x1B;			    										// It  is  pointer  on  resistor  in  slave      
			data=0x00;															// It  data  for  slave														// Giro  full  scale = +/- 4g
			HAL_I2C_Mem_Write(&hi2c1, (uint16_t) addres_devise_MPU6050<<1, addr, (uint16_t) 1, &data, (uint16_t) 1, (uint32_t) 1000);
			HAL_Delay(20);
			addr=0x1C;														  // It  is  pointer  on  resistor  in  slave    // Accelerometr
			data=0x00;		// +/-16 G -0x18					// It  data  for  slave  													// accelerometer  full scale = +/- 2g
			HAL_I2C_Mem_Write(&hi2c1, (uint16_t) addres_devise_MPU6050<<1, addr, (uint16_t) 1, &data, (uint16_t) 1, (uint32_t) 1000);
			HAL_Delay(20);
		
		  // Init MPU6050 for see magnitometr
		  addr=0x6A;														  // 
			data=0x00;						// 
			HAL_I2C_Mem_Write(&hi2c1, (uint16_t) addres_devise_MPU6050<<1, addr, (uint16_t) 1, &data, (uint16_t) 1, (uint32_t) 1000);  
		   
			addr=0x37;														  // 
			data=0x02;				// 
			HAL_I2C_Mem_Write(&hi2c1, (uint16_t) addres_devise_MPU6050<<1, addr, (uint16_t) 1, &data, (uint16_t) 1, (uint32_t) 1000);   
		  
			addr=0x6B;														  // 
			data=0x00;						// 
			HAL_I2C_Mem_Write(&hi2c1, (uint16_t) addres_devise_MPU6050<<1, addr, (uint16_t) 1, &data, (uint16_t) 1, (uint32_t) 1000);  
			// 
	}
}

void read_data_from_MPU6050(void)
{
	    uint8_t buff[6]={0};			// Bufer for data acceleration/giroscope
			uint16_t sizebuf=6;
	    uint32_t timeout=0xFF;
			// All reg
	    uint8_t ACCEL_XOUT_H=0x3B;        // Start read data acceleration
			uint8_t GYRO_XOUT_H=0x43;   			// / Start read data giroscope

	    // Read accelerometr data
			HAL_I2C_Mem_Read(&hi2c1, (uint16_t)addres_devise_MPU6050<<1,(uint16_t)ACCEL_XOUT_H, (uint16_t) 1, buff, (uint16_t) sizebuf,(uint32_t) timeout);
			ACCEL_X=(uint16_t)buff[0]<<8|buff[1];  //Convert accelerometr data
			ACCEL_Y=(uint16_t)buff[2]<<8|buff[3];
			ACCEL_Z=(uint16_t)buff[4]<<8|buff[5];
			// Read giroscope data
			HAL_I2C_Mem_Read(&hi2c1, (uint16_t)addres_devise_MPU6050<<1,(uint16_t)GYRO_XOUT_H, (uint16_t) 1, buff, (uint16_t) sizebuf,(uint32_t) timeout);
			GIRO_X=(uint16_t)buff[0]<<8|buff[1];   //Convert giroscope data
			GIRO_Y=(uint16_t)buff[2]<<8|buff[3];
			GIRO_Z=(uint16_t)buff[4]<<8|buff[5];
}

void read_data_from_HMC5883L(void)
{
			uint32_t timeout=0xFF;
			uint8_t STATUS=1;
	    // Init magnitometr for enabble Continuous-Measurement Mode
			STATUS=HAL_I2C_Mem_Write(&hi2c1, (uint16_t)addres_devise_HMC5883L<<1, (uint16_t) Mode_Register, (uint16_t) 1, &Continuous_Measurement_Moden, (uint16_t) 1, (uint32_t) 0xFF);
			if(STATUS==1)			// Dewise not respond
			{
						//ERROR
			}
			else							// Dewise respond OK
			{
						uint16_t sizebuf=6;
						uint8_t buff[6]={0};
						// Read data from magnitometr
						HAL_I2C_Mem_Read(&hi2c1, (uint16_t)addres_devise_HMC5883L<<1,(uint16_t)Data_Output_X_MSB_Register, (uint16_t) 1, buff, (uint16_t) sizebuf,(uint32_t) timeout);
						MAG_X=(int16_t)buff[0]<<8|buff[1];
						MAG_Y=(int16_t)buff[2]<<8|buff[3];
						MAG_Z=(int16_t)buff[4]<<8|buff[5];
			}
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
