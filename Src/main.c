/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
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
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "delay.h"
#include "key.h"
#include "usart.h"
#include "adc.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
//ADC_HandleTypeDef hadc1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;



/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
// static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
	uint8_t Second;	                                              //秒针
	uint8_t duanMa[]={0,1,2,3,4,5,6,7,
					             8,9,10};                              //显示0~9段码
	uint8_t weiMa[]={0,1,2,3,4,5,6,7};    //位码
	uint8_t DisplayData[8];
											 
void DigDisplay(){
		uint8_t i=0;
	for (i=0;i<8;i++){
		
		switch(weiMa[i]){
			
			case(0):
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET);break;
			case(1):
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);break;
			case(2):
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_RESET);break;
			case(3):
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_RESET);break;
			case(4):
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);break;
			case(5):
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);break;
			case(6):
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);break;
			case(7):
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);break;
				
			
		}
		//发送位码
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);   //位锁存
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);
		
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);
		
		switch(duanMa[DisplayData[i]]){
		  case(0):
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);break;
			case(1):
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);break;
			case(2):
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);break;
			case(3):
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);break;
			case(4):
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);break;
			case(5):
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET);break;
			case(6):
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);break;
			case(7):
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_SET);break;
			case(8):
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);break;
			case(9):
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET);break;
			case(10):
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);break;
		
		}
		//发送段码
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);   //段锁存
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET); 
		
		
				delay_us(500);//间隔一段时间扫描	
		/*
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);//消隐*/
	}
	
}				


/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  Second=1; 
	int flag=1;
	int time;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
	MX_USART1_UART_Init();
	delay_init(180);
	

  /* USER CODE BEGIN 2 */
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
	/*	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_SET);   //南北红
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_SET);   //南北黄
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,GPIO_PIN_SET);   //南北绿
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,GPIO_PIN_SET);   //东西红
		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_12,GPIO_PIN_SET);  //东西黄
		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_11,GPIO_PIN_SET);  //东西绿
		*/
		
		/*紧急通过*/
	if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_7)==0){
			flag=1;
			time=Second;
			Second=1;
			while(Second<=5&&flag==1){
					DisplayData[0] = 10;
					DisplayData[1] = 10;
					DisplayData[2] = duanMa[(5 - Second) % 100 / 10];
					DisplayData[3] = duanMa[(5 - Second) %10];
					DisplayData[4] = DisplayData[2];;
					DisplayData[5] = DisplayData[3];
					DisplayData[6] = 10;
					DisplayData[7] = 10;
					DigDisplay();
		
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_SET);   //南北黄
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,GPIO_PIN_SET);   //南北绿
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,GPIO_PIN_SET);   //东西红
					HAL_GPIO_WritePin(GPIOG,GPIO_PIN_12,GPIO_PIN_SET);  //东西黄
					HAL_GPIO_WritePin(GPIOG,GPIO_PIN_11,GPIO_PIN_SET);
		
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOG,GPIO_PIN_12,GPIO_PIN_RESET);
			
					//flag++;
					//delay_ms(1000);
			}
			/*五秒黄灯*/
			
			while(Second>5&&flag==1){
					DisplayData[0] = 10;
					DisplayData[1] = 10;
					DisplayData[2] = 9;
					DisplayData[3] = 9;
					DisplayData[4] = 9;
					DisplayData[5] = 9;
					DisplayData[6] = 10;
					DisplayData[7] = 10;
					DigDisplay();
		
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_SET);   //南北黄
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,GPIO_PIN_SET);   //南北绿
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,GPIO_PIN_SET);   //东西红
					HAL_GPIO_WritePin(GPIOG,GPIO_PIN_12,GPIO_PIN_SET);  //东西黄
					HAL_GPIO_WritePin(GPIOG,GPIO_PIN_11,GPIO_PIN_SET);
		
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,GPIO_PIN_RESET); 
			
					//flag++;
					//delay_ms(1000);
					
					if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_10)==0){
							flag=0;
							Second=time;
					
					}
			
			}
			
	}	
		/*红灯*/
		
		/*人行道*/
	if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_9)==0){
			Second=1;
			while (Second<=5){
			DisplayData[0] = 10;
			DisplayData[1] = 10;
			DisplayData[2] = duanMa[(5 - Second) % 100 / 10];
			DisplayData[3] = duanMa[(5 - Second) %10];
			DisplayData[4] = DisplayData[2];
			DisplayData[5] = DisplayData[3];
			DisplayData[6] = 10;
			DisplayData[7] = 10;
			DigDisplay();
		
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_SET);   //南北黄
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,GPIO_PIN_SET);   //南北绿
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,GPIO_PIN_SET);   //东西红
		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_12,GPIO_PIN_SET);  //东西黄
		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_11,GPIO_PIN_SET);
		
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOG,GPIO_PIN_12,GPIO_PIN_RESET);
			
			//delay_ms(1000);
			}
			Second=1;
	}	
	
	
	
	if(Second==55){
			Second=1;
	}
	if(Second<31){
		/*流量达到阈值*/
			if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_5)==0){
				printf("1");
					if(30-Second<=5){
						printf("2");
							Second=Second-20;
					}	
			}
		
			DisplayData[0] = 10;
			DisplayData[1] = 10;
			DisplayData[2] = duanMa[(30 - Second) % 100 / 10];
			DisplayData[3] = duanMa[(30 - Second) %10];
			DisplayData[4] = duanMa[(35 - Second) % 100 / 10];
			DisplayData[5] = duanMa[(35 - Second) %10];
			DisplayData[6] = 10;
			DisplayData[7] = 10;
			DigDisplay();
		
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_SET);   //南北黄
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,GPIO_PIN_SET);   //南北绿
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,GPIO_PIN_SET);   //东西红
		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_12,GPIO_PIN_SET);  //东西黄
		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_11,GPIO_PIN_SET);
		
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,GPIO_PIN_RESET);
		
	}
	else if(Second<36){
			DisplayData[0] = 10;
			DisplayData[1] = 10;
			DisplayData[2] = duanMa[(35 - Second) % 100 / 10];
			DisplayData[3] = duanMa[(35 - Second) %10];
			DisplayData[4] = DisplayData[2];
			DisplayData[5] = DisplayData[3];
			DisplayData[6] = 10;
			DisplayData[7] = 10;
			DigDisplay();
		
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_SET);   //南北黄
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,GPIO_PIN_SET);   //南北绿
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,GPIO_PIN_SET);   //东西红
		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_12,GPIO_PIN_SET);  //东西黄
		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_11,GPIO_PIN_SET);
		
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,GPIO_PIN_RESET);
		
		
	}
	else if(Second<51){
		
		/*流量达到阈值*/
			if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_5)==0){
					if(50-Second<=5){
						int cache=71-Second;
						flag=1;
						Second=1;
						while(Second<=cache&&flag==1){
						
						DisplayData[0] = 10;
						DisplayData[1] = 10;
						DisplayData[2] = duanMa[(cache-Second+5) % 100 / 10];
						DisplayData[3] = duanMa[(cache - Second+5) %10];
						DisplayData[4] = duanMa[(cache - Second) % 100 / 10];
						DisplayData[5] = duanMa[(cache - Second) %10];
						DisplayData[6] = 10;
						DisplayData[7] = 10;
						DigDisplay();
		
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_SET);   //南北黄
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,GPIO_PIN_SET);   //南北绿
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,GPIO_PIN_SET);   //东西红
		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_12,GPIO_PIN_SET);  //东西黄
		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_11,GPIO_PIN_SET);
		
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOG,GPIO_PIN_11,GPIO_PIN_RESET);
						
						}
						/*
						int cache=70-Second;
						for (int i=0;i<=cache;i++){
						
						DisplayData[0] = 10;
						DisplayData[1] = 10;
						DisplayData[2] = duanMa[(cache-i) % 100 / 10];
						DisplayData[3] = duanMa[(cache - i) %10];
						DisplayData[4] = duanMa[(cache - i) % 100 / 10];
						DisplayData[5] = duanMa[(cache - i) %10];
						DisplayData[6] = 10;
						DisplayData[7] = 10;
						DigDisplay();
		
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_SET);   //南北黄
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,GPIO_PIN_SET);   //南北绿
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,GPIO_PIN_SET);   //东西红
		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_12,GPIO_PIN_SET);  //东西黄
		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_11,GPIO_PIN_SET);
		
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOG,GPIO_PIN_11,GPIO_PIN_RESET);
						
						delay_ms(1000);
						}	
						
					//Second=Second-10;
					}*/
					
					Second=50;
			}
		}
		
		
		
			DisplayData[0] = 10;
			DisplayData[1] = 10;
			DisplayData[2] = duanMa[(55 - Second) % 100 / 10];
			DisplayData[3] = duanMa[(55 - Second) %10];
			DisplayData[4] = duanMa[(50 - Second) % 100 / 10];
			DisplayData[5] = duanMa[(50 - Second) %10];
			DisplayData[6] = 10;
			DisplayData[7] = 10;
			DigDisplay();
		
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_SET);   //南北黄
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,GPIO_PIN_SET);   //南北绿
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,GPIO_PIN_SET);   //东西红
		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_12,GPIO_PIN_SET);  //东西黄
		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_11,GPIO_PIN_SET);
		
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOG,GPIO_PIN_11,GPIO_PIN_RESET);
		
	}
	else{
			DisplayData[0] = 10;
			DisplayData[1] = 10;
			DisplayData[2] = weiMa[(55 - Second) % 100 / 10];
			DisplayData[3] = weiMa[(55 - Second) %10];
			DisplayData[4] = DisplayData[2];
			DisplayData[5] = DisplayData[3];
			DisplayData[6] = 10;
			DisplayData[7] = 10;
			DigDisplay();
		
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_SET);   //南北黄
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,GPIO_PIN_SET);   //南北绿
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,GPIO_PIN_SET);   //东西红
		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_12,GPIO_PIN_SET);  //东西黄
		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_11,GPIO_PIN_SET);
		
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOG,GPIO_PIN_12,GPIO_PIN_RESET);
	}
	
  }
  /* USER CODE END 3 */

}


	

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  HAL_PWREx_EnableOverDrive();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}



/* IWDG init function 
void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 500;
  HAL_IWDG_Init(&hiwdg);

}
*/

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 9999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 8999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);
	HAL_TIM_Base_Start_IT(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

}

/*定时器3中断服务函数
void TIM3_IRQHandler(void){
	HAL_TIM_IRQHandler(&htim3);
	
}*/

/*定时器3中断服务函数调用*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim==(&htim3)){
		Second++;
	}
}


/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();

  /*8位数码管端口*/
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode =GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA , &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_10|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	/*位锁存*/
	GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	/*段锁存*/
	GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	/*信号灯端口*/
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode =GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOD , &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode =GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD , &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode =GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD , &GPIO_InitStruct);
	/*南北信号灯*/
	
	GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode =GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD , &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode =GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOG , &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode =GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOG , &GPIO_InitStruct);
	/*东西交通灯*/
	
	/*按键*/
	GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode=GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE , &GPIO_InitStruct);
	/*到达阈值*/
	
	GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode =GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF , &GPIO_InitStruct);
	/*人行道*/
	
	GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode =GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF , &GPIO_InitStruct);
	/*紧急通过*/
	
	GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode =GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF , &GPIO_InitStruct);
	/*紧急通过结束*/

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED0_Pin|LED1_Pin, GPIO_PIN_SET);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 1);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 2);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 3);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

	HAL_UART_Receive_IT(&huart1,(uint8_t *)aRxBuffer,RXBUFFERSIZE);
}
/* USER CODE END 4 */

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
