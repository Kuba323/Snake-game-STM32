/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "stm32l4xx_hal.h"
//#include "stm32l4xx.h"
#include <stdlib.h>
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t width;
uint8_t height;
uint8_t isGameOver;

uint8_t snakeHeadX;
uint8_t snakeHeadY;

uint8_t snakeTailX[36];
uint8_t snakeTailY[36];
uint8_t snakeTailLen;

uint8_t foodX;
uint8_t foodY;

enum direction {STOP=0, LEFT, RIGHT, UP, DOWN};

enum direction dir;

///////////////////HARDWARE///////////////////////////
void soft_SPI_byte(uint8_t data){
	for(uint8_t i=8; i>=1; i--){
		HAL_GPIO_WritePin(sSCK_GPIO_Port, sSCK_Pin, 0);
		if((data&0x80) != 0)
			HAL_GPIO_WritePin(sMOSI_GPIO_Port, sMOSI_Pin, 1);
		else
			HAL_GPIO_WritePin(sMOSI_GPIO_Port, sMOSI_Pin, 0);
		data = data<<1;
		HAL_GPIO_WritePin(sSCK_GPIO_Port, sSCK_Pin, 1);

	}
}

void soft_SPI_word(uint8_t address, uint8_t data){

	HAL_GPIO_WritePin(sCS_GPIO_Port, sCS_Pin, 0);
	soft_SPI_byte(address);
	soft_SPI_byte(data);
	HAL_GPIO_WritePin(sCS_GPIO_Port, sCS_Pin, 1);
}

void matrix_init(void){
	soft_SPI_word(0x09, 0x00);
	soft_SPI_word(0x0a, 0x07);
	soft_SPI_word(0x0b, 0x07);
	soft_SPI_word(0x0c, 0x01);
	soft_SPI_word(0x0f, 0x00);
}

void matrix_row(uint8_t address, uint8_t data){
	if(address >=1 && address <=8) soft_SPI_word(address, data);
}

uint8_t matrix_buffer[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

void matrix_buffer_out(void){

	for(uint8_t i=1; i<=8; i++){
		matrix_row(i, matrix_buffer[i-1]);
	}
}

void set_buff(uint8_t x, uint8_t y){
	uint8_t sx = 7-(x&0b0111);
	uint8_t sy = (y&0b0111);
	matrix_buffer[sy] |= (1<<sx);
}

void clr_buff(uint8_t x, uint8_t y){
	uint8_t sx = 7-(x&0b0111);
	uint8_t sy = (y&0b0111);
	matrix_buffer[sy] &= ~(1<<sx);
}

void set_pixel(uint8_t x, uint8_t y){
	uint8_t sx = 7-(x&0b0111);
	uint8_t sy = (y&0b0111);
	matrix_buffer[sy] |= (1<<sx);
	matrix_buffer_out();
}

void clr_pixel(uint8_t x, uint8_t y){
	uint8_t sx = 7-(x&0b0111);
	uint8_t sy = (y&0b0111);
	matrix_buffer[sy] &= ~(1<<sx);
	matrix_buffer_out();
}

void draw_vertical_line(uint8_t x, uint8_t len){
	if(len<9){
		for(uint8_t i=0; i<len; i++){
			set_pixel(x, i);
		}
	}
}

void draw_horizontal_line(uint8_t y, uint8_t len){
	if(len<9){
		for(uint8_t i=0; i<len; i++){
			set_pixel(i, y);
		}
	}
}
///////////////////////SOFTWARE////////////////////////////////
void Setup(void){
	isGameOver = 0;
	width = 8;
	height = 8;

	dir = STOP;
	snakeHeadX = width/2;
	snakeHeadY = height/2;

	srand(HAL_GetTick());
	foodX = rand() % (width-1) + 1;
	foodY = rand() % (height-1) + 1;



	draw_horizontal_line(0, 8);
	draw_vertical_line(0, 8);
	draw_horizontal_line(7, 8);
	draw_vertical_line(7, 8);


}

void Draw(){

	 for(uint8_t i=1; i<width-1; i++){
		 for(uint8_t j=1; j<height-1; j++){
			 if(i == snakeHeadX && j==snakeHeadY) set_pixel(snakeHeadX, snakeHeadY);
			 else if(i == foodX && j==foodY) set_pixel(foodX, foodY);
			 else{
				 uint8_t isSnakeTailCoord=0;
				 for(uint8_t k=0; k<snakeTailLen; k++){
					 if(snakeTailX[k]==i && snakeTailY[k]==j){
						 set_pixel(i, j);
						 isSnakeTailCoord=1;
					 }
				 }
				 if(!isSnakeTailCoord) clr_pixel(i, j);
			 }
		 }
	 }

}

void Logic(){
	uint8_t prevX = snakeTailX[0];
	uint8_t prevY = snakeTailY[0];
	uint8_t prev2X, prev2Y;

	snakeTailX[0] = snakeHeadX;
	snakeTailY[0] = snakeHeadY;

	for(uint8_t i=1; i<snakeTailLen; i++){

        prev2X = snakeTailX[i];
        prev2Y = snakeTailY[i];
        snakeTailX[i] = prevX;
        snakeTailY[i] = prevY;
        prevX = prev2X;
        prevY = prev2Y;
	}

    switch (dir)
    {
    case LEFT:
        snakeHeadX--;
        break;
    case RIGHT:
        snakeHeadX++;
        break;
    case UP:
        snakeHeadY--;
        break;
    case DOWN:
        snakeHeadY++;
        break;
    }
    if (snakeHeadX > width - 2 || snakeHeadX<1 || snakeHeadY>height - 2 || snakeHeadY < 1) isGameOver = 1;

    for (int i = 0; i < snakeTailLen; i++)
    {
        if (snakeTailX[i] == snakeHeadX && snakeTailY[i] == snakeHeadY) isGameOver = 1;
    }
    if (snakeHeadX == foodX && snakeHeadY == foodY)
    {
        snakeTailLen++;

        srand(HAL_GetTick());
        foodX = rand() % (width-1) + 1;
        foodY = rand() % (height-1) + 1;

    }
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  matrix_init();

  Setup();
  uint32_t lastMoveTime = HAL_GetTick();
  uint8_t drawFlag=0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(!isGameOver){
		  	  uint32_t currentTime = HAL_GetTick();
		  	  if(currentTime - lastMoveTime >= 500){


			  Draw();
			  Logic();
			  lastMoveTime = currentTime;
		  }
	  }

	  if(isGameOver && !drawFlag){
		  for(uint8_t i=0; i<8; i++){
			  for(uint8_t j=0; j<8; j++){
				  clr_pixel(i, j);
			  }
		  }
		  drawFlag = 1;
		  if(drawFlag){
			  uint8_t new_matrix_buffer[] = {0x7e, 0xc1, 0xa5, 0xa1, 0xa1, 0xa5, 0xc1, 0x7e};
			  for(uint8_t i=0; i<8; i++){
				  matrix_buffer[i] = new_matrix_buffer[i];
			  }
			  matrix_buffer_out();
		  }
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_3){
		dir = UP;
	}
	else if(GPIO_Pin == GPIO_PIN_7){
		dir = DOWN;
	}
	else if(GPIO_Pin == GPIO_PIN_8){
		dir = LEFT;
	}
	else if(GPIO_Pin == GPIO_PIN_9){
		dir = RIGHT;
	}
}


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
