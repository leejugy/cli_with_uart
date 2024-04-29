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
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef enum {
	KEY_NONE=0,
	UP,
	DOWN,
	LEFT,
	RIGHT,
	END,
	HOME,
	DEL,
	BACKSPACE,
	ENTER,
	IS_WORD,
	KEY_ERROR
}keytype;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t command_tx[5];
uint8_t send_buffer[64];
typedef struct{
	uint8_t command_buffer[32];
	uint8_t command_index;
	uint8_t cursor_index;
}command_component;

command_component data[3];
uint8_t data_num=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void sum_word(uint8_t *buf,uint8_t len,char *fmt,...);
extern HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
extern HAL_StatusTypeDef UART_Start_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==huart1.Instance){
		HAL_UART_Receive_DMA(&huart1,&command_tx[0],5);
	}
	UNUSED(huart);
}

void sum_word(uint8_t *buf,uint8_t len,char *fmt,...){
	va_list arg;
	va_start(arg,fmt);
	vsnprintf((char *)buf,len,fmt,arg);
	va_end(arg);
}

void uart_send(uint8_t uart_num, char *fmt,...){
	va_list arg;
	va_start(arg,fmt);
	vsnprintf((char *)send_buffer,64,fmt,arg);
	if(uart_num==1){
		HAL_UART_Transmit(&huart1,send_buffer,64,100);
	}
	else if(uart_num==2){
		HAL_UART_Transmit(&huart2,send_buffer,64,100);
	}
	memset(send_buffer,0,64);
	va_end(arg);
}
keytype is_command(uint8_t *buf,uint8_t i){
	keytype type_key = KEY_NONE;
	switch (buf[i]){
		case 0x1b:
			HAL_Delay(1);
			switch(buf[(i+2)%5]){
				case 0x44:
					type_key = LEFT;
					break;
				case 0x43:
					type_key = RIGHT;
					break;
				case 0x42:
					type_key = DOWN;
					break;
				case 0x41:
					type_key = UP;
					break;
				case 0x34:
					type_key = END;
					break;
				case 0x31:
					type_key = HOME;
					break;
				default:
					type_key = KEY_ERROR;
					break;
			}
			break;
		case 0x7f:
			type_key = DEL;
			break;
		case 0x08:
			type_key = BACKSPACE;
			break;
		case 0x0d:
			type_key = ENTER;
			break;
		default:
			break;
	}
	if(buf[i]>=32 && buf[i]<=126){
		type_key=IS_WORD;
	}
	return type_key;
}

void cli_start(uint8_t *buf,uint8_t i,keytype key_input){
	switch (key_input){
		case LEFT:
			if(data[data_num].cursor_index>0){
				data[data_num].cursor_index--;
				uart_send(1,"\x1B[1D");
			}
			break;
		case RIGHT:
			if(data[data_num].cursor_index<32 && data[data_num].cursor_index<data[data_num].command_index){
				data[data_num].cursor_index++;
				uart_send(1,"\x1B[1C");
			}
			break;
		case UP:
			if(data[data_num].cursor_index>0){
				uart_send(1,"\x1B[%dD",data[data_num].cursor_index);
			}
			uart_send(1,"\x1B[0J");
			data_num++;
			data_num%=3;
			uart_send(1,"\x1B[4h%s\x1B[4l",data[data_num].command_buffer);
			data[data_num].cursor_index=data[data_num].command_index;
			break;
		case DOWN:
			if(data[data_num].cursor_index>0){
				uart_send(1,"\x1B[%dD",data[data_num].cursor_index);
			}
			uart_send(1,"\x1B[0J");
			if(data_num>0){
				data_num--;
			}
			else if(data_num==0){
				data_num=2;
			}
			uart_send(1,"\x1B[4h%s\x1B[4l",data[data_num].command_buffer);
			data[data_num].cursor_index=data[data_num].command_index;
			break;
		case END:
			if(data[data_num].command_index>data[data_num].cursor_index){
				uart_send(1,"\x1B[%dC",data[data_num].command_index-data[data_num].cursor_index);
				data[data_num].cursor_index=data[data_num].command_index;
			}
			break;
		case HOME:
			if(data[data_num].cursor_index>0){
				uart_send(1,"\x1B[%dD",data[data_num].cursor_index);
				data[data_num].cursor_index=0;
			}
			break;
		case ENTER:
			/*operating command when enter is pressed*/
			/*end operating*/
			uart_send(1,"\nCLI-# ");
			data_num++;
			data_num%=3;
			memset(data[data_num].command_buffer,0,32);
			data[data_num].command_index=0;
			data[data_num].cursor_index=0;
			break;
		case IS_WORD:
			if(data[data_num].command_index<32){
				data[data_num].command_index++;
				if(data[data_num].command_index-1!=data[data_num].cursor_index){
					for(int8_t j=data[data_num].command_index-1;j>=data[data_num].cursor_index;j--){
						data[data_num].command_buffer[j+1]=data[data_num].command_buffer[j];
					}
				}
				data[data_num].command_buffer[data[data_num].cursor_index++]=command_tx[i];
				uart_send(1,"\x1B[4h%c\x1B[4l",command_tx[i]);
			}
			break;
		case BACKSPACE:
			if(data[data_num].command_index>0 && data[data_num].cursor_index>0){
				data[data_num].command_index--;
				data[data_num].command_buffer[--data[data_num].cursor_index]=0;
				for(int8_t j=data[data_num].cursor_index;j<data[data_num].command_index;j++){
					data[data_num].command_buffer[j]=data[data_num].command_buffer[j+1];
				}
				data[data_num].command_buffer[data[data_num].command_index]=0;
				uart_send(1,"\x1B[1D\x1B[1P");
			}
			break;
		case DEL:
			if(data[data_num].command_index>0 && data[data_num].command_index!=data[data_num].cursor_index){
				data[data_num].command_index--;
				data[data_num].command_buffer[data[data_num].cursor_index]=0;
				for(int8_t j=data[data_num].cursor_index;j<data[data_num].command_index;j++){
					data[data_num].command_buffer[j]=data[data_num].command_buffer[j+1];
				}
				data[data_num].command_buffer[data[data_num].command_index]=0;
				uart_send(1,"\x1B[1P");
			}
			break;
		default:
			break;
	}
	uart_send(2,"\x1B[2J");
	for (int8_t j=0;j<3;j++){
		uart_send(2,"%d. buffer val:%s\n",j,data[j].command_buffer);
	}
	uart_send(2,"\n\n\n");
	uart_send(2,"num %d command buffer val:%s\n",data_num,data[data_num].command_buffer);
	uart_send(2,"command index:%d cursor index:%d\n",data[data_num].command_index,data[data_num].cursor_index);
}
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  UART_Start_Receive_DMA(&huart1,&command_tx[0],5);
  for (int8_t i=0;i<3;i++){
	  memset(data[i].command_buffer,0,32);
  }
  memset(send_buffer,0,32);
  uart_send(1,"CLI-# ");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	uint8_t i=0;
	keytype key=KEY_NONE;
	for (;;){
		i=i%5;
		key=is_command(&command_tx[0],i);
		if (key!=KEY_NONE){
			break;
		}
		i++;
	  }
	cli_start(&command_tx[0],i,key);
	memset(command_tx,0,5);
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
