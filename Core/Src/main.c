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
#include "adc.h"
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

#define send_buffer_length 64
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t command_tx[5];
uint8_t send_buffer[send_buffer_length];
typedef struct{
	uint8_t command_buffer[32];
	uint8_t command_index;
	uint8_t cursor_index;
}command_component;

command_component data[3];
uint8_t data_num=0;
uint16_t led1_toggle_time=100;
uint16_t led2_toggle_time=100;
bool led1_toggle=false;
bool led2_toggle=false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
extern HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
extern HAL_StatusTypeDef UART_Start_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
bool is_same_word(uint8_t *buf,uint8_t *compare,uint8_t start_compare);
keytype is_command(uint8_t *buf,uint8_t i);
void cli_start(uint8_t i,keytype key_input);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==huart1.Instance){
		HAL_UART_Receive_DMA(&huart1,&command_tx[0],5);
	}
	UNUSED(huart);
}

void uart_send(uint8_t uart_num, char *fmt,...){
	va_list arg;
	va_start(arg,fmt);
	vsnprintf((char *)send_buffer,send_buffer_length,fmt,arg);
	if(uart_num==1){
		HAL_UART_Transmit(&huart1,send_buffer,send_buffer_length,100);
	}
	else if(uart_num==2){
		HAL_UART_Transmit(&huart2,send_buffer,send_buffer_length,100);
	}
	memset(send_buffer,0,send_buffer_length);
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
/*
 * compare uint8_t buf and compare if they are completely same character return
 * true. else they are not same character return false
*/
bool is_same_word(uint8_t *buf,uint8_t *compare,uint8_t start_compare){
    bool ret=false;
    uint8_t end_compare=strlen((const char *)compare);
    for (uint8_t i=start_compare;i<start_compare+end_compare;i++){
        if(compare[i-start_compare]==0){
            ret=false;
            break;
        }
        if (buf[i]==compare[i-start_compare]){
            ret=true;
        }
        else if(buf[i]!=compare[i-start_compare]){
            ret=false;
            break;
        }
    }
    return ret;
}

void command_input(uint8_t *buf){
	bool existing_command = false;
	uint8_t word_len=0;
	if(is_same_word(buf,(uint8_t *)"help",0)){
		uart_send(1,"\n---------------------------------------------------");
		uart_send(1,"----------------\n\n");
		uart_send(1,"\t\t\tcommand_list\n\n");
		uart_send(1,"1. printf [string] : print string at terminal.\n");
		uart_send(1,"note) ledx : x can be 1, 2 value.\n");
		uart_send(1,"2. ledx [on/off] : on/off led\n");
		uart_send(1,"3. ledx toggle [number] : toggle led with user set number\n");
		uart_send(1,"4. adc read : read adc voltage val, where reference voltage,");
		uart_send(1,"is 3.3V.\n");
		uart_send(1,"5. clear : clear terminal.\n");
		uart_send(1,"\n---------------------------------------------------");
		uart_send(1,"----------------\n\n");
		existing_command = true;
	}
	else if(is_same_word(buf,(uint8_t *)"printf",0)){
		word_len=strlen("printf")+1;
		uart_send(1,"\n%s",buf+word_len);
		existing_command = true;
	}
	else if(is_same_word(buf,(uint8_t *)"clear",0)){
		uart_send(1,"\x1B[2J");
		existing_command = true;
	}
	else if(is_same_word(buf,(uint8_t *)"adc read",0)){
		uint32_t adc_val = HAL_ADC_GetValue(&hadc1);
		adc_val *= 3.3;
		uart_send(1,"\nadc voltage:%dV",(uint16_t)adc_val/4096);
		existing_command = true;
	}
	else if(is_same_word(buf,(uint8_t *)"led",0)){
		word_len=strlen("led");
		if(is_same_word(buf,(uint8_t *)"1",word_len)){
			word_len+=2;
			if(is_same_word(buf,(uint8_t *)"on",word_len)){
				led1_toggle=false;
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
				existing_command = true;
			}
			else if(is_same_word(buf,(uint8_t *)"off",word_len)){
				led1_toggle=false;
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
				existing_command = true;
			}
			else if(is_same_word(buf,(uint8_t *)"toggle",word_len)){
				word_len=strlen("led1 toggle ");
				uint8_t toggle_len=strlen((char *)buf+word_len);
				bool is_number = false;
				for(uint8_t i=0;i<toggle_len;i++){
					if(buf[word_len+i]>=(int)'0' && buf[word_len+1]<=(int)'9'){
						is_number=true;
					}
					else{
						is_number=false;
						break;
					}
				}
				if(!is_number){
					uart_send(1,(char *)"\nerror : toggle time value must be number!");
					return;
				}
				led1_toggle_time = 0;
				led1_toggle=true;
				for(uint8_t i=0;i<toggle_len;i++){
					led1_toggle_time = (buf[word_len+i]-48)+led1_toggle_time*10;
				}
				existing_command = true;
			}
		}
		else if(is_same_word(buf,(uint8_t *)"2",word_len)){
			word_len+=2;
			if(is_same_word(buf,(uint8_t *)"on",word_len)){
				led2_toggle=false;
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);
				existing_command = true;
			}
			else if(is_same_word(buf,(uint8_t *)"off",word_len)){
				led2_toggle=false;
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);
				existing_command = true;
			}
			else if(is_same_word(buf,(uint8_t *)"toggle",word_len)){
				word_len=strlen("led2 toggle ");
				uint8_t toggle_len=strlen((char *)buf+word_len);
				bool is_number = false;
				for(uint8_t i=0;i<toggle_len;i++){
					if(buf[word_len+i]>=(int)'0' && buf[word_len+1]<=(int)'9'){
						is_number=true;
					}
					else{
						is_number=false;
						break;
					}
				}
				if(!is_number){
					uart_send(1,(char *)"\nerror : toggle time value must be number!");
					return;
				}
				led2_toggle_time = 0;
				led2_toggle=true;
				for(uint8_t i=0;i<toggle_len;i++){
					led2_toggle_time = (buf[word_len+i]-48)+led2_toggle_time*10;
				}
				existing_command = true;
			}
		}
	}
	if (!existing_command){
		uart_send(1,"\ncommand is not existing. type \'help\' to get command list.");
	}
}

void gpio_led_toggle(uint32_t *led1_time,uint32_t *led2_time){
	if (led1_toggle){
		if (HAL_GetTick()-*led1_time>led1_toggle_time){
			*led1_time=HAL_GetTick();
			HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
		}
	}
	if (led2_toggle){
		if (HAL_GetTick()-*led2_time>led2_toggle_time){
			*led2_time=HAL_GetTick();
			HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_12);
		}
	}
}

void cli_start(uint8_t i,keytype key_input){
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
			command_input(data[data_num].command_buffer);
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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  UART_Start_Receive_DMA(&huart1,&command_tx[0],5);
  HAL_ADC_Start(&hadc1);
  uint32_t pre_time1=HAL_GetTick();
  uint32_t pre_time2=HAL_GetTick();
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
		gpio_led_toggle(&pre_time1,&pre_time2);
		key=is_command(&command_tx[0],i);
		if (key!=KEY_NONE){
			break;
		}
		i++;
	  }
	cli_start(i,key);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
