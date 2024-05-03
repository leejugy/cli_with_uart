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
#define FW_START_ADD  0x08000000+1024*60
#define F103_RB_START_ADD 0x08000000
#define F103_RB_END_ADD 0x0801FFFF
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
static const uint16_t crc_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

typedef enum{
	SOH=0x01,
	STX=0x02,
	EOT=0x04,
	ACK=0x06,
	NAK=0x15,
	CAN=0x18,
} ymodem_protocol_name;

typedef enum{
	HEX=16,
	DEC=10
} type_nubmer;

uint8_t rx_ymodem[1024+5];
uint8_t rx_1bit;
uint16_t rx_index=0;
uint8_t send_buffer[send_buffer_length];
bool uart1_send_flag = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//@user function begin

//@HAL function
extern HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
extern HAL_StatusTypeDef UART_Start_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);

//@is function
bool is_same_word(uint8_t *buf,uint8_t *compare,uint8_t start_compare);
keytype is_command(uint8_t *buf,uint8_t i);

//@cli function
void cli_start(uint8_t i,keytype key_input);
void command_input(uint8_t *buf);

//@translate function
uint8_t chartoint(uint8_t ascii);
uint32_t string_to_num(char *input_string, bool* is_number, type_nubmer num_type); //ref @type_nubmer

//@ymodem function
void ymodem_transmit();
void ymodem_fw_update();

//@uart function
void uart_send_1bit(uint8_t uart_num,uint8_t *send_1bit_buffer);
void uart_send(uint8_t uart_num, char *fmt,...);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

//@flash function
bool flash_erase(uint32_t start_address,uint8_t data_len);
void flash_read(uint32_t address,uint32_t bit_num);
bool flash_write(uint32_t start_address,uint32_t data_len,uint8_t *data);






uint32_t string_to_num(char *input_string, bool* is_number, type_nubmer num_type){
    uint16_t length = strlen(input_string);
    uint32_t ret = 0;
    if (num_type == DEC){
		for(uint32_t i=0;i<length;i++){
			if((input_string[i] >= (int)'0') && (input_string[i] <= (int) '9'))
			{
				*is_number=true;
				ret = input_string[i]-((int)'0') + ret * num_type;
			}
			else if(input_string[i] == ' '){
				return ret;
			}
			else{
				*is_number=false;
				return 0;
			}
		}
    }
    else if (num_type == HEX){
		for(uint32_t i=0;i<length;i++){
			if((input_string[i] >= (int)'0') && (input_string[i] <= (int) '9'))
			{
				*is_number=true;
				ret = input_string[i]-((int)'0') + ret * num_type;
			}
			else if((input_string[i] >= (int)'A') && (input_string[i] <= (int) 'F')){
				*is_number=true;
				ret = input_string[i]-55 + ret * num_type;
			}
			else if((input_string[i] >= (int)'a') && (input_string[i] <= (int) 'f')){
				*is_number=true;
				ret = input_string[i]-87 + ret * num_type;
			}
			else if(input_string[i] == ' '){
				return ret;
			}
			else{
				*is_number=false;
				return 0;
			}
		}
    }
    return ret;
}

void flash_read(uint32_t address,uint32_t bit_num){
	uint8_t *data=(uint8_t *)address;
	uart_send(1,"\n");
	for(uint32_t i=0;i<bit_num;i++){
		uart_send(1,"data adress : 0x%08X, data : 0x%02X\n",address+i,data[i]);
	}
}

bool flash_erase(uint32_t start_address,uint8_t data_len)
{
  uint32_t NbOfPages = 0;
  uint32_t start_page = (start_address - F103_RB_START_ADD)/FLASH_PAGE_SIZE;
  start_page = start_page * 1024 + F103_RB_START_ADD;
  uint32_t PageError = 0;
  /* Variable contains Flash operation status */
  HAL_StatusTypeDef status;
  FLASH_EraseInitTypeDef eraseinitstruct;

  /* Get the number of sector to erase from 1st sector */
  NbOfPages =
    (data_len / FLASH_PAGE_SIZE) + 1;
  eraseinitstruct.TypeErase = FLASH_TYPEERASE_PAGES;
  eraseinitstruct.PageAddress = start_page;
  eraseinitstruct.NbPages = NbOfPages;
  HAL_FLASH_Unlock();
  status = HAL_FLASHEx_Erase(&eraseinitstruct, &PageError);
  HAL_FLASH_Lock();
  if (status != HAL_OK)
  {
    return false;
  }
  return true;
}

bool flash_write(uint32_t start_address,uint32_t data_len,uint8_t *data){
	bool ret = false;
	flash_erase(start_address,data_len);
	HAL_FLASH_Unlock();
	uint16_t send_data=0;
	for(uint32_t i=0;i<data_len;i+=2){
		send_data+=data[i+1]<<8;
		send_data+=data[i];
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,start_address+i,(uint16_t)send_data)==HAL_OK){
			ret = true;
			send_data=0;
		}
		else{
			ret=false;
		}
	}
	HAL_FLASH_Lock();
	return ret;
}

uint8_t chartoint(uint8_t ascii){
	return ascii-48;
}

void uart_send_1bit(uint8_t uart_num,uint8_t *send_1bit_buffer){
	if(uart_num==1){
		HAL_UART_Transmit(&huart1,send_1bit_buffer,1,100);
	}
	else if(uart_num==2){
		HAL_UART_Transmit(&huart2,send_1bit_buffer,1,100);
	}
}

bool calculate_crc16(uint8_t *data, int length, bool *loop_do) {
    uint16_t crc = 0;
    for (int i = 0; i < length; i++) {
        crc = (crc << 8) ^ crc_table[((crc >> 8) ^ data[i]) & 0xFF];
    }
    if(crc!=0){
    	uart_send(2,"CRC is incorrect!\n");
    	*loop_do=false;
    	return false;
    }
    return true;
}
void ymodem_transmit(){
	HAL_UART_DMAStop(&huart1);
	UART_Start_Receive_DMA(&huart1,&rx_1bit,1);

	HAL_Delay(100);
	bool send_c_flag=true;
	bool first_send_eot=true;
	bool loop_do=true;

	uint8_t _1bit_buffer;
	uint32_t file_size=0;
	uint8_t file_index;
	uint8_t file_name[64];
	memset(file_name,0,64);
	memset(rx_ymodem,0,1029);
	rx_index=0;

	uart_send(1,"\n");
	while(loop_do){
		uart1_send_flag=true;
		if(send_c_flag){
			HAL_Delay(100);
			_1bit_buffer='C';
			uart_send_1bit(1,&_1bit_buffer);
			_1bit_buffer='C';
			uart_send_1bit(1,&_1bit_buffer);
		}
		if(rx_ymodem[0]==SOH){
			HAL_Delay(10);
			for(file_index=3;file_index<128;file_index++){
				file_name[file_index-3]=rx_ymodem[file_index];
				if(rx_ymodem[file_index]==0){
					file_index++;
					break;
				}
			}
			for(;file_index<128;file_index++){
				if(rx_ymodem[file_index]==' '){
					break;
				}
				else{
					file_size=chartoint(rx_ymodem[file_index])+file_size*10;
				}
			}
			calculate_crc16(rx_ymodem+1027, 2, &loop_do);

			memset(rx_ymodem,0,1029);
			rx_index=0;

			_1bit_buffer=ACK;
			uart_send_1bit(1,&_1bit_buffer);
			_1bit_buffer='C';
			uart_send_1bit(1,&_1bit_buffer);
			send_c_flag=false;
		}
		else if(rx_1bit==EOT && first_send_eot){
			_1bit_buffer=NAK;
			uart_send_1bit(1,&_1bit_buffer);
			first_send_eot=false;
		}
		else if(rx_1bit==EOT && !first_send_eot){
			first_send_eot=true;
			_1bit_buffer=ACK;
			uart_send_1bit(1,&_1bit_buffer);
			_1bit_buffer='C';
			uart_send_1bit(1,&_1bit_buffer);

			_1bit_buffer=ACK;
			uart_send_1bit(1,&_1bit_buffer);
			_1bit_buffer='C';
			uart_send_1bit(1,&_1bit_buffer);

			loop_do=false;
			uart1_send_flag=false;
			rx_1bit=0;

			uart_send(2,"\n+++++++++++++++++++++++++++++++++++++++++\n");
			uart_send(2,"\nsend_complete!\n");
			uart_send(2,"INFO)\nfile name : ");
			uart_send(2,"%s",file_name);
			uart_send(2,"\n");
			uart_send(2,"file size : ");
			uart_send(2,"%lu",file_size);
			uart_send(2,"\n");
			HAL_UART_DMAStop(&huart1);
			UART_Start_Receive_DMA(&huart1,&command_tx[0],5);;
		}
		else if(rx_ymodem[0]==STX){
			HAL_Delay(10);
			calculate_crc16(rx_ymodem+1027, 2, &loop_do);
			/*buffer send_occur*/
			for(uint16_t i=3;i<=1026;i++){
				uart_send_1bit(2,&rx_ymodem[i]);
			}
			memset(rx_ymodem,0,1029);
			rx_index=0;
			_1bit_buffer=ACK;
			uart_send_1bit(1,&_1bit_buffer);
			send_c_flag=false;
		  }
	}
}

void ymodem_fw_update(){
	HAL_UART_DMAStop(&huart1);
	UART_Start_Receive_DMA(&huart1,&rx_1bit,1);

	HAL_Delay(100);
	bool send_c_flag=true;
	bool first_send_eot=true;
	bool loop_do=true;

	uint8_t _1bit_buffer;
	uint32_t file_size=0;
	uint8_t file_index;
	uint8_t file_name[64];
	memset(file_name,0,64);
	memset(rx_ymodem,0,1029);
	rx_index=0;

	uart_send(1,"\n");
	while(loop_do){
		uart1_send_flag=true;
		if(send_c_flag){
			HAL_Delay(100);
			_1bit_buffer='C';
			uart_send_1bit(1,&_1bit_buffer);
			_1bit_buffer='C';
			uart_send_1bit(1,&_1bit_buffer);
		}
		if(rx_ymodem[0]==SOH){
			HAL_Delay(10);
			for(file_index=3;file_index<128;file_index++){
				file_name[file_index-3]=rx_ymodem[file_index];
				if(rx_ymodem[file_index]==0){
					file_index++;
					break;
				}
			}
			for(;file_index<128;file_index++){
				if(rx_ymodem[file_index]==' '){
					break;
				}
				else{
					file_size=chartoint(rx_ymodem[file_index])+file_size*10;
				}
			}
			calculate_crc16(rx_ymodem+1027, 2, &loop_do);

			memset(rx_ymodem,0,1029);
			rx_index=0;

			_1bit_buffer=ACK;
			uart_send_1bit(1,&_1bit_buffer);
			_1bit_buffer='C';
			uart_send_1bit(1,&_1bit_buffer);
			send_c_flag=false;
		}
		else if(rx_1bit==EOT && first_send_eot){
			_1bit_buffer=NAK;
			uart_send_1bit(1,&_1bit_buffer);
			first_send_eot=false;
		}
		else if(rx_1bit==EOT && !first_send_eot){
			first_send_eot=true;
			_1bit_buffer=ACK;
			uart_send_1bit(1,&_1bit_buffer);
			_1bit_buffer='C';
			uart_send_1bit(1,&_1bit_buffer);

			_1bit_buffer=ACK;
			uart_send_1bit(1,&_1bit_buffer);
			_1bit_buffer='C';
			uart_send_1bit(1,&_1bit_buffer);

			loop_do=false;
			uart1_send_flag=false;
			rx_1bit=0;

			uart_send(2,"\n+++++++++++++++++++++++++++++++++++++++++\n");
			uart_send(2,"\nsend_complete!\n");
			uart_send(2,"INFO)\nfile name : ");
			uart_send(2,"%s",file_name);
			uart_send(2,"\n");
			uart_send(2,"file size : ");
			uart_send(2,"%lu",file_size);
			uart_send(2,"\n");
			HAL_UART_DMAStop(&huart1);
			UART_Start_Receive_DMA(&huart1,&command_tx[0],5);;
		}
		else if(rx_ymodem[0]==STX){
			HAL_Delay(10);
			calculate_crc16(rx_ymodem+1027, 2, &loop_do);
			/*buffer send_occur*/
			for(uint16_t i=3;i<=1026;i++){
				uart_send_1bit(2,&rx_ymodem[i]);
			}
			memset(rx_ymodem,0,1029);
			rx_index=0;
			_1bit_buffer=ACK;
			uart_send_1bit(1,&_1bit_buffer);
			send_c_flag=false;
		  }
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1){
		if(uart1_send_flag){
			rx_ymodem[rx_index++]=rx_1bit;
			HAL_UART_Receive_DMA(&huart1,&rx_1bit,1);
		}
		else{
			HAL_UART_Receive_DMA(&huart1,&command_tx[0],5);
		}
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
	uint32_t flash_start_address;

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
		uart_send(1,"5. clear [x] : clear uart terminal, x can be 1,2.\n");
		uart_send(1,"6. ymodem transmit : transmit data with ymodem.\n");
		uart_send(1,"7. ymodem fw update : update fw.\n");
		uart_send(1,"note) start adrress must have 8 length.\n");
		uart_send(1,"8. flash read 0x[start adrress] [num]: read flash memory.\n");
		uart_send(1,"9. flash erase 0x[start adrress] [num] : erase flash memory.\n");
		uart_send(1,"10. flash write 0x[start adrress] : write flash memory.\n");
		uart_send(1,"\n---------------------------------------------------");
		uart_send(1,"----------------\n\n");
		existing_command = true;
	}
	else if(is_same_word(buf,(uint8_t *)"printf",0)){
		word_len=strlen("printf")+1;
		uart_send(1,"\n%s",buf+word_len);
		existing_command = true;
	}
	else if(is_same_word(buf,(uint8_t *)"flash read ",0)){
		word_len=strlen("flash read 0x");
		bool is_number;
		flash_start_address = string_to_num((char *)&buf[word_len],&is_number,HEX);
		if(!is_number){
			uart_send(1,"\nsend val %s is not number\n",buf+word_len);
		}
		word_len=strlen("flash read 0x00000000 ");
		uint16_t flash_read_val = string_to_num((char *)&buf[word_len],&is_number,DEC);
		if(is_number){
			flash_read(flash_start_address,flash_read_val);
		}
		else{
			uart_send(1,"\nsend val %s is not number\n",buf+word_len);
		}
		existing_command = true;
	}
	else if(is_same_word(buf,(uint8_t *)"flash erase ",0)){
		word_len=strlen("flash erase 0x");
		bool is_number;
		flash_start_address = string_to_num((char *)&buf[word_len],&is_number,HEX);
		if(!is_number){
			uart_send(1,"\nsend val %s is not number\n",buf+word_len);
		}
		word_len=strlen("flash erase 0x00000000 ");
		uint16_t flash_erase_val = string_to_num((char *)&buf[word_len],&is_number,DEC);
		if(is_number){
			if(flash_erase(flash_start_address,flash_erase_val)){
				uart_send(1,"\nerase complete!\n");
			}
			else{
				uart_send(1,"\nerase fail!\n");
			}
		}
		else{
			uart_send(1,"\nsend val %s is not number\n",buf+word_len);
		}
		existing_command = true;
		}
	else if(is_same_word(buf,(uint8_t *)"flash write ",0)){
		word_len=strlen("flash write 0x");
		bool is_number;
		flash_start_address = string_to_num((char *)&buf[word_len],&is_number,HEX);
		if(!is_number){
			uart_send(1,"\nsend val %s is not number\n",buf+word_len);
		}
		word_len=strlen("flash write 0x00000000 ");
		uint16_t flash_write_val = string_to_num((char *)&buf[word_len],&is_number,DEC);
		uint8_t *data=(uint8_t *)malloc(sizeof(uint8_t)*flash_write_val);
		memset(data,2,flash_write_val);
		if(is_number){
			if(flash_write(flash_start_address,flash_write_val,data)){
				uart_send(1,"\nwirte complete!\n");
			}
			else{
				uart_send(1,"\nwirte fail!\n");
			}
		}
		else{
			uart_send(1,"\nsend val %s is not number\n",buf+word_len);
		}
		existing_command = true;
		}
	else if(is_same_word(buf,(uint8_t *)"clear",0)){
		uart_send(1,"\x1B[2J");
		uart_send(2,"\x1B[2J");
		existing_command = true;
	}
	else if(is_same_word(buf,(uint8_t *)"clear 1",0)){
		uart_send(1,"\x1B[2J");
		existing_command = true;
	}
	else if(is_same_word(buf,(uint8_t *)"clear 2",0)){
		uart_send(2,"\x1B[2J");
		existing_command = true;
	}
	else if(is_same_word(buf,(uint8_t *)"ymodem transmit",0)){
		ymodem_transmit();
		existing_command = true;
	}
	else if(is_same_word(buf,(uint8_t *)"ymodem fw update",0)){
		ymodem_fw_update();
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
				bool is_number = false;
				led1_toggle_time = string_to_num((char *)buf+word_len,&is_number,DEC);
				if(!is_number){
					uart_send(1,(char *)"\nerror : toggle time value must be number!");
					return;
				}

				led1_toggle=true;
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
				bool is_number = false;
				led2_toggle_time = string_to_num((char *)buf+word_len,&is_number,DEC);
				if(!is_number){
					uart_send(1,(char *)"\nerror : toggle time value must be number!");
					return;
				}

				led2_toggle=true;
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
	/*
	uart_send(2,"\x1B[2J");
	for (int8_t j=0;j<3;j++){
		uart_send(2,"%d. buffer val:%s\n",j,data[j].command_buffer);
	}
	uart_send(2,"\n\n\n");
	uart_send(2,"num %d command buffer val:%s\n",data_num,data[data_num].command_buffer);
	uart_send(2,"command index:%d cursor index:%d\n",data[data_num].command_index,data[data_num].cursor_index);
	*/
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
