
#include "main.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#define BL_DEBUG_MSG_EN

CRC_HandleTypeDef hcrc;


UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;


#define D_UART   &huart3
#define C_UART   &huart2

#define FLASH_SECTOR5_BASE_ADDRESS      0x08020000U


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
static void printmsg(char *format,...);
void bootloader_jump_to_user_app(void);
void bootloader_usart_read_data(void);

void printmsg(char *format,...)
{
#ifdef BL_DEBUG_MSG_EN
	char str[80];
	va_list args;
	va_start(args,format);
	vsprintf(str,format,args);
	HAL_UART_Transmit(D_UART,(uint8_t *)&str,strlen(str),HAL_MAX_DELAY);
	va_end(args);
#endif
}


#define BL_RX_LEN    200
uint8_t bl_rx_buffer[BL_RX_LEN];
volatile uint8_t cmd_ready = 0;        // Command ready flag
volatile uint8_t cmd_len = 0;         // Length of the received command
uint8_t bl_rx_buffer[200];            // Buffer for received data
uint8_t common_cmd[] = {BL_GET_VER,BL_GET_HELP,BL_GET_CID,BL_GET_RDP_STATUS,
		             BL_GO_TO_ADDR,BL_FLASH_ERASE,BL_MEM_WRITE,BL_EN_RW_PROTECT,
					 BL_MEM_READ,BL_READ_SECTOR_P_STATUS,BL_OTP_READ};

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_CRC_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  printmsg("BL_DEBUG_MSG_EN: Welcome from Bootloader\n\r");
  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) ==  GPIO_PIN_SET){
	  printmsg("BL_DEBUG_MSG_EN: Button pressed.....Entering into Bootloader Mode\n\r");
	  bootloader_usart_read_data();
  }
  else{
	  printmsg("BL_DEBUG_MSG_EN: Button not pressed.....Executing Application\n\r");
	  bootloader_jump_to_user_app();
  }
}
void bootloader_jump_to_user_app(void){
	void (*app_reset_handler)(void);
	printmsg("BL_DEBUG_MSG_EN: Bootloader jumped to user application\n");
	uint32_t reset_handler = *(volatile uint32_t*)(FLASH_SECTOR5_BASE_ADDRESS + 4);
	printmsg("BL_DEBUG_MSG_EN:MSP value = %#x\n",reset_handler);
	app_reset_handler = (void (*)()) reset_handler;
	uint32_t msp_value = *(volatile uint32_t*)FLASH_SECTOR5_BASE_ADDRESS;
	printmsg("BL_DEBUG_MSG_EN:MSP value = %#x\n",msp_value);
	__set_MSP(msp_value);
	app_reset_handler();
}
void bootloader_usart_read_data(void)
{
    bootloader_uart_init();

    while (1)
    {
        if (cmd_ready)
        {
            cmd_ready = 0;

            printmsg("BL_DEBUG_MSG_EN: Bootloader command Length = %d\n", cmd_len);

            switch (bl_rx_buffer[1])
            {
            case BL_GET_VER:
                bootloader_handle_getver_cmd(bl_rx_buffer);
                break;
            case BL_GET_HELP:
                bootloader_handle_gethelp_cmd(bl_rx_buffer);
                break;
            case BL_GET_CID:
                bootloader_handle_getcid_cmd(bl_rx_buffer);
                break;
            case BL_GET_RDP_STATUS:
                bootloader_handle_getrdp_cmd(bl_rx_buffer);
                break;
            case BL_GO_TO_ADDR:
                bootloader_handle_go_cmd(bl_rx_buffer);
                break;
            case BL_FLASH_ERASE:
                bootloader_handle_flash_erase_cmd(bl_rx_buffer);
                break;
            case BL_MEM_WRITE:
                bootloader_handle_mem_write_cmd(bl_rx_buffer);
                break;
            case BL_EN_RW_PROTECT:
                bootloader_handle_en_rw_protect(bl_rx_buffer);
                break;
            case BL_READ_SECTOR_P_STATUS:
                bootloader_handle_read_sector_protection_status(bl_rx_buffer);
                break;
            case BL_OTP_READ:
                bootloader_handle_read_otp(bl_rx_buffer);
                break;
            case BL_DIS_R_W_PROTECT:
                bootloader_handle_dis_rw_protect(bl_rx_buffer);
                break;
            default:
                printmsg("BL_DEBUG_MSG: Invalid command code received from host\n");
                break;
            }
            HAL_UART_Receive_IT(C_UART, bl_rx_buffer, 1);
        }
    }
}

// UART RX Complete Callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    static uint8_t rx_index = 0;

    if (huart->Instance == USART2)
    {
        if (rx_index == 0)
        {
            cmd_len = bl_rx_buffer[0];
            HAL_UART_Receive_IT(C_UART, &bl_rx_buffer[1], cmd_len);
            rx_index++;
        }
        else
        {
            cmd_ready = 1;
            rx_index = 0;
        }
    }
}
void bootloader_uart_init(void)
{
    HAL_UART_Receive_IT(C_UART, bl_rx_buffer, 1);
}
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
}
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
    Error_Handler();
  }

}

static void MX_USART3_UART_Init(void)
{

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
}


static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitStruct.Pin = button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(button_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
/*************************bootloader command handle functions***************************/
void bootloader_handle_getver_cmd(uint8_t *bl_rx_buffer){
	   uint8_t bl_version;
	      printmsg("BL_DEBUG_MSG:bootloader_handle_getver_cmd\n");
		  uint32_t command_packet_len = bl_rx_buffer[0]+1 ;
		  uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	    if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	    {
	        printmsg("BL_DEBUG_MSG:checksum success !!\n");
	        bootloader_send_ack(bl_rx_buffer[0],1);
	        bl_version=get_bootloader_version();
	        printmsg("BL_DEBUG_MSG:BL_VER : %d %#x\n",bl_version,bl_version);
	        bootloader_uart_write_data(&bl_version,1);

	    }else
	    {
	        printmsg("BL_DEBUG_MSG:checksum fail !!\n");
	        bootloader_send_nack();
	    }
}
void bootloader_handle_gethelp_cmd(uint8_t *pBuffer){
	printmsg("BL_DEBUG_MSG:bootloader_handle_gethelp_cmd\n");
	uint32_t command_packet_len = bl_rx_buffer[0]+1;
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;
    if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
    {
        printmsg("BL_DEBUG_MSG:checksum success !!\n");
        bootloader_send_ack(bl_rx_buffer[0],sizeof(common_cmd));
        bootloader_uart_write_data(common_cmd,sizeof(common_cmd));

    }else
    {
        printmsg("BL_DEBUG_MSG:checksum fail !!\n");
        bootloader_send_nack();
    }
}
void bootloader_handle_getcid_cmd(uint8_t *pBuffer){
	uint16_t cid;
	printmsg("BL_DEBUG_MSG:bootloader_handle_getcid_cmd\n");
	uint32_t command_packet_len = bl_rx_buffer[0] + 1;
	uint32_t host_crc = *((uint32_t*)(bl_rx_buffer+command_packet_len - 4));
	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc)){
		printmsg("BL_DEBUG_MSG:checksum success !!\n");
		bootloader_send_ack(bl_rx_buffer[0],2);
		cid = get_mcu_chip_id();
		bootloader_uart_write_data((uint8_t*)&cid,sizeof(cid));
	}
	else{
		printmsg("BL_DEBUG_MSG:checksum fail !!\n");
		bootloader_send_nack();
	}

}
void bootloader_handle_getrdp_cmd(uint8_t *pBuffer){
	uint8_t rdp;
	printmsg("BL_DEBUG_MSG:bootloader_handle_getrdp_cmd\n");
	uint32_t command_packet_len = bl_rx_buffer[0] + 1;
	uint32_t host_crc = *((uint32_t*)(bl_rx_buffer+command_packet_len - 4));
	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc)){
		printmsg("BL_DEBUG_MSG:checksum success !!\n");
		bootloader_send_ack(bl_rx_buffer[0],1);
		rdp = get_flash_rdp_level();
		bootloader_uart_write_data((uint8_t*)&rdp,sizeof(rdp));
	}
	else{
		printmsg("BL_DEBUG_MSG:checksum fail !!\n");
		bootloader_send_nack();
	}

}
void bootloader_handle_go_cmd(uint8_t *pBuffer){
	uint8_t valid_addr = ADDR_VALID;
	uint8_t Invalid_addr = ADDR_INVALID;
	void (*jump_addr)(void);
	printmsg("BL_DEBUG_MSG:bootloader_handle_go_cmd\n");
	uint32_t command_packet_len = bl_rx_buffer[0] + 1;
	uint32_t host_crc = *((uint32_t*)(bl_rx_buffer+command_packet_len - 4));
	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc)){
		printmsg("BL_DEBUG_MSG:checksum success !!\n");
		bootloader_send_ack(bl_rx_buffer[0],1);
		uint32_t go_addr = *((uint32_t *)&bl_rx_buffer[2]);
		printmsg("BL_DEBUG_MSG:extracted Jump address = %d\n",go_addr);
		if(!verify_address(go_addr)){
			printmsg("BL_DEBUG_MSG: Goto address is VALID...\n");
			bootloader_uart_write_data((uint8_t*)&valid_addr,1);
			jump_addr = (void *)go_addr;
			printmsg("BL_DEBUG_MSG: Jumping to goto address....\n");
			jump_addr();
		}
		else{
			printmsg("BL_DEBUG_MSG: Goto address is INVALID...\n");
			bootloader_uart_write_data((uint8_t*)&Invalid_addr,1);
		}

	}
	else{
		printmsg("BL_DEBUG_MSG:checksum fail !!\n");
		bootloader_send_nack();
	}

}


void bootloader_handle_flash_erase_cmd(uint8_t *pBuffer){
	printmsg("BL_DEBUG_MSG:bootloader_handle_flash_erase_cmd\n");
	uint8_t status;
	uint32_t command_packet_len = bl_rx_buffer[0] + 1;
	uint32_t host_crc = *((uint32_t*)(bl_rx_buffer+command_packet_len - 4));
	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc)){
		printmsg("BL_DEBUG_MSG:checksum success !!\n");
		bootloader_send_ack(bl_rx_buffer[0],1);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1);
		 status = execute_flash_erase((uint8_t)bl_rx_buffer[2],(uint8_t)bl_rx_buffer[3]);
		 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);

		bootloader_uart_write_data((uint8_t*)&status,sizeof(status));
	}
	else{
		printmsg("BL_DEBUG_MSG:checksum fail !!\n");
		bootloader_send_nack();
	}
}
void bootloader_handle_mem_write_cmd(uint8_t *pBuffer)
{

	uint8_t write_status = 0x00;
	uint8_t payload_len = pBuffer[6];

	uint32_t mem_address = *((uint32_t *) ( &pBuffer[2]) );
    printmsg("BL_DEBUG_MSG:bootloader_handle_mem_write_cmd\n");
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;
	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
        printmsg("BL_DEBUG_MSG:checksum success !!\n");
        bootloader_send_ack(pBuffer[0],1);
        printmsg("BL_DEBUG_MSG: mem write address : %#x\n",mem_address);
		if( verify_address(mem_address) == ADDR_VALID )
		{

            printmsg("BL_DEBUG_MSG: valid mem write address\n");
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1);
            write_status = execute_mem_write(&pBuffer[7],mem_address, payload_len);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
            bootloader_uart_write_data(&write_status,1);

		}else
		{
            printmsg("BL_DEBUG_MSG: invalid mem write address\n");
            write_status = ADDR_INVALID;
            bootloader_uart_write_data(&write_status,1);
		}


	}else
	{
        printmsg("BL_DEBUG_MSG:checksum fail !!\n");
        bootloader_send_nack();
	}

}

/*Helper function to handle BL_EN_RW_PROTECT  command */
void bootloader_handle_en_rw_protect(uint8_t *pBuffer)
{
    uint8_t status = 0x00;
    printmsg("BL_DEBUG_MSG:bootloader_handle_endis_rw_protect\n");
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;
	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
        printmsg("BL_DEBUG_MSG:checksum success !!\n");
        bootloader_send_ack(pBuffer[0],1);
        status = configure_flash_sector_rw_protection(pBuffer[2] , pBuffer[3],0);
        printmsg("BL_DEBUG_MSG: flash erase status: %#x\n",status);
        bootloader_uart_write_data(&status,1);

	}else
	{
        printmsg("BL_DEBUG_MSG:checksum fail !!\n");
        bootloader_send_nack();
	}


}
/*Helper function to handle BL_EN_RW_PROTECT  command */
void bootloader_handle_dis_rw_protect(uint8_t *pBuffer)
{
    uint8_t status = 0x00;
    printmsg("BL_DEBUG_MSG:bootloader_handle_dis_rw_protect\n");
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;
	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
        printmsg("BL_DEBUG_MSG:checksum success !!\n");
        bootloader_send_ack(pBuffer[0],1);
        status = configure_flash_sector_rw_protection(0,0,1);
        printmsg("BL_DEBUG_MSG: flash erase status: %#x\n",status);
        bootloader_uart_write_data(&status,1);

	}else
	{
        printmsg("BL_DEBUG_MSG:checksum fail !!\n");
        bootloader_send_nack();
	}


}
void bootloader_handle_read_sector_protection_status(uint8_t *pBuffer)
{
	 uint16_t status;
	printmsg("BL_DEBUG_MSG:bootloader_handle_read_sector_protection_status\n");
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;
	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
        printmsg("BL_DEBUG_MSG:checksum success !!\n");
        bootloader_send_ack(pBuffer[0],2);
        status=read_OB_rw_protection_status();
        printmsg("BL_DEBUG_MSG: nWRP status: %#x\n",status);
        bootloader_uart_write_data((uint8_t*)&status,2);

	}else
	{
        printmsg("BL_DEBUG_MSG:checksum fail !!\n");
        bootloader_send_nack();
	}

}
void bootloader_handle_read_otp(uint8_t *pBuffer){

}
void bootloader_send_ack(uint8_t command_code, uint8_t follow_len){
	uint8_t ack_buf[2];
	ack_buf[0] = BL_ACK;
	ack_buf[1] = follow_len;
	HAL_UART_Transmit(C_UART, ack_buf,2,HAL_MAX_DELAY);
}
void bootloader_send_nack(void)
{
	uint8_t nack = BL_NACK;
	HAL_UART_Transmit(C_UART,&nack,1,HAL_MAX_DELAY);
}

uint8_t bootloader_verify_crc (uint8_t *pData, uint32_t len,uint32_t crc_host){
	uint32_t uwCRCValue = 0xff;
	for(uint32_t i = 0; i < len ; i++){
		uint32_t i_data = pData[i];
		uwCRCValue = HAL_CRC_Accumulate(&hcrc, &i_data, 1);
	}
	__HAL_CRC_DR_RESET(&hcrc);
	if( uwCRCValue == crc_host)
	{
		return VERIFY_CRC_SUCCESS;
	}

	return VERIFY_CRC_FAIL;
}
/* This function writes data in to C_UART */
void bootloader_uart_write_data(uint8_t *pBuffer,uint32_t len)
{
	HAL_UART_Transmit(C_UART,pBuffer,len,HAL_MAX_DELAY);

}

uint8_t get_bootloader_version(void)
{
  return (uint8_t)BL_VERSION;
}
uint16_t get_mcu_chip_id(void){
	uint16_t chip_id;
	volatile uint32_t *pCID = (volatile uint32_t*)0xE0042000;
	chip_id = (uint16_t)(*pCID & 0x0FFF);
	printmsg("BL_DEBUG_MSG: get_mcu_chip_id:%d\n",chip_id);
	return chip_id;
}
uint8_t get_flash_rdp_level(void){
	uint8_t rdp;
	volatile uint32_t *prdp = (volatile uint32_t*)0x1FFFC000;
	rdp = (uint8_t)((*prdp >> 8) & 0xFF);
	printmsg("BL_DEBUG_MSG: get_flash_rdp_level:%d\n",rdp);
	return rdp;
}
uint8_t verify_address(uint32_t jump_addr){
	if(jump_addr >= SRAM1_BASE && jump_addr <= SRAM1_END){
		return ADDR_VALID;
	}
	else if (jump_addr >= SRAM2_BASE && jump_addr <= SRAM2_END){
		return ADDR_VALID;
	}
	else if(jump_addr >= FLASH_BASE && jump_addr <= FLASH_END){
		return ADDR_VALID;
	}
	else if(jump_addr >= BKPSRAM_BASE && jump_addr <= BKPSRAM_END){
		return ADDR_VALID;
	}
	else{
		return ADDR_INVALID;
	}
}
uint8_t execute_flash_erase(uint8_t sector_number , uint8_t number_of_sector){
	FLASH_EraseInitTypeDef flashErase_handle;
	uint32_t sectorError;
	HAL_StatusTypeDef status;
	if(number_of_sector > 12){
		return INVALID_SECTOR;
	}
	if(sector_number == 0xff || sector_number <= 11){
		if(sector_number == 0xff){
			flashErase_handle.TypeErase = FLASH_TYPEERASE_MASSERASE;
		}
		else{
			uint8_t remaining_sector = 12 - sector_number;
			if(number_of_sector > remaining_sector){
				number_of_sector = remaining_sector;
			}
			flashErase_handle.TypeErase = FLASH_TYPEERASE_SECTORS;
			flashErase_handle.Sector = sector_number;
			flashErase_handle.NbSectors = number_of_sector;
		}
		flashErase_handle.Banks = FLASH_BANK_1;
		flashErase_handle.VoltageRange = FLASH_VOLTAGE_RANGE_3;
		HAL_FLASH_Unlock();
		status = (uint8_t) HAL_FLASHEx_Erase(&flashErase_handle, &sectorError);
		HAL_FLASH_Lock();
		return status;
	}
	return INVALID_SECTOR;
}

uint8_t execute_mem_write(uint8_t *pBuffer, uint32_t mem_address, uint32_t len)
{
    uint8_t status=HAL_OK;
    HAL_FLASH_Unlock();

    for(uint32_t i = 0 ; i <len ; i++)
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,mem_address+i,pBuffer[i] );
    }

    HAL_FLASH_Lock();

    return status;
}
uint8_t configure_flash_sector_rw_protection(uint8_t sector_details, uint8_t protection_mode, uint8_t disable)
{
    volatile uint32_t *pOPTCR = (uint32_t*) 0x40023C14;

	  if(disable)
		{
			HAL_FLASH_OB_Unlock();
			while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
			*pOPTCR |= (0xFF << 16);
			*pOPTCR |= ( 1 << 1);
			while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
			HAL_FLASH_OB_Lock();
			return 0;

		}

	   if(protection_mode == (uint8_t) 1)
    {
			HAL_FLASH_OB_Unlock();
			while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
			*pOPTCR &= ~ (sector_details << 16);
			*pOPTCR |= ( 1 << 1);
			while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
			HAL_FLASH_OB_Lock();
		}

		else if (protection_mode == (uint8_t) 2)
        {
			HAL_FLASH_OB_Unlock();
			while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
            *pOPTCR &= ~(0xff << 8);
			*pOPTCR |= (0xAA << 8);
			*pOPTCR |= ( 1 << 1);
			while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
			HAL_FLASH_OB_Lock();
        }

		return 0;
}
uint16_t read_OB_rw_protection_status(void)
{
	FLASH_OBProgramInitTypeDef OBInit;
	HAL_FLASH_OB_Unlock();
	HAL_FLASHEx_OBGetConfig(&OBInit);
	HAL_FLASH_Lock();
	return (uint16_t)OBInit.WRPSector;

}
