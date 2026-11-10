#include "rc.h"
#include "bsp_can.h"
#include "cmsis_os2.h"
#include "usart.h"
#include "Judge.h"

Remote_Mode remote_mode;
rc_info_t rc ={0};
uint8_t DBUS_buf[DBUS_BUFLEN];

int rc_restart_flag = 0;
uint8_t isStop_flag = 1;
uint8_t roll_start = 0;

void DBUS_Uart_Init(UART_HandleTypeDef *huart)
{
	HAL_UARTEx_ReceiveToIdle_DMA(huart, DBUS_buf, DBUS_BUFLEN);
	__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
	__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_TC);
	__HAL_UART_DISABLE_IT(huart, UART_IT_ERR);
	__HAL_UART_DISABLE_IT(huart, UART_IT_PE);
}

void Rc_Handler(rc_info_t *rc, uint8_t *buff)
{
	rc->ch0 = (buff[0] | buff[1] << 8) & 0x07FF;
	rc->ch0 -= 1024; 
	rc->ch1 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
	rc->ch1 -= 1024; 
	rc->ch2 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
	rc->ch2 -= 1024; 
	rc->ch3 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
	rc->ch3 -= 1024; 
	rc->roll = (buff[16] | (buff[17] << 8)) & 0x07FF;
	rc->sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
	rc->sw2 = (buff[5] >> 4) & 0x0003;
	
	if (rc->ch0 <= 5 && rc->ch0 >= -5)
		rc->ch0 = 0;
	if (rc->ch1 <= 5 && rc->ch1 >= -5)
		rc->ch1 = 0;
	if (rc->ch2 <= 5 && rc->ch2 >= -5)
		rc->ch2 = 0;
	if (rc->ch3 <= 5 && rc->ch3 >= -5)
		rc->ch3 = 0;
	
	rc->ch0 = rc->ch0 / 660.0f;
	rc->ch1 = rc->ch1 / 660.0f;
	rc->ch2 = rc->ch2 / 660.0f;
	rc->ch3 = rc->ch3 / 660.0f;
	rc->roll -= 1024;

	rc->mouse_x = buff[6] | (buff[7] << 8);
	rc->mouse_y = buff[8] | (buff[9] << 8);
	rc->mouse_z = buff[10] | (buff[11] << 8);
	rc->mouse_L = buff[12];
	rc->mouse_R = buff[13];
	
	rc->key_code = buff[14] | (buff[15] << 8);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart3)   
    {
        Rc_Handler(&rc, DBUS_buf);
		rc_restart_flag = 0;
    }
	if (huart == &huart1)
	{
		Judge_Receive();
	}
}

void Task_RcCallback(void)
{
	if(rc.sw1 == 2 || (!roll_start))
	{
		isStop_flag = 1;
		roll_start = 0;
	}
	else
	{
		isStop_flag = 0;
	}
	
	if((rc.sw1 != 2)&&(rc.roll > 400))
	{
		roll_start = 1;
	}
		
}
	
void OS_RcCallback(void *argument)
{
	for(;;)
	{
		rc_restart_flag++;

		if(rc_restart_flag >= 30)
		{	
			HAL_UARTEx_ReceiveToIdle_DMA(&DBUS_UART, DBUS_buf, DBUS_BUFLEN);
			__HAL_DMA_DISABLE_IT(DBUS_UART.hdmarx, DMA_IT_HT);
		}

		Task_RcCallback();
		osDelay(15);
	}
}
