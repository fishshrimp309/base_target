#ifndef __RC_H
#define __RC_H

#include "main.h"
#include "usart.h"

#define DBUS_BUFLEN 18
#define DBUS_UART huart3

#define mouse_sensitivity_X 0.009
#define mouse_sensitivity_Y 0.009
#define KEY_W            ((uint16_t)1 << 0)
#define KEY_S            ((uint16_t)1 << 1)
#define KEY_A            ((uint16_t)1 << 2)
#define KEY_D            ((uint16_t)1 << 3)
#define KEY_SHIFT        ((uint16_t)1 << 4)
#define KEY_CTRL         ((uint16_t)1 << 5)
#define KEY_Q            ((uint16_t)1 << 6)
#define KEY_E            ((uint16_t)1 << 7)
#define KEY_R            ((uint16_t)1 << 8)
#define KEY_F            ((uint16_t)1 << 9)
#define KEY_G            ((uint16_t)1 << 10)
#define KEY_Z            ((uint16_t)1 << 11)
#define KEY_X            ((uint16_t)1 << 12)
#define KEY_C            ((uint16_t)1 << 13)
#define KEY_V            ((uint16_t)1 << 14)
#define KEY_B            ((uint16_t)1 << 15)

typedef struct
{
	float ch0;
	float ch1;
	float ch2;
	float ch3;
	int16_t roll;
	uint8_t sw1;
	uint8_t sw2;
	
	int16_t mouse_x;
	int16_t mouse_y;
	int16_t mouse_z;
	uint8_t mouse_L;
	uint8_t mouse_R;
	
	uint16_t key_code;
}rc_info_t;

typedef enum
{
	REMOTE = 0,
	KEYBOARD,
}Remote_Mode;

extern uint8_t DBUS_buf[DBUS_BUFLEN];
extern rc_info_t rc;
extern uint8_t isStop_flag;
extern uint8_t roll_start;

void DBUS_Uart_Init(UART_HandleTypeDef *huart);

#endif
