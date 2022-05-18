/*
 * DFPLAYER_MINI.c
 *
 *  Created on: May 16, 2020
 *      Author: controllerstech
 */


#include "stm32l4xx_hal.h"
#include "stdio.h"

extern UART_HandleTypeDef huart1;
#define DF_UART &huart1

#define Source      0x02  // TF CARD

#define Previous_Key   GPIO_PIN_5
#define Previous_Port  GPIOB
#define Pause_Key      GPIO_PIN_4
#define Pause_Port     GPIOB
#define Next_Key       GPIO_PIN_1
#define Next_Port      GPIOB

/*************************************** NO CHANGES AFTER THIS *************************************************/

int ispause =0;
int isplaying=1;


# define Start_Byte 0x7E
# define End_Byte   0xEF
# define Version    0xFF
# define Cmd_Len    0x06
# define Feedback   0x00    //If need for Feedback: 0x01,  No Feedback: 0

void Send_cmd (uint8_t cmd, uint8_t Parameter1, uint8_t Parameter2)
{
	uint16_t Checksum = Version + Cmd_Len + cmd + Feedback + Parameter1 + Parameter2;
	Checksum = 0-Checksum;

	uint8_t CmdSequence[10] = { Start_Byte, Version, Cmd_Len, cmd, Feedback, Parameter1, Parameter2, (Checksum>>8)&0x00ff, (Checksum&0x00ff), End_Byte};

	HAL_UART_Transmit(DF_UART, CmdSequence, 10, HAL_MAX_DELAY);
}

void DF_PlayFromStart(void)
{
  Send_cmd(0x03,0x00,0x01);
  HAL_Delay(200);
}
void DF_PlayFrom(uint8_t num)
{
  Send_cmd(0x0F,0x00,num);
  HAL_Delay(200);
}

void DF_Init (uint8_t volume)
{
	Send_cmd(0x3F, 0x00, Source);
	HAL_Delay(200);
	Send_cmd(0x06, 0x00, volume);
	HAL_Delay(500);
}

void DF_Next (void)
{
	Send_cmd(0x01, 0x00, 0x00);
	HAL_Delay(200);
}

void DF_Pause (void)
{
	Send_cmd(0x0E, 0, 0);
	HAL_Delay(200);
}

void DF_Previous (void)
{
	Send_cmd(0x02, 0, 0);
	HAL_Delay(200);
}

void DF_Playback (void)
{
	Send_cmd(0x0D, 0, 0);
	HAL_Delay(200);
}

void Check_Key (void)
{
	if (HAL_GPIO_ReadPin(Pause_Port, Pause_Key))
	{
		while (HAL_GPIO_ReadPin(Pause_Port, Pause_Key));
		if (isplaying)
		{
			ispause = 1;
			isplaying = 0;
			DF_Pause();
		}

		else if (ispause)
		{
			isplaying = 1;
			ispause = 0;
			DF_Playback();
		}
	}

	if (HAL_GPIO_ReadPin(Previous_Port, Previous_Key))
	{
		while (HAL_GPIO_ReadPin(Previous_Port, Previous_Key));
		DF_Previous();
	}

	if (HAL_GPIO_ReadPin(Next_Port, Next_Key))
	{
		while (HAL_GPIO_ReadPin(Next_Port, Next_Key));
		DF_Next();
	}
}















