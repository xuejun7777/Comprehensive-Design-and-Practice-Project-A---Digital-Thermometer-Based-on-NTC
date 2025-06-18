#include "led_seg_display.h"
#include <stdlib.h>
#include <stdio.h>
#include "gpio.h"


__IO uint8_t dig_array[4];


const uint8_t seg_table[16] = {
	0x3f,		//"0"
	0x06,		//"1"
	0x5b,		//"2"
	0x4f,		//"3"
	0x66,		//"4"
	0x6d,		//"5"
	0x7d,		//"6"
	0x07,		//"7"
	0x7f,		//"8"
	0x6f,		//"9"
	0x77,		//"A"
	0x7c,		//"b"
	0x39,		//"C"
	0x5e,		//"d"
	0x79,		//"E"
	0x71		//"F"
};

void select_digit(uint8_t dig);
void write_segment(uint8_t seg);
//void display_ascii_str(char* p);
	


void DisplayCallback(void)
{
	uint8_t dig;
	uint32_t tick = HAL_GetTick();
	if((tick%LED_SEG_REFRESH_PERIOD) == 0)
	{
		dig = (tick/LED_SEG_REFRESH_PERIOD)%4;
		select_digit(dig);
		if(dig_array[dig]== 0xff)		//digit is off
		{
			write_segment(0x00);			
		}
		else if((dig_array[dig]&0x80) != 0)		//display minus
		{
			write_segment(0x40);	
		}
		else if((dig_array[dig]&0x10) != 0)		//add dot
		{
			write_segment(seg_table[dig_array[dig]&0x0f] | 0x80);
		}
		else
		{
			write_segment(seg_table[dig_array[dig]&0x0f]);
		}
	
	}
}


void select_digit(uint8_t dig)
{
	switch(dig)
	{
		case 0:
			HAL_GPIO_WritePin(GPIOD, LED_CC0_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, LED_CC1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, LED_CC2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, LED_CC3_Pin, GPIO_PIN_SET);
			break;
		case 1:
			HAL_GPIO_WritePin(GPIOD, LED_CC0_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, LED_CC1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, LED_CC2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, LED_CC3_Pin, GPIO_PIN_SET);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOD, LED_CC0_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, LED_CC1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, LED_CC2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, LED_CC3_Pin, GPIO_PIN_SET);
			break;
		case 3:
			HAL_GPIO_WritePin(GPIOD, LED_CC0_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, LED_CC1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, LED_CC2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, LED_CC3_Pin, GPIO_PIN_RESET);
			break;
		default:
			break;
	}
}


void write_segment(uint8_t seg)
{
	HAL_GPIO_WritePin(GPIOB, seg, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, ~seg, GPIO_PIN_RESET);
}



void DisplayDigitUINT32(uint32_t num)
{
	uint32_t tmp;
	if(num > 9999)
	{
		for(uint8_t i=0;i<8;i++)
			dig_array[i] = 0x0f;
	}
	else
	{
		tmp = num;
		for(uint8_t i=0;i<4;i++)
		{
			if((tmp == 0)&&(i!=0))
			{
				dig_array[i] = 0xff;
			}
			else
			{
				dig_array[i] = tmp%10;			
			}
			tmp = tmp/10;
		}
			
	}
}

void DisplayDigitINT32(int32_t num)
{
	int32_t tmp;
	if(num > 9999)
	{
		for(uint8_t i=0;i<8;i++)
			dig_array[i] = 0x0f;
	}
	else if(num < -999)
	{
		for(uint8_t i=0;i<4;i++)
			dig_array[i] = 0x0f;	
		dig_array[3] = 0x80;
	}
	else
	{
		tmp = num;
		for(uint8_t i=0;i<4;i++)
		{
			if((tmp == 0)&&(i!=0))
			{
				dig_array[i] = 0xff;
			}
			else
			{
				dig_array[i] = abs(tmp)%10;			
			}
			tmp = tmp/10;
		}
		if(num <0)
		{
			for(uint8_t i=0;i<4;i++)
			{
				if(dig_array[i] == 0xff)
				{
					dig_array[i] = 0x80;
					break;
				}
			}
		}
			
	}
}


