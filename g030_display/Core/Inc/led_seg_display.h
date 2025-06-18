#ifndef __LED_SEG_DISPLAY_H__
#define __LED_SEG_DISPLAY_H__

#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"

#define LED_SEG_REFRESH_PERIOD		5

extern __IO uint8_t dig_array[4];
void LEDSEG_Display_Init(void);

void select_digit(uint8_t dig);
void write_segment(uint8_t seg);
void DisplayDigitUINT32(uint32_t num);
void DisplayDigitINT32(int32_t num);
void DisplayDigitFLOAT(float num);
void DisplayCallback(void);


#ifdef __cplusplus
}
#endif
#endif /*__LED_SEG_DISPLAY_H__ */
