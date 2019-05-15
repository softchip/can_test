#ifndef __OLED_H__
#define __OLED_H__
//#include "stm32f10x_lib.h"
#include "stdlib.h"
#include "stm32f1xx_hal.h"

//#include "userinc/gpio.h"

//汉字大小，英文数字大小
#define 	TYPE8X16		1
#define 	TYPE16X16		2
#define 	TYPE6X8			3

//-----------------OLED端口定义----------------  					   
/*
#define LCD_SCL_CLR()	GPIO_ResetBits(GPIOB,GPIO_Pin_15)
#define LCD_SCL_SET()	GPIO_SetBits(GPIOB,GPIO_Pin_15)

#define LCD_SDA_CLR()	GPIO_ResetBits(GPIOB,GPIO_Pin_14)
#define LCD_SDA_SET()	GPIO_SetBits(GPIOB,GPIO_Pin_14)

#define LCD_RST_CLR()	GPIO_ResetBits(GPIOB,GPIO_Pin_13)
#define LCD_RST_SET()	GPIO_SetBits(GPIOB,GPIO_Pin_13)

#define LCD_DC_CLR()	GPIO_ResetBits(GPIOB,GPIO_Pin_12)
#define LCD_DC_SET()	GPIO_SetBits(GPIOB,GPIO_Pin_12)
*/

/*
#define LCD_SCL_CLR() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET)
#define LCD_SCL_SET() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET)

#define LCD_SDA_CLR()	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET)
#define LCD_SDA_SET() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET)

#define LCD_RST_CLR() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET)
#define LCD_RST_SET() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET)

#define LCD_DC_CLR() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)
#define LCD_DC_SET() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
*/

// SCL:PA10, SDA:PA11, RST:PA12, D/C:PA15
#define LCD_SCL_CLR() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET)
#define LCD_SCL_SET() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET)

#define LCD_SDA_CLR()	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET)
#define LCD_SDA_SET() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET)

#define LCD_RST_CLR() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET)
#define LCD_RST_SET() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET)

#define LCD_DC_CLR() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET)
#define LCD_DC_SET() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET)




extern void LCD_Init(void);
extern void LCD_CLS(void);
extern void LCD_CLS_y(char y);
extern void LCD_CLS_line_area(uint8_t start_x,uint8_t start_y,uint8_t width);
extern void LCD_P6x8Str(uint8_t x,uint8_t y,uint8_t *ch,const uint8_t *F6x8);
extern void LCD_P8x16Str(uint8_t x,uint8_t y,uint8_t *ch,const uint8_t *F8x16);
extern void LCD_P14x16Str(uint8_t x,uint8_t y,uint8_t ch[],const uint8_t *F14x16_Idx,const uint8_t *F14x16);
extern void LCD_P16x16Str(uint8_t x,uint8_t y,uint8_t *ch,const uint8_t *F16x16_Idx,const uint8_t *F16x16);
//extern void LCD_Print(u8 x, u8 y, u8 *ch);
extern void LCD_PutPixel(uint8_t x,uint8_t y);

//extern void LCD_Print(uint8_t x, uint8_t y, uint8_t *ch,uint8_t char_size, uint8_t ascii_size);
extern void LCD_Print(uint8_t x, uint8_t y, char *ch,uint8_t char_size, uint8_t ascii_size);

extern void LCD_Rectangle(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2,uint8_t gif);
extern void Draw_BMP(uint8_t x,uint8_t y,const uint8_t *bmp); 
extern void LCD_Fill(uint8_t dat);
#endif

