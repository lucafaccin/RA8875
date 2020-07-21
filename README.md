# RA8875


Library based on the Adafruit RA8875 library for Arduino converted C.
This library uses the STM32 FSMC controller to achive a 8-bit parallel data transfer (8080 interface) to the RA8875.

### Usage

Download or clone the project

Open the RA8875.h
```c
#include "stm32f3xx_hal.h"


/**************************************************************************************
 * CUSTOMIZE WITH YOUR MCU SETTINGS
 */
//FMC structure for 8-bit parallel LCD
typedef struct
{
  __IO uint8_t VAL;
} RA8875_TypeDef;


/**
 * Helpers for FMC: CUSTOMIZE IT BASING ON YOU HARDWARE SETUP
 *
 * See https://www.st.com/resource/en/application_note/cd00201397-tft-lcd-interfacing-with-the-highdensity-stm32f10xxx-fsmc-stmicroelectronics.pdf
 *
 *
 */
#define FMC_DATA_BASE_ADDR   		((uint32_t)(0x60000000))		//FMC Data base access registry
#define RA8875_DATA          		((RA8875_TypeDef *) FMC_DATA_BASE_ADDR)

#define FMC_REGISTER_BASE_ADDR    	((uint32_t)(0x600F0000))
#define RA8875_REGISTER        		((RA8875_TypeDef *) FMC_REGISTER_BASE_ADDR)
```

and change the following line
```c
#include "stm32f3xx_hal.h"
```
to reflect the correct header for your STM32 MCU HAL.

Change the base address of the FSMC for DATA and REGISTER 
```c
#define FMC_DATA_BASE_ADDR   		((uint32_t)(0x60000000))		//FMC Data base access registry
#define RA8875_DATA          		((RA8875_TypeDef *) FMC_DATA_BASE_ADDR)

#define FMC_REGISTER_BASE_ADDR    	((uint32_t)(0x600F0000))
#define RA8875_REGISTER        		((RA8875_TypeDef *) FMC_REGISTER_BASE_ADDR)
```
See  https://www.st.com/resource/en/application_note/cd00201397-tft-lcd-interfacing-with-the-highdensity-stm32f10xxx-fsmc-stmicroelectronics.pdf for reference on how to use FSMC as 8080-style interface

##### Initialization code
```c
//Init display with 480x272 and set color as 16-bit per pixel
RA8875_begin(LCD_RESET_GPIO_Port,LCD_RESET_Pin,LCD_WAIT_GPIO_Port,pLCD_WAIT_GPIO_Pin,RA8875_480x272,RA8875_16BPP);
//Power on the dispaly
RA8875_display_on(1);

//Backlight on
RA8875_pwm1_config(1,RA8875_PWM_CLK_DIV1);
RA8875_pwm1_out(250);

//Since in 480*272 i got 2 layers at 16-bit per pixel lets blank both screen with white
RA8875_set_write_layer(0);
RA8875_fill_screen(RA8875_WHITE);

RA8875_set_write_layer(1);
RA8875_fill_screen(RA8875_WHITE);

//Show layer 0
RA8875_set_layer_mode(RA8875_LTPR0_DM_L0);
```

### Example hardware configuration

Base address used was tested on a STM32F303VET6 with the following FSMC settings
##### FSMC NOR Flash/PSRAM/SRAM/ROM/LCD1 in STM32Cube MX
- [x] NE1 Chip Select
 

Parameter | Value
------------ | -------------
Memory Type|LCD Interface
Bank|Bank1 NOR/PSRAM 1
LCD Register Select| A17
Data|8 bits
Address setup time in HCLK clock cycles| 1
Data setup time in HCLK clock cycles| 1
BUS turn around time in HCLK clock cycle| 1

##### FSMC PIN Configuration


Pin name|Signal on Pin|GPIO output level|GPIO Mode|GPIO Pullup/Pulldown|Maximum output speed|Fast mode|User label
------------ | -------------| -------------| -------------| -------------| -------------| -------------|--------
PD0|FMC_D2|n/a|Alternate Function Push Pull|No pull up pull down|High|n/a
PD1|FMC_D3|n/a|Alternate Function Push Pull|No pull up pull down|High|n/a
PD4|FMC_NOE|n/a|Alternate Function Push Pull|No pull up pull down|High|n/a
PD5|FMC_NWE|n/a|Alternate Function Push Pull|No pull up pull down|High|n/a
PD7|FMC_NE1|n/a|Alternate Function Push Pull|No pull up pull down|High|n/a
PD12|FMC_A17|n/a|Alternate Function Push Pull|No pull up pull down|High|n/a
PD14|FMC_D0|n/a|Alternate Function Push Pull|No pull up pull down|High|n/a
PD15|FMC_D1|n/a|Alternate Function Push Pull|No pull up pull down|High|n/a
PE7|FMC_D4|n/a|Alternate Function Push Pull|No pull up pull down|High|n/a
PE8|FMC_D5|n/a|Alternate Function Push Pull|No pull up pull down|High|n/a
PE9|FMC_D6|n/a|Alternate Function Push Pull|No pull up pull down|High|n/a
PE10|FMC_D7|n/a|Alternate Function Push Pull|No pull up pull down|High|n/a

##### Other pin configuration
Pin name|Signal on Pin|GPIO output level|GPIO Mode|GPIO Pullup/Pulldown|Maximum output speed|Fast mode|User label
------------ | -------------| -------------| -------------| -------------| -------------| -------------|--------
PB12|n/a|n/a|Input mode|No pull up pull down|n/a|n/a|LCD_WAIT
PE6|n/a|Low|Output Push Pull|No pull up pull down|Low|n/a|LCD_RESET