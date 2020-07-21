/*
 * RA8875.h
 *
 *  Created on: 09 lug 2020
 *      Author: luca.faccin
 *      This file contains all the method to manage a RA8875 display driver
 */

#ifndef RA8875_H_
#define RA8875_H_
#include "stdint.h"
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


// Touchscreen Calibration and EEPROM Storage Defines
#define CFG_EEPROM_TOUCHSCREEN_CAL_AN 0       ///< EEPROM Storage Location
#define CFG_EEPROM_TOUCHSCREEN_CAL_BN 4       ///< EEPROM Storage Location
#define CFG_EEPROM_TOUCHSCREEN_CAL_CN 8       ///< EEPROM Storage Location
#define CFG_EEPROM_TOUCHSCREEN_CAL_DN 12      ///< EEPROM Storage Location
#define CFG_EEPROM_TOUCHSCREEN_CAL_EN 16      ///< EEPROM Storage Location
#define CFG_EEPROM_TOUCHSCREEN_CAL_FN 20      ///< EEPROM Storage Location
#define CFG_EEPROM_TOUCHSCREEN_CAL_DIVIDER 24 ///< EEPROM Storage Location
#define CFG_EEPROM_TOUCHSCREEN_CALIBRATED 28  ///< EEPROM Storage Location



// Sizes!

/**************************************************************************/
/*!
 @enum RA8875_sizes_e The Supported Screen Sizes
 */
/**************************************************************************/
typedef enum  {
  RA8875_480x80,  /*!<  480x80 Pixel Display */
  RA8875_480x128, /*!< 480x128 Pixel Display */
  RA8875_480x272, /*!< 480x272 Pixel Display */
  RA8875_800x480  /*!< 800x480 Pixel Display */
} RA8875_sizes_e;

/**************************************************************************/
/*!
 @enum RA8875_cursor_direction_e The Supported memory RW direction
 */
/**************************************************************************/
typedef enum
{
  LEFT_RIGHT_TOP_DOWN=0x00,
  RIGHT_LEFT_TOP_DOWN=0x04,
  TOP_DOWN_LEFT_RIGHT=0x08,
  TOP_DOWN_RIGHT_LEFT=0x0C,
} RA8875_cursor_direction_e;

/**************************************************************************/
/*!
 @enum RA8875_bpp_e The bit per pixels
 */
/**************************************************************************/

typedef enum
{
  RA8875_8BPP,
  RA8875_16BPP
} RA8875_bpp_e;

/**************************************************************************/
/*!
 @struct Point
 Calibration Point

 @var Point::x
    x-coordinate
 @var Point::y
    y-coordinate
 */
/**************************************************************************/
typedef struct Point {
  int32_t x;
  int32_t y;
} tsPoint_t; ///< Nameless struct variable!

/**************************************************************************/
/*!
 @struct tsMatrix_t
 Calibration Data Structure

 @var tsMatrix_t::An
 A Coefficient with the coarsest granularity
 @var tsMatrix_t::Bn
 B Coeffiecient
 @var tsMatrix_t::Cn
 C Coefficient
 @var tsMatrix_t::Dn
 D Coeffiecient
 @var tsMatrix_t::En
 E Coefficient
 @var tsMatrix_t::Fn
 F Coeffiecient with the finest granularity
 @var tsMatrix_t::Divider
 Divider for Coefficients
 */
/**************************************************************************/
typedef struct // Matrix
{
  int32_t An, Bn, Cn, Dn, En, Fn, Divider;
} tsMatrix_t;


/**
 * Table for RGB332 to RGB565 conversion
 */
extern const uint16_t rgb332_to_565[256];


/**************************************************************************/
/*!
 @brief  Functions for interacting with the RA8875 display controller.
 */
/**************************************************************************/

uint8_t RA8875_begin (GPIO_TypeDef *reset_port,uint16_t reset_pin,GPIO_TypeDef *wait_port,uint16_t wait_pin,RA8875_sizes_e s,RA8875_bpp_e bpp);
void RA8875_soft_reset(void);
void RA8875_display_on(uint8_t on);
void RA8875_sleep(uint8_t sleep);
void RA8875_disable_keyscan();


/* Text functions */
void RA8875_text_mode(void);
void RA8875_text_cursor_position(uint16_t x, uint16_t y);
void RA8875_text_color(uint16_t foreColor, uint16_t bgColor);
void RA8875_text_transparent_color(uint16_t foreColor);
void RA8875_text_scale(uint8_t scale);
void RA8875_text_write(const char *buffer, uint16_t len);
void RA8875_cursor_blink(uint8_t rate);

/* Primitive Graphics functions */
void RA8875_graphic_mode(void);
void RA8875_set_write_layer(uint8_t layer);
void RA8875_set_layer_mode(uint8_t layer_mode);
void RA8875_set_layer_transparent_color(uint16_t color);
void RA8875_set_active_window(uint16_t x,uint16_t y,uint16_t w,uint16_t h);

void RA8875_set_xy(uint16_t x, uint16_t y);
void RA8875_set_cursor_direction(RA8875_cursor_direction_e direction);
void RA8875_write_ram(void);
void RA8875_push_pixel(uint16_t p);
void RA8875_push_pixels(uint32_t num, uint16_t p);


/* Basic GFX functions */
void RA8875_draw_pixel(int16_t x, int16_t y, uint16_t color);
void RA8875_draw_pixels(uint16_t *p, uint32_t count, int16_t x, int16_t y);
void RA8875_draw_fast_vline(int16_t x, int16_t y, int16_t h, uint16_t color);
void RA8875_draw_fast_hline(int16_t x, int16_t y, int16_t w, uint16_t color);

/* HW accelerated wrapper functions*/
void RA8875_fill_screen(uint16_t color);
void RA8875_draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
void RA8875_draw_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void RA8875_draw_fill_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void RA8875_draw_circle(int16_t x, int16_t y, int16_t r, uint16_t color);
void RA8875_draw_fill_circle(int16_t x, int16_t y, int16_t r, uint16_t color);
void RA8875_draw_triangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2,int16_t y2, uint16_t color);
void RA8875_draw_fill_triangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2,int16_t y2, uint16_t color);
void RA8875_draw_draw_ellipse(int16_t xCenter, int16_t yCenter, int16_t longAxis,int16_t shortAxis, uint16_t color);
void RA8875_draw_fill_ellipse(int16_t xCenter, int16_t yCenter, int16_t longAxis,int16_t shortAxis, uint16_t color);
void RA8875_draw_curve(int16_t xCenter, int16_t yCenter, int16_t longAxis,int16_t shortAxis, uint8_t curvePart, uint16_t color);
void RA8875_draw_fill_curve(int16_t xCenter, int16_t yCenter, int16_t longAxis,int16_t shortAxis, uint8_t curvePart, uint16_t color);
void RA8875_draw_round_rect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r,uint16_t color);
void RA8875_draw_fill_round_rect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r,uint16_t color);

/* Scroll */
void RA8875_set_scroll_window(int16_t x, int16_t y, int16_t w, int16_t h,uint8_t mode);
void RA8875_scroll_x(int16_t dist);
void RA8875_scroll_y(int16_t dist);


/* Backlight */
void GPIOX(uint8_t on);
void RA8875_pwm1_config(uint8_t on, uint8_t clock);
void RA8875_pwm2_config(uint8_t on, uint8_t clock);
void RA8875_pwm1_out(uint8_t p);
void RA8875_pwm2_out(uint8_t p);

/* Touch screen */
void RA8875_enable_touch(uint8_t on);
uint8_t RA8875_touched(void);
uint8_t RA8875_touch_read(uint16_t *x, uint16_t *y);


/* Low level access */
void RA8875_write_reg(uint8_t reg, uint8_t val);
uint8_t RA8875_read_reg(uint8_t reg);
void RA8875_write_data(uint8_t d);
uint8_t RA8875_read_data(void);
void RA8875_write_command(uint8_t d);
uint8_t RA8875_read_status(void);
uint8_t RA8875_wait_poll(uint8_t r, uint8_t f);
uint16_t RA8875_width(void);
uint16_t RA8875_height(void);
void RA8875_set_rotation(int8_t rotation);
int8_t RA8875_get_rotation(void);

// Colors (RGB565)
#define RA8875_BLACK 0x0000   ///< Black Color
#define RA8875_BLUE 0x001F    ///< Blue Color
#define RA8875_RED 0xF800     ///< Red Color
#define RA8875_GREEN 0x07E0   ///< Green Color
#define RA8875_CYAN 0x07FF    ///< Cyan Color
#define RA8875_MAGENTA 0xF81F ///< Magenta Color
#define RA8875_YELLOW 0xFFE0  ///< Yellow Color
#define RA8875_WHITE 0xFFFF   ///< White Color

// Command/Data pins for SPI
#define RA8875_DATAWRITE 0x00 ///< See datasheet
#define RA8875_DATAREAD 0x40  ///< See datasheet
#define RA8875_CMDWRITE 0x80  ///< See datasheet
#define RA8875_CMDREAD 0xC0   ///< See datasheet

// Registers & bits
#define RA8875_PWRR 0x01           ///< See datasheet
#define RA8875_PWRR_DISPON 0x80    ///< See datasheet
#define RA8875_PWRR_DISPOFF 0x00   ///< See datasheet
#define RA8875_PWRR_SLEEP 0x02     ///< See datasheet
#define RA8875_PWRR_NORMAL 0x00    ///< See datasheet
#define RA8875_PWRR_SOFTRESET 0x01 ///< See datasheet

#define RA8875_MRWC 0x02 ///< See datasheet

#define RA8875_GPIOX 0xC7 ///< See datasheet

/**
 * Layers
 */
#define RA8875_LTPR0 0x52		///<Layer transparency register 0
#define RA8875_LTPR0_DM_BITMASK 0x07	///<Bitmask for display mode
#define RA8875_LTPR0_DM_L0 0x00	///<Display L1 only
#define RA8875_LTPR0_DM_L1 0x01	///<Display L2 only
#define RA8875_LTPR0_DM_TR 0x03	///<Transparent mode
#define RA8875_LTPR0_DM_LO 0x02	///<Light overlay
#define RA8875_LTPR0_DM_OR 0x04///<OR
#define RA8875_LTPR0_DM_AND 0x05	///<AND
#define RA8875_LTPR0_DM_FW 0x06	///<Floating window

#define RA8875_BGTR0 0x67 ///<Transparent color RED
#define RA8875_BGTR1 0x68 ///<Transparent color RED
#define RA8875_BGTR2 0x69 ///<Transparent color RED

#define RA8875_PLLC1 0x88         ///< See datasheet
#define RA8875_PLLC1_PLLDIV2 0x80 ///< See datasheet
#define RA8875_PLLC1_PLLDIV1 0x00 ///< See datasheet

#define RA8875_PLLC2 0x89        ///< See datasheet
#define RA8875_PLLC2_DIV1 0x00   ///< See datasheet
#define RA8875_PLLC2_DIV2 0x01   ///< See datasheet
#define RA8875_PLLC2_DIV4 0x02   ///< See datasheet
#define RA8875_PLLC2_DIV8 0x03   ///< See datasheet
#define RA8875_PLLC2_DIV16 0x04  ///< See datasheet
#define RA8875_PLLC2_DIV32 0x05  ///< See datasheet
#define RA8875_PLLC2_DIV64 0x06  ///< See datasheet
#define RA8875_PLLC2_DIV128 0x07 ///< See datasheet

#define RA8875_SYSR 0x10       ///< See datasheet
#define RA8875_SYSR_8BPP 0x00  ///< See datasheet
#define RA8875_SYSR_16BPP 0x0C ///< See datasheet
#define RA8875_SYSR_MCU8 0x00  ///< See datasheet
#define RA8875_SYSR_MCU16 0x03 ///< See datasheet

#define RA8875_PCSR 0x04       ///< See datasheet
#define RA8875_PCSR_PDATR 0x00 ///< See datasheet
#define RA8875_PCSR_PDATL 0x80 ///< See datasheet
#define RA8875_PCSR_CLK 0x00   ///< See datasheet
#define RA8875_PCSR_2CLK 0x01  ///< See datasheet
#define RA8875_PCSR_4CLK 0x02  ///< See datasheet
#define RA8875_PCSR_8CLK 0x03  ///< See datasheet

#define RA8875_DPCR 0x20	///< See datasheet
#define RA8875_DPCR_1_LAYER 0x00 ///< See datasheet
#define RA8875_DPCR_2_LAYER 0x80 ///< See datasheet
#define RA8875_DPCR_REVERSE_HDIR 0x08 ///< See datasheet
#define RA8875_DPCR_REVERSE_VDIR 0x04 ///< See datasheet

#define RA8875_HDWR 0x14 ///< See datasheet

#define RA8875_HNDFTR 0x15         ///< See datasheet
#define RA8875_HNDFTR_DE_HIGH 0x00 ///< See datasheet
#define RA8875_HNDFTR_DE_LOW 0x80  ///< See datasheet

#define RA8875_HNDR 0x16      ///< See datasheet
#define RA8875_HSTR 0x17      ///< See datasheet
#define RA8875_HPWR 0x18      ///< See datasheet
#define RA8875_HPWR_LOW 0x00  ///< See datasheet
#define RA8875_HPWR_HIGH 0x80 ///< See datasheet

#define RA8875_VDHR0 0x19     ///< See datasheet
#define RA8875_VDHR1 0x1A     ///< See datasheet
#define RA8875_VNDR0 0x1B     ///< See datasheet
#define RA8875_VNDR1 0x1C     ///< See datasheet
#define RA8875_VSTR0 0x1D     ///< See datasheet
#define RA8875_VSTR1 0x1E     ///< See datasheet
#define RA8875_VPWR 0x1F      ///< See datasheet
#define RA8875_VPWR_LOW 0x00  ///< See datasheet
#define RA8875_VPWR_HIGH 0x80 ///< See datasheet

#define RA8875_HSAW0 0x30 ///< See datasheet
#define RA8875_HSAW1 0x31 ///< See datasheet
#define RA8875_VSAW0 0x32 ///< See datasheet
#define RA8875_VSAW1 0x33 ///< See datasheet

#define RA8875_HEAW0 0x34 ///< See datasheet
#define RA8875_HEAW1 0x35 ///< See datasheet
#define RA8875_VEAW0 0x36 ///< See datasheet
#define RA8875_VEAW1 0x37 ///< See datasheet

#define RA8875_MCLR 0x8E            ///< See datasheet
#define RA8875_MCLR_START 0x80      ///< See datasheet
#define RA8875_MCLR_STOP 0x00       ///< See datasheet
#define RA8875_MCLR_READSTATUS 0x80 ///< See datasheet
#define RA8875_MCLR_FULL 0x00       ///< See datasheet
#define RA8875_MCLR_ACTIVE 0x40     ///< See datasheet

#define RA8875_DCR 0x90                   ///< See datasheet
#define RA8875_DCR_LINESQUTRI_START 0x80  ///< See datasheet
#define RA8875_DCR_LINESQUTRI_STOP 0x00   ///< See datasheet
#define RA8875_DCR_LINESQUTRI_STATUS 0x80 ///< See datasheet
#define RA8875_DCR_CIRCLE_START 0x40      ///< See datasheet
#define RA8875_DCR_CIRCLE_STATUS 0x40     ///< See datasheet
#define RA8875_DCR_CIRCLE_STOP 0x00       ///< See datasheet
#define RA8875_DCR_FILL 0x20              ///< See datasheet
#define RA8875_DCR_NOFILL 0x00            ///< See datasheet
#define RA8875_DCR_DRAWLINE 0x00          ///< See datasheet
#define RA8875_DCR_DRAWTRIANGLE 0x01      ///< See datasheet
#define RA8875_DCR_DRAWSQUARE 0x10        ///< See datasheet

#define RA8875_ELLIPSE 0xA0        ///< See datasheet
#define RA8875_ELLIPSE_STATUS 0x80 ///< See datasheet

#define RA8875_MWCR0 0x40         ///< See datasheet
#define RA8875_MWCR0_GFXMODE 0x00 ///< See datasheet
#define RA8875_MWCR0_TXTMODE 0x80 ///< See datasheet
#define RA8875_MWCR0_CURSOR 0x40  ///< See datasheet
#define RA8875_MWCR0_BLINK 0x20   ///< See datasheet

#define RA8875_MWCR0_DIRMASK 0x0C ///< Bitmask for Write Direction
#define RA8875_MWCR0_LRTD 0x00    ///< Left->Right then Top->Down
#define RA8875_MWCR0_RLTD 0x04    ///< Right->Left then Top->Down
#define RA8875_MWCR0_TDLR 0x08    ///< Top->Down then Left->Right
#define RA8875_MWCR0_DTLR 0x0C    ///< Down->Top then Left->Right

#define RA8875_MWCR1 0x41 ///< See datasheet
#define RA8875_MWCR1_LMASK 0x01 ///< Layer selection bitmask
#define RA8875_MWCR1_WDMASK 0x0C ///< Write desitnation bitmask
#define RA8875_MWCR1_WD_LAYER 0x00 ////<Write to layer
#define RA8875_MWCR1_WD_CGRAM 0x04 ////<Write to CGRAM
#define RA8875_MWCR1_WD_GP 0x08 ////<Write to Graphics cursor
#define RA8875_MWCR1_WD_PATTERN 0x0C ////<Write to pattern

#define RA8875_BTCR 0x44  ///< See datasheet
#define RA8875_CURH0 0x46 ///< See datasheet
#define RA8875_CURH1 0x47 ///< See datasheet
#define RA8875_CURV0 0x48 ///< See datasheet
#define RA8875_CURV1 0x49 ///< See datasheet

#define RA8875_P1CR 0x8A         ///< See datasheet
#define RA8875_P1CR_ENABLE 0x80  ///< See datasheet
#define RA8875_P1CR_DISABLE 0x00 ///< See datasheet
#define RA8875_P1CR_CLKOUT 0x10  ///< See datasheet
#define RA8875_P1CR_PWMOUT 0x00  ///< See datasheet

#define RA8875_P1DCR 0x8B ///< See datasheet

#define RA8875_P2CR 0x8C         ///< See datasheet
#define RA8875_P2CR_ENABLE 0x80  ///< See datasheet
#define RA8875_P2CR_DISABLE 0x00 ///< See datasheet
#define RA8875_P2CR_CLKOUT 0x10  ///< See datasheet
#define RA8875_P2CR_PWMOUT 0x00  ///< See datasheet

#define RA8875_P2DCR 0x8D ///< See datasheet

#define RA8875_PWM_CLK_DIV1 0x00     ///< See datasheet
#define RA8875_PWM_CLK_DIV2 0x01     ///< See datasheet
#define RA8875_PWM_CLK_DIV4 0x02     ///< See datasheet
#define RA8875_PWM_CLK_DIV8 0x03     ///< See datasheet
#define RA8875_PWM_CLK_DIV16 0x04    ///< See datasheet
#define RA8875_PWM_CLK_DIV32 0x05    ///< See datasheet
#define RA8875_PWM_CLK_DIV64 0x06    ///< See datasheet
#define RA8875_PWM_CLK_DIV128 0x07   ///< See datasheet
#define RA8875_PWM_CLK_DIV256 0x08   ///< See datasheet
#define RA8875_PWM_CLK_DIV512 0x09   ///< See datasheet
#define RA8875_PWM_CLK_DIV1024 0x0A  ///< See datasheet
#define RA8875_PWM_CLK_DIV2048 0x0B  ///< See datasheet
#define RA8875_PWM_CLK_DIV4096 0x0C  ///< See datasheet
#define RA8875_PWM_CLK_DIV8192 0x0D  ///< See datasheet
#define RA8875_PWM_CLK_DIV16384 0x0E ///< See datasheet
#define RA8875_PWM_CLK_DIV32768 0x0F ///< See datasheet

#define RA8875_TPCR0 0x70               ///< See datasheet
#define RA8875_TPCR0_ENABLE 0x80        ///< See datasheet
#define RA8875_TPCR0_DISABLE 0x00       ///< See datasheet
#define RA8875_TPCR0_WAIT_512CLK 0x00   ///< See datasheet
#define RA8875_TPCR0_WAIT_1024CLK 0x10  ///< See datasheet
#define RA8875_TPCR0_WAIT_2048CLK 0x20  ///< See datasheet
#define RA8875_TPCR0_WAIT_4096CLK 0x30  ///< See datasheet
#define RA8875_TPCR0_WAIT_8192CLK 0x40  ///< See datasheet
#define RA8875_TPCR0_WAIT_16384CLK 0x50 ///< See datasheet
#define RA8875_TPCR0_WAIT_32768CLK 0x60 ///< See datasheet
#define RA8875_TPCR0_WAIT_65536CLK 0x70 ///< See datasheet
#define RA8875_TPCR0_WAKEENABLE 0x08    ///< See datasheet
#define RA8875_TPCR0_WAKEDISABLE 0x00   ///< See datasheet
#define RA8875_TPCR0_ADCCLK_DIV1 0x00   ///< See datasheet
#define RA8875_TPCR0_ADCCLK_DIV2 0x01   ///< See datasheet
#define RA8875_TPCR0_ADCCLK_DIV4 0x02   ///< See datasheet
#define RA8875_TPCR0_ADCCLK_DIV8 0x03   ///< See datasheet
#define RA8875_TPCR0_ADCCLK_DIV16 0x04  ///< See datasheet
#define RA8875_TPCR0_ADCCLK_DIV32 0x05  ///< See datasheet
#define RA8875_TPCR0_ADCCLK_DIV64 0x06  ///< See datasheet
#define RA8875_TPCR0_ADCCLK_DIV128 0x07 ///< See datasheet

#define RA8875_TPCR1 0x71            ///< See datasheet
#define RA8875_TPCR1_AUTO 0x00       ///< See datasheet
#define RA8875_TPCR1_MANUAL 0x40     ///< See datasheet
#define RA8875_TPCR1_VREFINT 0x00    ///< See datasheet
#define RA8875_TPCR1_VREFEXT 0x20    ///< See datasheet
#define RA8875_TPCR1_DEBOUNCE 0x04   ///< See datasheet
#define RA8875_TPCR1_NODEBOUNCE 0x00 ///< See datasheet
#define RA8875_TPCR1_IDLE 0x00       ///< See datasheet
#define RA8875_TPCR1_WAIT 0x01       ///< See datasheet
#define RA8875_TPCR1_LATCHX 0x02     ///< See datasheet
#define RA8875_TPCR1_LATCHY 0x03     ///< See datasheet

#define RA8875_TPXH 0x72  ///< See datasheet
#define RA8875_TPYH 0x73  ///< See datasheet
#define RA8875_TPXYL 0x74 ///< See datasheet

#define RA8875_INTC1 0xF0     ///< See datasheet
#define RA8875_INTC1_KEY 0x10 ///< See datasheet
#define RA8875_INTC1_DMA 0x08 ///< See datasheet
#define RA8875_INTC1_TP 0x04  ///< See datasheet
#define RA8875_INTC1_BTE 0x02 ///< See datasheet

#define RA8875_INTC2 0xF1     ///< See datasheet
#define RA8875_INTC2_KEY 0x10 ///< See datasheet
#define RA8875_INTC2_DMA 0x08 ///< See datasheet
#define RA8875_INTC2_TP 0x04  ///< See datasheet
#define RA8875_INTC2_BTE 0x02 ///< See datasheet

#define RA8875_SCROLL_BOTH 0x00   ///< See datasheet
#define RA8875_SCROLL_LAYER1 0x40 ///< See datasheet
#define RA8875_SCROLL_LAYER2 0x80 ///< See datasheet
#define RA8875_SCROLL_BUFFER 0xC0 ///< See datasheet

#define RA8875_KSCR1 0xC0 ///< See datasheet
#endif /* RA8875_H_ */
