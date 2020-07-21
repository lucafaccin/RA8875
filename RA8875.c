/*
 * RA8875.c
 *
 *  Created on: 09 lug 2020
 *      Author: luca.faccin
 */

#include "RA8875.h"
#include "string.h"
#include "header.h"

/**
 * Static variables
 */
RA8875_sizes_e _size;
uint16_t _width, _height;
uint8_t _textScale;
uint8_t _rotation;
uint8_t _voffset;
GPIO_TypeDef *_reset_port = NULL;
uint16_t _reset_pin = 0;
GPIO_TypeDef *_wait_port = NULL;
uint16_t _wait_pin = 0;
RA8875_bpp_e _color_depth;

/**
 * Helpers for PIN
 */
#define LCD_RESET_OFF _reset_port->BRR=_reset_pin;
#define LCD_RESET_ON _reset_port->BSRR=_reset_pin;


//Wait is active low in RA8875
#define LCD_WAIT_STATUS (_wait_port->IDR & _wait_pin)

static void swap (int16_t *x, int16_t *y)
{
  int16_t temp = *x;
  *x = *y;
  *y = temp;
}

/**************************************************************************/
/*!
 Performs a SW-based reset of the RA8875
 */
/**************************************************************************/
void RA8875_soft_reset (void)
{
  RA8875_write_reg (RA8875_PWRR, RA8875_PWRR_SOFTRESET);
  HAL_Delay (10);
  RA8875_write_reg (RA8875_PWRR, RA8875_PWRR_NORMAL);
}

/**************************************************************************/
/*!
 Initialise the PLL
 */
/**************************************************************************/
static void RA885_pll_init (void)
{
  if (_size == RA8875_480x80 || _size == RA8875_480x128 || _size == RA8875_480x272)
  {
    RA8875_write_reg (RA8875_PLLC1, RA8875_PLLC1_PLLDIV1 + 8);
    HAL_Delay (1);
    RA8875_write_reg (RA8875_PLLC2, RA8875_PLLC2_DIV4);
    HAL_Delay (1);
  }
  else /* (_size == RA8875_800x480) */
  {
    RA8875_write_reg (RA8875_PLLC1, RA8875_PLLC1_PLLDIV1 + 11);
    HAL_Delay (1);
    RA8875_write_reg (RA8875_PLLC2, RA8875_PLLC2_DIV4);
    HAL_Delay (1);
  }
}
/**************************************************************************/
/*!
 Initialises the driver IC (clock setup, etc.)
 */
/**************************************************************************/
static void RA8875_init (uint16_t color_depth, uint16_t mcu_bus)
{

  RA885_pll_init ();
  RA8875_write_reg (RA8875_SYSR, RA8875_SYSR_16BPP | RA8875_SYSR_MCU8);

  /* Timing values */
  uint8_t pixclk;
  uint8_t hsync_start;
  uint8_t hsync_pw;
  uint8_t hsync_finetune;
  uint8_t hsync_nondisp;
  uint8_t vsync_pw;
  uint16_t vsync_nondisp;
  uint16_t vsync_start;

  /* Set the correct values for the display being used */
  if (_size == RA8875_480x80)
  {
    pixclk = RA8875_PCSR_PDATL | RA8875_PCSR_4CLK;
    hsync_nondisp = 10;
    hsync_start = 8;
    hsync_pw = 48;
    hsync_finetune = 0;
    vsync_nondisp = 3;
    vsync_start = 8;
    vsync_pw = 10;
    _voffset = 192; // This uses the bottom 80 pixels of a 272 pixel controller
  }
  else if (_size == RA8875_480x128 || _size == RA8875_480x272)
  {
    pixclk = RA8875_PCSR_PDATL | RA8875_PCSR_4CLK;
    hsync_nondisp = 10;
    hsync_start = 8;
    hsync_pw = 48;
    hsync_finetune = 0;
    vsync_nondisp = 3;
    vsync_start = 8;
    vsync_pw = 10;
    _voffset = 0;
  }
  else // (_size == RA8875_800x480)
  {
    pixclk = RA8875_PCSR_PDATL | RA8875_PCSR_2CLK;
    hsync_nondisp = 26;
    hsync_start = 32;
    hsync_pw = 96;
    hsync_finetune = 0;
    vsync_nondisp = 32;
    vsync_start = 23;
    vsync_pw = 2;
    _voffset = 0;
  }

  /**
   * Set color dept and MCU Bus
   */
  RA8875_write_reg (RA8875_SYSR, color_depth | mcu_bus);

  RA8875_write_reg (RA8875_PCSR, pixclk);
  HAL_Delay (1);

  /* Horizontal settings registers */
  RA8875_write_reg (RA8875_HDWR, (_width / 8) - 1); // H width: (HDWR + 1) * 8 = 480
  RA8875_write_reg (RA8875_HNDFTR, RA8875_HNDFTR_DE_HIGH + hsync_finetune);
  RA8875_write_reg (RA8875_HNDR, (hsync_nondisp - hsync_finetune - 2) / 8); // H non-display: HNDR * 8 + HNDFTR + 2 = 10
  RA8875_write_reg (RA8875_HSTR, hsync_start / 8 - 1); // Hsync start: (HSTR + 1)*8
  RA8875_write_reg (RA8875_HPWR,
  RA8875_HPWR_LOW + (hsync_pw / 8 - 1)); // HSync pulse width = (HPWR+1) * 8

  /* Vertical settings registers */
  RA8875_write_reg (RA8875_VDHR0, (uint16_t) (_height - 1 + _voffset) & 0xFF);
  RA8875_write_reg (RA8875_VDHR1, (uint16_t) (_height - 1 + _voffset) >> 8);
  RA8875_write_reg (RA8875_VNDR0, vsync_nondisp - 1); // V non-display period = VNDR + 1
  RA8875_write_reg (RA8875_VNDR1, vsync_nondisp >> 8);
  RA8875_write_reg (RA8875_VSTR0, vsync_start - 1); // Vsync start position = VSTR + 1
  RA8875_write_reg (RA8875_VSTR1, vsync_start >> 8);
  RA8875_write_reg (RA8875_VPWR,
  RA8875_VPWR_LOW + vsync_pw - 1); // Vsync pulse width = VPWR + 1

  /* Set active window X */
  RA8875_write_reg (RA8875_HSAW0, 0); // horizontal start point
  RA8875_write_reg (RA8875_HSAW1, 0);
  RA8875_write_reg (RA8875_HEAW0, (uint16_t) (_width - 1) & 0xFF); // horizontal end point
  RA8875_write_reg (RA8875_HEAW1, (uint16_t) (_width - 1) >> 8);

  /* Set active window Y */
  RA8875_write_reg (RA8875_VSAW0, 0 + _voffset); // vertical start point
  RA8875_write_reg (RA8875_VSAW1, 0 + _voffset);
  RA8875_write_reg (RA8875_VEAW0, (uint16_t) (_height - 1 + _voffset) & 0xFF); // vertical end point
  RA8875_write_reg (RA8875_VEAW1, (uint16_t) (_height - 1 + _voffset) >> 8);

  /**
   * LCD WITH 2 layer
   */
  RA8875_write_reg (RA8875_DPCR, RA8875_DPCR_2_LAYER);
  /* ToDo: Setup touch panel? */

  /* Clear the entire window */
  RA8875_write_reg (RA8875_MCLR, RA8875_MCLR_START | RA8875_MCLR_FULL);
  HAL_Delay (500);
}

/**************************************************************************/
/*!
 Initialises the LCD driver and any HW required by the display
 @param s The display size, which can be either:
 'RA8875_480x80'  (3.8" displays) or
 'RA8875_480x128' (3.9" displays) or
 'RA8875_480x272' (4.3" displays) or
 'RA8875_800x480' (5" and 7" displays)
 @return 1 if we reached the end 0 if some error occurs
 */
/**************************************************************************/
uint8_t RA8875_begin (GPIO_TypeDef *reset_port, uint16_t reset_pin, GPIO_TypeDef *wait_port, uint16_t wait_pin, RA8875_sizes_e s, RA8875_bpp_e bpp)
{
  _reset_port = reset_port;
  _reset_pin = reset_pin;
  _size = s;
  _wait_port = wait_port;
  _wait_pin = wait_pin;
  _color_depth = bpp;
  //Wait 50 ms
  HAL_Delay (50);

  if (_size == RA8875_480x80)
  {
    _width = 480;
    _height = 80;
  }
  else if (_size == RA8875_480x128)
  {
    _width = 480;
    _height = 128;
  }
  else if (_size == RA8875_480x272)
  {
    _width = 480;
    _height = 272;
  }
  else if (_size == RA8875_800x480)
  {
    _width = 800;
    _height = 480;
  }
  else
  {
    return 0;
  }
  _rotation = 0;

  //Display hardware reset. Reset is active LOW
  LCD_RESET_OFF
  ;
  HAL_Delay (10);
  LCD_RESET_ON
  ;
  HAL_Delay (10);
  RA8875_soft_reset ();
  HAL_Delay (10);
  RA8875_init (((bpp == RA8875_8BPP) ? RA8875_SYSR_8BPP : RA8875_SYSR_16BPP), RA8875_SYSR_MCU8);

  return 1;
}

/************************* Low Level ***********************************/

/**
 * Wait for BUS Free
 * @return 0 if bus not free, 1 otherwise
 */
static uint8_t RA8875_BUS_FREE ()
{
  while (LCD_WAIT_STATUS == 0)
  {
  }
  return 1;
}

/**************************************************************************/
/*!
 Write data to the specified register
 @param reg Register to write to
 @param val Value to write
 */
/**************************************************************************/
void RA8875_write_reg (uint8_t reg, uint8_t val)
{
  RA8875_write_command (reg);
  RA8875_write_data (val);
}

/**************************************************************************/
/*!
 Set the register to read from
 @param reg Register to read
 @return The value
 */
/**************************************************************************/
uint8_t RA8875_read_reg (uint8_t reg)
{
  RA8875_write_command (reg);
  return RA8875_read_data ();
}

/**************************************************************************/
/*!
 Write data to the current register
 @param d Data to write
 */
/**************************************************************************/
void RA8875_write_data (uint8_t d)
{
  if (LCD_WAIT_STATUS == 0)
  {
    if (!(RA8875_BUS_FREE (1))) return;
  }
  RA8875_DATA->VAL = d;
}

/**************************************************************************/
/*!
 Read the data from the current register
 @return The Value
 */
/**************************************************************************/
uint8_t RA8875_read_data (void)
{

  uint8_t data = 0;
  //Wait is active low
  if (LCD_WAIT_STATUS == 0)
  {
    if (!(RA8875_BUS_FREE (1))) return 0;
  }

  data = RA8875_DATA->VAL;
  return data;
}

/**************************************************************************/
/*!
 Write a command to the current register
 @param d The data to write as a command
 */
/**************************************************************************/
void RA8875_write_command (uint8_t d)
{
  //Wait is active low
  if (LCD_WAIT_STATUS == 0)
  {
    if (!(RA8875_BUS_FREE (1))) return;
  }
  RA8875_REGISTER->VAL = d;
}

/**************************************************************************/
/*!
 Read the status from the current register
 @return The value
 */
/**************************************************************************/
uint8_t RA8875_read_status (void)
{
  uint8_t status = 0;
  //Wait is active low
  if (LCD_WAIT_STATUS == 0)
  {
    if (!(RA8875_BUS_FREE (1))) return 0;
  }
  status = RA8875_REGISTER->VAL;
  return status;
}

/**************************************************************************/
/*!
 Returns the display width in pixels
 @return  The 1-based display width in pixels
 */
/**************************************************************************/
uint16_t RA8875_width (void)
{
  return _width;
}

/**************************************************************************/
/*!
 Returns the display height in pixels
 @return  The 1-based display height in pixels
 */
/**************************************************************************/
uint16_t RA8875_height (void)
{
  return _height;
}

/**************************************************************************/
/*!
 Returns the current rotation (0-3)
 @return  The Rotation Setting
 */
/**************************************************************************/
int8_t RA8875_get_rotation (void)
{
  return _rotation;
}

/**************************************************************************/
/*!
 Sets the current rotation (0-3)
 @param rotation The Rotation Setting
 */
/**************************************************************************/
void RA8875_set_rotation (int8_t rotation)
{
  switch (rotation)
  {
    case 2:
      _rotation = rotation;
      break;
    default:
      _rotation = 0;
      break;
  }
}

/************************* Graphics ***********************************/

/**************************************************************************/
/*!
 Sets the display in graphics mode (as opposed to text mode)
 */
/**************************************************************************/
void RA8875_graphic_mode (void)
{
  RA8875_write_command (RA8875_MWCR0);
  uint8_t temp = RA8875_read_data ();
  temp &= ~RA8875_MWCR0_TXTMODE; // bit #7
  RA8875_write_data (temp);
}

/**
 * Set the layer where we want to write/read
 * @param layer: 0 or 1
 */
void RA8875_set_write_layer (uint8_t layer)
{
  RA8875_write_reg (RA8875_MWCR1, (RA8875_read_reg (RA8875_MWCR1) & ~RA8875_MWCR1_LMASK & ~RA8875_MWCR1_WDMASK) | layer | RA8875_MWCR1_WD_LAYER);
}

/**
 * Set the actual layer mode
 * @param layer_mode: one of the RA8875_LPTR0_DM_XXX
 */
void RA8875_set_layer_mode(uint8_t layer_mode)
{
  RA8875_write_reg (RA8875_LTPR0, (RA8875_read_reg (RA8875_LTPR0) & ~RA8875_LTPR0_DM_BITMASK)|layer_mode);

}

/**
 * Set the transparent color for layer 0
 * @param color: color in RGB565
 */
void RA8875_set_layer_transparent_color(uint16_t color)
{
  //Red component
  RA8875_write_reg(RA8875_BGTR0,(color & 0xf800) >> 11);
  //Green
  RA8875_write_reg(RA8875_BGTR1,(color & 0x07e0) >> 5);
  //Blue
  RA8875_write_reg(RA8875_BGTR2,(color & 0x001f));
}

/*!
 Set the actual active window in the specified layer
 @oaram x: x position
 @oaram y: y position
 @oaram w: width
 @oaram h: height
 */
/**************************************************************************/
void RA8875_set_active_window (uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  RA8875_write_reg (RA8875_HSAW0, x);
  RA8875_write_reg (RA8875_HSAW1, x >> 8);
  RA8875_write_reg (RA8875_VSAW0, y);
  RA8875_write_reg (RA8875_VSAW1, y >> 8);

  RA8875_write_reg (RA8875_HEAW0, (x + w - 1));
  RA8875_write_reg (RA8875_HEAW1, (x + w - 1) >> 8);
  RA8875_write_reg (RA8875_VEAW0, (y + h - 1));
  RA8875_write_reg (RA8875_VEAW1, (y + h - 1) >> 8);
}

/**
 * Set the cursor direction according to the direction
 * @param direction
 */
void RA8875_set_cursor_direction (RA8875_cursor_direction_e direction)
{
  RA8875_write_reg (RA8875_MWCR0, (RA8875_read_reg (RA8875_MWCR0) & ~RA8875_MWCR0_DIRMASK) | direction);
}

/**
 * Send the command to start the write
 */
void RA8875_write_ram (void)
{
  RA8875_write_command (RA8875_MRWC);
}

/**************************************************************************/
/*!
 Waits for screen to finish by polling the status!
 @param regname The register name to check
 @param waitflag The value to wait for the status register to match
 @return 1 if the expected status has been reached
 */
/**************************************************************************/
uint8_t RA8875_wait_poll (uint8_t regname, uint8_t waitflag)
{
  /* Wait for the command to finish */
  while (1)
  {
    uint8_t temp = RA8875_read_reg (regname);
    if (!(temp & waitflag)) return 1;
  }
  return 0; // MEMEFIX: yeah i know, unreached! - add timeout?
}

/**************************************************************************/
/*!
 Sets the current X/Y position on the display before drawing
 @param x The 0-based x location
 @param y The 0-base y location
 */
/**************************************************************************/
void RA8875_set_xy (uint16_t x, uint16_t y)
{
  RA8875_write_reg (RA8875_CURH0, x);
  RA8875_write_reg (RA8875_CURH1, x >> 8);
  RA8875_write_reg (RA8875_CURV0, y);
  RA8875_write_reg (RA8875_CURV1, y >> 8);
}

/**************************************************************************/
/*!
 HW accelerated function to push a chunk of raw pixel data
 @param num The number of pixels to push
 @param p   The pixel color to use
 */
/**************************************************************************/
void RA8875_push_pixels (uint32_t num, uint16_t p)
{

  while (num--)
  {
    if (_color_depth == RA8875_16BPP) RA8875_write_data (p >> 8);
    RA8875_write_data (p);
  }
}

/**
 *!
 HW accelerated function to push a chunk of raw pixel data
 @param p   The pixel color to use
 */
void RA8875_push_pixel (uint16_t p)
{
  if (_color_depth == RA8875_16BPP) RA8875_write_data (p >> 8);
  RA8875_write_data (p);
}
/**************************************************************************/
/*!
 Apply current rotation in the X direction
 @return the X value with current rotation applied
 */
/**************************************************************************/
static int16_t RA8875_apply_rotation_x (int16_t x)
{
  switch (_rotation)
  {
    case 2:
      x = _width - 1 - x;
      break;
  }

  return x;
}

/**************************************************************************/
/*!
 Apply current rotation in the Y direction
 @return the Y value with current rotation applied
 */
/**************************************************************************/
static int16_t RA8875_apply_rotation_y (int16_t y)
{
  switch (_rotation)
  {
    case 2:
      y = _height - 1 - y;
      break;
  }

  return y + _voffset;
}

/**************************************************************************/
/*!
 Draws a single pixel at the specified location
 @param x     The 0-based x location
 @param y     The 0-base y location
 @param color The RGB565 color to use when drawing the pixel
 */
/**************************************************************************/
void RA8875_draw_pixel (int16_t x, int16_t y, uint16_t color)
{
  x = RA8875_apply_rotation_x (x);
  y = RA8875_apply_rotation_y (y);

  RA8875_write_reg (RA8875_CURH0, x);
  RA8875_write_reg (RA8875_CURH1, x >> 8);
  RA8875_write_reg (RA8875_CURV0, y);
  RA8875_write_reg (RA8875_CURV1, y >> 8);
  RA8875_write_command (RA8875_MRWC);
  if (_color_depth == RA8875_16BPP) RA8875_write_data (color >> 8);
  RA8875_write_data (color);
}

/**************************************************************************/
/*!
 Draws a series of pixels at the specified location without the overhead
 @param p     An array of RGB565 color pixels
 @param num   The number of the pixels to draw
 @param x     The 0-based x location
 @param y     The 0-base y location
 */
/**************************************************************************/
void RA8875_draw_pixels (uint16_t *p, uint32_t num, int16_t x, int16_t y)
{
  x = RA8875_apply_rotation_x (x);
  y = RA8875_apply_rotation_y (y);

  RA8875_write_reg (RA8875_CURH0, x);
  RA8875_write_reg (RA8875_CURH1, x >> 8);
  RA8875_write_reg (RA8875_CURV0, y);
  RA8875_write_reg (RA8875_CURV1, y >> 8);

  uint8_t dir = RA8875_MWCR0_LRTD;
  if (_rotation == 2)
  {
    dir = RA8875_MWCR0_RLTD;
  }
  RA8875_write_reg (RA8875_MWCR0, (RA8875_read_reg (RA8875_MWCR0) & ~RA8875_MWCR0_DIRMASK) | dir);

  RA8875_write_command (RA8875_MRWC);
  while (num--)
  {
    uint16_t *d = p++;
    if (_color_depth == RA8875_16BPP) RA8875_write_data ((*d) >> 8);
    RA8875_write_data (*d);
  }
}

/**************************************************************************/
/*!
 Draws a HW accelerated line on the display
 @param x0    The 0-based starting x location
 @param y0    The 0-base starting y location
 @param x1    The 0-based ending x location
 @param y1    The 0-base ending y location
 @param color The RGB565 color to use when drawing the pixel
 */
/**************************************************************************/
void RA8875_draw_line (int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
  x0 = RA8875_apply_rotation_x (x0);
  y0 = RA8875_apply_rotation_y (y0);
  x1 = RA8875_apply_rotation_x (x1);
  y1 = RA8875_apply_rotation_y (y1);

  /* Set X */
  RA8875_write_command (0x91);
  RA8875_write_data (x0);
  RA8875_write_command (0x92);
  RA8875_write_data (x0 >> 8);

  /* Set Y */
  RA8875_write_command (0x93);
  RA8875_write_data (y0);
  RA8875_write_command (0x94);
  RA8875_write_data (y0 >> 8);

  /* Set X1 */
  RA8875_write_command (0x95);
  RA8875_write_data (x1);
  RA8875_write_command (0x96);
  RA8875_write_data ((x1) >> 8);

  /* Set Y1 */
  RA8875_write_command (0x97);
  RA8875_write_data (y1);
  RA8875_write_command (0x98);
  RA8875_write_data ((y1) >> 8);

  /* Set Color */
  RA8875_write_command (0x63);
  RA8875_write_data ((color & 0xf800) >> 11);
  RA8875_write_command (0x64);
  RA8875_write_data ((color & 0x07e0) >> 5);
  RA8875_write_command (0x65);
  RA8875_write_data ((color & 0x001f));

  /* Draw! */
  RA8875_write_command (RA8875_DCR);
  RA8875_write_data (0x80);

  /* Wait for the command to finish */
  RA8875_wait_poll (RA8875_DCR, RA8875_DCR_LINESQUTRI_STATUS);
}

/**************************************************************************/
/*!
 Draw a vertical line
 @param x The X position
 @param y The Y position
 @param h Height
 @param color The color
 */
/**************************************************************************/
void RA8875_draw_fast_vline (int16_t x, int16_t y, int16_t h, uint16_t color)
{
  RA8875_draw_line (x, y, x, y + h, color);
}

/**************************************************************************/
/*!
 Draw a horizontal line
 @param x The X position
 @param y The Y position
 @param w Width
 @param color The color
 */
/**************************************************************************/
void RA8875_draw_fast_hline (int16_t x, int16_t y, int16_t w, uint16_t color)
{
  RA8875_draw_line (x, y, x + w, y, color);
}
/**************************************************************************/
/*!
 Helper function for higher level circle drawing code
 */
/**************************************************************************/
static void circleHelper (int16_t x, int16_t y, int16_t r, uint16_t color, uint8_t filled)
{
  x = RA8875_apply_rotation_x (x);
  y = RA8875_apply_rotation_y (y);

  /* Set X */
  RA8875_write_command (0x99);
  RA8875_write_data (x);
  RA8875_write_command (0x9a);
  RA8875_write_data (x >> 8);

  /* Set Y */
  RA8875_write_command (0x9b);
  RA8875_write_data (y);
  RA8875_write_command (0x9c);
  RA8875_write_data (y >> 8);

  /* Set Radius */
  RA8875_write_command (0x9d);
  RA8875_write_data (r);

  /* Set Color */
  RA8875_write_command (0x63);
  RA8875_write_data ((color & 0xf800) >> 11);
  RA8875_write_command (0x64);
  RA8875_write_data ((color & 0x07e0) >> 5);
  RA8875_write_command (0x65);
  RA8875_write_data ((color & 0x001f));

  /* Draw! */
  RA8875_write_command (RA8875_DCR);
  if (filled)
  {
    RA8875_write_data (RA8875_DCR_CIRCLE_START | RA8875_DCR_FILL);
  }
  else
  {
    RA8875_write_data (RA8875_DCR_CIRCLE_START | RA8875_DCR_NOFILL);
  }

  /* Wait for the command to finish */
  RA8875_wait_poll (RA8875_DCR, RA8875_DCR_CIRCLE_STATUS);
}

/**************************************************************************/
/*!
 Helper function for higher level rectangle drawing code
 */
/**************************************************************************/
static void rectHelper (int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color, uint8_t filled)
{
  x = RA8875_apply_rotation_x (x);
  y = RA8875_apply_rotation_y (y);
  w = RA8875_apply_rotation_x (w);
  h = RA8875_apply_rotation_y (h);

  /* Set X */
  RA8875_write_command (0x91);
  RA8875_write_data (x);
  RA8875_write_command (0x92);
  RA8875_write_data (x >> 8);

  /* Set Y */
  RA8875_write_command (0x93);
  RA8875_write_data (y);
  RA8875_write_command (0x94);
  RA8875_write_data (y >> 8);

  /* Set X1 */
  RA8875_write_command (0x95);
  RA8875_write_data (w);
  RA8875_write_command (0x96);
  RA8875_write_data ((w) >> 8);

  /* Set Y1 */
  RA8875_write_command (0x97);
  RA8875_write_data (h);
  RA8875_write_command (0x98);
  RA8875_write_data ((h) >> 8);

  /* Set Color */
  RA8875_write_command (0x63);
  RA8875_write_data ((color & 0xf800) >> 11);
  RA8875_write_command (0x64);
  RA8875_write_data ((color & 0x07e0) >> 5);
  RA8875_write_command (0x65);
  RA8875_write_data ((color & 0x001f));

  /* Draw! */
  RA8875_write_command (RA8875_DCR);
  if (filled)
  {
    RA8875_write_data (0xB0);
  }
  else
  {
    RA8875_write_data (0x90);
  }

  /* Wait for the command to finish */
  RA8875_wait_poll (RA8875_DCR, RA8875_DCR_LINESQUTRI_STATUS);
}

/**************************************************************************/
/*!
 Helper function for higher level triangle drawing code
 */
/**************************************************************************/
static void triangleHelper (int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color, uint8_t filled)
{
  x0 = RA8875_apply_rotation_x (x0);
  y0 = RA8875_apply_rotation_y (y0);
  x1 = RA8875_apply_rotation_x (x1);
  y1 = RA8875_apply_rotation_y (y1);
  x2 = RA8875_apply_rotation_x (x2);
  y2 = RA8875_apply_rotation_y (y2);

  /* Set Point 0 */
  RA8875_write_command (0x91);
  RA8875_write_data (x0);
  RA8875_write_command (0x92);
  RA8875_write_data (x0 >> 8);
  RA8875_write_command (0x93);
  RA8875_write_data (y0);
  RA8875_write_command (0x94);
  RA8875_write_data (y0 >> 8);

  /* Set Point 1 */
  RA8875_write_command (0x95);
  RA8875_write_data (x1);
  RA8875_write_command (0x96);
  RA8875_write_data (x1 >> 8);
  RA8875_write_command (0x97);
  RA8875_write_data (y1);
  RA8875_write_command (0x98);
  RA8875_write_data (y1 >> 8);

  /* Set Point 2 */
  RA8875_write_command (0xA9);
  RA8875_write_data (x2);
  RA8875_write_command (0xAA);
  RA8875_write_data (x2 >> 8);
  RA8875_write_command (0xAB);
  RA8875_write_data (y2);
  RA8875_write_command (0xAC);
  RA8875_write_data (y2 >> 8);

  /* Set Color */
  RA8875_write_command (0x63);
  RA8875_write_data ((color & 0xf800) >> 11);
  RA8875_write_command (0x64);
  RA8875_write_data ((color & 0x07e0) >> 5);
  RA8875_write_command (0x65);
  RA8875_write_data ((color & 0x001f));

  /* Draw! */
  RA8875_write_command (RA8875_DCR);
  if (filled)
  {
    RA8875_write_data (0xA1);
  }
  else
  {
    RA8875_write_data (0x81);
  }

  /* Wait for the command to finish */
  RA8875_wait_poll (RA8875_DCR, RA8875_DCR_LINESQUTRI_STATUS);
}

/**************************************************************************/
/*!
 Helper function for higher level ellipse drawing code
 */
/**************************************************************************/
static void ellipseHelper (int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint16_t color, uint8_t filled)
{
  xCenter = RA8875_apply_rotation_x (xCenter);
  yCenter = RA8875_apply_rotation_y (yCenter);

  /* Set Center Point */
  RA8875_write_command (0xA5);
  RA8875_write_data (xCenter);
  RA8875_write_command (0xA6);
  RA8875_write_data (xCenter >> 8);
  RA8875_write_command (0xA7);
  RA8875_write_data (yCenter);
  RA8875_write_command (0xA8);
  RA8875_write_data (yCenter >> 8);

  /* Set Long and Short Axis */
  RA8875_write_command (0xA1);
  RA8875_write_data (longAxis);
  RA8875_write_command (0xA2);
  RA8875_write_data (longAxis >> 8);
  RA8875_write_command (0xA3);
  RA8875_write_data (shortAxis);
  RA8875_write_command (0xA4);
  RA8875_write_data (shortAxis >> 8);

  /* Set Color */
  RA8875_write_command (0x63);
  RA8875_write_data ((color & 0xf800) >> 11);
  RA8875_write_command (0x64);
  RA8875_write_data ((color & 0x07e0) >> 5);
  RA8875_write_command (0x65);
  RA8875_write_data ((color & 0x001f));

  /* Draw! */
  RA8875_write_command (0xA0);
  if (filled)
  {
    RA8875_write_data (0xC0);
  }
  else
  {
    RA8875_write_data (0x80);
  }

  /* Wait for the command to finish */
  RA8875_wait_poll (RA8875_ELLIPSE, RA8875_ELLIPSE_STATUS);
}

/**************************************************************************/
/*!
 Helper function for higher level curve drawing code
 */
/**************************************************************************/
static void curveHelper (int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint8_t curvePart, uint16_t color, uint8_t filled)
{
  xCenter = RA8875_apply_rotation_x (xCenter);
  yCenter = RA8875_apply_rotation_y (yCenter);
  curvePart = (curvePart + _rotation) % 4;

  /* Set Center Point */
  RA8875_write_command (0xA5);
  RA8875_write_data (xCenter);
  RA8875_write_command (0xA6);
  RA8875_write_data (xCenter >> 8);
  RA8875_write_command (0xA7);
  RA8875_write_data (yCenter);
  RA8875_write_command (0xA8);
  RA8875_write_data (yCenter >> 8);

  /* Set Long and Short Axis */
  RA8875_write_command (0xA1);
  RA8875_write_data (longAxis);
  RA8875_write_command (0xA2);
  RA8875_write_data (longAxis >> 8);
  RA8875_write_command (0xA3);
  RA8875_write_data (shortAxis);
  RA8875_write_command (0xA4);
  RA8875_write_data (shortAxis >> 8);

  /* Set Color */
  RA8875_write_command (0x63);
  RA8875_write_data ((color & 0xf800) >> 11);
  RA8875_write_command (0x64);
  RA8875_write_data ((color & 0x07e0) >> 5);
  RA8875_write_command (0x65);
  RA8875_write_data ((color & 0x001f));

  /* Draw! */
  RA8875_write_command (0xA0);
  if (filled)
  {
    RA8875_write_data (0xD0 | (curvePart & 0x03));
  }
  else
  {
    RA8875_write_data (0x90 | (curvePart & 0x03));
  }

  /* Wait for the command to finish */
  RA8875_wait_poll (RA8875_ELLIPSE, RA8875_ELLIPSE_STATUS);
}

/**************************************************************************/
/*!
 Helper function for higher level rounded rectangle drawing code
 */
/**************************************************************************/
static void roundRectHelper (int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color, uint8_t filled)
{
  x = RA8875_apply_rotation_x (x);
  y = RA8875_apply_rotation_y (y);
  w = RA8875_apply_rotation_x (w);
  h = RA8875_apply_rotation_y (h);
  if (x > w) swap (&x, &w);
  if (y > h) swap (&y, &h);

  /* Set X */
  RA8875_write_command (0x91);
  RA8875_write_data (x);
  RA8875_write_command (0x92);
  RA8875_write_data (x >> 8);

  /* Set Y */
  RA8875_write_command (0x93);
  RA8875_write_data (y);
  RA8875_write_command (0x94);
  RA8875_write_data (y >> 8);

  /* Set X1 */
  RA8875_write_command (0x95);
  RA8875_write_data (w);
  RA8875_write_command (0x96);
  RA8875_write_data ((w) >> 8);

  /* Set Y1 */
  RA8875_write_command (0x97);
  RA8875_write_data (h);
  RA8875_write_command (0x98);
  RA8875_write_data ((h) >> 8);

  RA8875_write_command (0xA1);
  RA8875_write_data (r);
  RA8875_write_command (0xA2);
  RA8875_write_data ((r) >> 8);

  RA8875_write_command (0xA3);
  RA8875_write_data (r);
  RA8875_write_command (0xA4);
  RA8875_write_data ((r) >> 8);

  /* Set Color */
  RA8875_write_command (0x63);
  RA8875_write_data ((color & 0xf800) >> 11);
  RA8875_write_command (0x64);
  RA8875_write_data ((color & 0x07e0) >> 5);
  RA8875_write_command (0x65);
  RA8875_write_data ((color & 0x001f));

  /* Draw! */
  RA8875_write_command (RA8875_ELLIPSE);
  if (filled)
  {
    RA8875_write_data (0xE0);
  }
  else
  {
    RA8875_write_data (0xA0);
  }

  /* Wait for the command to finish */
  RA8875_wait_poll (RA8875_ELLIPSE, RA8875_DCR_LINESQUTRI_STATUS);
}

/**************************************************************************/
/*!
 Draws a HW accelerated rectangle on the display
 @param x     The 0-based x location of the top-right corner
 @param y     The 0-based y location of the top-right corner
 @param w     The rectangle width
 @param h     The rectangle height
 @param color The RGB565 color to use when drawing the pixel
 */
/**************************************************************************/
void RA8875_draw_rect (int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
  rectHelper (x, y, x + w - 1, y + h - 1, color, 0);
}

/**************************************************************************/
/*!
 Draws a HW accelerated filled rectangle on the display
 @param x     The 0-based x location of the top-right corner
 @param y     The 0-based y location of the top-right corner
 @param w     The rectangle width
 @param h     The rectangle height
 @param color The RGB565 color to use when drawing the pixel
 */
/**************************************************************************/
void RA8875_draw_fill_rect (int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
  rectHelper (x, y, x + w - 1, y + h - 1, color, 1);
}

/**************************************************************************/
/*!
 Fills the screen with the spefied RGB565 color
 @param color The RGB565 color to use when drawing the pixel
 */
/**************************************************************************/
void RA8875_fill_screen (uint16_t color)
{
  rectHelper (0, 0, _width - 1, _height - 1, color, 1);
}

/**************************************************************************/
/*!
 Draws a HW accelerated circle on the display
 @param x     The 0-based x location of the center of the circle
 @param y     The 0-based y location of the center of the circle
 @param r     The circle's radius
 @param color The RGB565 color to use when drawing the pixel
 */
/**************************************************************************/
void RA8875_draw_circle (int16_t x, int16_t y, int16_t r, uint16_t color)
{
  circleHelper (x, y, r, color, 0);
}

/**************************************************************************/
/*!
 Draws a HW accelerated filled circle on the display
 @param x     The 0-based x location of the center of the circle
 @param y     The 0-based y location of the center of the circle
 @param r     The circle's radius
 @param color The RGB565 color to use when drawing the pixel
 */
/**************************************************************************/
void RA8875_draw_fill_circle (int16_t x, int16_t y, int16_t r, uint16_t color)
{
  circleHelper (x, y, r, color, 1);
}

/**************************************************************************/
/*!
 Draws a HW accelerated triangle on the display
 @param x0    The 0-based x location of point 0 on the triangle
 @param y0    The 0-based y location of point 0 on the triangle
 @param x1    The 0-based x location of point 1 on the triangle
 @param y1    The 0-based y location of point 1 on the triangle
 @param x2    The 0-based x location of point 2 on the triangle
 @param y2    The 0-based y location of point 2 on the triangle
 @param color The RGB565 color to use when drawing the pixel
 */
/**************************************************************************/
void RA8875_draw_triangle (int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
  triangleHelper (x0, y0, x1, y1, x2, y2, color, 0);
}

/**************************************************************************/
/*!
 Draws a HW accelerated filled triangle on the display
 @param x0    The 0-based x location of point 0 on the triangle
 @param y0    The 0-based y location of point 0 on the triangle
 @param x1    The 0-based x location of point 1 on the triangle
 @param y1    The 0-based y location of point 1 on the triangle
 @param x2    The 0-based x location of point 2 on the triangle
 @param y2    The 0-based y location of point 2 on the triangle
 @param color The RGB565 color to use when drawing the pixel
 */
/**************************************************************************/
void RA8875_draw_fill_triangle (int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
  triangleHelper (x0, y0, x1, y1, x2, y2, color, 1);
}

/**************************************************************************/
/*!
 Draws a HW accelerated ellipse on the display
 @param xCenter   The 0-based x location of the ellipse's center
 @param yCenter   The 0-based y location of the ellipse's center
 @param longAxis  The size in pixels of the ellipse's long axis
 @param shortAxis The size in pixels of the ellipse's short axis
 @param color     The RGB565 color to use when drawing the pixel
 */
/**************************************************************************/
void RA8875_draw_draw_ellipse (int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint16_t color)
{
  ellipseHelper (xCenter, yCenter, longAxis, shortAxis, color, 0);
}

/**************************************************************************/
/*!
 Draws a HW accelerated filled ellipse on the display
 @param xCenter   The 0-based x location of the ellipse's center
 @param yCenter   The 0-based y location of the ellipse's center
 @param longAxis  The size in pixels of the ellipse's long axis
 @param shortAxis The size in pixels of the ellipse's short axis
 @param color     The RGB565 color to use when drawing the pixel
 */
/**************************************************************************/
void RA8875_draw_fill_ellipse (int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint16_t color)
{
  ellipseHelper (xCenter, yCenter, longAxis, shortAxis, color, 1);
}

/**************************************************************************/
/*!
 Draws a HW accelerated curve on the display
 @param xCenter   The 0-based x location of the ellipse's center
 @param yCenter   The 0-based y location of the ellipse's center
 @param longAxis  The size in pixels of the ellipse's long axis
 @param shortAxis The size in pixels of the ellipse's short axis
 @param curvePart The corner to draw, where in clock-wise motion:
 0 = 180-270°
 1 = 270-0°
 2 = 0-90°
 3 = 90-180°
 @param color     The RGB565 color to use when drawing the pixel
 */
/**************************************************************************/
void RA8875_draw_curve (int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint8_t curvePart, uint16_t color)
{
  curveHelper (xCenter, yCenter, longAxis, shortAxis, curvePart, color, 0);
}

/**************************************************************************/
/*!
 Draws a HW accelerated filled curve on the display
 @param xCenter   The 0-based x location of the ellipse's center
 @param yCenter   The 0-based y location of the ellipse's center
 @param longAxis  The size in pixels of the ellipse's long axis
 @param shortAxis The size in pixels of the ellipse's short axis
 @param curvePart The corner to draw, where in clock-wise motion:
 0 = 180-270°
 1 = 270-0°
 2 = 0-90°
 3 = 90-180°
 @param color     The RGB565 color to use when drawing the pixel
 */
/**************************************************************************/
void RA8875_draw_fill_curve (int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint8_t curvePart, uint16_t color)
{
  curveHelper (xCenter, yCenter, longAxis, shortAxis, curvePart, color, 1);
}

/**************************************************************************/
/*!
 Draws a HW accelerated rounded rectangle on the display
 @param x   The 0-based x location of the rectangle's upper left corner
 @param y   The 0-based y location of the rectangle's upper left corner
 @param w   The size in pixels of the rectangle's width
 @param h   The size in pixels of the rectangle's height
 @param r   The radius of the curves in the corners of the rectangle
 @param color  The RGB565 color to use when drawing the pixel
 */
/**************************************************************************/
void RA8875_draw_round_rect (int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color)
{
  roundRectHelper (x, y, x + w, y + h, r, color, 0);
}

/**************************************************************************/
/*!
 Draws a HW accelerated filled rounded rectangle on the display
 @param x   The 0-based x location of the rectangle's upper left corner
 @param y   The 0-based y location of the rectangle's upper left corner
 @param w   The size in pixels of the rectangle's width
 @param h   The size in pixels of the rectangle's height
 @param r   The radius of the curves in the corners of the rectangle
 @param color  The RGB565 color to use when drawing the pixel
 */
/**************************************************************************/
void RA8875_draw_fill_round_rect (int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color)
{
  roundRectHelper (x, y, x + w, y + h, r, color, 1);
}

/**************************************************************************/
/*!
 Set the scroll window
 @param x  X position of the scroll window
 @param y  Y position of the scroll window
 @param w  Width of the Scroll Window
 @param h  Height of the Scroll window
 @param mode Layer to Scroll
 */
/**************************************************************************/
void RA8875_set_scroll_window (int16_t x, int16_t y, int16_t w, int16_t h, uint8_t mode)
{
  // Horizontal Start point of Scroll Window
  RA8875_write_command (0x38);
  RA8875_write_data (x);
  RA8875_write_command (0x39);
  RA8875_write_data (x >> 8);

  // Vertical Start Point of Scroll Window
  RA8875_write_command (0x3a);
  RA8875_write_data (y);
  RA8875_write_command (0x3b);
  RA8875_write_data (y >> 8);

  // Horizontal End Point of Scroll Window
  RA8875_write_command (0x3c);
  RA8875_write_data (x + w);
  RA8875_write_command (0x3d);
  RA8875_write_data ((x + w) >> 8);

  // Vertical End Point of Scroll Window
  RA8875_write_command (0x3e);
  RA8875_write_data (y + h);
  RA8875_write_command (0x3f);
  RA8875_write_data ((y + h) >> 8);

  // Scroll function setting
  RA8875_write_command (0x52);
  RA8875_write_data (mode);
}

/**************************************************************************/
/*!
 Scroll in the X direction
 @param dist The distance to scroll
 */
/**************************************************************************/
void RA8875_scroll_x (int16_t dist)
{
  RA8875_write_command (0x24);
  RA8875_write_data (dist);
  RA8875_write_command (0x25);
  RA8875_write_data (dist >> 8);
}

/**************************************************************************/
/*!
 Scroll in the Y direction
 @param dist The distance to scroll
 */
/**************************************************************************/
void RA8875_scroll_y (int16_t dist)
{
  RA8875_write_command (0x26);
  RA8875_write_data (dist);
  RA8875_write_command (0x27);
  RA8875_write_data (dist >> 8);
}

/************************* Text Mode ***********************************/

/**************************************************************************/
/*!
 Sets the display in text mode (as opposed to graphics mode)
 */
/**************************************************************************/
void RA8875_text_mode (void)
{
  /* Set text mode */
  RA8875_write_command (RA8875_MWCR0);
  uint8_t temp = RA8875_read_data ();
  temp |= RA8875_MWCR0_TXTMODE; // Set bit 7
  RA8875_write_data (temp);

  /* Select the internal (ROM) font */
  RA8875_write_command (0x21);
  temp = RA8875_read_data ();
  temp &= ~((1 << 7) | (1 << 5)); // Clear bits 7 and 5
  RA8875_write_data (temp);
}

/**************************************************************************/
/*!
 Sets the display in text mode (as opposed to graphics mode)
 @param x The x position of the cursor (in pixels, 0..1023)
 @param y The y position of the cursor (in pixels, 0..511)
 */
/**************************************************************************/
void RA8875_text_cursor_position (uint16_t x, uint16_t y)
{
  x = RA8875_apply_rotation_x (x);
  y = RA8875_apply_rotation_y (y);

  /* Set cursor location */
  RA8875_write_command (0x2A);
  RA8875_write_data (x & 0xFF);
  RA8875_write_command (0x2B);
  RA8875_write_data (x >> 8);
  RA8875_write_command (0x2C);
  RA8875_write_data (y & 0xFF);
  RA8875_write_command (0x2D);
  RA8875_write_data (y >> 8);
}

/**************************************************************************/
/*!
 Sets the fore and background color when rendering text
 @param foreColor The RGB565 color to use when rendering the text
 @param bgColor   The RGB565 colot to use for the background
 */
/**************************************************************************/
void RA8875_text_color (uint16_t foreColor, uint16_t bgColor)
{
  /* Set Fore Color */
  RA8875_write_command (0x63);
  RA8875_write_data ((foreColor & 0xf800) >> 11);
  RA8875_write_command (0x64);
  RA8875_write_data ((foreColor & 0x07e0) >> 5);
  RA8875_write_command (0x65);
  RA8875_write_data ((foreColor & 0x001f));

  /* Set Background Color */
  RA8875_write_command (0x60);
  RA8875_write_data ((bgColor & 0xf800) >> 11);
  RA8875_write_command (0x61);
  RA8875_write_data ((bgColor & 0x07e0) >> 5);
  RA8875_write_command (0x62);
  RA8875_write_data ((bgColor & 0x001f));

  /* Clear transparency flag */
  RA8875_write_command (0x22);
  uint8_t temp = RA8875_read_data ();
  temp &= ~(1 << 6); // Clear bit 6
  RA8875_write_data (temp);
}

/**************************************************************************/
/*!
 Sets the fore color when rendering text with a transparent bg
 @param foreColor The RGB565 color to use when rendering the text
 */
/**************************************************************************/
void RA8875_text_transparent_color (uint16_t foreColor)
{
  /* Set Fore Color */
  RA8875_write_command (0x63);
  RA8875_write_data ((foreColor & 0xf800) >> 11);
  RA8875_write_command (0x64);
  RA8875_write_data ((foreColor & 0x07e0) >> 5);
  RA8875_write_command (0x65);
  RA8875_write_data ((foreColor & 0x001f));

  /* Set transparency flag */
  RA8875_write_command (0x22);
  uint8_t temp = RA8875_read_data ();
  temp |= (1 << 6); // Set bit 6
  RA8875_write_data (temp);
}

/**************************************************************************/
/*!
 Sets the text enlarge settings, using one of the following values:
 0 = 1x zoom
 1 = 2x zoom
 2 = 3x zoom
 3 = 4x zoom
 @param scale   The zoom factor (0..3 for 1-4x zoom)
 */
/**************************************************************************/
void RA8875_text_scale (uint8_t scale)
{
  if (scale > 3) scale = 3; // highest setting is 3

  /* Set font size flags */
  RA8875_write_command (0x22);
  uint8_t temp = RA8875_read_data ();
  temp &= ~(0xF); // Clears bits 0..3
  temp |= scale << 2;
  temp |= scale;

  RA8875_write_data (temp);

  _textScale = scale;
}

/**************************************************************************/
/*!
 Enable Cursor Visibility and Blink
 Here we set bits 6 and 5 in 40h
 As well as the set the blink rate in 44h
 The rate is 0 through max 255
 the lower the number the faster it blinks (00h is 1 frame time,
 FFh is 256 Frames time.
 Blink Time (sec) = BTCR[44h]x(1/Frame_rate)
 @param rate The frame rate to blink
 */
/**************************************************************************/

void RA8875_cursor_blink (uint8_t rate)
{

  RA8875_write_command (RA8875_MWCR0);
  uint8_t temp = RA8875_read_data ();
  temp |= RA8875_MWCR0_CURSOR;
  RA8875_write_data (temp);

  RA8875_write_command (RA8875_MWCR0);
  temp = RA8875_read_data ();
  temp |= RA8875_MWCR0_BLINK;
  RA8875_write_data (temp);

  if (rate > 255) rate = 255;
  RA8875_write_command (RA8875_BTCR);
  RA8875_write_data (rate);
}

/**************************************************************************/
/*!
 Renders some text on the screen when in text mode
 @param buffer    The buffer containing the characters to render
 @param len       The size of the buffer in bytes
 */
/**************************************************************************/
void RA8875_text_write (const char *buffer, uint16_t len)
{
  if (len == 0) len = strlen (buffer);
  RA8875_write_command (RA8875_MRWC);
  for (uint16_t i = 0; i < len; i++)
  {
    RA8875_write_data (buffer[i]);
    if (_textScale > 0) HAL_Delay (1);
  }
}

/************************* Mid Level ***********************************/

/**************************************************************************/
/*!
 Set the Extra General Purpose IO Register
 @param on Whether to turn Extra General Purpose IO on or not
 */
/**************************************************************************/
void GPIOX (uint8_t on)
{
  if (on) RA8875_write_reg (RA8875_GPIOX, 1);
  else RA8875_write_reg (RA8875_GPIOX, 0);
}

/**************************************************************************/
/*!
 * Disable the keyscan
 */
/**************************************************************************/
void RA8875_disable_keyscan ()
{
  RA8875_write_reg (RA8875_KSCR1, 0x0);
}

/**************************************************************************/
/*!
 Set the duty cycle of the PWM 1 Clock
 @param p The duty Cycle (0-255)
 */
/**************************************************************************/
void RA8875_pwm1_out (uint8_t p)
{
  RA8875_write_reg (RA8875_P1DCR, p);
}

/**************************************************************************/
/*!
 Set the duty cycle of the PWM 2 Clock
 @param p The duty Cycle (0-255)
 */
/**************************************************************************/
void RA8875_pwm2_out (uint8_t p)
{
  RA8875_write_reg (RA8875_P2DCR, p);
}

/**************************************************************************/
/*!
 Configure the PWM 1 Clock
 @param on Whether to enable the clock
 @param clock The Clock Divider
 */
/**************************************************************************/
void RA8875_pwm1_config (uint8_t on, uint8_t clock)
{
  if (on)
  {
    RA8875_write_reg (RA8875_P1CR, RA8875_P1CR_ENABLE | (clock & 0xF));
  }
  else
  {
    RA8875_write_reg (RA8875_P1CR, RA8875_P1CR_DISABLE | (clock & 0xF));
  }
}

/**************************************************************************/
/*!
 Configure the PWM 2 Clock
 @param on Whether to enable the clock
 @param clock The Clock Divider
 */
/**************************************************************************/
void RA8875_pwm2_config (uint8_t on, uint8_t clock)
{
  if (on)
  {
    RA8875_write_reg (RA8875_P2CR, RA8875_P2CR_ENABLE | (clock & 0xF));
  }
  else
  {
    RA8875_write_reg (RA8875_P2CR, RA8875_P2CR_DISABLE | (clock & 0xF));
  }
}

/**************************************************************************/
/*!
 Enables or disables the on-chip touch screen controller
 @param on Whether to turn touch sensing on or not
 */
/**************************************************************************/
void RA8875_enable_touch (uint8_t on)
{
  uint8_t adcClk = (uint8_t) RA8875_TPCR0_ADCCLK_DIV4;

  if (_size == RA8875_800x480) // match up touch size with LCD size
  adcClk = (uint8_t) RA8875_TPCR0_ADCCLK_DIV16;

  if (on)
  {
    /* Enable Touch Panel (Reg 0x70) */
    RA8875_write_reg (RA8875_TPCR0, RA8875_TPCR0_ENABLE | RA8875_TPCR0_WAIT_4096CLK |
    RA8875_TPCR0_WAKEENABLE | adcClk); // 10mhz max!
    /* Set Auto Mode      (Reg 0x71) */
    RA8875_write_reg (RA8875_TPCR1, RA8875_TPCR1_AUTO |
    // RA8875_TPCR1_VREFEXT |
			  RA8875_TPCR1_DEBOUNCE);
    /* Enable TP INT */
    RA8875_write_reg (RA8875_INTC1, RA8875_read_reg (RA8875_INTC1) | RA8875_INTC1_TP);
  }
  else
  {
    /* Disable TP INT */
    RA8875_write_reg (RA8875_INTC1, RA8875_read_reg (RA8875_INTC1) & ~RA8875_INTC1_TP);
    /* Disable Touch Panel (Reg 0x70) */
    RA8875_write_reg (RA8875_TPCR0, RA8875_TPCR0_DISABLE);
  }
}

/**************************************************************************/
/*!
 Checks if a touch event has occured
 @return  1 is a touch event has occured (reading it via
 touchRead() will clear the interrupt in memory)
 */
/**************************************************************************/
uint8_t RA8875_touched (void)
{
  if (RA8875_read_reg (RA8875_INTC2) & RA8875_INTC2_TP) return 1;
  return 0;
}

/**************************************************************************/
/*!
 Reads the last touch event
 @param x  Pointer to the uint16_t field to assign the raw X value
 @param y  Pointer to the uint16_t field to assign the raw Y value
 @return 1 if successful
 @note Calling this function will clear the touch panel interrupt on
 the RA8875, resetting the flag used by the 'touched' function
 */
/**************************************************************************/
uint8_t RA8875_touch_read (uint16_t *x, uint16_t *y)
{
  uint16_t tx, ty;
  uint8_t temp;

  tx = RA8875_read_reg (RA8875_TPXH);
  ty = RA8875_read_reg (RA8875_TPYH);
  temp = RA8875_read_reg (RA8875_TPXYL);
  tx <<= 2;
  ty <<= 2;
  tx |= temp & 0x03;        // get the bottom x bits
  ty |= (temp >> 2) & 0x03; // get the bottom y bits

  *x = tx;
  *y = ty;

  /* Clear TP INT Status */
  RA8875_write_reg (RA8875_INTC2, RA8875_INTC2_TP);

  return 1;
}

/**************************************************************************/
/*!
 Turns the display on or off
 @param on Whether to turn the display on or not
 */
/**************************************************************************/
void RA8875_display_on (uint8_t on)
{
  if (on) RA8875_write_reg (RA8875_PWRR, RA8875_PWRR_NORMAL | RA8875_PWRR_DISPON);
  else RA8875_write_reg (RA8875_PWRR, RA8875_PWRR_NORMAL | RA8875_PWRR_DISPOFF);
}

/**************************************************************************/
/*!
 Puts the display in sleep mode, or disables sleep mode if enabled
 @param sleep Whether to sleep or not
 */
/**************************************************************************/
void RA8875_sleep (uint8_t sleep)
{
  if (sleep) RA8875_write_reg (RA8875_PWRR, RA8875_PWRR_DISPOFF | RA8875_PWRR_SLEEP);
  else RA8875_write_reg (RA8875_PWRR, RA8875_PWRR_DISPOFF);
}
