#ifndef SSD1306_H
#define SSD1306_H

/* SSD1306 0.96" 128x64 I2C OLED driver - software (bit-bang) I2C on PB6/PB7.
 * Pins are configured as open-drain GPIO outputs by CubeMX (MX_GPIO_Init).
 * The module's onboard pull-ups (plus internal pull-ups) hold the bus high;
 * reading SDA in open-drain mode returns the real line level (ACK detection).
 *
 * Usage:  OLED_Init() once (before the scheduler is fine, <200ms),
 *         OLED_DrawString()/OLED_DrawHBar() into the framebuffer,
 *         OLED_Flush() to push only the changed (dirty) pages over I2C.
 *
 * Only DisplayTask calls the I2C routines after init - it is the sole owner
 * of the bus, and never touches the UART (CommTask is the only TX).
 */

#include "main.h"

#define OLED_ADDR   0x3C        /* 7-bit I2C address (0x78 with R/W bit) */
#define OLED_COLS   128
#define OLED_PAGES  8           /* 128x64 -> 8 pages of 8 pixel rows */

void OLED_Init(void);                       /* DWT + reset delay + init sequence + clear */
void OLED_Clear(void);                      /* clear framebuffer, mark all pages dirty   */
void OLED_ClearPage(uint8_t page);          /* clear one 8-row page                       */
void OLED_DrawChar(uint8_t page, uint8_t col, char c);      /* 6x8 glyph                */
void OLED_DrawString(uint8_t page, uint8_t col, const char *s);
void OLED_DrawHBar(uint8_t page, uint8_t col, uint8_t len); /* solid bar, 1px high      */
void OLED_Flush(void);                      /* push dirty pages over I2C                  */

extern volatile uint8_t g_oled_nack;        /* 1 if the last I2C byte was NACK'd          */

#endif /* SSD1306_H */
