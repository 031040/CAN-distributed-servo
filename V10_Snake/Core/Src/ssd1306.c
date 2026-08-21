/* SSD1306 0.96" 128x64 I2C OLED driver - software (bit-bang) I2C.
 *
 * Why software I2C on the F103: the STM32F1 hardware I2C peripheral is known
 * to lock up (BUSY flag) and has documented errata; with no logic analyzer a
 * hardware-I2C deadlock is a black box. A bit-banged bus is deterministic -
 * if the wiring is right it works. Bonus: it forces a real understanding of
 * the I2C protocol (START/STOP/ACK/address/control bytes) which is a good
 * interview story.
 *
 * Bus: PB6 = SCL, PB7 = SDA, both open-drain outputs (CubeMX MX_GPIO_Init).
 * The SSD1306 module has 4.7k pull-ups on the board; internal pull-ups are
 * enabled as a backup. In open-drain output mode, HAL_GPIO_ReadPin(SDA)
 * returns the real line level, so the ACK bit is read back for free.
 *
 * Timing: SysTick belongs to FreeRTOS (1ms) and HAL_Delay is 1ms granularity,
 * so microseconds come from the DWT cycle counter (8 MHz -> 8 cyc/us).
 * Each I2C phase is stretched by timer/HAL-call overhead and by higher
 * priority tasks preempting DisplayTask mid-byte - that is safe: the slave
 * simply waits on SCL (master-controlled clock stretching).
 */

#include "ssd1306.h"
#include <string.h>

/* ------------------------------------------------------------------------- */
/* Tunables                                                                  */
/* ------------------------------------------------------------------------- */
#define I2C_HALF_DELAY_US   2   /* ~200kHz; raise to 3-4 if the screen is   */
                                /* garbled on long wires, lower for speed   */

#define OLED_PORT   GPIOB
#define OLED_SCL    GPIO_PIN_6
#define OLED_SDA    GPIO_PIN_7

#define SCL_H()  HAL_GPIO_WritePin(OLED_PORT, OLED_SCL, GPIO_PIN_SET)
#define SCL_L()  HAL_GPIO_WritePin(OLED_PORT, OLED_SCL, GPIO_PIN_RESET)
#define SDA_H()  HAL_GPIO_WritePin(OLED_PORT, OLED_SDA, GPIO_PIN_SET)
#define SDA_L()  HAL_GPIO_WritePin(OLED_PORT, OLED_SDA, GPIO_PIN_RESET)

/* ------------------------------------------------------------------------- */
/* Module state                                                              */
/* ------------------------------------------------------------------------- */
volatile uint8_t g_oled_nack = 0;     /* set when the last I2C byte was NACK'd */

static uint8_t fb[OLED_PAGES][OLED_COLS];   /* 1KB framebuffer */
static uint8_t dirty_mask = 0xFF;           /* pages to push on next Flush   */

/* ------------------------------------------------------------------------- */
/* Microsecond delay via DWT cycle counter (8MHz -> 8 cycles per us)         */
/* ------------------------------------------------------------------------- */
static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * 8u;
    while ((DWT->CYCCNT - start) < ticks) { }
}

static void dwt_enable(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  /* TRCENA must go first */
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

/* ------------------------------------------------------------------------- */
/* I2C bit-bang primitives                                                   */
/* ------------------------------------------------------------------------- */
static void i2c_delay(void) { delay_us(I2C_HALF_DELAY_US); }

static void i2c_start(void)
{
    SDA_H(); SCL_H(); i2c_delay();   /* idle: bus high                      */
    SDA_L(); i2c_delay();            /* SDA falls while SCL high -> START   */
    SCL_L(); i2c_delay();
}

static void i2c_stop(void)
{
    SDA_L(); i2c_delay();
    SCL_H(); i2c_delay();
    SDA_H(); i2c_delay();            /* SDA rises while SCL high -> STOP    */
}

/* One byte out, returns 1 if the slave NACK'd (open-drain: read SDA back). */
static uint8_t i2c_write_byte(uint8_t byte)
{
    for (int8_t i = 7; i >= 0; i--)          /* MSB first */
    {
        if (byte & (1u << i)) SDA_H(); else SDA_L();
        i2c_delay();                         /* data setup while SCL low     */
        SCL_H(); i2c_delay();                /* slave latches on rising edge */
        SCL_L(); i2c_delay();
    }
    /* ACK window: release SDA and sample it on the 9th clock */
    SDA_H(); i2c_delay();
    SCL_H(); i2c_delay();
    uint8_t nack = (HAL_GPIO_ReadPin(OLED_PORT, OLED_SDA) == GPIO_PIN_SET) ? 1u : 0u;
    SCL_L(); i2c_delay();
    if (nack) g_oled_nack = 1;
    return nack;
}

/* ------------------------------------------------------------------------- */
/* SSD1306 transactions: command / data burst (one START-STOP per call)      */
/* ------------------------------------------------------------------------- */
static void oled_write_cmd(uint8_t cmd)
{
    g_oled_nack = 0;
    i2c_start();
    i2c_write_byte((uint8_t)(OLED_ADDR << 1));   /* write bit 0 */
    i2c_write_byte(0x00);                        /* control: command stream */
    i2c_write_byte(cmd);
    i2c_stop();
}

static void oled_write_data_burst(const uint8_t *buf, uint16_t len)
{
    g_oled_nack = 0;
    i2c_start();
    i2c_write_byte((uint8_t)(OLED_ADDR << 1));
    i2c_write_byte(0x40);                        /* control: data stream    */
    for (uint16_t i = 0; i < len; i++)
        i2c_write_byte(buf[i]);
    i2c_stop();
}

/* ------------------------------------------------------------------------- */
/* 6x8 ASCII font (public domain, bit 0 = top row), 0x20..0x7F               */
/* ------------------------------------------------------------------------- */
static const uint8_t font6x8[96][6] = {
    {0x00,0x00,0x00,0x00,0x00,0x00}, /* sp  */
    {0x00,0x00,0x00,0x2f,0x00,0x00}, /* !   */
    {0x00,0x00,0x07,0x00,0x07,0x00}, /* "   */
    {0x00,0x14,0x7f,0x14,0x7f,0x14}, /* #   */
    {0x00,0x24,0x2a,0x7f,0x2a,0x12}, /* $   */
    {0x00,0x62,0x64,0x08,0x13,0x23}, /* %   */
    {0x00,0x36,0x49,0x55,0x22,0x50}, /* &   */
    {0x00,0x00,0x05,0x03,0x00,0x00}, /* '   */
    {0x00,0x00,0x1c,0x22,0x41,0x00}, /* (   */
    {0x00,0x00,0x41,0x22,0x1c,0x00}, /* )   */
    {0x00,0x14,0x08,0x3e,0x08,0x14}, /* *   */
    {0x00,0x08,0x08,0x3e,0x08,0x08}, /* +   */
    {0x00,0x00,0x00,0xa0,0x60,0x00}, /* ,   */
    {0x00,0x08,0x08,0x08,0x08,0x08}, /* -   */
    {0x00,0x00,0x60,0x60,0x00,0x00}, /* .   */
    {0x00,0x20,0x10,0x08,0x04,0x02}, /* /   */
    {0x00,0x3e,0x51,0x49,0x45,0x3e}, /* 0   */
    {0x00,0x00,0x42,0x7f,0x40,0x00}, /* 1   */
    {0x00,0x42,0x61,0x51,0x49,0x46}, /* 2   */
    {0x00,0x21,0x41,0x45,0x4b,0x31}, /* 3   */
    {0x00,0x18,0x14,0x12,0x7f,0x10}, /* 4   */
    {0x00,0x27,0x45,0x45,0x45,0x39}, /* 5   */
    {0x00,0x3c,0x4a,0x49,0x49,0x30}, /* 6   */
    {0x00,0x01,0x71,0x09,0x05,0x03}, /* 7   */
    {0x00,0x36,0x49,0x49,0x49,0x36}, /* 8   */
    {0x00,0x06,0x49,0x49,0x29,0x1e}, /* 9   */
    {0x00,0x00,0x36,0x36,0x00,0x00}, /* :   */
    {0x00,0x00,0x56,0x36,0x00,0x00}, /* ;   */
    {0x00,0x08,0x14,0x22,0x41,0x00}, /* <   */
    {0x00,0x14,0x14,0x14,0x14,0x14}, /* =   */
    {0x00,0x00,0x41,0x22,0x14,0x08}, /* >   */
    {0x00,0x02,0x01,0x51,0x09,0x06}, /* ?   */
    {0x00,0x32,0x49,0x79,0x41,0x3e}, /* @   */
    {0x00,0x7e,0x11,0x11,0x11,0x7e}, /* A   */
    {0x00,0x7f,0x49,0x49,0x49,0x36}, /* B   */
    {0x00,0x3e,0x41,0x41,0x41,0x22}, /* C   */
    {0x00,0x7f,0x41,0x41,0x22,0x1c}, /* D   */
    {0x00,0x7f,0x49,0x49,0x49,0x41}, /* E   */
    {0x00,0x7f,0x09,0x09,0x09,0x01}, /* F   */
    {0x00,0x3e,0x41,0x49,0x49,0x7a}, /* G   */
    {0x00,0x7f,0x08,0x08,0x08,0x7f}, /* H   */
    {0x00,0x00,0x41,0x7f,0x41,0x00}, /* I   */
    {0x00,0x20,0x40,0x41,0x3f,0x01}, /* J   */
    {0x00,0x7f,0x08,0x14,0x22,0x41}, /* K   */
    {0x00,0x7f,0x40,0x40,0x40,0x40}, /* L   */
    {0x00,0x7f,0x02,0x0c,0x02,0x7f}, /* M   */
    {0x00,0x7f,0x04,0x08,0x10,0x7f}, /* N   */
    {0x00,0x3e,0x41,0x41,0x41,0x3e}, /* O   */
    {0x00,0x7f,0x09,0x09,0x09,0x06}, /* P   */
    {0x00,0x3e,0x41,0x51,0x21,0x5e}, /* Q   */
    {0x00,0x7f,0x09,0x19,0x29,0x46}, /* R   */
    {0x00,0x46,0x49,0x49,0x49,0x31}, /* S   */
    {0x00,0x01,0x01,0x7f,0x01,0x01}, /* T   */
    {0x00,0x3f,0x40,0x40,0x40,0x3f}, /* U   */
    {0x00,0x1f,0x20,0x40,0x20,0x1f}, /* V   */
    {0x00,0x3f,0x40,0x38,0x40,0x3f}, /* W   */
    {0x00,0x63,0x14,0x08,0x14,0x63}, /* X   */
    {0x00,0x07,0x08,0x70,0x08,0x07}, /* Y   */
    {0x00,0x61,0x51,0x49,0x45,0x43}, /* Z   */
    {0x00,0x00,0x7f,0x41,0x41,0x00}, /* [   */
    {0x00,0x02,0x04,0x08,0x10,0x20}, /* \   */
    {0x00,0x00,0x41,0x41,0x7f,0x00}, /* ]   */
    {0x00,0x04,0x02,0x01,0x02,0x04}, /* ^   */
    {0x00,0x40,0x40,0x40,0x40,0x40}, /* _   */
    {0x00,0x00,0x01,0x02,0x04,0x00}, /* `   */
    {0x00,0x20,0x54,0x54,0x54,0x78}, /* a   */
    {0x00,0x7f,0x48,0x44,0x44,0x38}, /* b   */
    {0x00,0x38,0x44,0x44,0x44,0x20}, /* c   */
    {0x00,0x38,0x44,0x44,0x48,0x7f}, /* d   */
    {0x00,0x38,0x54,0x54,0x54,0x18}, /* e   */
    {0x00,0x08,0x7e,0x09,0x01,0x02}, /* f   */
    {0x00,0x0c,0x52,0x52,0x52,0x3e}, /* g   */
    {0x00,0x7f,0x08,0x04,0x04,0x78}, /* h   */
    {0x00,0x00,0x44,0x7d,0x40,0x00}, /* i   */
    {0x00,0x20,0x40,0x44,0x3d,0x00}, /* j   */
    {0x00,0x7f,0x10,0x28,0x44,0x00}, /* k   */
    {0x00,0x00,0x41,0x7f,0x40,0x00}, /* l   */
    {0x00,0x7c,0x04,0x18,0x04,0x78}, /* m   */
    {0x00,0x7c,0x08,0x04,0x04,0x78}, /* n   */
    {0x00,0x38,0x44,0x44,0x44,0x38}, /* o   */
    {0x00,0x7c,0x14,0x14,0x14,0x08}, /* p   */
    {0x00,0x08,0x14,0x14,0x18,0x7c}, /* q   */
    {0x00,0x7c,0x08,0x04,0x04,0x08}, /* r   */
    {0x00,0x48,0x54,0x54,0x54,0x20}, /* s   */
    {0x00,0x04,0x3f,0x44,0x40,0x20}, /* t   */
    {0x00,0x3c,0x40,0x40,0x20,0x7c}, /* u   */
    {0x00,0x1c,0x20,0x40,0x20,0x1c}, /* v   */
    {0x00,0x3c,0x40,0x30,0x40,0x3c}, /* w   */
    {0x00,0x44,0x28,0x10,0x28,0x44}, /* x   */
    {0x00,0x0c,0x50,0x50,0x50,0x3c}, /* y   */
    {0x00,0x44,0x64,0x54,0x4c,0x44}, /* z   */
    {0x00,0x00,0x08,0x36,0x41,0x00}, /* {   */
    {0x00,0x00,0x00,0x7f,0x00,0x00}, /* |   */
    {0x00,0x00,0x41,0x36,0x08,0x00}, /* }   */
    {0x00,0x10,0x08,0x08,0x10,0x08}, /* ~   */
};

/* ------------------------------------------------------------------------- */
/* Public API                                                                */
/* ------------------------------------------------------------------------- */
void OLED_Init(void)
{
    dwt_enable();

    /* Let the display's internal power-on reset finish (4-pin module has no
     * RES pin wired to us, so we rely on power-on reset + a wait). */
    HAL_Delay(50);

    static const uint8_t init_seq[] = {
        0xAE,             /* display OFF                                 */
        0xD5, 0x80,       /* oscillator frequency                        */
        0xA8, 0x3F,       /* multiplex ratio 1/64 (128x64)               */
        0xD3, 0x00,       /* display offset 0                            */
        0x40,             /* start line 0                                */
        0x8D, 0x14,       /* charge pump ENABLE - without this the screen
                           * stays dark: the #1 cause of a blank OLED     */
        0x20, 0x02,       /* memory addressing mode: PAGE                 */
        0xA1,             /* segment remap (normal orientation)           */
        0xC8,             /* COM scan direction remapped                  */
        0xDA, 0x12,       /* COM pin config: alternative, 64 rows         */
        0x81, 0xCF,       /* contrast                                     */
        0xD9, 0xF1,       /* precharge period                             */
        0xDB, 0x40,       /* VCOMH deselect level                         */
        0xA4,             /* resume display from RAM content              */
        0xA6,             /* normal (non-inverted) display                */
        0xAF              /* display ON                                   */
    };
    for (uint16_t i = 0; i < sizeof(init_seq); i++)
        oled_write_cmd(init_seq[i]);

    OLED_Clear();
    OLED_Flush();         /* blank the screen (all pages dirty) */
}

void OLED_Clear(void)
{
    memset(fb, 0, sizeof(fb));
    dirty_mask = 0xFF;
}

void OLED_ClearPage(uint8_t page)
{
    if (page >= OLED_PAGES) return;
    memset(fb[page], 0, OLED_COLS);
    dirty_mask |= (uint8_t)(1u << page);
}

void OLED_DrawChar(uint8_t page, uint8_t col, char c)
{
    if (page >= OLED_PAGES) return;
    if (c < 0x20 || c > 0x7F) return;
    if (col + 6 > OLED_COLS) return;

    const uint8_t *g = font6x8[(uint8_t)c - 0x20];
    for (uint8_t i = 0; i < 6; i++)
        fb[page][col + i] = g[i];
    dirty_mask |= (uint8_t)(1u << page);
}

void OLED_DrawString(uint8_t page, uint8_t col, const char *s)
{
    while (s && *s)
    {
        OLED_DrawChar(page, col, *s);
        col += 6;
        if (col + 6 > OLED_COLS) break;
        s++;
    }
}

void OLED_DrawHBar(uint8_t page, uint8_t col, uint8_t len)
{
    if (page >= OLED_PAGES || col >= OLED_COLS) return;
    if (len > (OLED_COLS - col)) len = (uint8_t)(OLED_COLS - col);
    for (uint8_t i = 0; i < len; i++)
        fb[page][col + i] = 0xFF;
    dirty_mask |= (uint8_t)(1u << page);
}

/* Set or clear one pixel (x 0..127, y 0..63). Frame is page-organized: a page
 * holds 8 pixel rows (bit 0 = top row). Marks that page dirty so OLED_Flush()
 * pushes only the changed pages - the same local-redraw machinery the text
 * functions use. */
void OLED_SetPixel(uint8_t x, uint8_t y, uint8_t on)
{
    uint8_t page = y >> 3;
    uint8_t bit  = 1u << (y & 7);
    if (x >= OLED_COLS || page >= OLED_PAGES) return;
    if (on)
        fb[page][x] |= bit;
    else
        fb[page][x] &= (uint8_t)~bit;
    dirty_mask |= (uint8_t)(1u << page);
}

/* Filled rectangle, both corners inclusive. Caller must ensure x0<=x1, y0<=y1
 * within 0..127 / 0..63; SetPixel bounds-checks the page/column anyway. */
void OLED_FillRect(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t on)
{
    for (uint8_t y = y0; y <= y1; y++)
        for (uint8_t x = x0; x <= x1; x++)
            OLED_SetPixel(x, y, on);
}

void OLED_Flush(void)
{
    for (uint8_t page = 0; page < OLED_PAGES; page++)
    {
        if (!(dirty_mask & (1u << page))) continue;

        /* select page + start column 0 (page addressing mode) */
        g_oled_nack = 0;
        i2c_start();
        i2c_write_byte((uint8_t)(OLED_ADDR << 1));
        i2c_write_byte(0x00);                 /* command stream */
        i2c_write_byte((uint8_t)(0xB0 | page)); /* page address  */
        i2c_write_byte(0x00);                 /* column low = 0  */
        i2c_write_byte(0x10);                 /* column high = 0 */
        i2c_stop();

        /* push the whole 128-column page as one data burst */
        oled_write_data_burst(fb[page], OLED_COLS);

        dirty_mask &= (uint8_t)~(1u << page);
    }
}
