#include "lcd.h"

static void write_4_bits(uint8_t value);
static void lcd_enable(void);
static void mdelay(uint32_t cnt);
static void udelay(uint32_t cnt);

void lcd_send_command(uint8_t cmd)
{
    /* RS = 0, For LCD command */
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);

    /* RW = 0, Writing to LCD */
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

    write_4_bits(cmd >> 4);
    write_4_bits(cmd & 0xf);

}

void lcd_print_char(uint8_t data)
{
    /* RS = 1, For LCD data */
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_SET);

    /* RW = 0, Writing to LCD */
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

    write_4_bits(data >> 4);
    write_4_bits(data & 0xf);
}

void lcd_print_string(char *message)
{
    do {
        lcd_print_char((uint8_t)*message++);
    } while (*message != '\0');
}

void lcd_init(void)
{
    GPIO_Handle_t lcd_signal;

    //1. init GPIO pins
    lcd_signal.pGPIOx = LCD_GPIO_PORT;
    lcd_signal.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    lcd_signal.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    lcd_signal.GPIO_PinConfig.GPIO_PinPinPuPdControl = GPIO_NO_PUPD;


    lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RS;
    GPIO_Init(&lcd_signal);

    lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RW;
    GPIO_Init(&lcd_signal);

    lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_EN;
    GPIO_Init(&lcd_signal);

    lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D4;
    GPIO_Init(&lcd_signal);

    lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D5;
    GPIO_Init(&lcd_signal);

    lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D6;
    GPIO_Init(&lcd_signal);

    lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D7;
    GPIO_Init(&lcd_signal);

    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, GPIO_PIN_RESET);

    //2. do the LCD initialization

    mdelay(40);

    /* RS = 0, For LCD command */
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);

    /* RW = 0, Writing to LCD */
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

    write_4_bits(0x03);

    mdelay(5);

    write_4_bits(0x03);

    udelay(150);

    write_4_bits(0x03);
    write_4_bits(0x02);

    //function set command
    lcd_send_command(LCD_CMD_4DL_2N_5X8F);

    //display ON and cursor ON
    lcd_send_command(LCD_CMD_DON_CURSON);

    lcd_display_clear();

    //entry mode set
    lcd_send_command(LCD_CMD_INCADD);
}

void lcd_display_clear(void)
{
    lcd_send_command(LCD_CMD_DIS_CLEAR);
    mdelay(2);
}

void lcd_display_return_home(void)
{
    lcd_send_command(LCD_CMD_DIS_RETURN_HOME);
    mdelay(2);
}

void lcd_set_cursor(uint8_t row, uint8_t column)
{
    column--;

    switch (row) {
        case 1:
            lcd_send_command((column |= 0x80));
            break;
        case 2:
            lcd_send_command((column |= 0xC0));
            break;
        default:
            break;
    }
}

static void write_4_bits(uint8_t value)
{
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, ((value >> 0) & 0x1));
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, ((value >> 1) & 0x1));
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, ((value >> 2) & 0x1));
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, ((value >> 3) & 0x1));

    lcd_enable();
}

static void lcd_enable(void)
{
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_SET);
    udelay(10);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
    udelay(100);
}

static void mdelay(uint32_t cnt)
{
    uint32_t i;
    for (i = 0; i < (cnt * 1000); i++);
}

static void udelay(uint32_t cnt)
{
    uint32_t i;
    for (i = 0; i < cnt; i++);
}
