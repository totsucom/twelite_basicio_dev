#include "basicio.h"
#include "string.h"

/*
 * SWITCH SCIENCE の「I2C接続の小型LCD搭載ボード(3.3V版)」カタログ番号1405 用
 * http://akizukidenshi.com/download/ds/xiamen/AQM0802.pdf
 */

#define I2Cadr 0x3e  // 固定
uint8_t contrast = 35;  // コントラスト(0～63)

static void lcd_cmd(uint8_t x) {
    i2c_writeByte(I2Cadr, 0b00000000, x);
}

// 指定位置に表示 
void lcd_printXY(uint8_t x, uint8_t y, uint8_t *str) {
    i2c_writeByte(I2Cadr, 0b00000000, 0x80 | (y * 0x40 + x));

    //1文字目が表示されない。ダミーの'A'
    i2c_write(I2Cadr, 'A', (uint8_t *)str, (uint8_t)strlen(str));
}

// 文字の表示
void lcd_print(uint8_t *str) {
    //1文字目が表示されない。ダミーの'A'
    i2c_write(I2Cadr, 'A', (uint8_t *)str, (uint8_t)strlen(str));
}
 
// 表示位置の指定
void lcd_setCursor(uint8_t x, uint8_t y) {
  lcd_cmd(0x80 | (y * 0x40 + x));
}

//コントラストの設定 0-63
void lcd_setContrast(uint8_t c) {
  lcd_cmd(0x39);
  lcd_cmd(0b01110000 | (c & 0x0f)); // contrast Low
  lcd_cmd(0b01011100 | ((c >> 4) & 0x03)); // contast High/icon/power
  lcd_cmd(0x38);
}

static uint8_t init_step;
static uint8_t init_delay;

//setup()で呼ぶ
void lcd_init() {
    init_step = 0;
}

//関数がTRUEを返すまで、EVENT_TICK_TIMERで呼ぶ
//何回呼び出してもよい。一度TRUEを返したらその後はTRUEを返し続ける
//この関数がTRUEを返したら、LCDの使用を開始できる
bool_t lcd_enabled() {
    if (init_step == 0) {
        lcd_cmd(0b00111000); // function set
        lcd_cmd(0b00111001); // function set
        lcd_cmd(0b00000100); // EntryModeSet
        lcd_cmd(0b00010100); // interval osc
        lcd_cmd(0b01110000 | (contrast & 0xF)); // contrast Low
        lcd_cmd(0b01011100 | ((contrast >> 4) & 0x3)); // contast High/icon/power
        lcd_cmd(0b01101100); // follower control
        init_step = 1;
        init_delay = 0;
    } else if (init_step == 1) {
        if (++init_delay == 50) {//4ms x 50 = 200ms
            lcd_cmd(0b00111000); // function set
            lcd_cmd(0b00001100); // Display On
            lcd_cmd(0b00000001); // Clear Display
            init_step = 2;
            init_delay = 0;
        }
    } else if (init_step == 2) {
        if (++init_delay == 1) {//4ms
            init_step = 3;
        }
    } else {
        return TRUE;
    }
    return FALSE;
}

