
extern void lcd_init();
extern bool_t lcd_enabled();
extern void lcd_setCursor(uint8_t x, uint8_t y);
extern void lcd_printStr(const char *s);
extern void lcd_setContrast(uint8_t c);
extern void lcd_printXY(uint8_t x, uint8_t y, uint8_t *str);

