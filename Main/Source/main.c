#include "basicio.h"
//basicio_module.hでUSE_SERIALを宣言しておく

#define PB 1    //プッシュボタンのDIO番号
                //パルスカウンタ0のデフォルト入力ピンでもある
                //プッシュボタンは押されるとGNDに落ちる

void pcfunc() {
    pc_clear(0);
    serial_puts("5 time pressed\r\n");
}

void setup(bool warmWake, uint32 bitmWakeStatus) {

    // シリアル0を初期化する
    serial_init(SERIAL_BAUD_115200);

    //プッシュボタン用にプルアップ
    dio_pinMode(PB, INPUT_PULLUP);

    pc_attachCallback(0, PC_DEBOUNCE_8_MAX1200HZ, 5, FALSE, FALLING, pcfunc);
    pc_start(0);


//    pb_define(TRIG, FALSE);
/*
    dio_pinMode(PBA, INPUT_PULLUP);
    dio_pinMode(PBB, INPUT_PULLUP);

    pb_define(PBA, FALSE);
    pb_define(PBB, FALSE);
*/

    //BME280 電源ON
//    dio_write(10, LOW);
//    dio_pinMode(10, OUTPUT);


    //I2Cを初期化
    //i2c_enable(I2C_CLOCK_100KHZ, FALSE); //DIO14(SCL), DIO15(SDA)

    //SPIを初期化
//    spi_enable(1, SPI_MODE_0, SPI_CLOCK_400KHZ);

    //BME280を初期化、フォースモードで測定開始
    //initialized = bme280_init(0x76);    //SDOは10K抵抗でGNDに接続, アドレス0x76

    //if (!initialized) {
    //    serial_puts("bme280 init error\r\n");
    //}

    //dio_pinMode(5, INPUT);
    //pc_attachCallback(1, PC_DEBOUNCE_8_MAX1200HZ, 5, TRUE, FALLING, f);


    //comp_enable(COMP_SIGNAL_DIO17, COMP_REF_DIO16, COMP_HIS_10MV, FALSE);
    //comp_attachCallback(RISING, f);

    //uint16_t t = u64AHI_WakeTimerReadLarge(0);
    //serial_printf("%u", (uint32_t)(t & 0xffffffff));

    serial_puts((warmWake?"Warm wake\r\n":"Cold wake\r\n"));
    bool_t bw = FALSE;
    if (dioWake(bitmWakeStatus)) { serial_printf("dioWake %X\r\n", bitmWakeStatus & 0xfffff); bw = TRUE; }
    if (timerWake(bitmWakeStatus)) { serial_puts("timerWake\r\n"); bw = TRUE; }
    if (compWake(bitmWakeStatus)) { serial_puts("compWake\r\n"); bw = TRUE; }
    if (pc0Wake(bitmWakeStatus)) { serial_puts("pc0Wake\r\n"); bw = TRUE; }
    if (pc1Wake(bitmWakeStatus)) { serial_puts("pc1Wake\r\n"); bw = TRUE; }
    if (!bw) serial_puts("power on\r\n");
}

bool_t started = FALSE;
bool_t first = TRUE;

void loop(EVENTS event) {
/*
    if (event == EVENT_TICK_SECOND) {
        if (first) {
            serial_puts("FIRST\r\n");
            first = FALSE;
        } else
            serial_puts("ALIVE\r\n");
    }
    if (event == EVENT_TICK_TIMER) {
        if (started && bme280_sleeping()) {
            int32_t temp;
            uint32_t pres, humi;
            bme280_readData(&temp, &pres, &humi);
            serial_printf("T%d P%u H%u\r\n", temp, pres, humi);
            started = FALSE;
        }
        if (pb_pressed(TRIG) && !started) {
            bme280_init(0);     //SPI
            //bme280_init(0x76);  //I2C
            started = TRUE;
        }
    }
*/
}
