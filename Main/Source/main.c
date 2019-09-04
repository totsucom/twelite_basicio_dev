/*
 * TWELITE BLUE/RED (MONOSTICK含む) でシリアル通信
 * SDK バージョン 2018-05
 *
 */

#include "AppHardwareApi.h"
#include "utils.h"
#include "ToCoNet.h"

#include "basicio.h"

#include "bme280_i2c.h"

#define LED 17
#define SW  10

/*void tmr() {
    static int cnt = 0;
    cnt++;
    digitalWrite(LED, (cnt & 1) ? LOW : HIGH);
}*/

/*void sw() {
    static int cnt = 1;
    cnt++;

    if (cnt & 1) {
        detachTimer(4);
        attachTimerPWM(4, 1, 8, 128);
        startTimer(4);
        Serial0_puts("\r\nPWM");
    } else {
        detachTimer(4);
        pinMode(LED, OUTPUT);
        attachTimerInterrupt(4, 1, 8, tmr);
        startTimer(4);
        Serial0_puts("\r\nTimer");
    }
}*/

enum STATE {
    DOJOB,
    WAITING_SLEEP,
    SLEEPING
} state;

void radioRx(uint32_t u32SrcAddr,uint8_t u8CbId,uint8_t u8DataType,uint8_t *pu8Data,uint8_t u8Length,uint8_t u8Lqi)
{
    serial_printf("\r\nFrom:%u Id:%u Type:%u Lqi:%u ", u32SrcAddr,u8CbId,u8DataType,u8Lqi);
    serial_write(pu8Data, u8Length);
}

extern uint32_t millisValueTick;

void setup(bool warmWake, uint32 dioWakeStatus) {
    // UART0を初期化する
    serial_init(SERIAL_BAUD_115200);
    //Serial0_forDebug(5);//(働いてる?)

    dio_pinMode(DIO_MONOSTICK_LED, OUTPUT);
    dio_write(DIO_MONOSTICK_LED, HIGH);

    //attachEvent(EVENT_START_UP, su);
    //attachEvent(EVENT_TICK_SECOND, sec);

    /*pinMode(SW, INPUT_PULLUP);
    attachDioInterrupt(SW, sw, FALLING);

    attachTimerPWM(4, 1, 8, 128);
    startTimer(4);
*/
    //attachTimerPWM(1, 1, 8, 256);
    //startTimer(1);

    //pinMode(9, INPUT_PULLUP);
    //attachDioInterrupt(9, warikomi, FALLING);

    serial_puts((uint8 *)(warmWake ? "\r\nWarm wake" : "\r\nCold wake"));
    serial_printf("\r\nwake status = %x, tick=%u", dioWakeStatus, millisValueTick);

    state = DOJOB;

    //I2C_init(I2C_CLOCK_100KHZ);
    //I2C_selectPin(TRUE);
    //BME280_init();

//    SPI_enable(1,FALSE,SPI_MODE_3,SPI_CLOCK_8MHZ);

    radio_attachRxCallback(radioRx);
}

/*void adc(uint16 rawValue, int resultValue) {
    //detachAdcInterrupt();
    //disableAdcModule();

    Serial0_printf("\r\nadc result = %d, %d[mV]", (int)rawValue, resultValue);
}*/

void loop(EVENTS event) {
    static int x=0, w=0;

    if (event == EVENT_TICK_TIMER) {
        if (w > 0) {
            if (--w == 0) {
                dio_write(DIO_MONOSTICK_LED, HIGH);
            }
        }
    }

    if (event == EVENT_TICK_SECOND) {
        //attachAdcInterrupt(FALSE, TRUE, SOURCE_ADC4, adc);

        /*signed long int temp;
        unsigned long int pres;
        unsigned long int humi;
        BME280_readData(&temp, &pres, &humi);
        Serial0_printf("\r\nt=%d p=%u h=%u", temp, pres, humi);*/

        /*
	    vAHI_SpiConfigure(0,
					  E_AHI_SPIM_MSB_FIRST,
					  TRUE,
					  TRUE,
					  1,
					  E_AHI_SPIM_INT_DISABLE,
					  E_AHI_SPIM_AUTOSLAVE_DSABL);
        vAHI_SpiSelect(1);
        spi_writeByte(0x00);//(0xd0);
        int x = spi_readByte;
        serial_printf("\r\nid=%x", x);
        spi_stop;*/
/*
        radio_printf(RADIO_ADDR_BROADCAST, "%d", x++);
*/
        dio_write(DIO_MONOSTICK_LED, LOW);
        w=50;


        uint32_t t = u32AHI_TickTimerRead(), t1;
        while(1) {
            t1 = u32AHI_TickTimerRead();
            if (t != t1) break;
            //vWait(10);
        }
        serial_printf("\r\n%u %u %u", t, t1, millis());

    }

    if (event == EVENT_START_UP) {
        serial_printf("\r\nGood morning");
        //enableAdcModule(SAMPLE_8, CLOCK_250KHZ);

//        initBH1745NUC();
    }

/*    if (state == DOJOB && event == EVENT_TICK_SECOND) {
        Serial0_printf("\r\nGoto sleep");
        state = WAITING_SLEEP;
    }
    if (state == WAITING_SLEEP && Serial0_ready) {
        pinMode(SW, INPUT);
        setDioWake(SW, FALLING);

        sleep(10000, TRUE, FALSE);
        state = SLEEPING;
    }
*/
}
