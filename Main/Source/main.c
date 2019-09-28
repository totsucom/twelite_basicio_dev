#include "basicio.h"
//basicio_module.hでUSE_SERIALを宣言しておく

#define PB 7    //プッシュボタンのDIO番号

#define BOOT    18
#define BYP     19

bool_t bWarmWake;
bool_t bSleep = FALSE;
uint16_t u16Volt = 0;

void adcDone(uint16_t value) {

    if (u16Volt == 0) {
        //valueは電源電圧
        u16Volt = value;

        adc_attachCallback(FALSE, TRUE, ADC_SOURCE_1, adcDone);
    } else {
        adc_disable();

        //valueはVC2電圧
        value <<= 1; //蓄電電圧は半分に分圧されてるので、x２する

        sb_clear();
        sb_printf("PWR%u,VC2%u", (uint32_t)u16Volt, (uint32_t)value);
        radio_puts(RADIO_ADDR_BROADCAST, sb_getBuffer());
    }
}

void txDone(uint8_t u8CbId,bool_t bSuccess) {
    bSleep = TRUE;
}

void radioRaceived(uint32_t u32SrcAddr,uint8_t u8CbId,uint8_t u8DataType,uint8_t *pu8Data,uint8_t u8Length,uint8_t u8Lqi) {
    dio_write(DIO_MONOSTICK_LED, LOW);
    serial_write(pu8Data, u8Length);
}

//#define RECEIVE_MODE

void setup(bool_t warmWake, uint32_t bitmapWakeStatus) {

#ifndef RECEIVE_MODE
    dio_write(BOOT, LOW);
    dio_pinMode(BOOT, OUTPUT);

    dio_write(BYP, HIGH);
    dio_pinMode(BYP, OUTPUT);

    vAHI_BrownOutConfigure(0,//0:2.0V 1:2.3V
				FALSE,
				FALSE,
				FALSE,
				FALSE);
#endif


#ifndef RECEIVE_MODE
    bWarmWake = warmWake;
    dio_pinMode(PB, INPUT);


    adc_enable(ADC_SAMPLE_2, ADC_CLOCK_500KHZ, FALSE);// _2 もったいないので貯めない
    adc_attachCallback(FALSE, TRUE, ADC_SOURCE_VOLT, adcDone);

    radio_attachTxCallback(txDone);
#else
    // シリアルを初期化する
    serial_init(SERIAL_BAUD_115200, FALSE);

    dio_write(DIO_MONOSTICK_LED, HIGH);
    dio_pinMode(DIO_MONOSTICK_LED, OUTPUT);
    radio_attachRxCallback(radioRaceived);
#endif
}

void loop(EVENTS event) {
#ifndef RECEIVE_MODE
    if (event == EVENT_TICK_SECOND) {
        radio_puts(RADIO_ADDR_BROADCAST, "dummy");//念のため
    } else if (event == EVENT_TICK_TIMER) {
        if (bSleep) {// && serial_getTxCount() == 0) {
            dio_setWake(PB, FALLING);
            sleep(FALSE, FALSE);
        }
    }
#else
    if (event == EVENT_TICK_TIMER) dio_write(DIO_MONOSTICK_LED, HIGH);
#endif
}
