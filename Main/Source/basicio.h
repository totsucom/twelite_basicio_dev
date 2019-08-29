#ifndef __BASICIO_H
#include "AppHardwareApi.h"
#include "utils.h"
#include "ToCoNet.h"
#include "ToCoNet_mod_prototype.h"
#include "basicio_module.h"


/*
 * イベント
 */

typedef enum {
    EVENT_START_UP,          
    EVENT_TICK_TIMER,       //4[ms]
    EVENT_TICK_SECOND       //1[sec]
} EVENTS;


/*
 * ピン定数
 */

#define MONOSTICK_LED   16


/*
 * デジタルIO
 */

typedef enum {
    INPUT,
    INPUT_PULLUP,
    OUTPUT
} PINMODES;

#define LOW             0
#define HIGH            1

extern bool_t pinMode(uint8_t pinNo, PINMODES mode);
extern bool_t digitalWrite(uint8_t pinNo, uint8_t value);

//pinNO=0..19 返り値 LOW(0)/HIGH(1)
#define digitalRead(pinNo)  ((u32AHI_DioReadInput() & (1UL << pinNo)) ? HIGH : LOW)


/*
 * デジタルIO(割り込み)
 */

typedef enum {
    DISABLE,
    RISING,
    FALLING
} INTERRUPTIONEDGES;

extern bool_t attachDioCallback(uint8_t pinNo, void (*func)(), INTERRUPTIONEDGES mode);
extern bool_t detachDioCallback(uint8_t pinNo);


/*
 * タイマー／ＰＷＭ
 */

extern uint32_t millisValue;

//4ms刻みのカウントアップタイマー。ユーザーが書き込んでもOK
#define millis  millisValue

extern tsTimerContext sTimerApp[5];
extern bool_t attachTimerPWM(uint8_t timerNo, uint16_t hz, uint8_t prescale, uint16_t duty);
extern bool_t attachTimerCallback(uint8_t timerNo, uint16_t hz, uint8_t prescale, void (*func)());
extern bool_t detachTimer(uint8_t timerNo);

//PWMまたはタイマー割り込みを開始
#define startTimer(timerNo)     vTimerStart(&sTimerApp[timerNo])

//PWMまたはタイマー割り込みを停止
#define stopTimer(timerNo)      vTimerStop(&sTimerApp[timerNo])



/*
 * スリープ
 */

extern bool_t setDioWake(uint8_t pinNo, INTERRUPTIONEDGES mode);

//引数 スリープ時間[ms],インターバルスリープ(毎回自分のタイミングでsleep()する場合はFALSE),RAMの電源を切る
//ram_off=FALSEとした場合、グローバル変数のmillis値や関数内のstatic変数まで記憶されるので考慮すること
#define sleep(interval_ms,periodic,ram_off)   ToCoNet_vSleep(E_AHI_WAKE_TIMER_0, interval_ms, periodic, ram_off); 


/*
 * シリアル
 */

#if defined(USE_SERIAL0) || defined(USE_SERIAL1)
#include "serial.h"
#include "fprintf.h"

typedef enum {
    BAUDRATE_9600   = (0x80000000 | 104             ), //   9600bps
    BAUDRATE_19200  = (0x80000000 |  52             ), //  19200bps
    BAUDRATE_38400  = (0x80000000 |  26             ), //  38400bps
    BAUDRATE_57600  = (0x80000000 |  23 | (11 << 16)), //  57600bps
    BAUDRATE_76800  = (0x80000000 |  13             ), //  76800bps
    BAUDRATE_115200 = (0x80000000 |  10 | (13 << 16)), // 115200bps
    BAUDRATE_230400 = (0x80000000 |   5 | (13 << 16)), // 230400bps
    BAUDRATE_250000 = (0x80000000 |   4             )  // 250000bps
} BAUDRATES;
#endif //USE_SERIAL0 || USE_SERIAL1

#ifdef USE_SERIAL0
extern tsFILE sUartStream0;
extern tsSerialPortSetup sUartPort0;

extern bool_t Serial0_init(BAUDRATES baudRate);
extern bool_t Serial0_forDebug(uint8_t debugLevel);

#define Serial0_ready               ((SERIAL_bTxQueueEmpty(E_AHI_UART_0)) && \
                                        (u8AHI_UartReadLineStatus(E_AHI_UART_0) & E_AHI_UART_LS_THRE) && \
                                        (u8AHI_UartReadLineStatus(E_AHI_UART_0) & E_AHI_UART_LS_TEMT))
#define Serial0_printf(...)         vfPrintf(&sUartStream0, LB __VA_ARGS__)
#define Serial0_putc(c)             SERIAL_bTxChar(E_AHI_UART_0, c)
#define Serial0_puts(s)             SERIAL_bTxString(E_AHI_UART_0, (uint8_t *)s)
#define Serial0_write(p,len)        Serial_write(E_AHI_UART_0, p, len)
#define Serial0_flush               SERIAL_vFlush(E_AHI_UART_0)

#define Serial0_available           (!SERIAL_bRxQueueEmpty(E_AHI_UART_0))
#define Serial0_getc                SERIAL_i16RxChar(E_AHI_UART_0)
//バッファサイズになるか、0x0dを受け取るまでブロック。0x0dは文字列に含まれる
#define Serial0_readLine(p,len)     SERIAL_u32RxString(E_AHI_UART_0, p, len)
#endif //USE_SERIAL0

#ifdef USE_SERIAL1
extern tsFILE sUartStream1;
extern tsSerialPortSetup sUartPort1;

extern bool_t Serial1_init(BAUDRATES baudRate);
extern bool_t Serial1_forDebug(uint8_t debugLevel);

#define Serial1_ready               ((!SERIAL_bTxQueueEmpty(E_AHI_UART_1)) && \
                                        (u8AHI_UartReadLineStatus(E_AHI_UART_1) & E_AHI_UART_LS_THRE) && \
                                        (u8AHI_UartReadLineStatus(E_AHI_UART_1) & E_AHI_UART_LS_TEMT))
#define Serial1_printf(...)         vfPrintf(&sUartStream1, LB __VA_ARGS__)
#define Serial1_putc(c)             SERIAL_bTxChar(E_AHI_UART_1, c)
#define Serial1_puts(s)             SERIAL_bTxString(E_AHI_UART_1, s)
#define Serial1_write(p,len)        Serial_write(E_AHI_UART_1, p, len)
#define Serial1_flush               SERIAL_vFlush(E_AHI_UART_1)

#define Serial1_available           (!SERIAL_bRxQueueEmpty(E_AHI_UART_1))
#define Serial1_getc                SERIAL_i16RxChar(E_AHI_UART_1)
#define Serial1_readLine(p,len)     SERIAL_u32RxString(E_AHI_UART_1, p, len)
#endif //USE_SERIAL1

/*
 * ＡＤＣ
 */

typedef enum {
    ADC_SAMPLE_2 = E_AHI_AP_SAMPLE_2,
    ADC_SAMPLE_4 = E_AHI_AP_SAMPLE_4,
    ADC_SAMPLE_6 = E_AHI_AP_SAMPLE_6,
    ADC_SAMPLE_8 = E_AHI_AP_SAMPLE_8
} ADCSAMPLES;

typedef enum {
    ADC_CLOCK_2MHZ = E_AHI_AP_CLOCKDIV_2MHZ,
    ADC_CLOCK_1MHZ = E_AHI_AP_CLOCKDIV_1MHZ,
    ADC_CLOCK_500KHZ = E_AHI_AP_CLOCKDIV_500KHZ,
    ADC_CLOCK_250KHZ = E_AHI_AP_CLOCKDIV_250KHZ
} ADCCLOCKS;

typedef enum {
    SOURCE_ADC1 = E_AHI_ADC_SRC_ADC_1,
    SOURCE_ADC2 = E_AHI_ADC_SRC_ADC_2,
    SOURCE_ADC3 = E_AHI_ADC_SRC_ADC_3, //DIO0
    SOURCE_ADC4 = E_AHI_ADC_SRC_ADC_4, //DIO1
    SOURCE_TEMP = E_AHI_ADC_SRC_TEMP,
    SOURCE_VOLT = E_AHI_ADC_SRC_VOLT
} ADCSOURCES;

extern void enableAdcModule(ADCSAMPLES sample, ADCCLOCKS clock);
extern void disableAdcModule();
extern void attachAdcCallback(bool_t continuous, bool_t range2, ADCSOURCES source, void (*func)(uint16_t rawData, int16_t adcResult));
extern void detachAdcCallback();


/*
 * Ｉ２Ｃ
 */

//Operating frequency = 16/[(PreScaler + 1) x 5] MHz
typedef enum {
    I2C_CLOCK_66KHZ		= 47,
    I2C_CLOCK_100KHZ	= 31,
    I2C_CLOCK_200KHZ	= 15,
    I2C_CLOCK_400KHZ	= 7
} I2CPRESCALER;

//I2C初期化。標準のプリスケーラーは I2C_CLOCK_100KHZ
#define I2C_enable(prescaler)   vAHI_SiMasterConfigure(TRUE, FALSE, prescaler)
#define I2C_disable             vAHI_SiMasterDisable()

//TRUEでI2CのSCL,SDAピンをDIO14,15からDIO16,17に変更できる
#define I2C_selectPin(b)        vAHI_SiSetLocation(b)

extern bool_t I2C_write(uint8_t u8Address, uint8_t u8Command, const uint8* pu8Data, uint8_t u8Length);
extern bool_t I2C_read(uint8_t u8Address, uint8* pu8Data, uint8_t u8Length);
extern bool_t I2C_commandRead(uint8_t u8Address, uint8_t u8Command, uint8* pu8Data, uint8_t u8Length);
extern bool_t I2C_writeByte(uint8_t u8Address, uint8_t u8Command, uint8_t u8Data);
extern int16_t I2C_readByte(uint8_t u8Address);


/*
 * ＳＰＩ
 */

typedef enum {
    SPI_MODE_0,
    SPI_MODE_1,
    SPI_MODE_2,
    SPI_MODE_3
} SPIMODES;

typedef enum {
    SPI_CLOCK_8MHZ = 1,
    SPI_CLOCK_4MHZ = 2,
    SPI_CLOCK_2MHZ = 4,
    SPI_CLOCK_1MHZ = 8
} SPICLOCKS;

extern bool_t SPI_enable(uint8_t u8NumSlaves, bool_t bLsbFirst, SPIMODES u8Mode, SPICLOCKS u8Divider);

//SPIマスターを無効にする
#define SPI_disable             vAHI_SpiDisable()

extern bool_t SPI_selectPin(uint8_t slaveNo, bool_t bSecondPin);
extern void SPI_selectSlave(int8_t slaveNo);

//SPI_selectSlave(-1)と同意
#define SPI_stop            vAHI_SpiStop()

extern bool_t SPI_write(uint32_t u32Data, uint8_t u8BitLength);
extern void SPI_writeByte(uint8_t u8Data);

#define SPI_readDWord            u32AHI_SpiReadTransfer32()
#define SPI_readWord             u16AHI_SpiReadTransfer16()
#define SPI_readByte             u8AHI_SpiReadTransfer8()

 
#define __BASICIO_H
#endif