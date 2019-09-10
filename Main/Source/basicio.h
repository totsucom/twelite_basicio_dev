#ifndef __BASICIO_H
#define __BASICIO_H

#include "AppHardwareApi.h"
#include "utils.h"
#include "ToCoNet.h"

#include "basicio_module.h"

#if defined(USE_SERIAL) || defined(USE_SERIAL1)
#include "serial.h"
#include "fprintf.h"
#endif

#ifdef USE_RADIO
#include "ToCoNet_mod_prototype.h"
#include "sprintf.h"
#endif

#ifdef USE_EEPROM
#include "eeprom_6x.h"
#endif

#ifdef USE_FLASH
#include "ccitt8.h"
#endif


//TWELITE種類の判別
#if JN5164 == 5164      //TWELITE BLUE
#define TWELITE_BLUE
#elif JN5169 == 5169    //TWELITE RED
#define TWELITE_RED
#else
#error Unknown TWELITE model
#endif


/*
 * イベント
 */

typedef enum {
    EVENT_START_UP,          
    EVENT_TICK_TIMER,       //4[ms]
    EVENT_TICK_SECOND       //1[sec]
} EVENTS;



/*
 * いろいろ
 */

//vWait()の機能を調べること。


extern uint32_t millisValue;

//ミリ秒を計測するカウントアップタイマー。デフォルト精度は4ms。ユーザーが書き込んでもOK
//割り込みでなくコールバックでカウントしてるので、loop()を一旦抜けないとカウントアップしない
#define millis()            millisValue

//32bit乱数を生成
#define rand()              ToCoNet_u32GetRand()

//モジュールの32bitアドレス。モジュールのシリアル番号を元にシステムにより自動生成される
//モジュールを識別する番号として用いる
#define moduleAddress()     ToCoNet_u32GetSerial()

//引数 スリープ時間[ms],インターバルスリープ(毎回自分のタイミングでsleep()する場合はFALSE),RAMの電源を切る
//ram_off=FALSEとした場合、グローバル変数のmillis値や関数内のstatic変数まで記憶されるので考慮すること
#define sleep(interval_ms,periodic,ram_off)   ToCoNet_vSleep(E_AHI_WAKE_TIMER_0, interval_ms, periodic, ram_off); 


/*
 * デジタルIO
 */

#define DIO_MONOSTICK_LED   16

typedef enum {
    INPUT,
    INPUT_PULLUP,
    OUTPUT
} PINMODES;

#define LOW                 0
#define HIGH                1

extern bool_t dio_pinMode(uint8_t pinNo, PINMODES mode);
extern bool_t dio_write(uint8_t pinNo, uint8_t value);


//pinNO=0..19 返り値 LOW(0)/HIGH(1)
#define dio_read(pinNo)         ((u32AHI_DioReadInput() & (1UL << pinNo)) ? HIGH : LOW)

//DIO0～DIO19の出力状態を一度に設定する。
//onBits: HIGHにしたいDIOのビットを1に設定
//offBits: LOWにしたいDIOのビットを1に設定
#define dio_writeAll(onBits, offBits)   vAHI_DioSetOutput(onBits, offBits)

//DIO0～DIO19の入力状態を一度に読み込む。bit=0:LOW/1:HIGH
#define dio_readAll()           u32AHI_DioReadInput()


/*
 * デジタルIO(割り込み)
 */

typedef enum {
    DISABLE,
    RISING,
    FALLING
} INTERRUPTIONEDGES;

extern bool_t dio_attachCallback(uint8_t pinNo, void (*func)(uint32_t u32DioBitmap), INTERRUPTIONEDGES mode);
extern bool_t dio_detachCallback(uint8_t pinNo);
extern bool_t dio_setWake(uint8_t pinNo, INTERRUPTIONEDGES mode);



/*
 * タイマー／ＰＷＭ
 */

#ifdef USE_TIMER
extern tsTimerContext sTimerApp[5];
extern bool_t timer_attachCallback(uint8_t timerNo, uint16_t hz, uint8_t prescale, void (*func)());
extern bool_t timer_detachCallback(uint8_t timerNo);
extern bool_t timer_attachPWM(uint8_t timerNo, uint16_t hz, uint8_t prescale, uint16_t duty);
#define timer_detachPWM(timerNo)    timer_detachCallback(timerNo)

//PWMまたはタイマー割り込みを開始
#define timer_start(timerNo)     vTimerStart(&sTimerApp[timerNo])

//PWMまたはタイマー割り込みを停止
#define timer_stop(timerNo)      vTimerStop(&sTimerApp[timerNo])
#endif //USE_TIMER


/*
 * シリアル
 */

#if defined(USE_SERIAL) || defined(USE_SERIAL1)
//#include "serial.h"
//#include "fprintf.h"

typedef enum {
    SERIAL_BAUD_9600   = (0x80000000 | 104             ), //   9600bps
    SERIAL_BAUD_19200  = (0x80000000 |  52             ), //  19200bps
    SERIAL_BAUD_38400  = (0x80000000 |  26             ), //  38400bps
    SERIAL_BAUD_57600  = (0x80000000 |  23 | (11 << 16)), //  57600bps
    SERIAL_BAUD_76800  = (0x80000000 |  13             ), //  76800bps
    SERIAL_BAUD_115200 = (0x80000000 |  10 | (13 << 16)), // 115200bps
    SERIAL_BAUD_230400 = (0x80000000 |   5 | (13 << 16)), // 230400bps
    SERIAL_BAUD_250000 = (0x80000000 |   4             )  // 250000bps
} BAUDRATES;

bool_t serialx_init(uint8_t u8SerialPort, BAUDRATES baudRate, tsFILE *psUartStream, tsSerialPortSetup *psUartPort, tsUartOpt *psUartOpt);
//bool_t serialx_init(uint8_t u8SerialPort, BAUDRATES baudRate, tsFILE *psUartStream, tsSerialPortSetup *psUartPort);
bool_t serialx_forDebug(tsFILE *psUartStream, uint8_t debugLevel);
bool_t serialx_write(uint8_t u8SerialPort, uint8_t *pu8Data, uint8_t length);
int16_t serialx_readUntil(uint8_t u8SerialPort, uint8_t u8Terminate, uint8_t *pu8Buffer, uint16_t u16BufferLength);
#endif //USE_SERIAL || USE_SERIAL1

#ifdef USE_SERIAL
extern tsFILE sUartStream0;
extern tsSerialPortSetup sUartPort0;
extern tsUartOpt sUartOpt0;

//定数 SERIAL_BAUD_115200 などを渡すこと
#define serial_init(baudrate)       serialx_init(E_AHI_UART_0, baudrate, &sUartStream0, &sUartPort0, &sUartOpt0)

//debugLevel=0..5 (0:デバッグ出力無し)
#define serial_forDebug(debugLevel) serialx_forDebug(&sUartStream0, debugLevel)

#define serial_ready()              SERIAL_bTxQueueEmpty(E_AHI_UART_0)
/*                                     ((SERIAL_bTxQueueEmpty(E_AHI_UART_0)) && \
                                        (u8AHI_UartReadLineStatus(E_AHI_UART_0) & E_AHI_UART_LS_THRE) && \
                                        (u8AHI_UartReadLineStatus(E_AHI_UART_0) & E_AHI_UART_LS_TEMT))*/
#define serial_printf(...)         vfPrintf(&sUartStream0, LB __VA_ARGS__)
#define serial_putc(c)             SERIAL_bTxChar(E_AHI_UART_0, c)
#define serial_puts(s)             SERIAL_bTxString(E_AHI_UART_0, (uint8_t *)s)
#define serial_write(p,len)        serialx_write(E_AHI_UART_0, p, len)

#define serial_available()         (!SERIAL_bRxQueueEmpty(E_AHI_UART_0))
#define serial_getc()              SERIAL_i16RxChar(E_AHI_UART_0)

//バッファサイズになるか、0x0dを受け取るまでブロック。0x0dは文字列に含まれる
//#define serial_readLine(p,len)     SERIAL_u32RxString(E_AHI_UART_0, p, len)

//指定文字まで、または、バッファがいっぱいになるまで読む
//バッファを静的変数とし、loop()で1回だけ呼び出すことでブロックの処理を避けている
//エラーで-1, 途中なら0, 読み終わったら文字数を返す。指定文字まで読んだかどうかは最終文字を調べる
#define serial_readUntil(ch,p,len) serialx_readUntil(E_AHI_UART_0, ch, p, len)

#endif //USE_SERIAL

#ifdef USE_SERIAL1
extern tsFILE sUartStream1;
extern tsSerialPortSetup sUartPort1;
extern tsUartOpt sUartOpt1;

//定数 SERIAL_BAUD_115200 などを渡すこと
#define serial1_init(baudrate)       serialx_init(E_AHI_UART_1, baudrate, &sUartStream1, &sUartPort1, &sUartOpt1)

//debugLevel=0..5 (0:デバッグ出力無し)
#define serial1_forDebug(debugLevel) serialx_forDebug(&sUartStream1, debugLevel)

#define serial1_ready()             SERIAL_bTxQueueEmpty(E_AHI_UART_1)
/*                                    ((!SERIAL_bTxQueueEmpty(E_AHI_UART_1)) && \
                                        (u8AHI_UartReadLineStatus(E_AHI_UART_1) & E_AHI_UART_LS_THRE) && \
                                        (u8AHI_UartReadLineStatus(E_AHI_UART_1) & E_AHI_UART_LS_TEMT))*/
#define serial1_printf(...)         vfPrintf(&sUartStream1, LB __VA_ARGS__)
#define serial1_putc(c)             SERIAL_bTxChar(E_AHI_UART_1, c)
#define serial1_puts(s)             SERIAL_bTxString(E_AHI_UART_1, s)
#define serial1_write(p,len)        serialx_write(E_AHI_UART_1, p, len)

#define serial1_available()         (!SERIAL_bRxQueueEmpty(E_AHI_UART_1))
#define serial1_getc()              SERIAL_i16RxChar(E_AHI_UART_1)
//#define serial1_readLine(p,len)     SERIAL_u32RxString(E_AHI_UART_1, p, len)
#define serial1_readUntil(ch,p,len) serialx_readUntil(E_AHI_UART_1, ch, p, len)
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
    ADC_SOURCE_1 = E_AHI_ADC_SRC_ADC_1,
    ADC_SOURCE_2 = E_AHI_ADC_SRC_ADC_2,
    ADC_SOURCE_3 = E_AHI_ADC_SRC_ADC_3, //DIO0
    ADC_SOURCE_4 = E_AHI_ADC_SRC_ADC_4, //DIO1
    ADC_SOURCE_TEMP = E_AHI_ADC_SRC_TEMP,
    ADC_SOURCE_VOLT = E_AHI_ADC_SRC_VOLT
} ADCSOURCES;

extern void adc_enable(ADCSAMPLES sample, ADCCLOCKS clock);
extern void adc_disable();
extern void adc_attachCallback(bool_t continuous, bool_t range2, ADCSOURCES source, void (*func)(uint16_t rawData, int16_t adcResult));
extern void adc_detachCallback();


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
#define i2c_enable(prescaler)   vAHI_SiMasterConfigure(TRUE, FALSE, prescaler)
#define i2c_disable             vAHI_SiMasterDisable()

//TRUEでI2CのSCL,SDAピンをDIO14,15からDIO16,17に変更できる
#define i2c_selectPin(b)        vAHI_SiSetLocation(b)

extern bool_t i2c_write(uint8_t u8Address, uint8_t u8Command, const uint8* pu8Data, uint8_t u8Length);
extern bool_t i2c_read(uint8_t u8Address, uint8* pu8Data, uint8_t u8Length);
extern bool_t i2c_commandRead(uint8_t u8Address, uint8_t u8Command, uint8* pu8Data, uint8_t u8Length);
extern bool_t i2c_writeByte(uint8_t u8Address, uint8_t u8Command, uint8_t u8Data);
extern int16_t i2c_readByte(uint8_t u8Address);


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

extern bool_t spi_enable(uint8_t u8NumSlaves, bool_t bLsbFirst, SPIMODES u8Mode, SPICLOCKS u8Divider);

//SPIマスターを無効にする
#define spi_disable             vAHI_SpiDisable()

extern bool_t spi_selectPin(uint8_t slaveNo, bool_t bSecondPin);
extern void spi_selectSlave(int8_t slaveNo);

//spi_selectSlave(-1)と同意
#define spi_stop            vAHI_SpiStop()

extern bool_t spi_write(uint32_t u32Data, uint8_t u8BitLength);
extern void spi_writeByte(uint8_t u8Data);

#define spi_readDWord            u32AHI_SpiReadTransfer32()
#define spi_readWord             u16AHI_SpiReadTransfer16()
#define spi_readByte             u8AHI_SpiReadTransfer8()


/*
 * 無線通信
 */

#ifdef USE_RADIO
//#include "sprintf.h"

extern uint8_t u8NumRadioTx;

#define RADIO_ADDR_BROADCAST   TOCONET_MAC_ADDR_BROADCAST

//送信中のデータは無い
#define radio_txCompleted()       (u8NumRadioTx == 0)

extern void radio_attachTxCallback(void (*func)(uint8_t u8CbId, bool_t bSuccess));
extern void radio_attachRxCallback(void (*func)(uint32_t u32SrcAddr, uint8_t u8CbId, uint8_t u8DataType, uint8_t *pu8Data, uint8_t u8Length, uint8_t u8Lqi));

#define radio_detachTxCallback()  radio_attachTxCallback(NULL)
#define radio_detachRxCallback()  radio_attachRxCallback(NULL)

extern int16_t radio_write(uint32_t u32DestAddr, uint8_t *pu8Data, uint8_t u8Length, uint8_t u8DataType);
extern int16_t radio_puts(uint32_t u32DestAddr, uint8_t *pu8String);
extern uint16_t radio_printf(uint32_t u32DestAddr, const char* format, va_list args);
#endif
 

/*
 * EEPROM
 */

#ifdef USE_EEPROM
//#include "eeprom_6x.h"

#define eep_read(adr, buf, len)     EEP_6x_bRead(adr, len, buf)
#define eep_write(adr, buf, len)    EEP_6x_bWrite(adr, len, buf)

#endif


/*
 * FRASHメモリ
 */

#ifdef USE_FLASH
//#include "ccitt8.h"

extern bool_t flash_erase(uint8_t sector);
extern bool_t flash_write(uint8_t sector, uint32_t offset, uint8_t *pu8Data, uint16_t u16DataLength);
extern bool_t flash_read(uint8 sector, uint32 offset, uint8_t *pu8Data, uint16_t u16DataLength);
#endif



#endif  //__BASICIO_H