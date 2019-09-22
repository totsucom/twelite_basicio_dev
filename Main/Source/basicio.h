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

#if defined(USE_RADIO) || defined(USE_SBUTIL)
#include "sprintf.h"
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


extern volatile uint32_t millisValue;

//ミリ秒を計測するカウントアップタイマー。デフォルト精度は4ms。ユーザーが書き込んでもOK
//割り込みでなくコールバックでカウントしてるので、loop()を一旦抜けないとカウントアップしない
#define millis()            millisValue

//32bit乱数を生成
#define rand()              ToCoNet_u32GetRand()

//モジュールの32bitアドレス。モジュールのシリアル番号を元にシステムにより自動生成される
//モジュールを識別する番号として用いる
#define moduleAddress()     ToCoNet_u32GetSerial()

extern bool_t sleepTimer(uint64_t milliSeconds, bool_t bRAMPower);

#define sleep(b32kOsc, bRAMPower)   vAHI_Sleep(b32kOsc ? \
                    (bRAMPower ? E_AHI_SLEEP_OSCON_RAMON : E_AHI_SLEEP_OSCON_RAMOFF) : \
                    (bRAMPower ? E_AHI_SLEEP_OSCOFF_RAMON : E_AHI_SLEEP_OSCOFF_RAMOFF))

#define deepSleep()                 vAHI_Sleep(E_AHI_SLEEP_DEEP)


//setup()で使う
#define dioWake(ws)     ((ws & 0xfffff) ? TRUE : FALSE)
#define timerWake(ws)   ((ws & E_AHI_SYSCTRL_WK0_MASK) ? TRUE : FALSE)
#define compWake(ws)    ((ws & E_AHI_SYSCTRL_COMP0_MASK) ? TRUE : FALSE)
#define pc0Wake(ws)     ((ws & E_AHI_SYSCTRL_PC0_MASK) ? TRUE : FALSE)
#define pc1Wake(ws)     ((ws & E_AHI_SYSCTRL_PC1_MASK) ? TRUE : FALSE)



#ifdef SPRINTF_H_
#define sb_clear()              SPRINTF_vRewind()
#define sb_printf(...)          vfPrintf(SPRINTF_Stream, LB __VA_ARGS__)
#define sb_getBuffer()          SPRINTF_pu8GetBuff()
#endif

#ifdef USE_PBUTIL
extern void pb_define(uint8_t pinNo, bool_t bPressToHi);
extern bool_t pb_pressed(uint8_t pinNo);
extern bool_t pb_released(uint8_t pinNo);
extern void pb_reset();
#endif



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
    DISABLE,        //dio_attachCallback,dio_setWake,timer0_attachCounter
    RISING,         //dio_attachCallback,dio_setWake,timer0_attachCounter
    FALLING,        //dio_attachCallback,dio_setWake,timer0_attachCounter
    BOTHEDGE        //timer0_attachCounter
} INTERRUPTIONEDGES;

extern bool_t dio_attachCallback(uint8_t pinNo, INTERRUPTIONEDGES mode, void (*func)(uint32_t u32DioBitmap));
extern bool_t dio_detach(uint8_t pinNo);
extern bool_t dio_setWake(uint8_t pinNo, INTERRUPTIONEDGES mode);



/*
 * タイマー／ＰＷＭ
 */

#ifdef USE_TIMER

typedef struct {
    uint8_t u8Mode;         //用途 0:DIO 1:Timer 2:PWM 3:AnalogOut 4:MicroCounter 5:Capture(for Timer0) 6:Counter(for Timer0) 7:ADC sampling(DMA)
    bool_t bStartFromHi;    //for PWM
    uint16_t u16HiCount;    //count from start till signal change for Timer/PWM/MicroCounter
    uint16_t u16LoCount;    //count from start till end of cycle for Timer/PWM/MicroCounter
    volatile uint16_t u16HiMicroSeconds; //for MicroCounter
} tsTimerContext2;

extern tsTimerContext2 sTimerApp[5];

extern bool_t timer_attachCallback(uint8_t timerNo, uint8_t prescale, uint16_t cycleCount, void (*func)());
extern bool_t timer_attachCallbackByHz(uint8_t timerNo, uint16_t hz, void (*func)());

extern bool_t timer_attachPWM(uint8_t timerNo, uint8_t prescale, uint16_t cycleCount, uint16_t pulseCount, bool_t bStartFromLo, bool_t bUseSecondPin);
extern bool_t timer_attachPWMByHzDuty(uint8_t timerNo, uint16_t hz, uint16_t duty, bool_t bStartFromHi, bool_t bUseSecondPin);

extern bool_t timer_updatePWMPulseCount(uint8_t timerNo, uint16_t pulseCount);
extern bool_t timer_updatePWMDuty(uint8_t timerNo, uint16_t duty);


extern bool_t timer_detach(uint8_t timerNo);

//#define timer_start(timerNo)     vTimerStart(&sTimerApp[timerNo]) //ToCoNet版
//#define timer_stop(timerNo)      vTimerStop(&sTimerApp[timerNo])  //ToCoNet版

//タイマーを開始
extern bool_t timer_start(uint8_t timerNo);

//タイマーを停止
#define timer_stop(timerNo)         vAHI_TimerStop (timerNo)           //自前版

//16ビットのタイマーカウントを読む
//例えばtimer_attachCallbackでコールバック関数をNULL指定し、タイマーのみを走らせて、この関数で経過時間を測定するなどが可能
#define timer_readCount(timerNo)    u16AHI_TimerReadCount(timerNo)

extern bool_t timer_attachAnalogWrite(uint8_t timerNo, uint16_t power, bool_t bUseSecondPin);
extern bool_t timer_updateAnalogPower(uint8_t timerNo, uint16_t power);

extern bool_t timer_attachMicroCounter(uint8_t timerNo);
extern uint32_t timer_getMicroCount(uint8_t timerNo);

extern bool_t timer0_attachCapture(uint8_t prescale, uint32_t *pu32Buffer, uint16_t u16BufferLength, bool_t bUseSecondPin);
extern uint16_t timer0_getCaptureCount();
extern bool_t timer0_captureCompleted();

extern void timer0_attachCounter(uint8_t prescale, uint16_t count, bool_t bUseSecondPin, INTERRUPTIONEDGES mode, void (*func)());

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

extern bool_t serialx_init(uint8_t u8SerialPort, BAUDRATES baudRate, tsFILE *psUartStream, tsSerialPortSetup *psUartPort, tsUartOpt *psUartOpt);
//bool_t serialx_init(uint8_t u8SerialPort, BAUDRATES baudRate, tsFILE *psUartStream, tsSerialPortSetup *psUartPort);
extern bool_t serialx_forDebug(tsFILE *psUartStream, uint8_t debugLevel);
extern bool_t serialx_write(uint8_t u8SerialPort, uint8_t *pu8Data, uint8_t length);
extern int16_t serialx_readUntil(uint8_t u8SerialPort, uint8_t u8Terminate, uint8_t *pu8Buffer, uint16_t u16BufferLength);
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
//extern bool_t serial_printf(const char* format, ...);
//#define serial_printf(...)         serialx_printf(LB __VA_ARGS__)
//#define serial_printf(...)         do{SPRINTF_vRewind(); vfPrintf(SPRINTF_Stream, LB __VA_ARGS__); uint8_t *p=SPRINTF_pu8GetBuff(); uint8_t *p0=p;while(*p!=0)p++;*(p-2)=0;SERIAL_bTxString(E_AHI_UART_0,p0);}while(0)


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
#ifdef TWELITE_RED
    ADC_SOURCE_5 = E_AHI_ADC_SRC_ADC_5, //DIO2
    ADC_SOURCE_6 = E_AHI_ADC_SRC_ADC_6, //DIO3
#endif
    ADC_SOURCE_TEMP = E_AHI_ADC_SRC_TEMP,
    ADC_SOURCE_VOLT = E_AHI_ADC_SRC_VOLT
} ADCSOURCES;

typedef enum {
    ADC_SOURCE_BITMAP_1 = E_AHI_ADC_DMA_SRC_ADC_1_MASK,
    ADC_SOURCE_BITMAP_2 = E_AHI_ADC_DMA_SRC_ADC_2_MASK,
    ADC_SOURCE_BITMAP_3 = E_AHI_ADC_DMA_SRC_ADC_3_MASK,
    ADC_SOURCE_BITMAP_4 = E_AHI_ADC_DMA_SRC_ADC_4_MASK,
#ifdef TWELITE_RED
    ADC_SOURCE_BITMAP_5 = E_AHI_ADC_DMA_SRC_ADC_5_MASK,
    ADC_SOURCE_BITMAP_6 = E_AHI_ADC_DMA_SRC_ADC_6_MASK,
#endif
    ADC_SOURCE_BITMAP_TEMP = E_AHI_ADC_DMA_SRC_TEMP_MASK,
    ADC_SOURCE_BITMAP_VOLT = E_AHI_ADC_DMA_SRC_VOLT_MASK
} ADCSOURCEBITMAP;

typedef enum {
    ADC_INT_FULL = E_AHI_AP_INT_DMA_END_MASK,
    ADC_INT_HALF = E_AHI_AP_INT_DMA_MID_MASK
} ADCINTERRUPTIONMODE;

extern void adc_setVRef(uint32_t u32VRef);
extern void adc_enable(ADCSAMPLES sample, ADCCLOCKS clock, bool_t bUseExternalVRef);
extern void adc_disable();
extern void adc_attachCallback(bool_t continuous, bool_t range2, ADCSOURCES source, void (*func)(uint16_t value));
extern void adc_detachCallback();

#ifdef USE_TIMER
extern bool_t adc_attachCallbackWithTimerSampling(uint8_t timerNo, uint8_t prescale, uint16_t cycleCount,
        bool_t range2, ADCSOURCEBITMAP bitmap, uint16 *pu16Buffer, uint16 u16BufferSize, bool_t bBufferWrap, ADCINTERRUPTIONMODE mode, void (*func)());
extern bool_t adc_convertResults(ADCSOURCEBITMAP bitmap, uint16 *pu16Buffer, uint16 u16BufferSize);
#endif


/*
 *　コンパレータ
 */

typedef enum {
    COMP_SIGNAL_COMP1M = 0,
    COMP_SIGNAL_DIO17 = 0,
    COMP_SIGNAL_COMP1P = 1,
    COMP_SIGNAL_DIO16 = 1,
} COMPSIGNALSOURCE;

typedef enum {
    COMP_REF_COMP1M = 0,
    COMP_REF_DIO17 = 0,
    COMP_REF_COMP1P = 1,
    COMP_REF_DIO16 = 1,
    COMP_REF_VREF = 2
} COMPVREFSOURCE;

typedef enum {
    COMP_HIS_0MV  = E_AHI_COMP_HYSTERESIS_0MV,
    COMP_HIS_10MV = E_AHI_COMP_HYSTERESIS_10MV,
    COMP_HIS_20MV = E_AHI_COMP_HYSTERESIS_20MV,
    COMP_HIS_40MV = E_AHI_COMP_HYSTERESIS_40MV
} COMPHISTERESIS;

extern bool_t comp_enable(COMPSIGNALSOURCE signal, COMPVREFSOURCE vref, COMPHISTERESIS his, bool_t bLowPowerMode);
extern bool_t comp_attachCallback(INTERRUPTIONEDGES mode, void (*func)());
extern void comp_disable();
extern bool_t comp_setWake(INTERRUPTIONEDGES mode);

//コンパレータの比較結果を取得します。入力が基準より高い場合にTRUE(1)となり、低い場合にFALSE(0)を返します
//この関数の実行にはADCモジュールの電源が必要です(OFFだと再起動かかります)。comp_enabe()でADCモジュールがONされますので、意図的にOFFしない限り大丈夫
#define comp_read()                     (u8AHI_ComparatorStatus() & E_AHI_AP_COMPARATOR_MASK_1)


/*
 *　パルスカウンタ
 */

typedef enum {
    PC_DEBOUNCE_0_MAX100KHZ = 0,
    PC_DEBOUNCE_2_MAX3700HZ = 1,
    PC_DEBOUNCE_4_MAX2200HZ = 2,
    PC_DEBOUNCE_8_MAX1200HZ = 3
} PCDEBOUNCEMODE;

typedef enum {
    PC_INT_BITMAP_0 = E_AHI_SYSCTRL_PC0_MASK,
    PC_INT_BITMAP_1 = E_AHI_SYSCTRL_PC1_MASK,
    PC_INT_BITMAP_32 = E_AHI_SYSCTRL_PC0_MASK
} PCINTBITMAPS;

extern bool_t pc_attachCallback(uint8_t pcNo, PCDEBOUNCEMODE debounce, uint16_t count, bool_t bUseSecondPin, INTERRUPTIONEDGES mode, void (*func)());
extern uint16_t pc_read(uint8_t pcNo);
extern bool_t pc_countReached(uint8_t pcNo);
extern bool_t pc_setWake(uint8_t pcNo, PCDEBOUNCEMODE debounce, uint16_t count, bool_t bUseSecondPin, INTERRUPTIONEDGES mode);

//パルスカウンタを開始します。カウンタは変更されません。pc_attachCallback()の後は実行する必要があります。
#define pc_start(pcNo)                  bAHI_StartPulseCounter(pcNo)

//パルスカウンタを停止します。カウンタは失われません。
#define pc_stop(pcNo)                   bAHI_StopPulseCounter(pcNo)

//パルスカウンタをクリア（０）にします。
#define pc_clear(pcNo)                  bAHI_Clear16BitPulseCounter(pcNo)


extern bool_t pc32_attachCallback(PCDEBOUNCEMODE debounce, uint32_t count, uint8_t pinNo, INTERRUPTIONEDGES mode, void (*func)());
extern uint32_t pc32_read();
extern bool_t pc32_setWake(PCDEBOUNCEMODE debounce, uint32_t count, uint8_t pinNo, INTERRUPTIONEDGES mode);

//パルスカウンタを開始します。カウンタは変更されません。pc32_attachCallback()の後は実行する必要があります。
#define pc32_start()                    bAHI_StartPulseCounter(0)

//パルスカウンタを停止します。カウンタは失われません。
#define pc32_stop()                     bAHI_StopPulseCounter(0)

//パルスカウンタをクリア（０）にします。
#define pc32_clear()                    bAHI_Clear32BitPulseCounter()

//カウンタが設定値に到達したかどうかを返します。
//読み取ったらフラグはクリアされます。
#define pc32_countReached()             pc_countReached(0)



/*
 * Ｉ２Ｃ
 */

typedef enum {
    I2C_ADDRESS_7BIT = 7,
    I2C_ADDRESS_10BIT = 10
} I2CADDRESSINGMODE;
extern I2CADDRESSINGMODE i2cAddressingMode;

#define i2c_setAddressingMode(m)    (i2cAddressingMode=m)

//Operating frequency = 16/[(PreScaler + 1) x 5] MHz
typedef enum {
    I2C_CLOCK_66KHZ		= 47,
    I2C_CLOCK_100KHZ	= 31,
    I2C_CLOCK_200KHZ	= 15,
    I2C_CLOCK_400KHZ	= 7
} I2CCLOCKS;

extern void i2c_enable(I2CCLOCKS clock, bool_t bUseSecondPin);
#define i2c_disable             vAHI_SiMasterDisable()

//コマンド書き込みを伴う書き込み
extern bool_t i2c_write(uint16_t u16Address, uint8_t u8Command, const uint8* pu8Data, uint8_t u8Length);
extern bool_t i2c_writeByte(uint16_t u16Address, uint8_t u8Command, uint8_t u8Data);

//コマンド書き込みを伴う読み込み
extern bool_t i2c_read(uint16_t u16Address, uint8_t u8Command, uint8* pu8Data, uint8_t u8Length);
extern int16_t i2c_readByte(uint16_t u16Address, uint8_t u8Command);

//書き込むだけ
extern bool_t i2c_writeOnly(uint16_t u16Address, const uint8* pu8Data, uint8_t u8Length);

//読み込むだけ
extern bool_t i2c_readOnly(uint16_t u16Address, uint8* pu8Data, uint8_t u8Length);
extern int16_t i2c_readByteOnly(uint16_t u16Address);


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
    SPI_CLOCK_1MHZ = 8,
    SPI_CLOCK_800KHZ = 10,
    SPI_CLOCK_500KHZ = 16,
    SPI_CLOCK_400KHZ = 20,
    SPI_CLOCK_250KHZ = 32
} SPICLOCKS;

extern bool_t spi_enable(uint8_t u8NumSlaves, SPIMODES u8Mode, SPICLOCKS u8Divider);

//SPIマスターを無効にする
#define spi_disable()           vAHI_SpiDisable()

extern bool_t spi_selectSlavePin(uint8_t slaveNo, bool_t bSecondPin);
extern void spi_selectSlave(int8_t slaveNo);

//spi_selectSlave(-1)と同意
#define spi_stop()              vAHI_SpiStop()

extern uint8_t spi_readByte(int8_t slaveNo, uint8_t u8Command);
extern void spi_read(int8_t slaveNo, uint8_t u8Command, uint8* pu8Data, uint8_t u8Length);
extern void spi_writeByte(int8_t slaveNo, uint8_t u8Command, uint8_t u8Data);


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
extern bool_t radio_printf(uint32_t u32DestAddr, const char* format, va_list args);
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