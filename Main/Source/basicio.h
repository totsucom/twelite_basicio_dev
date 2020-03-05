/*
 * basicio.h
 * バージョン 3.1
 * 2020/3/5 totsucom
 */

#ifndef __BASICIO_H
#define __BASICIO_H

#define BASICIO_MAJOR_VER   3
#define BASICIO_MINOR_VER   1


#include "AppHardwareApi.h"
#include "utils.h"
#include "ToCoNet.h"
#include "string.h"

#define ToCoNet_USE_MOD_RAND_XOR_SHIFT
#include "ToCoNet_mod_prototype.h"

#include "basicio_module.h"

#if defined(USE_SERIAL) || defined(USE_SERIAL1)
#include "serial.h"
#endif

//printf()用
#include <stdarg.h>


/*
 * 条件コンパイル
 */

//シリアル0のフロー制御にTimerを用いる
#if defined(USE_SERIAL) && !defined(USE_TIMER)
#define USE_TIMER
#endif

//ＡＤＣ，コンパレータ，パルスカウンタでDIOを使用
#if (defined(USE_ADC) || defined(USE_COMP) || defined(USE_PC) || defined(USE_LEDUTIL)) && !defined(USE_DIO)
#define USE_DIO
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
 * 定数
 */

#define DIO_MONOSTICK_LED   16

#define LOW                 0
#define HIGH                1

//フラッシュメモリ定数
#define FLASH_SECTOR_SIZE (32L* 1024L) // 32KB
#ifdef TWELITE_RED
#define FLASH_LAST_SECTOR   15  //RED
#else
#define FLASH_LAST_SECTOR   4   //BLUE
#endif




/*
 * 未定義定数についてデフォルト値を設定
 */

//チャタリング防止のためのディレイ
#ifndef PB_JUDGE_COUNT
#define PB_JUDGE_COUNT 5
#endif

//DIO割り込みのコールバック関数登録バッファの大きさ
#ifndef MAX_DIO_INTERRUPT_FUNCS
#define MAX_DIO_INTERRUPT_FUNCS 5
#endif

/*未対応
#if defined(SERIAL_HW_FLOW_CONTROL) && defined(SERIAL_XON_XOFF_FLOW_CONTROL)
#error You can not choose both flow control 'SERIAL_HW_FLOW_CONTROL' and 'SERIAL_XON_XOFF_FLOW_CONTROL' same time
#endif

//シリアル0のソフトウェアフロー制御 XONコード
#ifndef SERIAL_XON
#define SERIAL_XON      0x11
#endif

//シリアル0のソフトウェアフロー制御 XOFFコード
#ifndef SERIAL_XOFF
#define SERIAL_XOFF     0x13
#endif
*/

//シリアル0送信FIFOバッファ(16～2047)
#ifndef SERIAL_TX_BUFFER_SIZE
#define SERIAL_TX_BUFFER_SIZE 96
#endif

//シリアル0受信FIFOバッファ(16～2047)
#ifndef SERIAL_RX_BUFFER_SIZE
#define SERIAL_RX_BUFFER_SIZE 32
#endif

//シリアル1送信FIFOバッファ(16～2047)
#ifndef SERIAL1_TX_BUFFER_SIZE
#define SERIAL1_TX_BUFFER_SIZE 96
#endif

//シリアル1受信FIFOバッファ(16～2047)
#ifndef SERIAL1_RX_BUFFER_SIZE
#define SERIAL1_RX_BUFFER_SIZE 32
#endif

//I2Cスレーブでマスタの書き込むデータを受けるバッファサイズ
#ifndef I2CS_MW_BUFFER_SIZE
#define I2CS_MW_BUFFER_SIZE 20
#endif

//I2Cスレーブでマスターの読み込みが行われるときにデータが準備されなかった場合に返す値
#ifndef I2CS_MR_DATA_NOT_READY
#define I2CS_MR_DATA_NOT_READY 0
#endif

//I2Cスレーブでマスターの読み込みが行われるとき、準備したデータの範囲外を参照した場合に返す値
#ifndef I2CS_MR_OUT_OF_RANGE
#define I2CS_MR_OUT_OF_RANGE 255
#endif

//無線送信データが相手に届かない場合、リトライ送信する回数
#ifndef TX_RETRY
#define TX_RETRY 2
#endif

//sb_printf()等で使用するバッファサイズを指定します 32/64/128/256/512/1024のいずれか
#ifndef SB_BUFFER_SIZE
#define SB_BUFFER_SIZE 128
#endif

//led_define()で登録できるLEDの最大数
#ifndef MAX_LED
#define MAX_LED     2
#endif

//led_setPattern()で同時に付加できるパターンの最大数
#ifndef MAX_LED_PATTERN
#define MAX_LED_PATTERN 4
#endif

//TICK TIMER 1秒間の回数
#ifndef TICK_COUNT
#define TICK_COUNT  250
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


extern uint32_t millis();


//32bit乱数を生成
#define random()                      ToCoNet_u32GetRand()

//モジュールの32bitアドレス。モジュールのシリアル番号を元にシステムにより自動生成される
//モジュールを識別する番号として用いる
#define getModuleAddress()          ToCoNet_u32GetSerial()

extern bool_t sleepCalibratedTimer(uint64_t milliSeconds, bool_t bRAMPower, uint32_t calibValue);
extern uint32_t wakeTimer_getCalibrationValue();

//ウェイクタイマーが動いているか返します
//タイマー起床後のsetup()内でのみTRUEを返します
#define wakeTimer_isRunning()       (u8AHI_WakeTimerStatus() & E_AHI_WAKE_TIMER_MASK_0 ? TRUE : FALSE)

//ウェイクタイマーの現在のカウント値を返します。uint16_t 型であることに注意してください
//ウェイクタイマーは約32KHzのカウントダウンタイマーで、0を過ぎると0x1FFFFFFFFFFになります。
#define wakeTimer_readCount()       u64AHI_WakeTimerReadLarge(E_AHI_WAKE_TIMER_0)

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

//キュー関数
typedef struct {
    uint8_t             *pau8Buff;
    volatile uint16_t   u16BuffSize;
    volatile uint16_t   u16DataSize;
    volatile uint16_t   u16WriteIndex;
    volatile uint16_t   u16ReadIndex;
    volatile bool_t     bDataLost;
   	volatile uint32_t   u32InterruptStore;
} BYTEQUE;
extern void que_init(BYTEQUE *psQue, uint8_t *pau8Buff, uint16_t u16BuffSize);
extern void que_clear(BYTEQUE *psQue);
#define que_getCount(que)               ((que)->u16DataSize)
#define que_bufferEmpty(que)            ((que)->u16DataSize == 0)
#define que_bufferFull(que)             ((que)->u16DataSize == (que)->u16BuffSize)
extern bool_t que_dataLost(BYTEQUE *psQue);
extern void que_append(BYTEQUE *psQue, uint8_t byte);
extern int16_t que_get(BYTEQUE *psQue);

//printf関数
bool_t myprintf(bool_t (*__putc)(char), const char *fmt, va_list ap);


#ifdef USE_SBUTIL
extern void sb_clear();
extern bool_t sb_putc(char c);
extern bool_t sb_puts(const char *str);
extern bool_t sb_printf(const char *fmt, ...);
extern const char *sb_getBuffer();
#endif

#ifdef USE_PBUTIL
extern void pb_define(uint8_t pinNo, bool_t bPressToHi);
extern bool_t pb_pressed(uint8_t pinNo);
extern bool_t pb_released(uint8_t pinNo);
extern bool_t pb_currentState(uint8_t pinNo);
extern void pb_reset();
#endif

#ifdef USE_LEDUTIL
extern bool_t led_define(uint8_t pinNo, bool_t bHighToOn, bool_t bDefaultIsOn);
extern uint8_t led_setPattern(uint8_t pinNo, uint16_t u16OnTime, uint16_t u16OffTime, bool_t bStartFromOn, int16_t i16Cycle, uint8_t u8Priority, void (*callbackFunction)(uint8_t u8Id));
extern void led_clearPattern(uint8_t u8Id);
extern void led_clearDioPattern(uint8_t pinNo);
#endif

typedef enum {
    DISABLE,        //dio_attachCallback,dio_setWake,timer0_attachCounter
    RISING,         //dio_attachCallback,dio_setWake,timer0_attachCounter
    FALLING,        //dio_attachCallback,dio_setWake,timer0_attachCounter
    BOTHEDGE        //timer0_attachCounter
} INTERRUPTIONEDGES;




/*
 * デジタルIO
 */

#ifdef USE_DIO

typedef enum {
    INPUT,
    INPUT_PULLUP,
    OUTPUT
} PINMODES;

extern bool_t dio_pinMode(uint8_t pinNo, PINMODES mode);
extern bool_t dio_write(uint8_t pinNo, uint8_t value);

extern bool_t do_enable(bool_t bEnable);
extern bool_t do_write(uint8_t pinNo, uint8_t value);


//pinNO=0..19 返り値 LOW(0)/HIGH(1)
#define dio_read(pinNo)         ((u32AHI_DioReadInput() & (1UL << pinNo)) ? HIGH : LOW)

//DIO0～DIO19の出力状態を一度に設定する。
//onBits: HIGHにしたいDIOのビットを1に設定
//offBits: LOWにしたいDIOのビットを1に設定
#define dio_writeAll(onBits, offBits)   vAHI_DioSetOutput(onBits, offBits)

//DIO0～DIO19の入力状態を一度に読み込む。bit=0:LOW/1:HIGH
#define dio_readAll()           u32AHI_DioReadInput()


extern bool_t dio_attachCallback(uint8_t pinNo, INTERRUPTIONEDGES mode, void (*func)(uint32_t u32DioBitmap));
extern bool_t dio_detach(uint8_t pinNo);
extern bool_t dio_setWake(uint8_t pinNo, INTERRUPTIONEDGES mode);

#endif //USE_DIO



/*
 * タイマー／ＰＷＭ
 */

#ifdef USE_TIMER

typedef struct {
    uint8_t u8Mode;         //用途 0:DIO 1:Timer 2:PWM 3:AnalogOut 4:MicroCounter 5:Capture(for Timer0) 6:Counter(for Timer0) 7:ADC sampling(DMA) 8:PWM(Sync)
    bool_t bStartFromHi;    //for PWM
    uint16_t u16HiCount;    //count from start till signal change for Timer/PWM/MicroCounter
    uint16_t u16LoCount;    //count from start till end of cycle for Timer/PWM/MicroCounter
    volatile uint16_t u16HiMicroSeconds; //for MicroCounter
    uint16_t u16ReservedHiCount; //for PWM(Sync)
} tsTimerContext2;

extern tsTimerContext2 sTimerApp[5];

extern bool_t timer_attachCallback(uint8_t timerNo, uint8_t prescale, uint16_t cycleCount, bool_t bStartNow, void (*func)());
extern bool_t timer_attachCallbackByHz(uint8_t timerNo, uint16_t hz, bool_t bStartNow, void (*func)());

typedef enum {
    DEFAULT_PIN = 0,    //FALSE
    SECOND_PIN = 1,     //TRUE
    DO_PIN = 2          //Timer2,3専用
} TIMEROPINSELECTION;

extern bool_t timer_attachPWM(uint8_t timerNo, uint8_t prescale, uint16_t cycleCount, uint16_t pulseCount, bool_t bStartFromLo, TIMEROPINSELECTION pinSelection, bool_t bSyncUpdate, bool_t bStartNow);
extern bool_t timer_attachPWMByHzDuty(uint8_t timerNo, uint16_t hz, uint16_t duty, bool_t bStartFromHi, TIMEROPINSELECTION pinSelection, bool_t bSyncUpdate, bool_t bStartNow);

extern int32_t timer_getPWMPulseCountULimit(uint8_t timerNo);
extern bool_t timer_updatePWM(uint8_t timerNo, uint16_t pulseCount);
extern bool_t timer_updatePWMDuty(uint8_t timerNo, uint16_t duty);


extern bool_t timer_detach(uint8_t timerNo);

//#define timer_start(timerNo)     vTimerStart(&sTimerApp[timerNo]) //ToCoNet版
//#define timer_stop(timerNo)      vTimerStop(&sTimerApp[timerNo])  //ToCoNet版

//タイマーを開始
extern bool_t timer_start(uint8_t timerNo);

//タイマーを停止
#define timer_stop(timerNo)         vAHI_TimerStop (timerNo)

//16ビットのタイマーカウントを読む
//例えばtimer_attachCallbackでコールバック関数をNULL指定し、タイマーのみを走らせて、この関数で経過時間を測定するなどが可能
#define timer_readCount(timerNo)    u16AHI_TimerReadCount(timerNo)

extern bool_t timer_attachAnalogWrite(uint8_t timerNo, uint16_t power, TIMEROPINSELECTION pinSelection);
extern bool_t timer_updateAnalogPower(uint8_t timerNo, uint16_t power);

extern bool_t timer_attachMicroCounter(uint8_t timerNo, bool_t bStartNow);
extern uint32_t timer_getMicroCount(uint8_t timerNo);

extern bool_t timer0_attachCapture(uint8_t prescale, uint32_t *pu32Buffer, uint16_t u16BufferLength, bool_t bUseSecondPin);
extern uint16_t timer0_getCaptureCount();
extern bool_t timer0_captureCompleted();

//取得した32ビットデータからパルスのLOW時間[μ秒]を取得する
#define timer0_captureValueToLowTime(prescale, u32Value)    (prescale >= 4 ? (u32Value >> 16) << (prescale - 4) : (u32Value >> 16) >> (4 - prescale)) 

//取得した32ビットデータからパルスの周期[μ秒]を取得する
#define timer0_captureValueToCycleTime(prescale, u32Value)  (prescale >= 4 ? (u32Value & 0xffff) << (prescale - 4) : (u32Value & 0xffff) >> (4 - prescale)) 

//取得した32ビットデータからパルスのパルス幅[μ秒]を取得する
#define timer0_captureValueToPulseTime(prescale, u32Value)  (timer0_captureValueToCycleTime(prescale, u32Value) - timer0_captureValueToLowTime(prescale, u32Value))

extern bool_t timer0_attachCounter(uint8_t prescale, uint16_t count, bool_t bUseSecondPin, INTERRUPTIONEDGES mode, void (*func)());

#endif //USE_TIMER


/*
 * シリアル
 */

#if defined(USE_SERIAL) || defined(USE_SERIAL1)

typedef enum {
    SERIAL_BAUD_4800   = E_AHI_UART_RATE_4800,  //(4800 bps)
    SERIAL_BAUD_9600   = E_AHI_UART_RATE_9600,  //(9600 bps)
    SERIAL_BAUD_19200  = E_AHI_UART_RATE_19200, //(19200 bps)
    SERIAL_BAUD_38400  = E_AHI_UART_RATE_38400, //(38400 bps)
    SERIAL_BAUD_76800  = E_AHI_UART_RATE_76800, //(76800 bps)
    SERIAL_BAUD_115200 = E_AHI_UART_RATE_115200 //(115200 bps)
} SERIALBAUD;

typedef enum {
    SERIAL_PARITY_NONE,
    SERIAL_PARITY_EVEN,
    SERIAL_PARITY_ODD
} SERIALPARITY;

typedef enum {
    SERIAL_STOP_1BIT = E_AHI_UART_1_STOP_BIT,
    SERIAL_STOP_2BIT = E_AHI_UART_2_STOP_BITS
} SERIALSTOPBIT;

typedef enum {
    SERIAL_LENGTH_5BITS = E_AHI_UART_WORD_LEN_5,
    SERIAL_LENGTH_6BITS = E_AHI_UART_WORD_LEN_6,
    SERIAL_LENGTH_7BITS = E_AHI_UART_WORD_LEN_7,
    SERIAL_LENGTH_8BITS = E_AHI_UART_WORD_LEN_8
} SERIALBITLENGTH;

typedef enum {
    SERIAL_HWFC_NONE = -1,
    SERIAL_HWFC_TIMER0 = 0,
    SERIAL_HWFC_TIMER1 = 1,
    SERIAL_HWFC_TIMER2 = 2,
    SERIAL_HWFC_TIMER3 = 3,
    SERIAL_HWFC_TIMER4 = 4
} SERIALHWFLOWCONTROL;


//共通関数
extern bool_t serialx_putc(uint8_t serialNo, uint8_t u8Data);
extern bool_t serialx_write(uint8_t serialNo, const uint8_t *pau8Data, uint16_t u16Length);
extern bool_t serialx_puts(uint8_t serialNo, const char *pau8String);

#endif //USE_SERIAL || USE_SERIAL1


#ifdef USE_SERIAL

extern bool_t serial_initEx(SERIALBAUD baudRate, SERIALPARITY parity, SERIALBITLENGTH bitLength, SERIALSTOPBIT stopBit, bool_t bUseSecondPin, SERIALHWFLOWCONTROL flowControl);
//extern bool_t serial_initEx(SERIALBAUD baudRate, SERIALPARITY parity, SERIALBITLENGTH bitLength, SERIALSTOPBIT stopBit, bool_t bUseSecondPin);
extern void serial_disable();
extern bool_t serial_dataLost();
extern uint16_t serial_getRxCount();
extern int16_t serial_getc();
extern int16_t serial_readUntil(uint8_t u8Terminate, uint8_t *pu8Buffer, uint16_t u16Length);
//extern bool_t serial_printf(const char* format, va_list args);
//#define serial_printf(...)   do{ SPRINTF_vRewind(); vfPrintf(SPRINTF_Stream, LB __VA_ARGS__); serialx_write(E_AHI_UART_0, (uint8_t *)SPRINTF_pu8GetBuff(), (uint16_t)strlen((const char *)SPRINTF_pu8GetBuff())); }while(0)
extern bool_t serial_printf(const char *fmt, ...);
extern bool_t serial_ready();

//シリアル0を初期化する
#define serial_init(baudRate)               serial_initEx(baudRate, SERIAL_PARITY_NONE, SERIAL_LENGTH_8BITS, SERIAL_STOP_1BIT, FALSE, SERIAL_HWFC_NONE)

//シリアル0に1バイト書き出す。バッファがいっぱいの場合は書き出さずにFALSEを返す
#define serial_putc(u8Data)                 serialx_putc(E_AHI_UART_0, u8Data)

//シリアル0にバイト配列を書き出す。バッファがいっぱいの場合は書き出さずにFALSEを返す
#define serial_write(pau8Data, u16Length)   serialx_write(E_AHI_UART_0, pau8Data, u16Length)

//シリアル0に文字列を書き出す。バッファがいっぱいの場合は書き出さずにFALSEを返す
#define serial_puts(pau8String)             serialx_puts(E_AHI_UART_0, pau8String)

//シリアル0に文字列を書き出す。バッファがいっぱいの場合は書き出さずにFALSEを返す
//#define serial_puts(pau8String)             serialx_write(E_AHI_UART_0, (uint8_t *)pau8String, (uint16_t)strlen((const char *)pau8String))

//シリアル0の送信バッファのデータ数を返す
#define serial_getTxCount()                 u16AHI_UartReadTxFifoLevel(E_AHI_UART_0)

//シリアル0の送信バッファをクリアする
#define serial_resetTx()                    vAHI_UartReset(E_AHI_UART_0, TRUE, FALSE)

//シリアル0の受信バッファをクリアする
extern void serial_resetRx();

#endif //USE_SERIAL


#ifdef USE_SERIAL1

extern bool_t serial1_initEx(SERIALBAUD baudRate, SERIALPARITY parity, SERIALBITLENGTH bitLength, SERIALSTOPBIT stopBit, bool_t bUseTxOnly, bool_t bUseSecondPin);
#define serial1_disable()                        vAHI_UartDisable(E_AHI_UART_1)
extern bool_t serial1_dataLost();

//シリアル1の受信バッファのデータ数を返す
#define serial1_getRxCount()           u16AHI_UartReadRxFifoLevel(E_AHI_UART_1)
extern int16_t serial1_getc();
extern int16_t serial1_readUntil(uint8_t u8Terminate, uint8_t *pu8Buffer, uint16_t u16Length);
//extern bool_t serial1_printf(const char* format, va_list args);
//#define serial1_printf(...)   do{ SPRINTF_vRewind(); vfPrintf(SPRINTF_Stream, LB __VA_ARGS__); serialx_write(E_AHI_UART_1, (uint8_t *)SPRINTF_pu8GetBuff(), (uint16_t)strlen((const char *)SPRINTF_pu8GetBuff())); }while(0)
extern bool_t serial1_printf(const char *fmt, ...);

//シリアル1を初期化する
#define serial1_init(baudRate)   serial1_initEx(baudRate, SERIAL_PARITY_NONE, SERIAL_LENGTH_8BITS, SERIAL_STOP_1BIT, FALSE, FALSE)

//シリアル1に1バイト書き出す。バッファがいっぱいの場合は書き出さずにFALSEを返す
#define serial1_putc(u8Data)                serialx_putc(E_AHI_UART_1, u8Data)

//シリアル1にバイト配列を書き出す。バッファがいっぱいの場合は書き出さずにFALSEを返す
#define serial1_write(pau8Data, u16Length)  serialx_write(E_AHI_UART_1, pau8Data, u16Length)

//シリアル1に文字列を書き出す。バッファがいっぱいの場合は書き出さずにFALSEを返す
#define serial1_puts(pau8String)            serialx_puts(E_AHI_UART_1, pau8String)

//シリアル1に文字列を書き出す。バッファがいっぱいの場合は書き出さずにFALSEを返す
//#define serial1_puts(pau8String)            serialx_write(E_AHI_UART_1, (uint8_t *)pau8String, (uint16_t)strlen(pau8String))

//シリアル1の送信バッファのデータ数を返す
#define serial1_getTxCount()                u16AHI_UartReadTxFifoLevel(E_AHI_UART_1)

//送信可能かを返す。フロー制御を持たないため常にTRUEを返す
//#define serial1_ready()                     TRUE

//シリアル0の送信バッファをクリアする
#define serial1_resetTx()                   vAHI_UartReset(E_AHI_UART_1, TRUE, FALSE)

//シリアル0の受信バッファをクリアする
#define serial1_resetRx()                   vAHI_UartReset(E_AHI_UART_1, FALSE, TRUE)

#endif //USE_SERIAL1



/*
 * ＡＤＣ
 */

#if defined(USE_ADC) || defined(USE_COMP)
extern void adc_disable();
#endif

#ifdef USE_ADC

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
    ADC_SOURCE_BITMAP_1 = E_AHI_ADC_DMA_SRC_ADC_1_MASK,//1<<0
    ADC_SOURCE_BITMAP_2 = E_AHI_ADC_DMA_SRC_ADC_2_MASK,
    ADC_SOURCE_BITMAP_3 = E_AHI_ADC_DMA_SRC_ADC_3_MASK,
    ADC_SOURCE_BITMAP_4 = E_AHI_ADC_DMA_SRC_ADC_4_MASK,//1<<3
#ifdef TWELITE_RED
    //ADC_SOURCE_BITMAP_5 = E_AHI_ADC_DMA_SRC_ADC_5_MASK,//1<<8
    //ADC_SOURCE_BITMAP_6 = E_AHI_ADC_DMA_SRC_ADC_6_MASK,//1<<9
#endif
    ADC_SOURCE_BITMAP_TEMP = E_AHI_ADC_DMA_SRC_TEMP_MASK,//1<<4
    ADC_SOURCE_BITMAP_VOLT = E_AHI_ADC_DMA_SRC_VOLT_MASK//1<<5
} ADCSOURCEBITMAP;

typedef enum {
    ADC_INT_FULL = E_AHI_AP_INT_DMA_END_MASK,
    ADC_INT_HALF = E_AHI_AP_INT_DMA_MID_MASK,
    ADC_INT_HALF_FULL = (E_AHI_AP_INT_DMA_END_MASK | E_AHI_AP_INT_DMA_MID_MASK)
} ADCINTERRUPTIONMODE;

extern void adc_setVRef(uint32_t u32VRef);
extern void adc_enable(ADCSAMPLES sample, ADCCLOCKS clock, bool_t bUseExternalVRef);
extern void adc_attachCallback(bool_t continuous, bool_t range2, ADCSOURCES source, void (*func)(uint16_t value));
extern void adc_detach();

#ifdef USE_TIMER
extern bool_t adc_attachCallbackWithTimer(uint8_t timerNo, uint8_t prescale, uint16_t cycleCount,
        bool_t range2, ADCSOURCEBITMAP bitmap, uint16_t *pu16Buffer, uint16_t u16BufferSize, bool_t bBufferWrap, ADCINTERRUPTIONMODE mode, void (*func)());
extern bool_t adc_convertResults(ADCSOURCEBITMAP bitmap, uint16_t *pu16Buffer, uint16_t u16BufferSize);
#endif

#endif //USE_ADC



/*
 *　コンパレータ
 */

#ifdef USE_COMP

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
extern bool_t comp_read();

#define comp_detach()               comp_attachCallback(FALLING, NULL)

#endif //USE_COMP



/*
 *　パルスカウンタ
 */

#ifdef USE_PC

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

extern bool_t pc_enable(uint8_t pcNo, PCDEBOUNCEMODE debounce, uint16_t count, bool_t bUseSecondPin, INTERRUPTIONEDGES mode, bool_t bStartNow);
extern bool_t pc_disable(uint8_t pcNo);
extern bool_t pc_attachCallback(uint8_t pcNo, void (*func)());
extern uint16_t pc_read(uint8_t pcNo);
extern bool_t pc_countReached(uint8_t pcNo);

//コールバック関数を削除します。パルスカウンタと割り込みは停止しません。
#define pc_detach(pcNo)                 pc_attachCallback(pcNo, NULL)

//パルスカウンタを開始します。カウンタは変更されません。pc_attachCallback()の後は実行する必要があります。
#define pc_start(pcNo)                  bAHI_StartPulseCounter(pcNo)

//パルスカウンタを停止します。カウンタは失われません。
#define pc_stop(pcNo)                   bAHI_StopPulseCounter(pcNo)

//パルスカウンタをクリア（０）にします。
#define pc_clear(pcNo)                  bAHI_Clear16BitPulseCounter(pcNo)

//パルスカウンタを起床条件に設定します
//事前にpc_enable()でパルスカウンタを開始しておく必要があります。
//この関数の実行でカウンタはクリアされません。必要に応じてpc_clear()を呼び出してください。
//debounceにPC_DEBOUNCE_0_MAX100KHZ以外を使用する場合はスリープで32kHzRCオシレータを停止してはいけません。
//スリープ関数の直前に呼び出してください。
#define pc_setWake()                    u32AHI_PulseCounterStatus()    //clear flag

extern bool_t pc32_enable(PCDEBOUNCEMODE debounce, uint32_t count, uint8_t pinNo, INTERRUPTIONEDGES mode, bool_t bStartNow);
extern uint32_t pc32_read();
extern bool_t pc32_setWake(PCDEBOUNCEMODE debounce, uint32_t count, uint8_t pinNo, INTERRUPTIONEDGES mode);

//32ビットパルスカウンタを無効にします
#define pc32_disable()                  pc_disable(0)

//32ビットパルスカウンタにコールバック関数を登録します。
//パルスカウンタは事前にpc32_enable()で開始しておく必要があります。
//func=NULLで登録を解除できます(パルスカウンタは停止しません)。
#define pc32_attachCallback(func)       pc_attachCallback(0, func)

//コールバック関数を削除します。パルスカウンタと割り込みは停止しません。
#define pc32_detach()                   pc_attachCallback(0, NULL)

//パルスカウンタを開始します。カウンタは変更されません。pc32_attachCallback()の後は実行する必要があります。
#define pc32_start()                    bAHI_StartPulseCounter(0)

//パルスカウンタを停止します。カウンタは失われません。
#define pc32_stop()                     bAHI_StopPulseCounter(0)

//パルスカウンタをクリア（０）にします。
#define pc32_clear()                    bAHI_Clear32BitPulseCounter()

//カウンタが設定値に到達したかどうかを返します。
//読み取ったらフラグはクリアされます。
#define pc32_countReached()             pc_countReached(0)

//パルスカウンタを起床条件に設定します
//事前にpc_enable()でパルスカウンタを開始しておく必要があります。
//この関数の実行でカウンタはクリアされません。必要に応じてpc_clear()を呼び出してください。
//debounceにPC_DEBOUNCE_0_MAX100KHZ以外を使用する場合はスリープで32kHzRCオシレータを停止してはいけません。
//スリープ関数の直前に呼び出してください。
#define pc32_setWake()                  u32AHI_PulseCounterStatus()    //clear flag

#endif //USE_PC



/*
 * Ｉ２Ｃ
 */

#ifdef USE_I2C

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
extern void i2c_disable();

//コマンド書き込みを伴う書き込み
extern bool_t i2c_write(uint16_t u16Address, uint8_t u8Command, const uint8* pu8Data, uint8_t u8Length);
extern bool_t i2c_writeByte(uint16_t u16Address, uint8_t u8Command, uint8_t u8Data);

//コマンド書き込みを伴う読み込み
extern bool_t i2c_read(uint16_t u16Address, uint8_t u8Command, uint8* pu8Data, uint8_t u8Length);
extern int16_t i2c_readByte(uint16_t u16Address, uint8_t u8Command);

//書き込むだけ
extern bool_t i2c_writeOnly(uint16_t u16Address, const uint8* pu8Data, uint8_t u8Length);
extern bool_t i2c_writeByteOnly(uint16_t u16Address, uint8_t u8Data);

//読み込むだけ
extern bool_t i2c_readOnly(uint16_t u16Address, uint8* pu8Data, uint8_t u8Length);
extern int16_t i2c_readByteOnly(uint16_t u16Address);

#endif //USE_I2C


/*
 * スレーブ
 */

#ifdef USE_I2CS
extern void i2cs_enable(uint8_t u16Address, bool_t b10BitAddress, bool_t bUseSecondPin, void (*prepareFunc)(uint8_t), void (*receivedFunc)(uint8_t*, uint8_t));
extern void i2cs_disable();
extern void i2cs_write(const uint8_t *pau8Data, uint8_t u8Length);
#endif //USE_I2CS



/*
 * ＳＰＩ
 */

#ifdef USE_SPI

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

extern bool_t spi_enable(uint8_t u8NumSlaves, SPIMODES u8Mode, SPICLOCKS clock);

//SPIマスターを無効にする
#define spi_disable()           vAHI_SpiDisable()

extern bool_t spi_selectSlavePin(uint8_t slaveNo, bool_t bSecondPin);

extern uint8_t spi_readByte(int8_t slaveNo, uint8_t u8Command);
extern void spi_read(int8_t slaveNo, uint8_t u8Command, uint8* pu8Data, uint8_t u8Length);
extern void spi_writeByte(int8_t slaveNo, uint8_t u8Command, uint8_t u8Data);


//低レベル入出力
#define spi_select(slaveNo)     vAHI_SpiSelect(1 << slaveNo)
#define spi_deselect()          vAHI_SpiSelect(0)
extern void spi_write8(uint8_t u8Data);
extern void spi_write16(uint16_t u16Data);
extern void spi_write32(uint32_t u32Data);
#define spi_read8()             u8AHI_SpiReadTransfer8()
#define spi_read16()            u16AHI_SpiReadTransfer16()
#define spi_read32()            u32AHI_SpiReadTransfer32()

#endif //USE_SPI


/*
 * 無線通信
 */

#ifdef USE_RADIO
//#include "sprintf.h"

#define RADIO_ADDR_BROADCAST    TOCONET_MAC_ADDR_BROADCAST

typedef enum {
    RADIO_MODE_OFF,      //無線通信を使用しない
    RADIO_MODE_TXONLY,   //送信のみ行う
    RADIO_MODE_TXRX      //送受信を行う
} RADIOMODE;

extern bool_t radio_setupInit(RADIOMODE mode, uint32_t appid, uint8_t channel, uint8_t txPower);
extern void radio_setupShortAddress(uint16_t u16ShortAddress);
extern bool_t radio_setRetry(uint8_t retryCount, uint16_t retryDuration);
extern void radio_setCbId(uint8_t u8CbId);

extern void radio_attachCallback(void (*txFunc)(uint8_t u8CbId, bool_t bSuccess), void (*rxFunc)(uint32_t u32SrcAddr, bool_t bBroadcast, uint8_t u8CbId, uint8_t u8DataType, uint8_t *pu8Data, uint8_t u8Length, uint8_t u8Lqi));
extern void radio_setRxGateCallback(bool_t (*gateFunc)(uint32_t u32SrcAddr, uint8_t u8CbId));
#define radio_detach()          radio_attachCallback(NULL, NULL)

/*
extern void radio_attachTxCallback(void (*func)(uint8_t u8CbId, bool_t bSuccess));
#ifdef USE_RADIO
extern void radio_attachRxCallback(void (*func)(uint32_t u32SrcAddr, uint8_t u8CbId, uint8_t u8DataType, uint8_t *pu8Data, uint8_t u8Length, uint8_t u8Lqi));
#endif

#define radio_detachTxCallback()  radio_attachTxCallback(NULL)
#define radio_detachRxCallback()  radio_attachRxCallback(NULL)
*/

extern int16_t radio_write(uint32_t u32DestAddr, uint8_t u8DataType, uint8_t *pu8Data, uint8_t u8Length);
extern int16_t radio_puts(uint32_t u32DestAddr, uint8_t u8DataType, const char *pu8String);

//extern bool_t radio_printf(uint32_t u32DestAddr, const char* format, va_list args);
//#define radio_printf(u32DestAddr,u8DataType, ...)     do{ SPRINTF_vRewind(); vfPrintf(SPRINTF_Stream, LB __VA_ARGS__); radio_puts(u32DestAddr, (const char *)SPRINTF_pu8GetBuff(), u8DataType); }while(0)
extern int16_t radio_printf(uint32_t u32DestAddr, uint8_t u8DataType, const char* fmt, ...);
extern uint8_t radio_txCount();

#endif //USE_RADIO
 

/*
 * EEPROM
 */

#ifdef USE_EEPROM

extern uint16_t eeprom_getSegmentCount();
extern uint8_t eeprom_getSegmentSize();
extern bool_t eeprom_erased(uint16_t u16SegIndex);


//EEPROMを初期化(消去)します。
//全てのビットは0にセットされます。
//成功でTRUEを返します。
#define eeprom_erase(u16SegIndex)                                   (iAHI_EraseEEPROMsegment(u16SegIndex) == 0)

//EEPROMからデータを読み出します。
//成功でTRUEを返します。
#define eeprom_read(u16SegIndex, u8Offset, pu8Buffer, u8Length)     (iAHI_ReadDataFromEEPROMsegment(u16SegIndex, u8Offset, pu8Buffer, u8Length) == 0)

//EEPROMにデータを書き込みます。
//成功でTRUEを返します。
#define eeprom_write(u16SegIndex, u8Offset, pu8Buffer, u8Length)    (iAHI_WriteDataIntoEEPROMsegment(u16SegIndex, u8Offset, pu8Buffer, u8Length) == 0)

#endif //USE_EEPROM


/*
 * FRASHメモリ
 */

#ifdef USE_FLASH

extern bool_t flash_erase(uint8_t sector);
extern bool_t flash_erased(uint8_t sector, uint16_t offset, uint16_t u16Length);
extern bool_t flash_write(uint8_t sector, uint16_t offset, uint8_t *pu8Data, uint16_t u16Length);
extern bool_t flash_read(uint8_t sector, uint32 offset, uint8_t *pu8Data, uint16_t u16Length);

#endif //USE_FLASH


#endif  //__BASICIO_H