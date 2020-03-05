/*
 * basicio.h
 * バージョン 3.1
 * 2020/3/5 totsucom
 */

#include "basicio.h"


//myprintf()用書き込み補助関数
static char *__printf_putc_ptr;
static uint16_t __printf_putc_count;
static uint16_t __printf_putc_size;

#define __printf_putc_init(p,l) if(1) { __printf_putc_ptr = p; __printf_putc_count = 0; __printf_putc_size = l; }

static bool_t __printf_putc(char c) {
    if (__printf_putc_count == __printf_putc_size) return FALSE;
    *__printf_putc_ptr++ = c;
    __printf_putc_count++;
    return TRUE;
}


static volatile uint32_t millisValue;
static uint32_t millisValueTick;

//ミリ秒を計測するカウントアップタイマー。カウントアップ値はTICK_TIMERによる。デフォルト4ms。
uint32_t millis() {
    return millisValue;
}


#ifdef USE_PBUTIL

static uint32_t u32PBDefined;       //ボタン登録されたDIO
static uint32_t u32PBPositive;      //押されたときにHになるボタン
static uint32_t u32PBCurIOStatus;   //現在のボタンのDIO状態(押されていると0)
static uint8_t u8PBDelayCount[20];  //判定用ディレイカウンタ
static uint32_t u32PBPressed;       //押されたボタン
static uint32_t u32PBReleased;      //離されたボタン

//指定したピンをプッシュボタンとする
//事前にdio_pinMode()でINPUTまたはINPUT_PULLUPに設定しておく
void pb_define(uint8_t pinNo, bool_t bPressToHi) {
    uint32_t bit = 1UL << pinNo;
    uint32_t mask = (1UL << pinNo) ^ 0xfffff;
    u32PBDefined |= bit;
    if (bPressToHi) {
        u32PBPositive |= bit;
    } else {
        u32PBPositive &= mask;
    }
    u32PBCurIOStatus = (u32PBCurIOStatus & mask) | ((u32AHI_DioReadInput() ^ u32PBPositive) & bit);
}

//プッシュボタンが押されたかどうか知る
//この関数の実行により内部のフラグがクリアされるので、ボタンを１回押すごとにこの関数は１回しかTRUEを返さない
bool_t pb_pressed(uint8_t pinNo) {
    if ((u32PBDefined & (1UL << pinNo)) == 0) return FALSE;
    bool_t b = (u32PBPressed & (1UL << pinNo)) != 0;
    u32PBPressed &= (1UL << pinNo) ^ 0xfffff;
    return b;
}

//プッシュボタンが離されたかどうか知る
//この関数の実行により内部のフラグがクリアされるので、ボタンを１回離すごとにこの関数は１回しかTRUEを返さない
bool_t pb_released(uint8_t pinNo) {
    if ((u32PBDefined & (1UL << pinNo)) == 0) return FALSE;
    bool_t b = (u32PBReleased & (1UL << pinNo)) != 0;
    u32PBReleased &= (1UL << pinNo) ^ 0xfffff;
    return b;
}

//プッシュボタンの現在の状態を得る
bool_t pb_currentState(uint8_t pinNo) {
    uint32_t b = 1UL << pinNo;
    if ((u32PBDefined & b) == 0) return FALSE;
    return (u32PBCurIOStatus & b) == 0;
}

//プッシュボタンの状態をリセットする。
void pb_reset() {
    u32PBCurIOStatus = u32AHI_DioReadInput() ^ u32PBPositive;
    u32PBPressed = 0;
    u32PBReleased = 0;
    memset(u8PBDelayCount, 0, sizeof(u8PBDelayCount));
}

static void pb_update() {
    uint32_t diffBits = (u32AHI_DioReadInput() ^ u32PBPositive) ^ u32PBCurIOStatus;
    uint8_t i;
    uint32_t b = 1;
    for (i = 0; i < 20; i++) {
        if (u32PBDefined & b) {
            if (diffBits & b) {
                if (++u8PBDelayCount[i] >= PB_JUDGE_COUNT) {
                    u32PBCurIOStatus ^= b;
                    if ((u32PBCurIOStatus & b) == 0) {
                        //Pressed
                        u32PBPressed |= b;
                        //u32PBReleased &= b ^ 0xfffff;
                    } else {
                        //Released
                        u32PBReleased |= b;
                        //u32PBPressed &= b ^ 0xfffff;
                    }
                    u8PBDelayCount[i] = 0;
                }
            } else {
                u8PBDelayCount[i] = 0;
            }
        }
        b <<= 1;
    }
}
#endif

#ifdef USE_SBUTIL
//v2.0で刷新

static char __sb_buf[SB_BUFFER_SIZE + 1];

//書き込み補助関数
static char *__sb_ptr;
static uint16_t __sb_count;

void sb_clear() {
    __sb_ptr = __sb_buf;
    __sb_count = 0;
}

bool_t sb_putc(char c) {
    if (__sb_count == SB_BUFFER_SIZE) return FALSE;
    *__sb_ptr++ = c;
    __sb_count++;
    return TRUE;
}

bool_t sb_puts(const char *str) {
    uint16_t l = (uint16_t)strlen(str);
    if (l > SB_BUFFER_SIZE - __sb_count) return FALSE;
    memcpy(__sb_ptr, str, l);
    __sb_ptr += l;
    __sb_count += l;
    return TRUE;
}

bool_t sb_printf(const char *fmt, ...) {
    //パラメータを取得
    va_list ap;
    va_start(ap, fmt);

    //退避
    char *p = __sb_ptr;
    uint16_t c = __sb_count;

    //bufにprintf
    if (!myprintf(sb_putc, fmt, ap)) {
        //バッファオーバーフロー
        va_end(ap);

        //復帰
        __sb_ptr = p;
        __sb_count = c;
        return FALSE;
    }
    va_end(ap);
    return TRUE;
}

const char *sb_getBuffer() {
    *__sb_ptr = '\0';
    return __sb_buf;
}
#endif




/*
 * スリープ
 */


//スリープに使用される32kHzオシレータの校正値を取得します
//返り値を sleepCalibratedTimer() に渡すことでスリープ時間の精度向上が望めます
//この関数は実行に約0.5ミリ秒を要します
uint32_t wakeTimer_getCalibrationValue() {
    vAHI_WakeTimerStop(E_AHI_WAKE_TIMER_0);
    u8AHI_WakeTimerFiredStatus();
    return u32AHI_WakeTimerCalibrate();
}

//デフォルトの32kHzRCオシレータを使用する場合は、calibValueには wakeTimer_getCalibrationValue() で得た値を渡します。
//外部クロックを使用する場合は、 wakeTimer_getCalibrationValue() で計測してもよいですが、
//正確なクロックが予めわかっている場合は計算値を渡すこともできます。
//例えば 32.768kHz の水晶発振子を使用した場合、calibValue = 10000 x 32000 ÷ 32768 = 9765 となります。
bool_t sleepCalibratedTimer(uint64_t milliSeconds, bool_t bRAMPower, uint32_t calibValue) {
    if (calibValue == 0) return FALSE;

    uint64_t count = milliSeconds * 320000 / calibValue; //x32 x1000 /calibValue
    if (count > 0x1FFFFFFFFFF) return FALSE; 

    vAHI_WakeTimerEnable(0, TRUE);
    vAHI_WakeTimerStartLarge(0, count); //有効範囲 2～0x1FFFFFFFFFF
    vAHI_Sleep(bRAMPower ? E_AHI_SLEEP_OSCON_RAMON : E_AHI_SLEEP_OSCON_RAMOFF);
    return TRUE;
}

//milliSeconds = 1..68719476735ミリ秒(795日+)
bool_t sleepTimer(uint64_t milliSeconds, bool_t bRAMPower) {
    uint64_t count = milliSeconds << 5; //x32
    if (count > 0x1FFFFFFFFFF) return FALSE; 

    vAHI_WakeTimerEnable(0, TRUE);
    vAHI_WakeTimerStartLarge(0, count); //有効範囲 2～0x1FFFFFFFFFF
    vAHI_Sleep(bRAMPower ? E_AHI_SLEEP_OSCON_RAMON : E_AHI_SLEEP_OSCON_RAMOFF);
    return TRUE;
}

/*
            E_AHI_SLEEP_OSCON_RAMON   32kHz oscillator on and RAM on (warm restart)
            E_AHI_SLEEP_OSCON_RAMOFF    32kHz oscillator on and RAM off (cold restart)
            E_AHI_SLEEP_OSCOFF_RAMON    32kHz oscillator off and RAM on (warm restart)
            E_AHI_SLEEP_OSCOFF_RAMOFF   32kHz oscillator off and RAM off (cold restart)
            E_AHI_SLEEP_DEEP            Deep Sleep (all components off - cold restart)

[RAM ON/OFFの違い]
RAM OFFだとColdStart扱いになる。
起床に関する割り込みビットはいずれも取得できる。

[OSC ON/OFFの違い]
OSC(32kHz RCオシレータ)がOFFになると、それに関するモジュールが使えない。
　タイマー起床、パルスカウンタのデバウンス
起床に関する割り込みビットはいずれも取得できる。

[DEEP SLEEP]
常にColdStart扱いになる。
DIO割り込み起床は可能だが、起床に関する割り込みビットは取得できなくなる。

*/


/*
 * キュー
 */

void que_init(BYTEQUE *psQue, uint8_t *pau8Buff, uint16_t u16BuffSize) {
    psQue->pau8Buff = pau8Buff;
    psQue->u16BuffSize = u16BuffSize;
    psQue->u16DataSize = 0;
    psQue->u16WriteIndex = 0;
    psQue->bDataLost = FALSE;
    psQue->u16ReadIndex = 0;
}

void que_clear(BYTEQUE *psQue) {
    psQue->u16DataSize = 0;
    psQue->u16WriteIndex = 0;
    psQue->bDataLost = FALSE;
    psQue->u16ReadIndex = 0;
}

bool_t que_dataLost(BYTEQUE *psQue) {
    bool_t b = psQue->bDataLost;
    psQue->bDataLost = FALSE;
    return b;
}

void que_append(BYTEQUE *psQue, uint8_t byte) {
    if (psQue->u16DataSize < psQue->u16BuffSize) {
        *(psQue->pau8Buff + psQue->u16WriteIndex) = byte;
        if (++(psQue->u16WriteIndex) >= psQue->u16BuffSize) psQue->u16WriteIndex -= psQue->u16BuffSize;
        psQue->u16DataSize++;
    } else {
        psQue->bDataLost = TRUE;
    }
}

int16_t que_get(BYTEQUE *psQue) {
    if (psQue->u16DataSize == 0) return -1;

    int8_t byte = *(psQue->pau8Buff + psQue->u16ReadIndex);
    if (++(psQue->u16ReadIndex) >= psQue->u16BuffSize) psQue->u16ReadIndex -= psQue->u16BuffSize;
    psQue->u16DataSize--;
    return (int16_t)byte;
}




/*
 * デジタルIO
 */

#ifdef USE_DIO

bool_t do_enable(bool_t bEnable) {
    if (bEnable) {
        if (!bAHI_DoEnableOutputs(bEnable)) return FALSE;
        vAHI_DoSetPullup(0, 3); //PULLUP OFF
    } else {
        //元に戻す
        vAHI_DoSetPullup(3, 0);     //PULLUP ON
        vAHI_DoSetDataOut(3, 0);    //HIGH
        if (!bAHI_DoEnableOutputs(bEnable)) return FALSE;
    }
    return TRUE;
}

bool_t do_write(uint8_t pinNo, uint8_t value) {
    if (pinNo > 1) return FALSE;

    if (value == LOW) {
        vAHI_DoSetDataOut(0, 1 << pinNo);
    } else {
        vAHI_DoSetDataOut(1 << pinNo, 0);
    }
    return TRUE;
}


//pinNO=0..19 mode=INPUT/INPUT_PULLUP/OUTPUT
bool_t dio_pinMode(uint8_t pinNo, PINMODES mode) {
    if (pinNo > 20) return FALSE;

    if (mode == OUTPUT) {
        //OUTPUTモードにする
        vAHI_DioSetDirection(0, 1UL << pinNo);
    } else {
        //INPUTモードにする
        vAHI_DioSetDirection(1UL << pinNo, 0);
        if (mode == INPUT_PULLUP) {
            //プルアップ有効
            vAHI_DioSetPullup(1UL << pinNo, 0);
        } else {
            //プルアップ無効
            vAHI_DioSetPullup(0, 1UL << pinNo);
        }
    }
    return TRUE;
}

//pinNO=0..19 value=LOW/HIGH
//事前にpinModeでOUTPUTを設定しておく
bool_t dio_write(uint8_t pinNo, uint8_t value) {
    if (pinNo > 20) return FALSE;

    if (value == LOW) {
        vAHI_DioSetOutput(0, 1UL << pinNo);
    } else {
        vAHI_DioSetOutput(1UL << pinNo, 0);
    }
    return TRUE;
}


//割り込みルーチンのポインタを保持
void (*dioCallbackFunctions[MAX_DIO_INTERRUPT_FUNCS])(uint32_t);  //コールバック関数のポインタ
uint8_t dioCallbackFuncIndices[20];        //ピン番号0-19に対するdioCallbackFunctions[]のインデックスを保持, 0xffで初期化

//pinNO=0..19 mode=RISING(立ち上がり)/FALLING(立ち下がり)/DISABLE funcは引数を持たない関数
//一つのピンに一つの関数しか登録できない
bool_t dio_attachCallback(uint8_t pinNo, INTERRUPTIONEDGES mode, void (*func)(uint32_t u32DioBitmap)) {
    if (mode == DISABLE) {
        return dio_detach(pinNo);
    }

    if (mode != RISING && mode != FALLING) return FALSE;
    if (pinNo > 19 || func == NULL) return FALSE;

    uint8_t i, freeIndex = 0xff;
    dioCallbackFuncIndices[pinNo] = 0xff;
    for(i = 0; i < MAX_DIO_INTERRUPT_FUNCS; i++) {
        if (dioCallbackFunctions[i] == func) {
            //コールバック関数がすでに登録済みなので、ここを参照
            dioCallbackFuncIndices[pinNo] = i;
            break;
        } else if (freeIndex == 0xff && dioCallbackFunctions[i] == NULL) {
            //空きインデックスを記憶
            freeIndex = i;
        }
    }
    if (i == MAX_DIO_INTERRUPT_FUNCS) {
        if (freeIndex == 0xff) return FALSE;    //空きが無い

        //処理ルーチンのポインタを新規登録
        dioCallbackFunctions[freeIndex] = func;
        dioCallbackFuncIndices[pinNo] = freeIndex;
    }

    //割り込みを有効にする
    vAHI_DioInterruptEnable(1UL << pinNo, 0);

    if (mode == RISING) {
        //立ち上がりで発生
        vAHI_DioInterruptEdge(1UL << pinNo, 0);
    } else {
        //立下りで発生
        vAHI_DioInterruptEdge(0, 1UL << pinNo); 
    }
    return TRUE;
}

//pinNO=0..19
bool_t dio_detach(uint8_t pinNo) {
    if (pinNo > 19) return FALSE;

    uint8_t i, index = dioCallbackFuncIndices[pinNo];
    for (i = 0; i < 20; i++) {
        if (i != pinNo && dioCallbackFuncIndices[i] == index) {
            //他のピンで同じコールバック関数を参照している
            break;
        }
    }
    if (i == 20) {
        //コールバック関数は誰にも使用されていない
        dioCallbackFunctions[index] = NULL;
    }
    dioCallbackFuncIndices[pinNo] = 0xff;

    //割り込みを無効にする
    vAHI_DioInterruptEnable(0, 1UL << pinNo);
    return TRUE;
}

//DIOピンによるウェイクアップ pinNO=0..19 mode=RISING(立ち上がり)/FALLING(立ち下がり)/DISABLE
//この関数によりDIOはINPUTモードに設定されます（スリープ中はINPUT_PULLUPは使えないので自前で準備すること）
//この関数はsleep()の直前に呼び出してしてください。
bool_t dio_setWake(uint8_t pinNo, INTERRUPTIONEDGES mode) {
    if (pinNo > 19) return FALSE;
    if (mode != DISABLE && mode != RISING && mode != FALLING) return FALSE;

    (void)u32AHI_DioInterruptStatus(); // clear interrupt register
    if (mode != DISABLE) {
        dio_pinMode(pinNo, INPUT);
        vAHI_DioWakeEnable(1UL << pinNo, 0); // enable ports
        if (mode == RISING) {
            vAHI_DioWakeEdge(1UL << pinNo, 0); // set edge (rising)
        } else {
            vAHI_DioWakeEdge(0, 1UL << pinNo); // set edge (falling)
        }
    } else {
        vAHI_DioWakeEnable(0, 1UL << pinNo);
    }
    return TRUE;
}

#endif //USE_DIO



/*
 * タイマー／ＰＷＭ
 */

#ifdef USE_TIMER

//割り込みルーチンのポインタを保持
void (*timerCallbackFunctions[5])();

//タイマーコンテキストを保持
//tsTimerContext sTimerApp[5];  //ToCoNet関数をやめて下記で実装
tsTimerContext2 sTimerApp[5];

//vAHI_TimerFineGrainDIOControl()の値を保持（DIOピンを汎用かPWMで使うか）
uint8_t timerFineGrainDIOControlValue;

const uint8_t timerDeviceIndices[5] = {E_AHI_DEVICE_TIMER0,E_AHI_DEVICE_TIMER1,E_AHI_DEVICE_TIMER2,E_AHI_DEVICE_TIMER3,E_AHI_DEVICE_TIMER4};

//プレスケールとカウンタをhzから計算する
//hz=1..65536(0)
//うまく割り切れない場合は誤差が発生するが、hzが大きいほどその誤差は大きくなる傾向になる
//hz=1..100:最大誤差0.0012%
//hz=101..1000:最大誤差0.00295%
//hz=1001..10000:最大誤差0.03083%
//hz=10001..30000:最大誤差0.09304%
//hz=30001..50000:最大誤差0.15624%
//hz=50001..65535:最大誤差0.20442%
static void timerCalcParamFromHz(uint16_t hz, uint8_t *pu8Prescale, uint16_t *pu16CycleCount) {
    uint8_t prescale;
    uint32_t cycleCount;
    for (prescale = 0; prescale <= 16; prescale++) {
        cycleCount = (16000000 >> prescale) / (hz != 0 ? hz : 65536);

        //16ビットに収まらなければダメ
        if (cycleCount > 65536) continue;

        //精度上げるため割り算結果を四捨五入
        uint32_t w = (32000000 >> prescale) / (hz != 0 ? hz : 65536);
        if (w & 1) cycleCount++;

        *pu8Prescale = prescale;
        *pu16CycleCount = cycleCount < 65536 ? cycleCount : 0;
        return;
    }
    //ここには来ない
}


//タイマーを開始する。内部カウンタは０に戻る
bool_t timer_start(uint8_t timerNo) {
    if (timerNo > 4) return FALSE;
    if (sTimerApp[timerNo].u8Mode == 0) return FALSE;

    if (sTimerApp[timerNo].u8Mode == 4) {   //Micro counter
        sTimerApp[timerNo].u16HiMicroSeconds = 0;
    }
    vAHI_TimerStartRepeat(timerNo, sTimerApp[timerNo].u16HiCount, sTimerApp[timerNo].u16LoCount);
    return TRUE;
}

//PWM
//timerNo=0..4, prescale=0..16, cycleCount=2..65536(0), pulseCount=1..(cycleCount-1), bStartFromLo TRUE:L->H, FALSE:H->L
//周期 = (1 << prescale) * cycleCount / 16000000 [秒]
//pinSelection       Timer0 Timer1 Timer2 Timer3 Timer4
//  DEFAULT_PINの場合 DIO10  DIO11  DIO12  DIO13  DIO17
//  SECOND_PINの場合  DIO4   DIO5   DIO6   DIO7   DIO8
//  DO_PINの場合     (DIO4) (DIO5)  DO0    DO1   (DIO8)
//bSyncUpdate: TRUEの場合、timer_updatePWM(),timer_updatePWMDuty()でタイミングをとってパルスカウントを変更する
bool_t timer_attachPWM(uint8_t timerNo, uint8_t prescale, uint16_t cycleCount, uint16_t pulseCount, bool_t bStartFromHi,
                        TIMEROPINSELECTION pinSelection, bool_t bSyncUpdate, bool_t bStartNow) {
    if (timerNo > 4) return FALSE;
    if (cycleCount != 0 && pulseCount > cycleCount) return FALSE;

    vAHI_TimerSetLocation(timerNo, (pinSelection != DEFAULT_PIN), (pinSelection == DO_PIN));

    uint8_t b = 4 << timerNo;
    if ((timerFineGrainDIOControlValue & b) != 0) {
        //DIOを汎用からタイマー用に切り替える
        timerFineGrainDIOControlValue &= b ^ 255;
        vAHI_TimerFineGrainDIOControl(timerFineGrainDIOControlValue);
    }

    vAHI_TimerEnable(timerNo,
        prescale,
        FALSE, //bool_t bIntRiseEnable, 
        bSyncUpdate, //bool_t bIntPeriodEnable,     //v3.1 同期更新のために割り込みを発生させる
        TRUE);//bool_t bOutputEnable);

    vAHI_TimerConfigureOutputs(timerNo,
        bStartFromHi, //TRUE:出力反転
        TRUE); //disable clock gation input

    sTimerApp[timerNo].u8Mode = bSyncUpdate ? 8 : 2; //PWM(Sync) または PWM    
    sTimerApp[timerNo].bStartFromHi = bStartFromHi;
    sTimerApp[timerNo].u16HiCount = bStartFromHi ? pulseCount : cycleCount - pulseCount; //開始から変化までのカウント
    sTimerApp[timerNo].u16LoCount = cycleCount;             //開始から終了までのカウント=サイクル
    timerCallbackFunctions[timerNo] = NULL;

    if (bStartNow) timer_start(timerNo);
    return TRUE;
}

//timer_attachPWM()の簡易版。HzとDutyで指定 Hz:1～65536(0) Duty:0～32768
//pinSelection       Timer0 Timer1 Timer2 Timer3 Timer4
//  DEFAULT_PINの場合 DIO10  DIO11  DIO12  DIO13  DIO17
//  SECOND_PINの場合  DIO4   DIO5   DIO6   DIO7   DIO8
//  DO_PINの場合     (DIO4) (DIO5)  DO0    DO1   (DIO8)
//bSyncUpdate: TRUEの場合、timer_updatePWM(),timer_updatePWMDuty()でタイミングをとってパルスカウントを変更する
bool_t timer_attachPWMByHzDuty(uint8_t timerNo, uint16_t hz, uint16_t duty, bool_t bStartFromHi,
                                TIMEROPINSELECTION pinSelection, bool_t bSyncUpdate, bool_t bStartNow) {
    if (timerNo > 4 || duty > 32768) return FALSE;

    uint8_t prescale;
    uint16_t cycleCount;
    timerCalcParamFromHz(hz, &prescale, &cycleCount);

    uint16_t pulseCount;
    if (cycleCount == 0 && duty == 32768) {
        pulseCount = 65535;
    } else {
        pulseCount = ((uint32_t)cycleCount * (uint32_t)duty) >> 15;
    }
    return timer_attachPWM(timerNo, prescale, cycleCount, pulseCount, bStartFromHi, pinSelection, bSyncUpdate, bStartNow);
}

//PWMに設定可能なpulseCount上限値を返します
int32_t timer_getPWMPulseCountULimit(uint8_t timerNo) {
    if (timerNo > 4) return -1;
    if (sTimerApp[timerNo].u8Mode != 2 && sTimerApp[timerNo].u8Mode != 8) return -1; //not PWM
    if (sTimerApp[timerNo].u16LoCount == 0) {
        return 65535;
    } else{
        return sTimerApp[timerNo].u16LoCount;
    }
}

//PWMのデューティー比を変更するためにパルスカウントを再設定する
bool_t timer_updatePWM(uint8_t timerNo, uint16_t pulseCount) {
    if (timerNo > 4) return FALSE;
    if (sTimerApp[timerNo].u8Mode != 2 && sTimerApp[timerNo].u8Mode != 8) return FALSE; //not PWM

    uint16_t cycleCount = sTimerApp[timerNo].u16LoCount;
    if (cycleCount != 0 && pulseCount > cycleCount) return FALSE;

    if (sTimerApp[timerNo].u8Mode == 2) {
        //値を変更
        sTimerApp[timerNo].u16HiCount = sTimerApp[timerNo].bStartFromHi ? pulseCount : cycleCount - pulseCount; //開始から変化までのカウント
        vAHI_TimerStartRepeat(timerNo, sTimerApp[timerNo].u16HiCount, cycleCount);
    } else {
        //同期の場合は割り込みで変更を行うので、ここでは保存のみ
        sTimerApp[timerNo].u16ReservedHiCount = sTimerApp[timerNo].bStartFromHi ? pulseCount : cycleCount - pulseCount; //開始から変化までのカウント
    }

    return TRUE;
}

//timer_updatePWM()の簡易版。Duty:0～32768
bool_t timer_updatePWMDuty(uint8_t timerNo, uint16_t duty) {
    if (timerNo > 4 || duty > 32768) return FALSE;
    if (sTimerApp[timerNo].u8Mode != 2 && sTimerApp[timerNo].u8Mode != 8) return FALSE; //not PWM

    uint16_t cycleCount = sTimerApp[timerNo].u16LoCount;
    uint16_t pulseCount;
    if (cycleCount == 0 && duty == 32768) {
        pulseCount = 65535;
    } else {
        pulseCount = ((uint32_t)cycleCount * (uint32_t)duty) >> 15;
    }
    
    if (sTimerApp[timerNo].u8Mode == 2) {
        //値を変更
        sTimerApp[timerNo].u16HiCount = sTimerApp[timerNo].bStartFromHi ? pulseCount : cycleCount - pulseCount; //開始から変化までのカウント
        vAHI_TimerStartRepeat(timerNo, sTimerApp[timerNo].u16HiCount, cycleCount);
    } else {
        //同期の場合は割り込みで変更を行うので、ここでは保存のみ
        sTimerApp[timerNo].u16ReservedHiCount = sTimerApp[timerNo].bStartFromHi ? pulseCount : cycleCount - pulseCount; //開始から変化までのカウント
    }

    return TRUE;
}

//timerNo=0..4, prescale=0..16, cycleCount=1..65536(0)
//周期 = (1 << prescale) * cycleCount / 16000000 [秒]
bool_t timer_attachCallback(uint8_t timerNo, uint8_t prescale, uint16_t cycleCount, bool_t bStartNow, void (*func)()) {
    if (timerNo > 4) return FALSE;

    uint8_t b = 4 << timerNo;
    if ((timerFineGrainDIOControlValue & b) == 0) {
        //DIOをタイマー用から汎用に切り替える
        timerFineGrainDIOControlValue |= b;
        vAHI_TimerFineGrainDIOControl(timerFineGrainDIOControlValue);
    }

    vAHI_TimerEnable(timerNo,
        prescale,
        FALSE, //bool_t bIntRiseEnable,
        (func != NULL), //bool_t bIntPeriodEnable, TRUEでcbToCoNet_u8HwInt()割り込みイベント起動。func=NULLの場合、タイマーだけ走らせることもできる
        FALSE);//bool_t bOutputEnable);
    //vAHI_TimerConfigureOutputs(timerNo, 出力しないので不要
    //    FALSE, //disable inversion
    //    TRUE); //disable clock gation input
    sTimerApp[timerNo].u8Mode = 1; //Timer    
    sTimerApp[timerNo].u16HiCount = cycleCount;      //無意味だが何かしら設定
    sTimerApp[timerNo].u16LoCount = cycleCount;      //countまでカウントしたら割り込みがかかる
    timerCallbackFunctions[timerNo] = func;

    if (bStartNow) timer_start(timerNo);
    return TRUE;
}

//timer_attachCallback()の簡易版。Hzで指定 1～65536(0)
bool_t timer_attachCallbackByHz(uint8_t timerNo, uint16_t hz, bool_t bStartNow, void (*func)()) {
    uint8_t prescale;
    uint16_t cycleCount;
    timerCalcParamFromHz(hz, &prescale, &cycleCount);
    return timer_attachCallback(timerNo, prescale, cycleCount, bStartNow, *func);
}

//疑似アナログ出力
//timerNo=0..4, power=0..65536(0)
//pinSelection       Timer0 Timer1 Timer2 Timer3 Timer4
//  DEFAULT_PINの場合 DIO10  DIO11  DIO12  DIO13  DIO17
//  SECOND_PINの場合  DIO4   DIO5   DIO6   DIO7   DIO8
//  DO_PINの場合     (DIO4) (DIO5)  DO0    DO1   (DIO8)
bool_t timer_attachAnalogWrite(uint8_t timerNo, uint16_t power, TIMEROPINSELECTION pinSelection) {
    if (timerNo > 4) return FALSE;

    vAHI_TimerSetLocation(timerNo, (pinSelection != DEFAULT_PIN), (pinSelection == DO_PIN));

    uint8_t b = 4 << timerNo;
    if ((timerFineGrainDIOControlValue & b) != 0) {
        //DIOを汎用からタイマー用に切り替える
        timerFineGrainDIOControlValue &= b ^ 255;
        vAHI_TimerFineGrainDIOControl(timerFineGrainDIOControlValue);
    }

    sTimerApp[timerNo].u8Mode = 3; //AnalogOut

    vAHI_TimerEnable(timerNo,
        8,
        FALSE, //bool_t bIntRiseEnable, 
        FALSE, //bool_t bIntPeriodEnable, 
        TRUE);//bool_t bOutputEnable);
    vAHI_TimerStartDeltaSigma(timerNo,
        power,
        0x0000,
        FALSE);//bool_t bRtzEnable

    return TRUE;
}

//疑似アナログ出力の出力を変更する
bool_t timer_updateAnalogPower(uint8_t timerNo, uint16_t power) {
    if (timerNo > 4) return FALSE;
    if (sTimerApp[timerNo].u8Mode != 3) return FALSE; //not AnalogOut

    vAHI_TimerStartDeltaSigma(timerNo,
        power,
        0x0000,
        FALSE);//bool_t bRtzEnable
    return TRUE;
}

//1μ秒で増加する32ビットカウンタをタイマーに割り当てる
bool_t timer_attachMicroCounter(uint8_t timerNo, bool_t bStartNow) {
    if (timerNo > 4) return FALSE;

    uint8_t b = 4 << timerNo;
    if ((timerFineGrainDIOControlValue & b) == 0) {
        //DIOをタイマー用から汎用に切り替える
        timerFineGrainDIOControlValue |= b;
        vAHI_TimerFineGrainDIOControl(timerFineGrainDIOControlValue);
    }

    vAHI_TimerEnable(timerNo,
        4,      //prescale
        FALSE, //bool_t bIntRiseEnable,
        TRUE, //bool_t bIntPeriodEnable
        FALSE);//bool_t bOutputEnable);
    sTimerApp[timerNo].u8Mode = 4; //Micro counter    
    sTimerApp[timerNo].u16HiCount = 0;      //無意味だが何かしら設定
    sTimerApp[timerNo].u16LoCount = 0;      //65536までカウントしたら割り込みがかかる
    timerCallbackFunctions[timerNo] = NULL;

    if (bStartNow) timer_start(timerNo);
    return TRUE;
}

//1μ秒で増加する32ビットカウンタ値を読み出す
uint32_t timer_getMicroCount(uint8_t timerNo) {
    if (timerNo > 4) return 0xffffffff;
    if (sTimerApp[timerNo].u8Mode != 4) return 0xffffffff; //Not Micro counter

    uint16_t l1 = u16AHI_TimerReadCount(timerNo);
    uint16_t h = sTimerApp[timerNo].u16HiMicroSeconds;
    uint16_t l2 = u16AHI_TimerReadCount(timerNo);
    if (l2 >= l1) {
        return ((uint32_t)h << 16) | (uint32_t)l2;
    } else {
        //割り込みがかかってそうな時はもっかい読み出す
        return ((uint32_t)(sTimerApp[timerNo].u16HiMicroSeconds) << 16) | (uint32_t)u16AHI_TimerReadCount(timerNo);
    }
}

static uint16_t *pu16Timer0CapBuf;
static uint16_t u16Timer0CapBufSize;
static uint16_t u16Timer0CapCount;
static bool_t bTimer0FirstCap;

//timer0_attachCapture()で読み取ったデータの個数を返す
//パルスが無い場合にこの数値は増えない
uint16_t timer0_getCaptureCount() {
    if (sTimerApp[0].u8Mode != 5) return 0; //Not Capture
    return u16Timer0CapCount;
}

//timer0_attachCapture()で開始したバッファリングが完了したかを返す
//完了した時点でサンプリングは自動停止する
bool_t timer0_captureCompleted() {
    if (sTimerApp[0].u8Mode != 5) return FALSE; //Not Capture
    return u16Timer0CapCount == u16Timer0CapBufSize;
}

//タイマー０でパルス幅をサンプリング＆バッファに保存する
//prescale=0..16, pu32Buffer=バッファ, u16BufferLength=バッファのサイズ, bUseSecondPin=入力ピンの選択。FALSE:DIO9/TRUE:DIO3
//バッファの上位16ビットにLOWレベルの期間、下位16ビットにHIGHレベルの期間を示すカウンタ値が格納される
//1カウント値あたりの時間は 1 / (16000000 / (2^prescale)) 秒である
bool_t timer0_attachCapture(uint8_t prescale, uint32_t *pu32Buffer, uint16_t u16BufferLength, bool_t bUseSecondPin) {
    if (pu32Buffer == NULL || u16BufferLength == 0) return FALSE;

    pu16Timer0CapBuf = (uint16_t *)pu32Buffer;
    u16Timer0CapBufSize = u16BufferLength;
    u16Timer0CapCount = 0;
    bTimer0FirstCap = TRUE;
    sTimerApp[0].u8Mode = 5; //Capture

    vAHI_TimerSetLocation(0, bUseSecondPin, FALSE);

    if ((timerFineGrainDIOControlValue & 0x02) != 0) {
        //DIOを汎用からタイマー用に切り替える
        timerFineGrainDIOControlValue &= 0xfd;
        vAHI_TimerFineGrainDIOControl(timerFineGrainDIOControlValue);
    }

    vAHI_TimerEnable(0,
        prescale,   //u8Prescale,
        FALSE,  //bIntRiseEnable,
        TRUE,   //bIntPeriodEnable,
        FALSE); //bOutputEnable

    vAHI_TimerConfigureInputs(0,
        FALSE,  //bInvCapt,
        FALSE); //bEventEdge
    vAHI_TimerStartCapture(0);

    return TRUE;
}

//Callback/PWM/AnalogOut/MicroCounter/Capture のデタッチ
//timerNo = 0..4
bool_t timer_detach(uint8_t timerNo) {
    if (timerNo > 4) return FALSE;
    
    vAHI_TimerStop(timerNo);
    vAHI_TimerDisable(timerNo);
    timerCallbackFunctions[timerNo] = NULL;
    sTimerApp[timerNo].u8Mode = 0; //DIO

    //カウンタモードで外部クロックにしているので、ここで戻す
    vAHI_TimerClockSelect(0,
        FALSE,   //bool_t bExternalClock,
        FALSE); //bool_t bInvertClock);

    uint8_t b = 4 << timerNo;
    if ((timerFineGrainDIOControlValue & b) == 0) {
        //DIOをタイマー用から汎用に切り替える
        timerFineGrainDIOControlValue |= b;
        vAHI_TimerFineGrainDIOControl(timerFineGrainDIOControlValue);
    }
    return TRUE;
}

//bUseSecondPin=FALSE:DIO8/TRUE:DIO2
bool_t timer0_attachCounter(uint8_t prescale, uint16_t count, bool_t bUseSecondPin, INTERRUPTIONEDGES mode, void (*func)()) {
    if (mode == DISABLE) {
        timer_detach(0);
        return TRUE;
    }

    sTimerApp[0].u8Mode = 6; //Counter
    timerCallbackFunctions[0] = func;

    vAHI_TimerSetLocation(0, bUseSecondPin, FALSE);

    if ((timerFineGrainDIOControlValue & 0x01) != 0) {
        //DIOを汎用からタイマー用に切り替える
        timerFineGrainDIOControlValue &= 0xfe;
        vAHI_TimerFineGrainDIOControl(timerFineGrainDIOControlValue);
    }

    vAHI_TimerEnable(0,
        prescale,   //uint8 u8Prescale,
        FALSE,      //bool_t bIntRiseEnable,
        func != NULL, //bool_t bIntPeriodEnable,
        FALSE);     //bool_t bOutputEnable);

    vAHI_TimerClockSelect(0,
        TRUE,   //bool_t bExternalClock,
        mode == FALLING); //bool_t bInvertClock);

    vAHI_TimerConfigureInputs(0,
        FALSE,  //bool_t bInvCapt,      capture用
        mode == BOTHEDGE); //bool_t bEventEdge);   FALSE:立ち上がりエッジ/TRUE:両エッジでカウント

    vAHI_TimerStartRepeat(0,
        count,  //no use
        count); //本命
    return TRUE;
}


#endif //USE_TIMER



/*
 * シリアル
 */

#ifdef USE_SERIAL
//ここからシリアル0関数


// 送信FIFOバッファ(16～2047)
static uint8_t au8SerialTxBuffer0[SERIAL_TX_BUFFER_SIZE]; 

// 受信FIFOバッファ(16～2047)
static uint8_t au8SerialRxBuffer0[SERIAL_RX_BUFFER_SIZE];   //ハードウェアフロー制御のときは二次バッファとなる  

//u8AHI_UartReadLineStatus()の値を保持
uint8_t serial0StatusBit;

//#ifdef SERIAL_HW_FLOW_CONTROL
static uint8_t au8SerialRxBuffer0_internal[20]; //一次バッファ
static BYTEQUE sSerialRxQue0;                   //二次バッファ管理用キュー
static int8_t i8HwFCTimerNo;                    //ハードウェアフロー制御に使うタイマー番号。使わない場合は-1。ハードウェアフロー制御判定フラグにも使う

static void serial0UpdateRxBuffer();
//#endif


//New serial_printf()
//出力できなかった場合は１文字も出力せずにFALSEを返します
bool_t serial_printf(const char *fmt, ...) {
    //パラメータを取得
    va_list ap;
    va_start(ap, fmt);

    //バッファへの書き込みパラメータを設定
    char buf[SERIAL_TX_BUFFER_SIZE];
    __printf_putc_init(buf, SERIAL_TX_BUFFER_SIZE);

    //bufにprintf
    if (!myprintf(__printf_putc, fmt, ap)) {
        //バッファオーバーフロー
        va_end(ap);
        return FALSE;
    }
    va_end(ap);

    //シリアルに書き出す
    return serial_write((uint8_t *)buf, __printf_putc_count);
}

//シリアル0を初期化する
//bUseSecondPin RXD   TXD   RTS   CTS
//  FALSE       DIO7  DIO6  DIO5  DIO4
//  TRUE        DIO15 DIO14 DIO13 DIO12
//ハードウェアフロー制御を行わない場合はRTS,CTSピンは汎用DIOとして使用できる
bool_t serial_initEx(SERIALBAUD baudRate, SERIALPARITY parity, SERIALBITLENGTH bitLength, SERIALSTOPBIT stopBit, bool_t bUseSecondPin, SERIALHWFLOWCONTROL flowControl) {

    i8HwFCTimerNo = (uint8_t)flowControl;

    vAHI_UartSetLocation(E_AHI_UART_0, bUseSecondPin);
    vAHI_UartSetRTSCTS(E_AHI_UART_0, (i8HwFCTimerNo != -1)); //RTS,CTSピンを使う

    if (!bAHI_UartEnable(E_AHI_UART_0,
        au8SerialTxBuffer0,     //uint8 *pu8TxBufAd,
        SERIAL_TX_BUFFER_SIZE,  //uint16 u16TxBufLen,
        (i8HwFCTimerNo != -1) ? au8SerialRxBuffer0_internal : au8SerialRxBuffer0, //uint8 *pu8RxBufAd,
        (i8HwFCTimerNo != -1) ? 20 : SERIAL_RX_BUFFER_SIZE)  //uint16 u16RxBufLen);
    ) return FALSE;

    u8AHI_UartReadLineStatus(E_AHI_UART_0); //clear
    serial0StatusBit = 0;

    vAHI_UartSetBaudRate(E_AHI_UART_0, baudRate);

    vAHI_UartSetControl(E_AHI_UART_0,
        (parity == SERIAL_PARITY_EVEN), //bool_t bEvenParity,
        (parity != SERIAL_PARITY_NONE), //bool_t bEnableParity,
        bitLength,                      //uint8 u8WordLength,
        (stopBit == SERIAL_STOP_1BIT),  //bool_t bOneStopBit,
        FALSE);                         //bool_t bRtsValue); FALSE:"RequestToSend"

    vAHI_UartSetAutoFlowCtrl(E_AHI_UART_0,
        E_AHI_UART_FIFO_ARTS_LEVEL_15,  //uint8 u8RxFifoLevel, 一次バッファが15バイトでRTSがHになる
        FALSE,                          //bool_t bFlowCtrlPolarity,
        (i8HwFCTimerNo != -1),          //bool_t bAutoRts,  受信制御自動
        (i8HwFCTimerNo != -1));         //bool_t bAutoCts); 送信制御自動

    if (i8HwFCTimerNo != -1) {
        //二次バッファ用キューを初期化
        que_init(&sSerialRxQue0, au8SerialRxBuffer0, SERIAL_RX_BUFFER_SIZE);

        //一次バッファ監視用にタイマー起動
        timer_attachCallback(i8HwFCTimerNo, 4, 1000, TRUE, serial0UpdateRxBuffer);  //1ms
    }

    return TRUE;
}

void serial_disable() {
    vAHI_UartDisable(E_AHI_UART_0);
    if (i8HwFCTimerNo != -1) timer_detach(i8HwFCTimerNo);
}

//フロー制御時に一次バッファから二次バッファにデータを移送
//タイマーコールバック関数(遅延割り込み)も兼ねる
static void serial0UpdateRxBuffer() {
    while (u16AHI_UartReadRxFifoLevel(E_AHI_UART_0) > 0 && !que_bufferFull(&sSerialRxQue0)) {
        que_append(&sSerialRxQue0, u8AHI_UartReadData(E_AHI_UART_0));
    }
}

//シリアル0でバッファフルなどによる欠落が発生したか
bool_t serial_dataLost() {
    serial0StatusBit |= u8AHI_UartReadLineStatus(E_AHI_UART_0); //フラグ合成

    bool_t b;
    if (i8HwFCTimerNo != -1) {
        b = ((serial0StatusBit & E_AHI_UART_LS_OE) || que_dataLost(&sSerialRxQue0));
    } else {
        b = (serial0StatusBit & E_AHI_UART_LS_OE);
    }
    serial0StatusBit &= (E_AHI_UART_LS_OE ^ 0xff);  //フラグクリア
    return b;
}

//シリアル0の受信バッファのデータ数を返す
uint16_t serial_getRxCount() {
    if (i8HwFCTimerNo != -1) {
        serial0UpdateRxBuffer(); //データがあれば読み込む
        return que_getCount(&sSerialRxQue0);
    } else {
        return u16AHI_UartReadRxFifoLevel(E_AHI_UART_0);
    }
}

//シリアル0から1バイト読み出す。データが無い場合は-1を返す
int16_t serial_getc() {
    if (i8HwFCTimerNo != -1) {
        serial0UpdateRxBuffer();        //データがあれば読み込む
        return que_get(&sSerialRxQue0); //無いときは-1を返す
    } else {
        if (u16AHI_UartReadRxFifoLevel(E_AHI_UART_0) == 0) return -1;
        return (int16_t)u8AHI_UartReadData(E_AHI_UART_0);
    }
}

//シリアル0からバッファに読み込む。読み込み終了条件は以下の通り
//・u8Terminateが見つかった場合。u8Terminateを含む。
//・バッファいっぱい、(u16Length-1)文字読み込むまで。
//・読み込むデータがなくなった場合。
//バッファにnull終端'\0'が付加されるため、最大読み取りバイト数は(u16Length-1)となる。
//関数はnull終端を含めない、読み込んだバイト数を返す。エラーの場合-1。
int16_t serial_readUntil(uint8_t u8Terminate, uint8_t *pu8Buffer, uint16_t u16Length) {
    if (u16Length <= 1) return -1;

    if (i8HwFCTimerNo != -1) serial0UpdateRxBuffer(); //データがあれば読み込む

    int16_t len = 0;
    while (u16Length-- > 1) {
        uint8_t c;
        if (i8HwFCTimerNo != -1) {
            if (que_getCount(&sSerialRxQue0) == 0) break;
            c= que_get(&sSerialRxQue0);
        } else {
            if (u16AHI_UartReadRxFifoLevel(E_AHI_UART_0) == 0) break;
            c = u8AHI_UartReadData(E_AHI_UART_0);
        }
        *pu8Buffer++ = c;
        len++;
        if (c == u8Terminate) break;
    }
    *pu8Buffer = '\0';
    return len;
}

//送信可能かを返す。ハードウェアフロー制御を行わない場合は常にTRUEを返す
bool_t serial_ready() {
    if (i8HwFCTimerNo == -1) return TRUE;
    return ((u8AHI_UartReadModemStatus(E_AHI_UART_0) & 0x10) ? TRUE : FALSE);
}

//シリアル0に書式文字列を書き出す。バッファがいっぱいの場合は書き出さずにFALSEを返す
/*bool_t serial_printf(const char* format, va_list args) {
    SPRINTF_vRewind();
    vfPrintf(SPRINTF_Stream, format, args);
    va_end(args);
    return serialx_puts(E_AHI_UART_0, SPRINTF_pu8GetBuff());
}*/

//シリアル0の受信バッファをクリアする
void serial_resetRx() {
    vAHI_UartReset(E_AHI_UART_0, FALSE, TRUE);

    if (i8HwFCTimerNo != -1) {
        que_clear(&sSerialRxQue0);
    }
}

#endif //USE_SERIAL


#ifdef USE_SERIAL1
//ここからシリアル1関数


// 送信FIFOバッファ(16～2047)
static uint8_t au8SerialTxBuffer1[SERIAL1_TX_BUFFER_SIZE]; 

// 受信FIFOバッファ(16～2047)
static uint8_t au8SerialRxBuffer1[SERIAL1_RX_BUFFER_SIZE];

//u8AHI_UartReadLineStatus()の値を保持
uint8_t serial1StatusBit;

//New serial1_printf()
//出力できなかった場合は１文字も出力せずにFALSEを返します
bool_t serial1_printf(const char *fmt, ...) {
    //パラメータを取得
    va_list ap;
    va_start(ap, fmt);

    //バッファへの書き込みパラメータを設定
    char buf[SERIAL1_TX_BUFFER_SIZE];
    __printf_putc_init(buf, SERIAL1_TX_BUFFER_SIZE);

    //bufにprintf
    if (!myprintf(__printf_putc, fmt, ap)) {
        //バッファオーバーフロー
        va_end(ap);
        return FALSE;
    }
    va_end(ap);

    //シリアルに書き出す
    return serial1_write((uint8_t *)buf, __printf_putc_count);
}

//シリアル1を初期化する
//bUseSecondPin RXD   TXD
//  FALSE       DIO15 DIO14
//  TRUE        DIO9  DIO11
//bUseTxOnly=TRUEの場合、RXDピンは使用しないのでDIOとして使用できます。
bool_t serial1_initEx(SERIALBAUD baudRate, SERIALPARITY parity, SERIALBITLENGTH bitLength, SERIALSTOPBIT stopBit, bool_t bUseTxOnly, bool_t bUseSecondPin) {

    vAHI_UartSetLocation(E_AHI_UART_1, bUseSecondPin);

    vAHI_UartTxOnly(E_AHI_UART_1, bUseTxOnly);

    if (!bAHI_UartEnable(E_AHI_UART_1,
        au8SerialTxBuffer1,     //uint8 *pu8TxBufAd,
        SERIAL1_TX_BUFFER_SIZE, //uint16 u16TxBufLen,
        au8SerialRxBuffer1,             //uint8 *pu8RxBufAd,
        SERIAL1_RX_BUFFER_SIZE)         //uint16 u16RxBufLen);
    ) return FALSE;

    u8AHI_UartReadLineStatus(E_AHI_UART_1); //clear
    serial1StatusBit = 0;

    vAHI_UartSetBaudRate(E_AHI_UART_1, baudRate);

    vAHI_UartSetControl(E_AHI_UART_1,
        (parity == SERIAL_PARITY_EVEN), //bool_t bEvenParity,
        (parity != SERIAL_PARITY_NONE), //bool_t bEnableParity,
        bitLength,                      //uint8 u8WordLength,
        (stopBit == SERIAL_STOP_1BIT),  //bool_t bOneStopBit,
        FALSE);                         //bool_t bRtsValue); FALSE:"RequestToSend"

    return TRUE;
}

//シリアル1でバッファフルなどによる欠落が発生したか
bool_t serial1_dataLost() {
    serial1StatusBit |= u8AHI_UartReadLineStatus(E_AHI_UART_1); //フラグ合成
    bool_t b = (serial1StatusBit & E_AHI_UART_LS_OE);
    serial1StatusBit &= (E_AHI_UART_LS_OE ^ 0xff);  //フラグクリア
    return b;
}

//シリアル1から1バイト読み出す。データが無い場合は-1を返す
int16_t serial1_getc() {
    if (u16AHI_UartReadRxFifoLevel(E_AHI_UART_1) == 0) return -1;
    return (int16_t)u8AHI_UartReadData(E_AHI_UART_1);
}

//シリアル1からバッファに読み込む。読み込み終了条件は以下の通り
//・u8Terminateが見つかった場合。u8Terminateを含む。
//・バッファいっぱい、(u16Length-1)文字読み込むまで。
//・読み込むデータがなくなった場合。
//バッファにnull終端'\0'が付加されるため、最大読み取りバイト数は(u16Length-1)となる。
//関数はnull終端を含めない、読み込んだバイト数を返す。エラーの場合-1。
int16_t serial1_readUntil(uint8_t u8Terminate, uint8_t *pu8Buffer, uint16_t u16Length) {
    if (u16Length <= 1) return -1;
    int16_t len = 0;
    while (u16Length-- > 1) {
        if (u16AHI_UartReadRxFifoLevel(E_AHI_UART_1) == 0) break;
        uint8_t c = u8AHI_UartReadData(E_AHI_UART_1);
        *pu8Buffer++ = c;
        len++;
        if (c == u8Terminate) break;
    }
    *pu8Buffer = '\0';
    return len;
}

//シリアル1に書式文字列を書き出す。バッファがいっぱいの場合は書き出さずにFALSEを返す
/*bool_t serial1_printf(const char* format, va_list args) {
    SPRINTF_vRewind();
    vfPrintf(SPRINTF_Stream, format, args);
    va_end(args);
    return serialx_puts(E_AHI_UART_1, SPRINTF_pu8GetBuff());
}*/

#endif //USE_SERIAL1


#if defined(USE_SERIAL) || defined(USE_SERIAL1)
//ここからシリアル共通関数


//シリアルに1バイト書き出す。バッファがいっぱいの場合は書き出さずにFALSEを返す
bool_t serialx_putc(uint8_t serialNo, uint8_t u8Data) {
    if (u16AHI_UartReadTxFifoLevel(serialNo) == SERIAL_TX_BUFFER_SIZE) return FALSE;
    vAHI_UartWriteData(serialNo, u8Data);
    return TRUE;
}

//シリアルにバイト配列を書き出す。バッファがいっぱいの場合は書き出さずにFALSEを返す
bool_t serialx_write(uint8_t serialNo, const uint8_t *pau8Data, uint16_t u16Length) {
    if (((serialNo == E_AHI_UART_0 ? SERIAL_TX_BUFFER_SIZE : SERIAL1_TX_BUFFER_SIZE) - u16AHI_UartReadTxFifoLevel(serialNo)) < u16Length) return FALSE;
    while (u16Length-- > 0) {
        vAHI_UartWriteData(serialNo, *pau8Data++);
    }
    return TRUE;
}

//シリアルに文字列を書き出す。バッファがいっぱいの場合は書き出さずにFALSEを返す
bool_t serialx_puts(uint8_t serialNo, const char *pau8String) {
    return serialx_write(serialNo, (const uint8_t *)pau8String, (uint16_t)strlen(pau8String));
}

#endif //USE_SERIAL || USE_SERIAL1



/*
 * ＡＤＣ
 */

#if defined(USE_ADC) || defined(USE_COMP)
void adc_disable() {
    if (bAHI_APRegulatorEnabled()) {
        vAHI_ApConfigure(E_AHI_AP_REGULATOR_DISABLE,    //OFF
            E_AHI_AP_INT_DISABLE,                       //OFF
            E_AHI_AP_SAMPLE_4,//ADC_SAMPLE_4,
            E_AHI_AP_CLOCKDIV_500KHZ,//ADC_CLOCK_500KHZ,
            E_AHI_AP_INTREF);
    }
}
#endif

#ifdef USE_ADC

//割り込みルーチンのポインタを保持
static void (*adcCallbackFunction)(uint16_t);

//attachAdcCallback()またはadc_attachCallbackWithTimer()の設定を保持
static bool_t adcIsContinuous;
static bool_t adcIsRange2;
static bool_t adcIsExternalVRef;
static ADCSOURCES adcLastSource;        //adc_attachCallbackWithTimer()実行時は0xff
//adc_attachCallbackWithTimer()専用
//static ADCSOURCEBITMAP adcSourceBitmap;
//static uint16_t *adcBuffer;
//static uint16_t adcBufferSize;
static uint8_t adcIntCountTillEnd;

static uint32_t adcExternalVRef_3072Kdiv1023; //外部入力基準電圧[V] x 3072000 ÷ 1023
static uint32_t adcExternalVRef_2048Kdiv1023; //外部入力基準電圧[V] x 2048000 ÷ 1023
static uint32_t adcExternalVRef_1024Kdiv1023; //外部入力基準電圧[V] x 1024000 ÷ 1023


//VREF(A2)に入力する基準電圧[V]を1024000倍したものを渡す
//これはadc_enable()でbUseExternalVRef＝TRUEとした場合に使用される
void adc_setVRef(uint32_t u32VRef) {
    adcExternalVRef_3072Kdiv1023 = (u32VRef + u32VRef + u32VRef) / 1023;
    adcExternalVRef_2048Kdiv1023 = (u32VRef << 1) / 1023;
    adcExternalVRef_1024Kdiv1023 = u32VRef / 1023;
}

//sample = サンプリング数, clock = ADCモジュールのクロック(500KHZが推奨)
//AD変換時間は (サンプリング数 x 3 + 13)クロック 
void adc_enable(ADCSAMPLES sample, ADCCLOCKS clock, bool_t bUseExternalVRef) {
    //アナログ部の電源投入
    vAHI_ApConfigure(E_AHI_AP_REGULATOR_ENABLE, // DISABLE にするとアナログ部の電源断
        E_AHI_AP_INT_ENABLE,    // 割り込み
        sample,                 // サンプル数 2,4,6,8 が選択可能
        clock,                  // 周波数 250K/500K/1M/2M
        bUseExternalVRef);

    adcIsExternalVRef = bUseExternalVRef;

    while(!bAHI_APRegulatorEnabled()); // 安定するまで待つ（一瞬）
}

//contiuous=TRUE:連続,FALSE:1SHOT  range2=FALSE:0～Vref[V],TRUE:0～2*Vref[V] *Vrefは約1.235V
//ADC_SOURCE_3,4はそれぞれDIO0,1と共用
void adc_attachCallback(bool_t continuous, bool_t range2, ADCSOURCES source, void (*func)(uint16_t value)) {

    switch (source) {
    case ADC_SOURCE_3:
        dio_pinMode(0, INPUT);
        break;
    case ADC_SOURCE_4:
        dio_pinMode(1, INPUT);
        break;
#ifdef TWELITE_RED
    case ADC_SOURCE_5:
        dio_pinMode(2, INPUT);
        break;
    case ADC_SOURCE_6:
        dio_pinMode(3, INPUT);
        break;
#endif
/*
    case ADC_SOURCE_TEMP:
        range2 = FALSE;
        break;
    case ADC_SOURCE_VOLT:
        range2 = TRUE;
*/
    }

    adcIsContinuous = continuous;
    adcIsRange2 = range2;
    adcLastSource = source;
    adcCallbackFunction = func;

    vAHI_AdcEnable(continuous, range2, source);
    vAHI_AdcStartSample(); // ADC開始
}

void adc_detach()  {

    //adc_attachCallbackWithTimer()の後片付け
    if (adcLastSource == 0xff) {
        adcLastSource = 0;

        vAHI_AdcDisableSampleBuffer();//完了時にこれをやっとかないと2回目が実行できない

#ifdef USE_TIMER
        //タイマー停止
        uint8_t i;
        for(i=0; i<=4; i++) {
            if (sTimerApp[i].u8Mode == 7) { //ADC sampling(DMA)
                timer_detach(i);
                break;
            }
        }
#endif
    }

    adcCallbackFunction = NULL;
    vAHI_AdcDisable();
}

static uint16_t adcConvertADCx(uint16_t src) {
    if (adcIsRange2) {
        //1023 = 2.470V, 2.470V * 1024/1023 *1000 = 2472
        if (!adcIsExternalVRef) {
            return (2472 * (int32_t)src) >> 10; //[mV]
        } else {
            return (adcExternalVRef_2048Kdiv1023 * (int32_t)src) >> 10;
        }
   } else {
        //1023 = 1.235V, 1.235V * 1024/1023 *1000 = 1236
        if (!adcIsExternalVRef) {
            return (1236 * (int32_t)src) >> 10; //[mV]
        } else {
            return (adcExternalVRef_1024Kdiv1023 * (int32_t)src) >> 10;
        }
    }
}

static uint16_t adcConvertTemp(uint16_t src) {
    //温度センサーのスペック
    //730mV@25℃, -1.66mV/℃

    uint32_t resultValue;
    if (adcIsRange2) {
        if (!adcIsExternalVRef) {
            resultValue = (int32_t)src * 2472;     //x1024[mV]
        } else {
            resultValue = (int32_t)src * adcExternalVRef_2048Kdiv1023;
        }
    } else {
        if (!adcIsExternalVRef) {
            resultValue = (int32_t)src * 1236;     //x1024[mV]
        } else {
            resultValue = (int32_t)src * adcExternalVRef_1024Kdiv1023;
        }
        /*resultValue -= 730 * 1024;              //x1024
        resultValue *= -771;                    //771=(1/1.66)*1280  x1024x1280
                                                //1280は最終結果をx10[℃]にしたいため
        return (resultValue >> 17) + 250; //x10[℃]*/
    }
    resultValue -= 730 * 1024;              //x1024
    resultValue *= -771;                    //771=(1/1.66)*1280  x1024x1280
                                            //1280は最終結果をx10[℃]にしたいため
    return (resultValue >> 17) + 250; //x10[℃]
}

static uint16_t adcConvertVolt(uint16_t src) {

    if (adcIsRange2) {
        //電源電圧は通常、こっち(VREFx2)で測定

        //自身の電圧は2/3に分圧された値を測定しているので、1023のとき3.705V
        //1.235V * 2 * 1.5 = 3.705V
        //3709 = 3.705V / 1023 * 1000 * 1024

        if (!adcIsExternalVRef) {
            return ((int32_t)src * 3709) >> 10; //[mV]
        } else {
            return ((int32_t)src * adcExternalVRef_3072Kdiv1023) >> 10;
        }

    } else {

        //自身の電圧は2/3に分圧された値を測定しているので、1023のとき1.853V
        //1.235V * 1.5 = 1.853V
        //1855 = 1.853V / 1023 * 1000 * 1024

        if (!adcIsExternalVRef) {
            return ((int32_t)src * 1855) >> 10; //[mV]　　こっちは実質不可能。ADCレンジを超えるため
        } else {
            return ((int32_t)src * adcExternalVRef_3072Kdiv1023) >> 11;   //外部基準電圧を1.7V、電源電圧を2.4Vならここで測定できる（誰がやるのか?）
        }
    }
}

#ifdef USE_TIMER
bool_t adc_attachCallbackWithTimer(uint8_t timerNo, uint8_t prescale, uint16_t cycleCount,
        bool_t range2, ADCSOURCEBITMAP bitmap, uint16_t *pu16Buffer, uint16_t u16BufferSize, bool_t bBufferWrap, ADCINTERRUPTIONMODE mode, void (*func)()) {

    if (timerNo > 4) return FALSE;
    if (prescale > 16) return FALSE;
    if (pu16Buffer == NULL || u16BufferSize < 2) return FALSE;
    if (mode == 0 || func == NULL) return FALSE;

    uint8_t b = 4 << timerNo;
    if ((timerFineGrainDIOControlValue & b) == 0) {
        //DIOをタイマー用から汎用に切り替える
        timerFineGrainDIOControlValue |= b;
        vAHI_TimerFineGrainDIOControl(timerFineGrainDIOControlValue);
    }

    vAHI_TimerEnable(timerNo,
        prescale,
        FALSE, //bool_t bIntRiseEnable,
        FALSE, //bool_t bIntPeriodEnable,
        FALSE);//bool_t bOutputEnable);

    sTimerApp[timerNo].u8Mode = 7; //ADC sampling(DMA)
    sTimerApp[timerNo].u16HiCount = cycleCount;      //無意味だが何かしら設定
    sTimerApp[timerNo].u16LoCount = cycleCount;      //countまでカウントしたら割り込みがかかる
    timerCallbackFunctions[timerNo] = NULL;

    adcIsContinuous = bBufferWrap;
    adcIsRange2 = range2;
    adcLastSource = 0xff;
    //adcSourceBitmap = bitmap;
    //adcBuffer = pu16Buffer;
    //adcBufferSize = u16BufferSize;
    adcCallbackFunction = (void (*)(uint16_t))func;
    adcIntCountTillEnd = (mode & ADC_INT_HALF ? 1 : 0) + (mode & ADC_INT_FULL ? 1 : 0);

    if (bitmap & ADC_SOURCE_BITMAP_3) dio_pinMode(0, INPUT);
    if (bitmap & ADC_SOURCE_BITMAP_4) dio_pinMode(1, INPUT);
    //if (bitmap & ADC_SOURCE_BITMAP_5) dio_pinMode(2, INPUT);
    //if (bitmap & ADC_SOURCE_BITMAP_6) dio_pinMode(3, INPUT);

    vAHI_TimerStartRepeat(timerNo, cycleCount, cycleCount);

    return bAHI_AdcEnableSampleBuffer(
        range2,         //bool_t bInputRange x2,
        timerNo,        //uint8 u8Timer (0-4),
        bitmap,         //uint8 u8SourceBitmap,
        pu16Buffer,     //uint16 *pu16Buffer,
        u16BufferSize,  //uint16 u16BufferSize, 1-2047
        bBufferWrap,    //bool_t bBufferWrap, 
        mode);          //組み合わせ可能。ADC_INT_OVERを使えるのは bBufferWrap=FALSE のときだけ
}

/*
//テスト関数
bool_t adc_attachCallbackWithTimer2(uint8_t timerNo, bool_t range2, ADCSOURCEBITMAP bitmap, uint16_t *pu16Buffer, uint16_t u16BufferSize, bool_t bBufferWrap, ADCINTERRUPTIONMODE mode, void (*func)()) {

    if (timerNo > 4) return FALSE;
    if (pu16Buffer == NULL || u16BufferSize < 2) return FALSE;
    if (mode == 0 || func == NULL) return FALSE;

    adcIsContinuous = bBufferWrap;
    adcIsRange2 = range2;
    adcLastSource = 0xff;
    //adcSourceBitmap = bitmap;
    //adcBuffer = pu16Buffer;
    //adcBufferSize = u16BufferSize;
    adcCallbackFunction = (void (*)(uint16_t))func;
    adcIntCountTillEnd = (mode & ADC_INT_HALF ? 1 : 0) + (mode & ADC_INT_FULL ? 1 : 0);

    if (bitmap & ADC_SOURCE_BITMAP_3) dio_pinMode(0, INPUT);
    if (bitmap & ADC_SOURCE_BITMAP_4) dio_pinMode(1, INPUT);
    //if (bitmap & ADC_SOURCE_BITMAP_5) dio_pinMode(2, INPUT);
    //if (bitmap & ADC_SOURCE_BITMAP_6) dio_pinMode(3, INPUT);

    return bAHI_AdcEnableSampleBuffer(
        range2,         //bool_t bInputRange x2,
        timerNo,        //uint8 u8Timer (0-4),
        bitmap,         //uint8 u8SourceBitmap,
        pu16Buffer,     //uint16 *pu16Buffer,
        u16BufferSize,  //uint16 u16BufferSize, 1-2047
        bBufferWrap,    //bool_t bBufferWrap, 
        mode);          //組み合わせ可能。ADC_INT_OVERを使えるのは bBufferWrap=FALSE のときだけ
}*/

bool_t adc_convertResults(ADCSOURCEBITMAP bitmap, uint16_t *pu16Buffer, uint16_t u16BufferSize) {
    if (bitmap == 0 || u16BufferSize == 0 || pu16Buffer == NULL) return FALSE;
    while(1) {
        if (bitmap & ADC_SOURCE_BITMAP_1) { *pu16Buffer = adcConvertADCx(*pu16Buffer); pu16Buffer++; if(--u16BufferSize == 0) break; }
        if (bitmap & ADC_SOURCE_BITMAP_2) { *pu16Buffer = adcConvertADCx(*pu16Buffer); pu16Buffer++; if(--u16BufferSize == 0) break; }
        if (bitmap & ADC_SOURCE_BITMAP_3) { *pu16Buffer = adcConvertADCx(*pu16Buffer); pu16Buffer++; if(--u16BufferSize == 0) break; }
        if (bitmap & ADC_SOURCE_BITMAP_4) { *pu16Buffer = adcConvertADCx(*pu16Buffer); pu16Buffer++; if(--u16BufferSize == 0) break; }
        if (bitmap & ADC_SOURCE_BITMAP_TEMP) { *pu16Buffer = adcConvertTemp(*pu16Buffer); pu16Buffer++; if(--u16BufferSize == 0) break; }
        if (bitmap & ADC_SOURCE_BITMAP_VOLT) { *pu16Buffer = adcConvertVolt(*pu16Buffer); pu16Buffer++; if(--u16BufferSize == 0) break; }
        //if (bitmap & ADC_SOURCE_BITMAP_5) { *pu16Buffer = adcConvertADCx(*pu16Buffer); pu16Buffer++; if(--u16BufferSize == 0) break; }
        //if (bitmap & ADC_SOURCE_BITMAP_6) { *pu16Buffer = adcConvertADCx(*pu16Buffer); pu16Buffer++; if(--u16BufferSize == 0) break; }
    }
    return TRUE;
}
#endif //USE_TIMER

#endif //USE_ADC



/*
 *　コンパレータ
 */

#ifdef USE_COMP

//割り込みルーチンのポインタを保持
static void (*compCallbackFunction)();


//コンパレーターを開始する。
//ADCモジュールの電源がONされます。
//選択された外部入力ピンはINPUTモードに設定されます。
//bLowPowerMode=TRUEにより73μA->0.8μAまで消費電力が下がりますが、反応速度が遅くなります。
bool_t comp_enable(COMPSIGNALSOURCE signal, COMPVREFSOURCE vref, COMPHISTERESIS his, bool_t bLowPowerMode) {
    uint8_t sel;
    if (signal == COMP_SIGNAL_COMP1P) {
        if (vref == COMP_REF_COMP1M) {
            sel = E_AHI_COMP_SEL_EXT;
            dio_pinMode(17, INPUT);
        } else if (vref == COMP_REF_VREF) {
            sel = E_AHI_COMP_SEL_BANDGAP;
        } else {
            return FALSE;
        }
        dio_pinMode(16, INPUT);
    } else if (signal == COMP_SIGNAL_COMP1M) {
        if (vref == COMP_REF_COMP1P) {
            sel = E_AHI_COMP_SEL_EXT_INVERSE;
            dio_pinMode(16, INPUT);
        } else if (vref == COMP_REF_VREF) {
            sel = E_AHI_COMP_SEL_BANDGAP_INVERSE;
        } else {
            return FALSE;
        }
        dio_pinMode(17, INPUT);
    } else {
        return FALSE;
    } 

//    bool_t turned_on = FALSE;
    if (!bAHI_APRegulatorEnabled()) {
        //設定には電源が必要
        vAHI_ApConfigure(E_AHI_AP_REGULATOR_ENABLE, // DISABLE にするとアナログ部の電源断
            FALSE,                      // 割り込みなし
            E_AHI_AP_SAMPLE_2,          // 意味なし サンプル数 2,4,6,8 が選択可能
            E_AHI_AP_CLOCKDIV_500KHZ,   // 意味なし 周波数 250K/500K/1M/2M
            FALSE);                     // 内部基準電圧を使う

        while(!bAHI_APRegulatorEnabled()); // 安定するまで待つ（一瞬）
//        turned_on = TRUE;
    }

    vAHI_ComparatorEnable(
        E_AHI_AP_COMPARATOR_1, //uint8 u8Comparator,
        his,    //uint8 u8Hysteresis,
        sel);   //uint8 u8SignalSelect);

    vAHI_ComparatorLowPowerMode(bLowPowerMode);
/*
    if (turned_on && vref != COMP_REF_VREF) {
        //もう使用しない
        vAHI_ApConfigure(E_AHI_AP_REGULATOR_DISABLE, // アナログ部の電源断
            FALSE,
            E_AHI_AP_SAMPLE_2,
            E_AHI_AP_CLOCKDIV_500KHZ,
            FALSE);
    }
*/
    return TRUE;
}

//comp_enable()で開始されたコンパレーターに変化を受け取るコールバック関数を設定します。
//mode = FALLING または RISING のいずれか
bool_t comp_attachCallback(INTERRUPTIONEDGES mode, void (*func)()) {
    if (mode != FALLING && mode != RISING) return FALSE;

    if (!bAHI_APRegulatorEnabled()) {
        //設定には電源が必要
        vAHI_ApConfigure(E_AHI_AP_REGULATOR_ENABLE, // DISABLE にするとアナログ部の電源断
            FALSE,                      // 割り込みなし
            E_AHI_AP_SAMPLE_2,          // 意味なし サンプル数 2,4,6,8 が選択可能
            E_AHI_AP_CLOCKDIV_500KHZ,   // 意味なし 周波数 250K/500K/1M/2M
            FALSE);                     // 意味なし 内部基準電圧を使う

        while(!bAHI_APRegulatorEnabled()); // 安定するまで待つ（一瞬）
    }
    compCallbackFunction = func;

    vAHI_ComparatorIntEnable(
        E_AHI_AP_COMPARATOR_1,  //uint8 u8Comparator,
        (func != NULL),         //bool_t bIntEnable, コールバックを呼ぶとき、ウェイク条件にするときはTRUE
        (mode == RISING));      //bool_t bRisingNotFalling);

    return TRUE;
}

//コンパレーターを停止します。VREFを基準電圧としていた場合はadc_disable()を別途呼び出す必要があります。
void comp_disable() {
    vAHI_ComparatorDisable(E_AHI_AP_COMPARATOR_1);
    compCallbackFunction = NULL;
}

//コンパレータ割り込み条件をウェイク条件に設定します。
//mode = FALLING または RISING のいずれか
//あらかじめcomp_enable()でコンパレータを起動しておく必要があります。
//この場合、基準電圧にCOMP_REF_VREFは使用できません。スリープ中はADCモジュールの電源がOFFされるためです。
//コンパレータは低消費電力モードに設定されます。
//この関数はsleep()の直前に呼び出してしてください。
bool_t comp_setWake(INTERRUPTIONEDGES mode) {
    if (mode != FALLING && mode != RISING) return FALSE;

    //if (!comp_attachCallback(mode, NULL)) return FALSE;

    compCallbackFunction = NULL;

/*    vAHI_ApConfigure(TRUE, // アナログ部の電源断
        TRUE,
        E_AHI_AP_SAMPLE_2,
        E_AHI_AP_CLOCKDIV_500KHZ,
        FALSE);*/
    
    if (!bAHI_APRegulatorEnabled()) {
        //設定には電源が必要
        vAHI_ApConfigure(E_AHI_AP_REGULATOR_ENABLE, // DISABLE にするとアナログ部の電源断
            FALSE,                      // 割り込みなし
            E_AHI_AP_SAMPLE_2,          // 意味なし サンプル数 2,4,6,8 が選択可能
            E_AHI_AP_CLOCKDIV_500KHZ,   // 意味なし 周波数 250K/500K/1M/2M
            FALSE);                     // 意味なし 内部基準電圧を使う

        while(!bAHI_APRegulatorEnabled()); // 安定するまで待つ（一瞬）
    }

    vAHI_ComparatorIntEnable(
        E_AHI_AP_COMPARATOR_1,  //uint8 u8Comparator,
        TRUE,                   //bool_t bIntEnable, コールバックを呼ぶとき、ウェイク条件にするときはTRUE
        (mode == RISING));      //bool_t bRisingNotFalling);

    vAHI_ComparatorLowPowerMode(TRUE);  //必須

    u8AHI_ComparatorWakeStatus();   //clear interrupt flag
    return TRUE;
}

//コンパレータの比較結果を取得します。入力が基準より高い場合にTRUE、低い場合にFALSEを返します
bool_t comp_read() {
    if (!bAHI_APRegulatorEnabled()) {
        //設定には電源が必要
        vAHI_ApConfigure(E_AHI_AP_REGULATOR_ENABLE, // DISABLE にするとアナログ部の電源断
            FALSE,                      // 割り込みなし
            E_AHI_AP_SAMPLE_2,          // 意味なし サンプル数 2,4,6,8 が選択可能
            E_AHI_AP_CLOCKDIV_500KHZ,   // 意味なし 周波数 250K/500K/1M/2M
            FALSE);                     // 意味なし 内部基準電圧を使う

        while(!bAHI_APRegulatorEnabled()); // 安定するまで待つ（一瞬）
    }
    return (u8AHI_ComparatorStatus() & E_AHI_AP_COMPARATOR_MASK_1);
}


#endif //USE_COMP



/*
 *　パルスカウンタ
 */

#ifdef USE_PC

//割り込みルーチンのポインタを保持
static void (*pcCallbackFunctions[2])();


//u32AHI_PulseCounterStatus()の値を保持(1回読むとフラグがクリアされるので)
static uint32_t pcCountStatus;


//パルスカウンタを開始します。入力ピンは事前にdio_pinMode()でINPUTまたはINPUT_PULLUPに設定しておいてください
//pcNo=0/1, mode=RISING/FALLING, count=何回目でコールバック関数を呼び出すかを指定
//bUseSecondPin PC0  PC1
//       FALSE  DIO1 DIO8
//       TRUE   DIO4 DIO5
bool_t pc_enable(uint8_t pcNo, PCDEBOUNCEMODE debounce, uint16_t count, bool_t bUseSecondPin, INTERRUPTIONEDGES mode, bool_t bStartNow) {
    if (pcNo > 1) return FALSE;
    if (mode != FALLING && mode != RISING) return FALSE;

    pcCallbackFunctions[pcNo] = NULL;

    vAHI_PulseCounterSetLocation(pcNo, bUseSecondPin);

    bAHI_PulseCounterConfigure(
        pcNo,               //パルスカウンター0または1、結合する場合は0
        (mode == FALLING),  //FALSE:RISING TRUE:FALLING
        debounce,           //デバウンス
                            //0: No debounce (maximum input frequency of 100kHz)
                            //1: 2 samples (maximum input frequency of 3.7kHz)
                            //2: 4 samples (maximum input frequency of 2.2kHz)
                            //3: 8 samples (maximum input frequency of 1.2kHz)
        0,                  //結合
                            //E_AHI_PC_COMBINE_OFF (0 - Pulse counters not combined)
                            //E_AHI_PC_COMBINE_ON0 (1 - Counters combined using PC0 input)
                            //E_AHI_PC_COMBINE_ON1 (2 - Counters combined using PC1 input)
        TRUE);              //bool_t bIntEnable); カウンタリセットに必要。
        //↑ここで割り込み有効にするとu32AHI_PulseCounterStatus()のフラグが立たなくなる。
        //そのため割り込みからpcCountStatusのフラグを立てている
        //この割り込みフラグはスリープに影響しない(テストで確認)

    bAHI_SetPulseCounterRef(pcNo, (uint32_t)(count - 1));
    pc_clear(pcNo);
    if (bStartNow) pc_start(pcNo);

    return TRUE;
}

//パルスカウンタを無効にします
bool_t pc_disable(uint8_t pcNo) {
    if (pcNo > 1) return FALSE;

    pcCallbackFunctions[pcNo] = NULL;

    bAHI_PulseCounterConfigure(
        pcNo,               //パルスカウンター0または1、結合する場合は0
        FALSE,              //FALSE:RISING TRUE:FALLING
        0,                  //デバウンス
                            //0: No debounce (maximum input frequency of 100kHz)
                            //1: 2 samples (maximum input frequency of 3.7kHz)
                            //2: 4 samples (maximum input frequency of 2.2kHz)
                            //3: 8 samples (maximum input frequency of 1.2kHz)
        0,                  //結合
                            //E_AHI_PC_COMBINE_OFF (0 - Pulse counters not combined)
                            //E_AHI_PC_COMBINE_ON0 (1 - Counters combined using PC0 input)
                            //E_AHI_PC_COMBINE_ON1 (2 - Counters combined using PC1 input)
        FALSE);              //bool_t bIntEnable);
        return TRUE;
}

//パルスカウンタのコールバック関数を登録します。
//パルスカウンタは事前にpc_enable()で開始しておく必要があります。
//func=NULLで登録を解除できます(パルスカウンタは停止しません)。
bool_t pc_attachCallback(uint8_t pcNo, void (*func)()) {
    if (pcNo > 1) return FALSE;
    pcCallbackFunctions[pcNo] = func;
    return TRUE;
}

//パルスカウンタのカウント値を読み出します
uint16_t pc_read(uint8_t pcNo) {
    if (pcNo > 1) return 0xffff;

    uint16_t value;
    if (!bAHI_Read16BitCounter(pcNo, &value)) return 0xffff;
    return value;
}

//カウンタが設定値に到達したかどうかを返します。
//読み取ったらフラグはクリアされます。
bool_t pc_countReached(uint8_t pcNo) {
    if (pcNo > 1) return FALSE;
    
    pcCountStatus |= u32AHI_PulseCounterStatus();
    bool_t r;
    if (pcNo == 0) {
        r = (pcCountStatus & PC_INT_BITMAP_0) ? TRUE : FALSE;
        pcCountStatus &= PC_INT_BITMAP_0 ^ 0xffffffff;
    } else {
        r = (pcCountStatus & PC_INT_BITMAP_1) ? TRUE : FALSE;
        pcCountStatus &= PC_INT_BITMAP_1 ^ 0xffffffff;
    }
    return r;
}

//32ビットパルスカウンタを開始します。PC0とPC1を結合して使用するため、併用はできません。
//入力ピンは事前にdio_pinMode()でINPUTまたはINPUT_PULLUPに設定しておいてください
//pcNo=0/1, mode=RISING/FALLING, count=何回目でコールバック関数を呼び出すかを指定
//pinNo=DIO1,4,5,8のいずれか
//カウンタは０にクリアされます。
bool_t pc32_enable(PCDEBOUNCEMODE debounce, uint32_t count, uint8_t pinNo, INTERRUPTIONEDGES mode, bool_t bStartNow) {
    if (mode != FALLING && mode != RISING) return FALSE;

    uint8_t combine;
    bool_t bUseSecondPin;
    switch (pinNo) {
        case 1: combine = E_AHI_PC_COMBINE_ON0; bUseSecondPin = FALSE; break;
        case 4: combine = E_AHI_PC_COMBINE_ON0; bUseSecondPin = TRUE; break;
        case 5: combine = E_AHI_PC_COMBINE_ON1; bUseSecondPin = TRUE; break;
        case 8: combine = E_AHI_PC_COMBINE_ON1; bUseSecondPin = FALSE; break;
        default: return FALSE;
    }

    pcCallbackFunctions[0] = NULL;

    vAHI_PulseCounterSetLocation(0, bUseSecondPin);

    bAHI_PulseCounterConfigure(
        0,                  //パルスカウンター0または1、結合する場合は0
        (mode == FALLING),  //FALSE:RISING TRUE:FALLING
        debounce,           //デバウンス
                            //0: No debounce (maximum input frequency of 100kHz)
                            //1: 2 samples (maximum input frequency of 3.7kHz)
                            //2: 4 samples (maximum input frequency of 2.2kHz)
                            //3: 8 samples (maximum input frequency of 1.2kHz)
        combine,            //結合
                            //E_AHI_PC_COMBINE_OFF (0 - Pulse counters not combined)
                            //E_AHI_PC_COMBINE_ON0 (1 - Counters combined using PC0 input)
                            //E_AHI_PC_COMBINE_ON1 (2 - Counters combined using PC1 input)
        TRUE);              //bool_t bIntEnable); カウンタリセットに必要

    bAHI_SetPulseCounterRef(0, count - 1);
    pc32_clear();
    if (bStartNow) pc32_start();

    return TRUE;
}

//32ビットパルスカウンタのカウント値を読み出します
uint32_t pc32_read() {
    uint32_t value;
    if (!bAHI_Read32BitCounter(&value)) return 0xffffffff;
    return value;
}

#endif //USE_PC



/*
 * I2Cマスター
 */

#if defined(USE_I2C) || defined(USE_I2CS)
static uint8_t i2cI2csInitFlag; //1:i2c initialized 2:i2cs initialized
#endif

#ifdef USE_I2C

I2CADDRESSINGMODE i2cAddressingMode;

//I2C初期化。標準のクロックは I2C_CLOCK_100KHZ
//bUseSecondPin  SCL   SDA
// = FALSE      DIO14 DIO15
// = TRUE       DIO16 DIO17
void i2c_enable(I2CCLOCKS clock, bool_t bUseSecondPin) {
    if (i2cI2csInitFlag == 2) vAHI_SiSlaveDisable();
    vAHI_SiSetLocation(bUseSecondPin);
    vAHI_SiMasterConfigure(TRUE, FALSE, clock);
    i2cI2csInitFlag = 1;
}

void i2c_disable() {
    vAHI_SiMasterDisable();
    if (i2cI2csInitFlag == 1) i2cI2csInitFlag = 0;
}

static bool_t i2cWait()
{
	while(bAHI_SiMasterPollTransferInProgress()); //処理待ち

	if (bAHI_SiMasterPollArbitrationLost() | bAHI_SiMasterCheckRxNack()) {
		//通信ロストかNACKが返された
        //バスを解放する
		vAHI_SiMasterSetCmdReg(E_AHI_SI_NO_START_BIT,
						 E_AHI_SI_STOP_BIT,         //STOP
						 E_AHI_SI_NO_SLAVE_READ,
						 E_AHI_SI_SLAVE_WRITE,      //WRITE
						 E_AHI_SI_SEND_ACK,         //ACK
						 E_AHI_SI_NO_IRQ_ACK);
        return FALSE;
	}
	return TRUE;
}

static bool_t setAddress(bool_t bRead, uint16_t u16Address) {
    if (i2cAddressingMode == I2C_ADDRESS_7BIT) {
        //7ビットアドレス設定
        vAHI_SiMasterWriteSlaveAddr((uint8_t)u16Address, bRead);  //FALSE:書き込み
        vAHI_SiMasterSetCmdReg(E_AHI_SI_START_BIT,  //START
                        E_AHI_SI_NO_STOP_BIT,
                        E_AHI_SI_NO_SLAVE_READ,
                        E_AHI_SI_SLAVE_WRITE,      //WRITE
                        E_AHI_SI_SEND_ACK,         //ACK
                        E_AHI_SI_NO_IRQ_ACK);
    } else {
        //10ビットアドレス設定
        //上位2ビットアドレス設定
        vAHI_SiMasterWriteSlaveAddr(0x78 | (u16Address >> 8), bRead); //FALSE:書き込み
        vAHI_SiMasterSetCmdReg(E_AHI_SI_START_BIT,  //START
                        E_AHI_SI_NO_STOP_BIT,
                        E_AHI_SI_NO_SLAVE_READ,
                        E_AHI_SI_SLAVE_WRITE,      //WRITE
                        E_AHI_SI_SEND_ACK,         //ACK
                        E_AHI_SI_NO_IRQ_ACK);
        if (!i2cWait()) return FALSE;

        //下位8ビットアドレス設定
        vAHI_SiMasterWriteData8(u16Address & 0xff);
        vAHI_SiMasterSetCmdReg(E_AHI_SI_NO_START_BIT,
                            E_AHI_SI_NO_STOP_BIT,
                            E_AHI_SI_NO_SLAVE_READ,
                            E_AHI_SI_SLAVE_WRITE,      //WRITE
                            E_AHI_SI_SEND_ACK,         //ACK
                            E_AHI_SI_NO_IRQ_ACK);
    }
    if (!i2cWait()) return FALSE;
    return TRUE;
}

//I2Cで指定アドレスにコマンドとデータを書き込む。データが無いときはpu8Data=NULL,u8Length=0とする
//事前にi2c_setAddressingMode()でアドレスサイズを設定しておくこと。デフォルトは７ビット
bool_t i2c_write(uint16_t u16Address, uint8_t u8Command, const uint8* pu8Data, uint8_t u8Length)
{
    if (pu8Data == NULL) u8Length = 0;

    //アドレス設定
    if (!setAddress(FALSE, u16Address)) return FALSE;

	bool_t bCommandSent = FALSE;

	while(bCommandSent == FALSE || u8Length > 0){
		if(!bCommandSent){
			//コマンド送信
			vAHI_SiMasterWriteData8(u8Command);
			bCommandSent = TRUE;
		} else {
			u8Length--;
			//データ送信
			vAHI_SiMasterWriteData8(*pu8Data++);
		}
		if(u8Length == 0){
            //最後のバイトを送り出す
			vAHI_SiMasterSetCmdReg(E_AHI_SI_NO_START_BIT,
							 E_AHI_SI_STOP_BIT,         //STOP
							 E_AHI_SI_NO_SLAVE_READ,
							 E_AHI_SI_SLAVE_WRITE,      //WRITE
							 E_AHI_SI_SEND_ACK,         //ACK
							 E_AHI_SI_NO_IRQ_ACK);

		} else {
            //途中のバイトを送り出す
			vAHI_SiMasterSetCmdReg(E_AHI_SI_NO_START_BIT,
							 E_AHI_SI_NO_STOP_BIT,
							 E_AHI_SI_NO_SLAVE_READ,
							 E_AHI_SI_SLAVE_WRITE,      //WRITE
							 E_AHI_SI_SEND_ACK,         //ACK
							 E_AHI_SI_NO_IRQ_ACK);
		}
		if(!i2cWait()) return FALSE;
	}
	return TRUE;
}

//I2Cで指定アドレスにコマンドとデータを書き込む。データが無いときはpu8Data=NULL,u8Length=0とする
//事前にi2c_setAddressingMode()でアドレスサイズを設定しておくこと。デフォルトは７ビット
bool_t i2c_writeOnly(uint16_t u16Address, const uint8* pu8Data, uint8_t u8Length)
{
    //アドレス設定
    if (!setAddress(FALSE, u16Address)) return FALSE;

	while(u8Length > 0){
        u8Length--;

        //データ送信
        vAHI_SiMasterWriteData8(*pu8Data++);

		if(u8Length == 0){
            //最後のバイトを送り出す
			vAHI_SiMasterSetCmdReg(E_AHI_SI_NO_START_BIT,
							 E_AHI_SI_STOP_BIT,         //STOP
							 E_AHI_SI_NO_SLAVE_READ,
							 E_AHI_SI_SLAVE_WRITE,      //WRITE
							 E_AHI_SI_SEND_ACK,         //ACK
							 E_AHI_SI_NO_IRQ_ACK);

		} else {
            //途中のバイトを送り出す
			vAHI_SiMasterSetCmdReg(E_AHI_SI_NO_START_BIT,
							 E_AHI_SI_NO_STOP_BIT,
							 E_AHI_SI_NO_SLAVE_READ,
							 E_AHI_SI_SLAVE_WRITE,      //WRITE
							 E_AHI_SI_SEND_ACK,         //ACK
							 E_AHI_SI_NO_IRQ_ACK);
		}
		if(!i2cWait()) return FALSE;
	}
	return TRUE;
}

//I2Cで指定アドレスからデータを読み出す
bool_t i2c_readOnly(uint16_t u16Address, uint8* pu8Data, uint8_t u8Length)
{
    //アドレス設定
    if (!setAddress(TRUE, u16Address)) return FALSE;

	while (u8Length-- > 0){
		if(u8Length == 0){
            //最後のバイトを読む
			vAHI_SiMasterSetCmdReg(E_AHI_SI_NO_START_BIT,
							 E_AHI_SI_STOP_BIT,         //STOP
							 E_AHI_SI_SLAVE_READ,       //READ
							 E_AHI_SI_NO_SLAVE_WRITE,
							 E_AHI_SI_SEND_NACK,        //NACK
							 E_AHI_SI_NO_IRQ_ACK);
		} else {
            //途中のバイトを読む
			vAHI_SiMasterSetCmdReg(E_AHI_SI_NO_START_BIT,
							 E_AHI_SI_NO_STOP_BIT,
							 E_AHI_SI_SLAVE_READ,       //READ
							 E_AHI_SI_NO_SLAVE_WRITE,
							 E_AHI_SI_SEND_ACK,         //ACK
							 E_AHI_SI_NO_IRQ_ACK);
		}
		while(bAHI_SiMasterPollTransferInProgress()); //待ち
		*pu8Data++ = u8AHI_SiMasterReadData8();
	}
	return TRUE;
}

//I2Cで指定アドレスのコマンドからデータを読み出します
bool_t i2c_read(uint16_t u16Address, uint8_t u8Command, uint8* pu8Data, uint8_t u8Length) {
    if (!i2c_write(u16Address, u8Command, NULL, 0)) return FALSE;
    if (pu8Data != NULL && u8Length > 0) {
        if (!i2c_readOnly(u16Address, pu8Data, u8Length)) return FALSE;
    }
    return TRUE;
}

//I2Cで指定アドレスのコマンドから1バイトのデータを読み出します。失敗で-1
int16_t i2c_readByte(uint16_t u16Address, uint8_t u8Command) {
    if (!i2c_write(u16Address, u8Command, NULL, 0)) return -1;
    uint8_t data;
    if (!i2c_readOnly(u16Address, &data, 1)) return -1;
    return (int16_t)data;
}

//I2Cで指定アドレスにコマンドと1バイトのデータを書き込みます。
bool_t i2c_writeByte(uint16_t u16Address, uint8_t u8Command, uint8_t u8Data) {
    return i2c_write(u16Address, u8Command, &u8Data, 1);
}

//I2Cで指定アドレスに1バイトのデータを書き込みます。
bool_t i2c_writeByteOnly(uint16_t u16Address, uint8_t u8Data) {
    return i2c_writeOnly(u16Address, &u8Data, 1);
}

//I2Cで指定アドレスからデータを1バイト読み出します。失敗で-1
int16_t i2c_readByteOnly(uint16_t u16Address) {
    uint8_t data;
    if (!i2c_readOnly(u16Address, &data, 1)) return -1;
    return (int16_t)data;
}

#endif //USE_I2C


/*
 * I2Cスレーブ
 */

#ifdef USE_I2CS

static uint8_t i2csu8LastInt;
static uint8_t i2csu8cmd;
static bool_t i2csbNewCmd;

static uint8_t i2csu8MWBufferSelect; //ダブルバッファのどっちに書き込むか
static uint8_t i2csau8MWBuffer[2][I2CS_MW_BUFFER_SIZE]; //ダブルバッファにする必要あるかも
static uint8_t i2csu8MWIndex[2];

static const uint8_t *i2cspau8MRBuffer;  //by user
static uint8_t i2csu8MRBufferSize; //by user
static uint8_t i2csu8MRIndex;
static bool_t i2csbDataNotReady;

static int16_t i2csi16CmdForMRCallback;
static void (*i2csMRCallbackFunction)(uint8_t);

static int16_t i2csi16BufSelForMWCallback;
static void (*i2csMWCallbackFunction)(uint8_t *, uint8_t);

void i2cs_enable(uint8_t u16Address, bool_t b10BitAddress, bool_t bUseSecondPin, void (*prepareFunc)(uint8_t), void (*receivedFunc)(uint8_t*, uint8_t)) {
    if (i2cI2csInitFlag == 1) vAHI_SiMasterDisable();

    i2csMRCallbackFunction = prepareFunc;
    i2csMWCallbackFunction = receivedFunc;

    vAHI_SiSetLocation(bUseSecondPin);
    vAHI_SiSlaveConfigure(
        u16Address,     //uint16 u16SlaveAddress,
        b10BitAddress,  //bool_t bExtendAddr,
        TRUE,           //bool_t bPulseSuppressionEnable,
        E_AHI_SIS_DATA_RR_MASK |
        E_AHI_SIS_DATA_WA_MASK |
        E_AHI_SIS_LAST_DATA_MASK |
        E_AHI_SIS_ERROR_MASK, //uint8 u8InMaskEnable,
        FALSE);         //bool_t bFlowCtrlMode);
/*
    0 E_AHI_SIS_DATA_RR_MASK    マスターに送るデータをバッファに書き込む必要あり。　Data buffer must be written with data to be read by SI master
    1 E_AHI_SIS_DATA_RTKN_MASK  マスターに送る次のデータをバッファに書き込めるで。　Data taken from buffer by SI master - buffer free for next data
    2 E_AHI_SIS_DATA_WA_MASK    データバッファにマスターからのデータが入ってるで。　Data buffer contains data from SI master to be read by SI slave
    3 E_AHI_SIS_LAST_DATA_MASK  最後のデータが送信されたよ。　　　　　　　　　　　Last data transferred (end of burst)
    4 E_AHI_SIS_ERROR_MASK      I2C protocol error
*/
    i2cI2csInitFlag = 2;
}

void i2cs_disable() {
    vAHI_SiSlaveDisable()
    if (i2cI2csInitFlag == 2) i2cI2csInitFlag = 0;
}

void i2cs_write(const uint8_t *pau8Data, uint8_t u8Length) {
    i2cspau8MRBuffer = pau8Data;
    i2csu8MRBufferSize = u8Length;
}

#endif //USE_I2CS



/*
 * SPIマスター
 */

#ifdef USE_SPI

/* SPI MODE
0 Data latched on rising edge of clock
1 Data latched on falling edge of clock
2 Clock inverted and data latched on falling edge of clock
3 Clock inverted and data latched on rising edge of clock
*/

//SPIマスターを有効にする
//u8NumSlaves=1..3
//使用するピン CLK:DO0(ｼﾙｸC), MISO(in):DO1(ｼﾙｸI), MOSI(out):DIO18, SS(CSB):DIO19(slave0)
bool_t spi_enable(uint8_t u8NumSlaves, SPIMODES u8Mode, SPICLOCKS clock) {
    if (u8NumSlaves == 0 || u8NumSlaves > 3) return FALSE;

    bool_t polarity, phase;
    switch (u8Mode) {
    case SPI_MODE_0: polarity=FALSE; phase=FALSE; break;
    case SPI_MODE_1: polarity=FALSE; phase=TRUE; break;
    case SPI_MODE_2: polarity=TRUE; phase=FALSE; break;
    case SPI_MODE_3: polarity=TRUE; phase=TRUE; break;
    default: return FALSE;
    }

    vAHI_SpiConfigure(u8NumSlaves - 1,
        E_AHI_SPIM_MSB_FIRST,       //bool_t bLsbFirst,  最上ビットから送信
        polarity,                   //bool_t bPolarity,
        phase,                      //bool_t bPhase,
        clock,                      //uint8 u8ClockDivider,
        E_AHI_SPIM_INT_DISABLE,     //bool_t bInterruptEnable,
        E_AHI_SPIM_AUTOSLAVE_DSABL  //bool_t bAutoSlaveSelect
    );
    return TRUE;
}

//スレーブを選択するSSピンを変更する。spi_selectSlave()に適用
//スレーブ0　DIO19固、変更できない
//スレーブ1　DIO0(FALSE, デフォルト) / DIO14(TRUE)
//スレーブ2　DIO1(FALSE, デフォルト) / DIO15(TRUE)
bool_t spi_selectSlavePin(uint8_t slaveNo, bool_t bSecondPin) {
    if (slaveNo != 1 && slaveNo != 2) return FALSE;
    vAHI_SpiSelSetLocation(slaveNo, bSecondPin);
    return TRUE;
}

//SPIスレーブ 0..2を選択。その他の値で無選択となる
//具体的には0..2で次のDIO(SSピン)がLになる DIO19,DIO0*,DIO1*  *spi_selectPin()で変更可能
/*void spi_begin(int8_t slaveNo) {
    if (slaveNo >= 0 && slaveNo <= 2) {
        vAHI_SpiSelect(1 << slaveNo);
    } else {
        vAHI_SpiSelect(0);
    }
}*/

//1バイトのコマンドを送信し、1バイトのデータを読み取る
//slaveNo=0..2
uint8_t spi_readByte(int8_t slaveNo, uint8_t u8Command) {
    vAHI_SpiSelect(1 << slaveNo);
    uint16_t w = (uint16_t)u8Command << 8;
    vAHI_SpiStartTransfer(15, w);
    while(bAHI_SpiPollBusy());
    uint8_t r= u8AHI_SpiReadTransfer8();
    vAHI_SpiSelect(0);
    return r;
}

//1バイトのコマンドを送信し、複数バイトのデータを読み取る
//slaveNo=0..2
void spi_read(int8_t slaveNo, uint8_t u8Command, uint8* pu8Data, uint8_t u8Length) {
    if (pu8Data == NULL || u8Length == 0) return;

    vAHI_SpiSelect(1 << slaveNo);

    vAHI_SpiStartTransfer(7, u8Command++);
    while(bAHI_SpiPollBusy());

    while(u8Length-- > 1) {
        vAHI_SpiStartTransfer(7, u8Command++);
        while(bAHI_SpiPollBusy());
        *pu8Data++ = u8AHI_SpiReadTransfer8();
    }

    vAHI_SpiStartTransfer(7, 0x00);
    while(bAHI_SpiPollBusy());
    *pu8Data++ = u8AHI_SpiReadTransfer8();
    vAHI_SpiSelect(0);
}

//1バイトのコマンドを送信し、1バイトのデータを書き込む
//slaveNo=0..2
void spi_writeByte(int8_t slaveNo, uint8_t u8Command, uint8_t u8Data) {
    vAHI_SpiSelect(1 << slaveNo);
    uint16_t w = ((uint16_t)u8Command << 8) | u8Data;
    vAHI_SpiStartTransfer(15, w);
    while(bAHI_SpiPollBusy());
    vAHI_SpiSelect(0);
}

void spi_write8(uint8_t u8Data) {
    vAHI_SpiStartTransfer(7, u8Data);
    while(bAHI_SpiPollBusy());
}

void spi_write16(uint16_t u16Data) {
    vAHI_SpiStartTransfer(15, u16Data);
    while(bAHI_SpiPollBusy());
}

void spi_write32(uint32_t u32Data) {
    vAHI_SpiStartTransfer(31, u32Data);
    while(bAHI_SpiPollBusy());
}

#endif //USE_SPI



/*
 * 無線通信
 */

#ifdef USE_RADIO

/*
ネットワーク層  暗号化  宛先    送信元   ペイロードの最大
    なし       なし    ショート ショート    104     //u16MyShortAddress != 0xFFFF
    なし       なし    ショート ロング      98      //u16MyShortAddress == 0xFFFF
    なし       なし    ロング   ロング      92      //u16MyShortAddress == 0xFFFF
*/

//送信完了割り込みルーチンのポインタを保持
static void (*radioTxCallbackFunction)(uint8_t, bool_t);

//受信割り込みルーチンのポインタを保持
static void (*radioRxCallbackFunction)(uint32_t, bool_t, uint8_t, uint8_t, uint8_t *, uint8_t, uint8_t);

//受信重複判定のコールバック関数へのポインタを保持
static bool_t (*radioRxDuplicateJudgementCallbackFunction)(uint32_t, uint8_t);

//送信シーケンス番号を保持
static uint8_t u8RadioSeqNo;

//送信中のデータ数を保持
static uint8_t u8NumRadioTx;

//ACKが得られない場合の再送回数
static uint8_t u8RadioRetryCount;

//再送間隔
static uint16_t u16RadioRetryDuration;

//自分の12bitショートアドレス。未使用時 0xFFFF
static uint16_t u16MyShortAddress;

//送信IDを強制的に指定する。未使用時-1
static int16_t i16RadioNextCbId;


//重複受信回避用の構造体
typedef struct {
    uint32_t u32SrcAddr; //0:未使用
    uint32_t u32Millis;
    uint8_t u8seq;
} RADIORECEIVEHISTORY;
#define RADIORECEIVEHISTRY_BUFSIZE 5
static RADIORECEIVEHISTORY radioReceiveHistory[RADIORECEIVEHISTRY_BUFSIZE];


//setup()関数内で使用すること
//有効な appid の範囲。0xHHHHLLLLの場合、HHHH,LLL共に0x0001～0x7FFF
//有効な channel の範囲。11～26
//txPower 送信出力。0～3 (弱～強)
bool_t radio_setupInit(RADIOMODE mode, uint32_t appid, uint8_t channel, uint8_t txPower) {
    uint16_t w = appid & 0xffff;
    if (w < 0x0001 || w > 0x7fff) return FALSE;
    w = appid >> 16;
    if (w < 0x0001 || w > 0x7fff) return FALSE;
    if (channel < 11 || channel > 26) return FALSE;
    if (txPower > 3) return FALSE;

    sToCoNet_AppContext.u8MacInitPending = (mode == RADIO_MODE_OFF);
    sToCoNet_AppContext.u32AppId = appid;
    sToCoNet_AppContext.u8Channel = channel;
    sToCoNet_AppContext.bRxOnIdle = (mode == RADIO_MODE_TXRX);
	sToCoNet_AppContext.u8TxPower = txPower;
    return TRUE;
}

//アドレスは12ビット(0x000～0xFFF)です。
void radio_setupShortAddress(uint16_t u16ShortAddress) {
	sToCoNet_AppContext.u16ShortAddress = u16ShortAddress;
    u16MyShortAddress = u16ShortAddress;
}

//ACKが得られない場合の再送信回数を設定 0..7。デフォルトは 2
//ブロードキャスト送信の場合は常に u8Retry+1 回の送信が行われる
//retryDurationは再送間隔(ミリ秒)。デフォルトは10ms
bool_t radio_setRetry(uint8_t retryCount, uint16_t retryDuration) {
    if (retryCount > 7) return FALSE;
    u8RadioRetryCount = retryCount;
    u16RadioRetryDuration = retryDuration;
    return TRUE;
}

//次の送信関数実行時の送信IDを強制的に指定します
void radio_setCbId(uint8_t u8CbId) {
    i16RadioNextCbId = u8CbId;
}

//送信中のパケット数
uint8_t radio_txCount() {
    return u8NumRadioTx;
}

//無線送信完了、受信割り込みルーチンを設定する
void radio_attachCallback(void (*txFunc)(uint8_t u8CbId, bool_t bSuccess), void (*rxFunc)(uint32_t u32SrcAddr, bool_t bBroadcast, uint8_t u8CbId, uint8_t u8DataType, uint8_t *pu8Data, uint8_t u8Length, uint8_t u8Lqi)) {
    radioTxCallbackFunction = txFunc;
    radioRxCallbackFunction = rxFunc;
}

//無線受信時の重複受信回避処理をユーザーコールバック関数に置き換える
//ユーザーコールバック関数は受信を許可したときにTRUEを返す
//ユーザーコールバック関数のポンタにNULLを渡したときはデフォルトの重複受信回避アルゴリズムが使用される
void radio_setRxGateCallback(bool_t (*gateFunc)(uint32_t u32SrcAddr, uint8_t u8CbId)) {
    radioRxDuplicateJudgementCallbackFunction = gateFunc;
}


//無線で特定の相手に送信する
//basicio_module.hでUSE_RADIOを宣言し、送信モジュールと同じAPP_ID,CHANNELに設定したモジュールかつ、関数の引数でu32DistAddrに指定したモジュールが受信できる
//u32DestAddr=相手のモジュールアドレス。事前にSerial0_printf("%u", moduleAddress)等を実行してTWELITE毎のモジュールアドレスを知っておくとよい
//pu8Data=データ, u8Length=データ長さ, u8DataType=データの簡易識別番号(0..7)
//簡易識別番号は受け取り側が何のデータか知るために使う。使用しない場合は値はなんでもよい
//関数はエラーで-1、送信開始で8bitの送信Id(u8CbId)を返す。これは送信完了コールバックで送信データの識別に使用される。
int16_t radio_write(uint32_t u32DestAddr, uint8_t u8DataType, uint8_t *pu8Data, uint8_t u8Length)
{
    uint8_t maxDataLength;
    if (u16MyShortAddress == 0xFFFF) {
        //ロングアドレスモード
        if ((u32DestAddr & 0xffff0000) == 0) {
            //⇒ショートアドレス
            maxDataLength = 98;
        } else {
            //⇒ロングアドレス
            maxDataLength = 92;
        }
    } else {
        //ショートアドレスモード
        if ((u32DestAddr & 0xffff0000) == 0) {
            //⇒ショートアドレス
            maxDataLength = 104;
        } else {
            //⇒ロングアドレス
            return -1;  //自分がショートアドレスの時、ロングアドレスには送信できない
        }
    }
    if (u8Length > maxDataLength) return -1;

    u8RadioSeqNo++;

    if (i16RadioNextCbId != -1 && i16RadioNextCbId == (u8RadioSeqNo + 1)) {
        //送信IDを強制指定しているときで、次回の送信時の自動生成送信ID(u8RadioSeqNo)と値がダブる場合、
        //次回の送信が相手先で拒否される可能性があるので、ダブらないように値をずらす
        u8RadioSeqNo++;
    }

    tsTxDataApp tsTx;
    memset(&tsTx, 0, sizeof(tsTxDataApp));

    tsTx.u32SrcAddr = (u16MyShortAddress == 0xFFFF) ? getModuleAddress() : u16MyShortAddress; //送信元アドレス
    tsTx.u32DstAddr = u32DestAddr;                  //送信先アドレス

	tsTx.u8Cmd = u8DataType;                        //データ種別 (0..7)。データの簡易識別子。
	tsTx.u8Len = u8Length; 		                    //データ長

    if (i16RadioNextCbId == -1) {
    	tsTx.u8Seq = u8RadioSeqNo; 		            //シーケンス番号(複数回送信時に、この番号を調べて重複受信を避ける)
	    tsTx.u8CbId = u8RadioSeqNo;	                //送信識別ID。送信完了イベントの引数として渡され、送信イベントとの整合を取るために使用する
        //受信側ではu8CbIdを受け取れないので、u8Seqと同じ値を用いる
    }
    else {
    	tsTx.u8CbId = (uint8_t)i16RadioNextCbId;    //送信IDを強制指定
    	tsTx.u8Seq = (uint8_t)i16RadioNextCbId;     //
        i16RadioNextCbId = -1;
    }
    memcpy(tsTx.auData, pu8Data, u8Length);

	tsTx.bAckReq = (u32DestAddr != TOCONET_MAC_ADDR_BROADCAST); //TRUE Ack付き送信を行う
	tsTx.u8Retry = u8RadioRetryCount;  		        //MACによるAck付き送信失敗時に、さらに再送する場合(ToCoNet再送)の再送回数

	//tsTx.u16ExtPan = 0;                           //0:外部PANへの送信ではない 1..0x0FFF: 外部PANへの送信 (上位4bitはリザーブ)

	//tsTx.u16DelayMax = 0;                         //送信開始までのディレー(最大)[ms]。指定しない場合は 0 にし、指定する場合は Min 以上にすること。
	//tsTx.u16DelayMin = 0;                         //送信開始までのディレー(最小)[ms]
	tsTx.u16RetryDur = u16RadioRetryDuration;       //再送間隔[ms]。

    //送信
    if (ToCoNet_bMacTxReq(&tsTx)) {
         
        //送信中データカウント
        u8NumRadioTx++;

        //送信データIdを返す
        return tsTx.u8CbId;
    } else {
        return -1;
    }
}


//radio_write()の簡易版
//関数はエラーで-1、送信開始で8bitの送信Id(u8CbId)を返す。これは送信完了コールバックで送信データの識別に使用される
//u8DataType=データの簡易識別番号(0..7)
int16_t radio_puts(uint32_t u32DestAddr, uint8_t u8DataType, const char *pu8String)
{
    uint32_t len = (uint32_t)strlen(pu8String);
    return radio_write(u32DestAddr, u8DataType, (uint8_t *)pu8String, (uint8_t)len);
}

//radio_write()のprintf版
/*bool_t radio_printf(uint32_t u32DestAddr, const char* format, va_list args) {
    SPRINTF_vRewind();
    vfPrintf(SPRINTF_Stream, format, args);
    va_end(args);
    return radio_puts(u32DestAddr, SPRINTF_pu8GetBuff());
}*/

//New radio_printf()
//出力できなかった場合は１文字も出力せずに-1を返します
int16_t radio_printf(uint32_t u32DestAddr, uint8_t u8DataType, const char* fmt, ...) {
    //パラメータを取得
    va_list ap;
    va_start(ap, fmt);

    //バッファへの書き込みパラメータを設定
    char buf[104];
    __printf_putc_init(buf, 104);

    //bufにprintf
    if (!myprintf(__printf_putc, fmt, ap)) {
        //バッファオーバーフロー
        va_end(ap);
        return -1;
    }
    va_end(ap);

    //無線送信する
    return radio_write(u32DestAddr, u8DataType, (uint8_t *)buf, __printf_putc_count);
}

#endif //USE_RADIO


/*
 * EEPROM
 */

#ifdef USE_EEPROM

static uint16_t u16NumberOfEEPROMSegments; //EEPROMセグメントの数 BLUE:63 RED:255
static uint8_t u8EEPROMSegmentSize;        //EEPROMセグメントの大きさ 64 bytes

//EEPROMセグメント数を返します。
//TWELITE BLUE: 63
//TWELITE RED: 255
uint16_t eeprom_getSegmentCount() {
    return u16NumberOfEEPROMSegments;
}

//EEPROMセグメントの大きさを返します。
//64バイト
uint8_t eeprom_getSegmentSize() {
    return u8EEPROMSegmentSize;
}

//指定したセグメントがすべて0x00のときTRUEを返す
bool_t eeprom_erased(uint16_t u16SegIndex)
{
    uint8_t buf[64];
    if (!eeprom_read(u16SegIndex, 0, buf, 64)) return FALSE;

    uint8_t i;
    for(i=0; i<64; i++) {
        if (buf[i] != 0) return FALSE;
    }
    return TRUE;
}

#endif



/*
 * FLASHメモリ
 */

#ifdef USE_FLASH

//sector=0..4(BLUE)/0..15(RED)。プログラムはセクタ0から書き込まれるので、使用していないセクタに書き込むこと
//flash_write()はビットを1から0にしか書き換えられないので、この関数により消去(ビットを1にする)した部分にしか書き込めない
bool_t flash_erase(uint8_t sector)
{
    if (sector > FLASH_LAST_SECTOR) return FALSE;
    return bAHI_FlashEraseSector(sector);
}

//sector=0..4(BLUE)/0..15(RED)。プログラムはセクタ0から書き込まれるので、使用していないセクタに書き込むこと
//書き込み単位は16の倍数で、一度に書き込める最大長は1セクタ(32KB)。セクタ境界を越えることは問題ない
//offsetはセクタ内オフセット値で、16の倍数とする
//事前にflash_erase()で領域をフォーマットしておくこと
bool_t flash_write(uint8_t sector, uint16_t offset, uint8_t *pu8Data, uint16_t u16Length)
{
    if (sector > FLASH_LAST_SECTOR) return FALSE;
    if ((offset & 15) != 0 || (u16Length & 15) != 0) return FALSE;
    if (u16Length > FLASH_SECTOR_SIZE) return FALSE;

    uint32_t addr = offset + (uint32)sector * FLASH_SECTOR_SIZE;
    return bAHI_FullFlashProgram(addr, u16Length, pu8Data);
}

//指定した範囲がすべて0xFFのときTRUEを返す
bool_t flash_erased(uint8_t sector, uint16_t offset, uint16_t u16Length)
{
    if ((u16Length & 15) != 0) return FALSE;

    uint16_t n = u16Length >> 4;
    uint32_t buf[4];
    while (n-- > 0) {
        if (offset >= 32768) {
            offset -= 32768;
            sector++;
        }
        if (!flash_read(sector, offset, (uint8_t *)&buf[0], 16)) return FALSE;
        if (buf[0] != 0xffffffff) return FALSE;
        if (buf[1] != 0xffffffff) return FALSE;
        if (buf[2] != 0xffffffff) return FALSE;
        if (buf[3] != 0xffffffff) return FALSE;
        offset += 16;
    }
    return TRUE;
}

bool_t flash_read(uint8_t sector, uint32 offset, uint8_t *pu8Data, uint16_t u16Length)
{
    if (sector > FLASH_LAST_SECTOR) return FALSE;
    if ((offset & 15) != 0 || (u16Length & 15) != 0) return FALSE;
    if ((uint32_t)offset + (uint32_t)u16Length > FLASH_SECTOR_SIZE) return FALSE;

    uint32_t addr = offset + (uint32)sector * FLASH_SECTOR_SIZE;
    return bAHI_FullFlashRead(addr, u16Length, pu8Data);
}
#endif



/*
 * 隠ぺいされた基本関数
 */


//ユーザーが準備する関数
extern void setup(bool_t warmWake, uint32_t bitmapWakeStatus);
extern void loop(EVENTS event);

// イベントハンドラ
static void vProcessEvCore(tsEvent *pEv, teEvent eEvent, uint32_t u32evarg)
{
    //ユーザーが設定した処理ルーチンを呼び出す
    switch (eEvent) {
	case E_EVENT_START_UP:
        loop(EVENT_START_UP);
        break;
    case E_EVENT_TICK_TIMER: 
#ifdef USE_PBUTIL
        pb_update();    //プッシュボタンの状態を更新
#endif
        loop(EVENT_TICK_TIMER);
        break;
    case E_EVENT_TICK_SECOND:
        loop(EVENT_TICK_SECOND);
        break;
    }
}

//変数や構造体を初期化
void resetVars()
{
#ifdef USE_PBUTIL
    u32PBDefined = 0;
    u32PBPositive = 0;
    u32PBPressed = 0;
    u32PBReleased = 0;
    memset(u8PBDelayCount, 0, sizeof(u8PBDelayCount));
#endif

#ifdef USE_DIO
    memset(dioCallbackFunctions, 0, sizeof(dioCallbackFunctions));
    memset(dioCallbackFuncIndices, 0xff, sizeof(dioCallbackFuncIndices));
#endif

#ifdef USE_TIMER
    memset(timerCallbackFunctions, 0, sizeof(timerCallbackFunctions));
    memset(sTimerApp, 0, sizeof(sTimerApp));
    timerFineGrainDIOControlValue = 0xFF;
#endif

#ifdef USE_ADC
    adcCallbackFunction = NULL;
#endif

#ifdef USE_COMP
    compCallbackFunction = NULL;
#endif

#ifdef USE_PC
    memset(pcCallbackFunctions, 0, sizeof(pcCallbackFunctions));
    pcCountStatus = 0;
#endif

#if defined(USE_I2C) || defined(USE_I2CS)
    i2cI2csInitFlag = 0;
#endif

#ifdef USE_I2C
    i2cAddressingMode = I2C_ADDRESS_7BIT;
#endif

#ifdef USE_I2CS
    i2csu8LastInt = 1;
    i2csbNewCmd = FALSE;
    i2csu8MWIndex[0] = 0;
    i2csu8MWIndex[1] = 0;
    i2csu8MRBufferSize = 0;
    i2csMRCallbackFunction = NULL;
    i2csi16CmdForMRCallback = -1;
    i2csMWCallbackFunction = NULL;
    i2csi16BufSelForMWCallback = -1;
#endif

#ifdef USE_RADIO
    radioTxCallbackFunction = NULL;
    radioRxCallbackFunction = NULL;
    u8RadioSeqNo = 0;
    u8NumRadioTx = 0;
    u8RadioRetryCount = 2;
    u16RadioRetryDuration = 10;
    u16MyShortAddress = 0xFFFF;
    i16RadioNextCbId = -1;
    memset(radioReceiveHistory, 0, sizeof(radioReceiveHistory));
    radioRxDuplicateJudgementCallbackFunction = NULL;
#endif

    millisValue = 0;

#ifdef USE_SBUTIL
    __sb_ptr = __sb_buf;
    __sb_count = 0;
/*
#if SB_BUFFER_SIZE == 32
        SPRINTF_vInit32();
#elif SB_BUFFER_SIZE == 64
        SPRINTF_vInit64();
#elif SB_BUFFER_SIZE == 128
        SPRINTF_vInit128();
#elif SB_BUFFER_SIZE == 256
        SPRINTF_vInit256();
#elif SB_BUFFER_SIZE == 512
        SPRINTF_vInit512();
#elif SB_BUFFER_SIZE == 1024
        SPRINTF_vInit1024();
#else
#error Invalid SB_BUFFER_SIZE value.
#endif
*/
#endif
}

void initAppContext()
{
#ifdef USE_RADIO
    //radio_setupInit() が呼ばれない場合はペンディングモードで起動
	sToCoNet_AppContext.u8MacInitPending = TRUE; //!< TRUE:MAC 層の初期化をシステム始動時に行わない。無線部を使用せずに動作させる場合に設定します。
    //以下、適当
	sToCoNet_AppContext.u32AppId = 0x44444444;  //!< 32bitのアプリケーションID。本IDでToCoNet同士の識別を行う。
	sToCoNet_AppContext.u8Channel = 15;         //!< モジュールのチャネル。NWK層の動作により変更される場合がある。
	sToCoNet_AppContext.bRxOnIdle = FALSE;      //!< TRUE:無線回路アイドル時も受信回路をオープンする。受信が必要な場合は必ずTRUEに設定する。
	sToCoNet_AppContext.u8TxPower = 3; 	        //!< モジュールの出力 3:最大 2: -11.5db 2: -23db 0:-34.5db となる (規定値は 3)
	//uint32 u32ChMask; 			            //!< 利用するチャネル群。NWK層やNeibourScanで利用する。(必須設定項目)
	//uint16 u16ShortAddress; 	                //!< モジュールのショートアドレス。指定しなければモジュールのシリアル番号から自動生成される。0xFFFFは指定出来ない。Nwk層利用時は指定しないこと。
#endif

#ifdef CPU_CLOCK
	sToCoNet_AppContext.u8CPUClk = CPU_CLOCK; 	//!< 通常稼働時のCPUクロック。3:32MHz, 2:16Mhz, 1:8Mhz, 0:4Mhz を指定する(規定値は 2)
#endif

	//uint8_t u8TxMacRetry; 		//!< MAC層の再送回数 7..0 を指定する。(規定値は3, Nwk層では１を推奨)
	//bool_t bPromiscuousMode; 	//!< テスト受信モード。通常は設定してはいけません (規定値は FALSE)
	//bool_t bSkipBootCalib;		//!< 始動時のキャリブレーション処理を省略する
	//uint8 u8Osc32Kmode; 		//!< 32K 水晶のモード (0x00: RC, 0x02: 32K水晶, 0x03: 32K発振器)
	//uint8 u8CCA_Retry; 			//!< CCA のリトライ回数 (通常は変更しない)
	//uint8 u8CCA_Level; 			//!< CCA アルゴリズムの開始レベル (通常は変更しない)

	//bool_t bNoAckMode;			//!< Ack を一切返さない。起動時の設定のみ反映され、起動後は変更できない。(通常は変更しない)
	//bool_t bRxExtPan;			//!< 他のPANからのパケットを受信する (1.0.8)

	sToCoNet_AppContext.u8RandMode = 3; //!< 乱数生成方法の指定。0:ハード 1:システム経過時間を元に生成 2:MT法 3:XorShift法 (32kOscモードで外部水晶が利用されたときは 0 の場合 XorShift 方を採用する)
                                //※ハードウェア乱数では同一ループ内では同じ値しか生成できない。MT法は高品質だがライセンス表記必要。    
	sToCoNet_AppContext.u16TickHz = TICK_COUNT; //!< システムの Tick カウントの周期(規定値は250=4ms, 1000で割り切れる値にすること。事実上 1000, 500, 250, 200, 100 のみ)
    millisValueTick = 1000 / TICK_COUNT;
}

//ToCoNet_REG_MOD_ALL()を展開
//詳細がわからないので動作確認＆予想で削っている
void regMod() {

    //チャネルの入力レベルを測定します
    //ToCoNet_REG_MOD_ENERGYSCAN();

    //近隣のモジュールを探索します
    //ToCoNet_REG_MOD_NBSCAN();
    //ToCoNet_REG_MOD_NBSCAN_SLAVE();

    //乱数生成アルゴリズムを登録。登録しない場合はハードウェア乱数を使用
    //ToCoNet_REG_MOD_MTRAND(); //MT法を使用する場合はライセンス表記が必要
    //ToCoNet_vReg_mod_Rand_Xor_Shift();
    ToCoNet_REG_MOD_RAND_XOR_SHIFT();

#ifdef USE_RADIO
    //送受信キューを確保する(はず)
    ToCoNet_REG_MOD_TXRXQUEUE();

    //レイヤーツリー型ネットワーク層を利用します
    //ToCoNet_REG_MOD_NWK_LAYERTREE();

    //チャネルアジリティを利用します
    //ToCoNet_REG_MOD_CHANNEL_MGR();

    //メッセージプール機能を利用します
    //ToCoNet_REG_MOD_NWK_MESSAGE_POOL();

    //送信モジュール(?)
    ToCoNet_REG_MOD_TX();

    //パケットの重複チェッカ
    //ToCoNet_REG_MOD_DUPCHK();
#endif
}

uint32_t getWakeStatus() {
    uint16_t ps = u16AHI_PowerStatus();
    uint32_t w;

    //wake from deepp sleep
    if (ps & (1 << 11)) {
        return 0;
    }

    w = u8AHI_WakeTimerFiredStatus();

    //Timer0(sleep)によるウェイク
    //タイマーが走ってことも確認しないとオシレーターOFFスリープからの起床でうまく判別できない
    if ((u8AHI_WakeTimerStatus() & E_AHI_WAKE_TIMER_MASK_0) && (w & E_AHI_WAKE_TIMER_MASK_0)) {
        return E_AHI_SYSCTRL_WK0_MASK;
    }

    //Timer1によるウェイク（未確認）
    if ((u8AHI_WakeTimerStatus() & E_AHI_WAKE_TIMER_MASK_1) && (w & E_AHI_WAKE_TIMER_MASK_1)) {
        return E_AHI_SYSCTRL_WK1_MASK;
    }

    //コンパレータによるウェイク
    if (u8AHI_ComparatorWakeStatus() & E_AHI_AP_COMPARATOR_MASK_1) {
        return E_AHI_SYSCTRL_COMP0_MASK;
    }

    w = u32AHI_PulseCounterStatus();

    //パルスカウンタ0または32によるウェイク
    if (w & E_AHI_SYSCTRL_PC0_MASK) {
        return E_AHI_SYSCTRL_PC0_MASK;
    }

    //パルスカウンタ1によるウェイク
    if (w & E_AHI_SYSCTRL_PC1_MASK) {
        return E_AHI_SYSCTRL_PC1_MASK;
    }

    //DIOによるウェイク
    return u32AHI_DioWakeStatus() & 0xfffff;
}


// 電源オンによるスタート
void cbAppColdStart(bool_t bAfterAhiInit)
{
    static uint32_t bitmapWakeStatus;

	if (!bAfterAhiInit) {

        //起動原因を取得, 電源ONとDEEP SLEEPからの起床は0になる
        bitmapWakeStatus = getWakeStatus();

        //モジュールを登録
        regMod();
	} else {
        initAppContext();

        //変数や構造体を初期化
        resetVars();


        //イベントハンドラを登録
        ToCoNet_Event_Register_State_Machine(vProcessEvCore);

#ifdef USE_FLASH
        //デフォルトで内蔵フラッシュメモリを選択
        bAHI_FlashInit(E_FL_CHIP_INTERNAL, NULL);
#endif

#ifdef USE_EEPROM
        //EEPROM情報を取得
        u16NumberOfEEPROMSegments = u16AHI_InitialiseEEP(&u8EEPROMSegmentSize);
#endif

        //ユーザーの初期化ルーチンを呼び出す
        setup(FALSE, bitmapWakeStatus);

#ifdef USE_RADIO
        // MAC 層開始
        ToCoNet_vMacStart();
#endif
	}
}

// スリープからの復帰
void cbAppWarmStart(bool_t bAfterAhiInit)
{
    static uint32_t bitmapWakeStatus;

	if (!bAfterAhiInit) {

        //起動原因を取得
        bitmapWakeStatus = getWakeStatus();

        //モジュールを登録
        regMod();
	} else {

        //変数や構造体を初期化
        resetVars();

        //イベントハンドラを登録
        ToCoNet_Event_Register_State_Machine(vProcessEvCore);

#ifdef USE_FLASH
        //デフォルトで内蔵フラッシュメモリを選択
        bAHI_FlashInit(E_FL_CHIP_INTERNAL, NULL);
#endif

#ifdef USE_EEPROM
        //EEPROM情報を取得
        u16NumberOfEEPROMSegments = u16AHI_InitialiseEEP(&u8EEPROMSegmentSize);
#endif

        //ユーザーの初期化ルーチンを呼び出す
        setup(TRUE, bitmapWakeStatus);

#ifdef USE_RADIO
        // MAC 層開始
        ToCoNet_vMacStart();
#endif
	}
}

// ネットワークイベント発生時
void cbToCoNet_vNwkEvent(teEvent eEvent, uint32_t u32arg)
{
}

// パケット受信時
void cbToCoNet_vRxEvent(tsRxDataApp *pRx)
{
#ifdef USE_RADIO
#define RADIO_CBID_LIFESPAN 200 //[ms] 最大リトライを考えると200くらい必要

    if (radioRxDuplicateJudgementCallbackFunction != NULL) {
        //ユーザーによる重複受信回避処理
        if (!(*radioRxDuplicateJudgementCallbackFunction)(pRx->u32SrcAddr, pRx->u8Seq)) return;
    }
    else {
        //デフォルトの重複受信回避処理

        //100ms以内に受信した送信元アドレスと送信IDを5つまで保持し
        //重複受信を回避している

        uint32_t oldestTimePassed = 0;  //RADIO_ID_TIME_LIMIT以内で最も古い履歴の経過時間
        int8_t oldestIndex = -1;        //RADIO_ID_TIME_LIMIT以内で最も古い履歴のインデックス
        int8_t emptyIndex = -1;         //空データのインデックス
        uint8_t i;
        for (i=0; i<RADIORECEIVEHISTRY_BUFSIZE; i++) {
            if (radioReceiveHistory[i].u32SrcAddr != 0) {
                uint32_t timePassed = millis() - radioReceiveHistory[i].u32Millis;
                if (timePassed > RADIO_CBID_LIFESPAN) {
                    //一定時間経過した履歴は無効、削除する
                    radioReceiveHistory[i].u32SrcAddr = 0;
                    emptyIndex = i;
                }
                else if (radioReceiveHistory[i].u32SrcAddr == pRx->u32SrcAddr &&
                    radioReceiveHistory[i].u8seq == pRx->u8Seq) {
                    //このパケットは受信済み

                    //受信時刻を更新する
                    radioReceiveHistory[i].u32Millis = millis();
                    return;
                }
                else if (timePassed > oldestTimePassed) {
                    //有効な履歴

                    //そのなかでも古い履歴なので記憶
                    oldestTimePassed = timePassed;
                    oldestIndex = i;
                }
            } else {
                //配列は使用されていない
                emptyIndex = i;
            }
        }

        //空があればそこに履歴を書き込むが、そうでない場合は
        //最も古い履歴に上書き更新する。
        i = (emptyIndex >= 0) ? emptyIndex : oldestIndex;
        radioReceiveHistory[i].u32SrcAddr = pRx->u32SrcAddr;
        radioReceiveHistory[i].u32Millis = millis();
        radioReceiveHistory[i].u8seq = pRx->u8Seq;
    }

    if (radioRxCallbackFunction != NULL) {
        //受信ルーチンを呼び出す
        (*radioRxCallbackFunction)(pRx->u32SrcAddr, pRx->u32DstAddr == RADIO_ADDR_BROADCAST, pRx->u8Seq, pRx->u8Cmd, pRx->auData, pRx->u8Len, pRx->u8Lqi);
    }
#endif
}

// パケット送信完了時
void cbToCoNet_vTxEvent(uint8_t u8CbId, uint8_t bStatus)
{
#ifdef USE_RADIO
    //送信中データカウント
    u8NumRadioTx--;

    if (radioTxCallbackFunction != NULL) {
        //送信完了ルーチンを呼び出す
        (*radioTxCallbackFunction)(u8CbId, (bool_t)bStatus);
    }
#endif
}

// ハードウェア割り込み発生後（遅延呼び出し）
void cbToCoNet_vHwEvent(uint32_t u32DeviceId, uint32_t u32ItemBitmap)
{
    //割り込みに対する処理は通常ここで行う。
    switch (u32DeviceId) {

#ifdef USE_I2S
    case E_AHI_DEVICE_SI:   //I2Cマスタースレーブ共通
        if (i2cI2csInitFlag == 2) {
            //I2C スレーブ
            if (i2csi16CmdForMRCallback != -1 && i2csMRCallbackFunction != NULL) {
                //マスターが読み出すかもしれないデータを準備
                (*i2csMRCallbackFunction)(i2csi16CmdForMRCallback);
                i2csi16CmdForMRCallback = -1;
            }
            if (i2csi16BufSelForMWCallback != -1 && i2csMWCallbackFunction != NULL) {
                //マスターが書き込んだデータを処理
                (*i2csMWCallbackFunction)(i2csau8MWBuffer[i2csi16BufSelForMWCallback], i2csu8MWIndex[i2csi16BufSelForMWCallback]);
                i2csi16BufSelForMWCallback = -1;
            }
        }
        break;
#endif

#ifdef USE_TIMER
    case E_AHI_DEVICE_TIMER0:
        //タイマー0割り込み処理ルーチンの呼び出し
        if ((sTimerApp[0].u8Mode == 1 || sTimerApp[0].u8Mode == 6) && timerCallbackFunctions[0] != NULL) {
            (*timerCallbackFunctions[0])();
        }
        break;

    case E_AHI_DEVICE_TIMER1:
        //タイマー1割り込み処理ルーチンの呼び出し
        if (sTimerApp[1].u8Mode == 1 && timerCallbackFunctions[1] != NULL) {
            (*timerCallbackFunctions[1])();
        }
        break;

    case E_AHI_DEVICE_TIMER2:
        //タイマー2割り込み処理ルーチンの呼び出し
        if (sTimerApp[2].u8Mode == 1 && timerCallbackFunctions[2] != NULL) {
            (*timerCallbackFunctions[2])();
        }
        break;

    case E_AHI_DEVICE_TIMER3:
        //タイマー3割り込み処理ルーチンの呼び出し
        if (sTimerApp[3].u8Mode == 1 && timerCallbackFunctions[3] != NULL) {
            (*timerCallbackFunctions[3])();
        }
        break;

    case E_AHI_DEVICE_TIMER4:
        //タイマー4割り込み処理ルーチンの呼び出し
        if (sTimerApp[4].u8Mode == 1 && timerCallbackFunctions[4] != NULL) {
            (*timerCallbackFunctions[4])();
        }
        break;
#endif

    case E_AHI_DEVICE_SYSCTRL:
        _C {
#ifdef USE_DIO
            if (u32ItemBitmap & 0xfffff) {
                //DIO割り込み
                uint32_t bitmap = u32ItemBitmap;
                uint8_t pinNo;
                for(pinNo = 0; pinNo < 20; pinNo++) {
                    if (bitmap & 1) { //DIO番号に対応するビットが1になっている
                        uint8_t i = dioCallbackFuncIndices[pinNo];
                        if (i < MAX_DIO_INTERRUPT_FUNCS && dioCallbackFunctions[i] != NULL)
                            (*dioCallbackFunctions[i])(u32ItemBitmap);
                    }
                    bitmap >>= 1;
                    if (bitmap == 0) break;
                }
            }
#endif
#ifdef USE_COMP
            if ((u32ItemBitmap & E_AHI_SYSCTRL_COMP0_MASK) != 0 && compCallbackFunction != NULL) {
                //コンパレータ割り込み
                (*compCallbackFunction)();
            }
#endif
#ifdef USE_PC
            if ((u32ItemBitmap & E_AHI_SYSCTRL_PC0_MASK) != 0 && pcCallbackFunctions[0] != NULL) {
                //パルスカウンタ0割り込み
                (*pcCallbackFunctions[0])();
            }
            if ((u32ItemBitmap & E_AHI_SYSCTRL_PC1_MASK) != 0 && pcCallbackFunctions[1] != NULL) {
                //パルスカウンタ1割り込み
                (*pcCallbackFunctions[1])();
            }
#endif


/* E_AHI_DEVICE_SYSCTRLにおけるbitmap値
E_AHI_SYSCTRL_CKEM_MASK (31)    System clock source has been changed
E_AHI_SYSCTRL_RNDEM_MASK (30)   A new value has been generated by the Random Number Generator
E_AHI_SYSCTRL_COMP1_MASK (29)
E_AHI_SYSCTRL_COMP0_MASK (28)   Comparator (0 and 1) events
E_AHI_SYSCTRL_WK1_MASK (27)
E_AHI_SYSCTRL_WK0_MASK (26)     Wake Timer events
E_AHI_SYSCTRL_VREM_MASK (25)    Brownout condition entered
E_AHI_SYSCTRL_VFEM_MASK (24)    Brownout condition exited
E_AHI_SYSCTRL_PC1_MASK (23)
E_AHI_SYSCTRL_PC0_MASK (22)     Pulse Counter (0 or 1) has reached its pre-configured reference value
E_AHI_DIO20_INT (20)
E_AHI_DIO19_INT (19)
E_AHI_DIO18_INT (18)
E_AHI_DIO17_INT (17)
.
.
.
E_AHI_DIO0_INT (0)              Digital IO (DIO) events
*/

        }
        break;

    case E_AHI_DEVICE_ANALOGUE:
#ifdef USE_ADC
        //ADC(完了)割り込みルーチンの呼び出し

        switch(adcLastSource) {
            case 0xff:
                //adc_attachCallbackWithTimer()割り込み
                _C {
                    if (!adcIsContinuous && --adcIntCountTillEnd == 0) {
                        vAHI_AdcDisableSampleBuffer();//完了時にこれをやっとかないと2回目が実行できない
                        adcLastSource = 0;  //0xffのままだとadc_detach()で余計な処理が入るのでクリア
#ifdef USE_TIMER
                        //タイマー停止
                        uint8_t i;
                        for(i=0; i<=4; i++) {
                            if (sTimerApp[i].u8Mode == 7) { //ADC sampling(DMA)
                                timer_detach(i);
                                break;
                            }
                        }
#endif
                    }
                    (*((void (*)())adcCallbackFunction))();//パラメータ無し
                }
                break;
            case ADC_SOURCE_VOLT:
                (*adcCallbackFunction)((int16_t)adcConvertVolt(u16AHI_AdcRead()));
                //(*adcCallbackFunction)(u16AHI_AdcRead());
                break;
            case ADC_SOURCE_TEMP:
                (*adcCallbackFunction)((int16_t)adcConvertTemp(u16AHI_AdcRead()));
                break;
            default:
                (*adcCallbackFunction)((int16_t)adcConvertADCx(u16AHI_AdcRead()));
                break;
        }
#endif
    case E_AHI_DEVICE_TICK_TIMER:
        break;
    }
}

// ハードウェア割り込み発生時
uint8_t cbToCoNet_u8HwInt(uint32_t u32DeviceId, uint32_t u32ItemBitmap)
{
    //割り込みで最初に呼ばれる。最短で返さないといけない。

    //注)ここで目的の割り込み処理を実行したときだけTRUEを返すことで
    //　cbToCoNet_vHwEvent()の呼び出しを無効化できる

    switch (u32DeviceId) {

#ifdef USE_TIMER
    case E_AHI_DEVICE_TIMER0:
        if (sTimerApp[0].u8Mode == 4) {         //Micro counter    
            sTimerApp[0].u16HiMicroSeconds++;
            return TRUE;
        }
        else if (sTimerApp[0].u8Mode == 5) {    //Capture
            if (bTimer0FirstCap) {
                //最初の1つはダミー読み込み
                vAHI_TimerReadCaptureFreeRunning(0, pu16Timer0CapBuf, pu16Timer0CapBuf + 1);
                bTimer0FirstCap = FALSE;
            } else if (u16Timer0CapCount < u16Timer0CapBufSize) {
                if (++u16Timer0CapCount < u16Timer0CapBufSize) {
                    vAHI_TimerReadCaptureFreeRunning(0, pu16Timer0CapBuf, pu16Timer0CapBuf + 1);
                    pu16Timer0CapBuf += 2;
                } else {
                    vAHI_TimerReadCapture(0, pu16Timer0CapBuf, pu16Timer0CapBuf + 1);

                    //パルスは外部入力なのでtimer_stop()ではダメ。元から止める
                    vAHI_TimerDisable(0);
                }
            }
            return TRUE;
        }
        else if (sTimerApp[0].u8Mode == 8 && sTimerApp[0].u16HiCount != sTimerApp[0].u16ReservedHiCount) {
            //PWM0パルスカウンタの同期変更
            sTimerApp[0].u16HiCount = sTimerApp[0].u16ReservedHiCount;
            vAHI_TimerStartRepeat(0, sTimerApp[0].u16HiCount, sTimerApp[0].u16LoCount);
            return TRUE;
        }
        break;

    case E_AHI_DEVICE_TIMER1:
        if (sTimerApp[1].u8Mode == 4) {         //Micro counter    
            sTimerApp[1].u16HiMicroSeconds++;
            return TRUE;
        } else if (sTimerApp[1].u8Mode == 8 && sTimerApp[1].u16HiCount != sTimerApp[1].u16ReservedHiCount) {
            //PWM1パルスカウンタの同期変更
            sTimerApp[1].u16HiCount = sTimerApp[1].u16ReservedHiCount;
            vAHI_TimerStartRepeat(1, sTimerApp[1].u16HiCount, sTimerApp[1].u16LoCount);
            return TRUE;
        }
        break;

    case E_AHI_DEVICE_TIMER2:
        if (sTimerApp[2].u8Mode == 4) {         //Micro counter    
            sTimerApp[2].u16HiMicroSeconds++;
            return TRUE;
        } else if (sTimerApp[2].u8Mode == 8 && sTimerApp[2].u16HiCount != sTimerApp[2].u16ReservedHiCount) {
            //PWM2パルスカウンタの同期変更
            sTimerApp[2].u16HiCount = sTimerApp[2].u16ReservedHiCount;
            vAHI_TimerStartRepeat(2, sTimerApp[2].u16HiCount, sTimerApp[2].u16LoCount);
            return TRUE;
        }
        break;

    case E_AHI_DEVICE_TIMER3:
        if (sTimerApp[3].u8Mode == 4) {         //Micro counter    
            sTimerApp[3].u16HiMicroSeconds++;
            return TRUE;
        } else if (sTimerApp[3].u8Mode == 8 && sTimerApp[3].u16HiCount != sTimerApp[3].u16ReservedHiCount) {
            //PWM3パルスカウンタの同期変更
            sTimerApp[3].u16HiCount = sTimerApp[3].u16ReservedHiCount;
            vAHI_TimerStartRepeat(3, sTimerApp[3].u16HiCount, sTimerApp[3].u16LoCount);
            return TRUE;
        }
        break;

    case E_AHI_DEVICE_TIMER4:
        if (sTimerApp[4].u8Mode == 4) {         //Micro counter    
            sTimerApp[4].u16HiMicroSeconds++;
            return TRUE;
        } else if (sTimerApp[4].u8Mode == 8 && sTimerApp[4].u16HiCount != sTimerApp[4].u16ReservedHiCount) {
            //PWM4パルスカウンタの同期変更
            sTimerApp[4].u16HiCount = sTimerApp[4].u16ReservedHiCount;
            vAHI_TimerStartRepeat(4, sTimerApp[4].u16HiCount, sTimerApp[4].u16LoCount);
            return TRUE;
        }
        break;
#endif

#ifdef USE_I2CS
    case E_AHI_DEVICE_SI: //I2Cマスタースレーブ共通
        if (i2cI2csInitFlag == 2 && u32ItemBitmap & 0x1d) {
            //I2C スレーブ割り込み

            //書き込みパターンの割り込み 4,4,...4,8
            //読み込みパターンの割り込み 4,8,1,...1

            bool_t bDone = TRUE;

            if (u32ItemBitmap & 1) {//E_AHI_SIS_DATA_RR_MASK
                if (i2csu8LastInt == 8) {
                    //マスターの読み出し開始
                    i2csu8MRIndex = 0;
                    i2csbDataNotReady = (i2csu8MRBufferSize == 0);
                }
                if (i2csbDataNotReady) {
                    //準備していない
                    vAHI_SiSlaveWriteData8(I2CS_MR_DATA_NOT_READY);
                } else if (i2csu8MRIndex < i2csu8MRBufferSize) {
                    //データを渡してポインタを進める
                    vAHI_SiSlaveWriteData8(*(i2cspau8MRBuffer + i2csu8MRIndex));
                    i2csu8MRIndex++;
                } else {
                    //範囲外
                    vAHI_SiSlaveWriteData8(I2CS_MR_OUT_OF_RANGE);
                }
                i2csu8LastInt = 1;
            }
            else if (u32ItemBitmap & 4) {//E_AHI_SIS_DATA_WA_MASK
                uint8_t c = u8AHI_SiSlaveReadData8();
                if (i2csu8LastInt == 1 || i2csu8LastInt == 8 || i2csu8LastInt == 16) {
                    i2csu8cmd = c;
                    i2csbNewCmd = TRUE;

                    //遅延割り込みでデータ準備コールバックを呼びたいので
                    //コマンド番号をセット
                    i2csi16CmdForMRCallback = i2csu8cmd;
                    bDone = FALSE;

                } else if (i2csu8LastInt == 4) {
                    if (i2csbNewCmd) {
                        i2csbNewCmd = FALSE;
                        //データ保存開始
                        i2csu8MWBufferSelect = (i2csu8MWBufferSelect + 1) & 1; //次のバッファ
                        i2csu8MWIndex[i2csu8MWBufferSelect] = 0;
                        if (i2csu8MWIndex[i2csu8MWBufferSelect] < I2CS_MW_BUFFER_SIZE) i2csau8MWBuffer[i2csu8MWBufferSelect][i2csu8MWIndex[i2csu8MWBufferSelect]++] = i2csu8cmd;
                    }
                    //データ保存
                    if (i2csu8MWIndex[i2csu8MWBufferSelect] < I2CS_MW_BUFFER_SIZE) i2csau8MWBuffer[i2csu8MWBufferSelect][i2csu8MWIndex[i2csu8MWBufferSelect]++] = c;
                }
                i2csu8LastInt = 4;
                i2csbDataNotReady = FALSE;
            }
            else if (u32ItemBitmap & 8) {//E_AHI_SIS_LAST_DATA_MASK
                if (i2csu8LastInt == 4 && i2csu8MWIndex[i2csu8MWBufferSelect] > 0) {
                    //遅延割り込みで書き込まれたデータをユーザーに渡したいので、
                    //ダブルバッファインデックスをセット
                    i2csi16BufSelForMWCallback = i2csu8MWBufferSelect;
                    bDone = FALSE;
                }
                i2csu8LastInt = 8;
            }
            else if (u32ItemBitmap & 16) {//E_AHI_SIS_ERROR_MASK
                i2csu8LastInt = 16;
            }
            return bDone;
        }
        break;
#endif

    case E_AHI_DEVICE_TICK_TIMER:               //For millis()
        //u32AHI_TickTimerRead()は1秒毎にリセットされてるみたいなので、ここで独自にカウント
        millisValue += millisValueTick;

        //E_AHI_DEVICE_TICK_TIMERは例外的にTRUEを返してはいけない (固まる)
        break;

    case E_AHI_DEVICE_SYSCTRL:
#ifdef USE_PC
        if ((u32ItemBitmap & E_AHI_SYSCTRL_PC0_MASK) != 0) {
            //パルスカウンタ0割り込み
            pc_clear(0);                        //パルスカウンタは延々とカウントアップするのでクリアする
            pcCountStatus |= PC_INT_BITMAP_0;   //フラグを立てる(割り込み有効にするとu32AHI_PulseCounterStatus()でフラグが立たなくなるため)

            if (pcCallbackFunctions[0] == NULL) return TRUE;    //遅延割り込みで呼び出すコールバックが無いので、ここで終了
        }
        if ((u32ItemBitmap & E_AHI_SYSCTRL_PC1_MASK) != 0) {
            //パルスカウンタ1割り込み
            pc_clear(1);                        //パルスカウンタは延々とカウントアップするのでクリアする
            pcCountStatus |= PC_INT_BITMAP_1;   //フラグを立てる(割り込み有効にするとu32AHI_PulseCounterStatus()でフラグが立たなくなるため)

            if (pcCallbackFunctions[1] == NULL) return TRUE;    //遅延割り込みで呼び出すコールバックが無いので、ここで終了
        }

        //遅延割り込みでコールバック関数を呼び出すのでTRUEを返しません
#endif
        break;
    }

	return FALSE;//FALSEによりcbToCoNet_vHwEvent()が呼ばれる
}

// メイン
void cbToCoNet_vMain(void)
{
}




//下記をベースにTWELITE使用に改造しました。感謝！
//http://blog.livedoor.jp/hiroumauma/archives/1676244.html

#define _isnumc(x) ( (x) >= '0' && (x) <= '9' )
#define _ctoi(x)   ( (x) -  '0' )


#define    ZERO_PADDING         (1<<1)  //0埋め
#define    ALTERNATIVE          (1<<2)  //形式を表示。16進数で 0x など
#define    THOUSAND_GROUP       (1<<3)  //3桁毎に ,
#define    CAPITAL_LETTER       (1<<4)  //16進数表記などで大文字
#define    WITH_SIGN_CHAR       (1<<5)  //符号付
#define    LEFT_JUSTIFIED       (1<<6)  //指定桁数が表示桁数より大きいときに左詰め

static bool_t put_integerD(bool_t (*__putc)(char), uint64_t n, int32_t length, int8_t sign, uint8_t flags);
static bool_t put_integerX(bool_t (*__putc)(char), uint64_t n, int32_t length, uint8_t flags);
static bool_t put_integerO(bool_t (*__putc)(char), uint64_t n, int32_t length, uint8_t flags);
static bool_t put_integerB(bool_t (*__putc)(char), uint64_t n, int32_t length, uint8_t flags);

//__putc()がFALSEを返したときに__printf()もFALSEを返す
//末尾の'\0'は書き込まない
bool_t myprintf(bool_t (*__putc)(char), const char *fmt, va_list ap) {
    //va_list ap;
    //va_start(ap, fmt);

    for (;;) {
        uint64_t ui;
        int64_t i;
        uint8_t *s = NULL;
        uint8_t sign = 0;
        uint8_t flags = 0;
        int32_t length = 0;
        int32_t tmp = 0;
        uint8_t int_type = 32;

        // % まで進める
        while (*fmt && *fmt != '%') {
            if (!__putc(*fmt++)) return FALSE;
        }

        if (*fmt == '\0') {
            //va_end(ap);
            break;
        }

        //フラグを取得
        fmt++;
        while (strchr("'-+ #0", *fmt)) {
            switch (*fmt++) {
                case '\'': flags |= THOUSAND_GROUP;             break;
                case  '-': flags |= LEFT_JUSTIFIED;             break;
                case  '+': flags |= WITH_SIGN_CHAR; sign = '+'; break;
                case  '#': flags |= ALTERNATIVE;                break;
                case  '0': flags |= ZERO_PADDING;               break;
                case  ' ': flags |= WITH_SIGN_CHAR; sign = ' '; break;
            }
        }

        //表示幅
        if (*fmt == '*') {
            //変数で指定できる
            length = va_arg(ap, int32_t);
            fmt++;
        }
        else {
            while (_isnumc(*fmt)) {
                length = (length * 10) + _ctoi(*fmt++);
            }
        }

        //浮動小数点は扱わない
        /*if (*fmt == '.') {
            fmt++;
            if (*fmt == '*') {
                fmt++;
                precision = va_arg(ap, int);
            }
            else {
                while (_isnumc(*fmt) ) {
                    precision = precision * 10 + _ctoi(*fmt++);
                }
            }
        }*/

        //型指定は ll (64bit)のみに対応
        while (strchr("hljzt", *fmt)) {
            if (*fmt == 'h') {
                fmt++;
                if (*fmt == 'h') {
                    int_type = 8;  //hh
                    fmt++;
                } else {
                    int_type = 16;
                }
            } else if (*fmt == 'l') {
                fmt++;
                if (*fmt == 'l') {
                    int_type = 64;  //ll
                    fmt++;
                } else {
                    int_type = 32;
                }
            } else {
                int_type = 32;
                fmt++;
            }
        }

        switch (*fmt) {
            case 'd':
            case 'i':
                if (int_type == 64) {
                    i = va_arg(ap, int64_t);
                } else {
                    i = va_arg(ap, int32_t);
                    if (int_type == 16) {
                        i &= 0xffff;
                    } else if (int_type == 8) {
                        i &= 0xff;
                    }
                }
                if (i < 0) {
                    i = -i;
                    sign = '-';
                }
                if (!put_integerD(__putc, i, length, sign, flags)) return FALSE;
                break;

            case 'u':
                if (int_type == 64) {
                    ui = va_arg(ap, uint64_t);
                } else {
                    ui = va_arg(ap, uint32_t);
                    if (int_type == 16) {
                        ui &= 0xffff;
                    } else if (int_type == 8) {
                        ui &= 0xff;
                    }
                }
                if (!put_integerD(__putc, ui, length, sign, flags)) return FALSE;
                break;

            case 'o':
                if (int_type == 64) {
                    ui = va_arg(ap, uint64_t);
                } else {
                    ui = va_arg(ap, uint32_t);
                    if (int_type == 16) {
                        ui &= 0xffff;
                    } else if (int_type == 8) {
                        ui &= 0xff;
                    }
                }
                if (!put_integerO(__putc, ui, length, flags)) return FALSE;
                break;

            case 'b':
                if (int_type == 64) {
                    ui = va_arg(ap, uint64_t);
                } else {
                    ui = va_arg(ap, uint32_t);
                    if (int_type == 16) {
                        ui &= 0xffff;
                    } else if (int_type == 8) {
                        ui &= 0xff;
                    }
                }
                if (!put_integerB(__putc, ui, length, flags)) return FALSE;
                break;

            case 'p':
                length = 8;
                int_type = 32;
                sign = 0;
                flags = ZERO_PADDING | ALTERNATIVE;
            case 'X':
                flags |= CAPITAL_LETTER;
            case 'x':
                if (int_type == 64) {
                    ui = va_arg(ap, uint64_t);
                } else {
                    ui = va_arg(ap, uint32_t);
                    if (int_type == 16) {
                        ui &= 0xffff;
                    } else if (int_type == 8) {
                        ui &= 0xff;
                    }
                }
                if (!put_integerX(__putc, ui, length, flags)) return FALSE;
                break;

            case 'c':
                //i = get_signed(ap, 8);
                //__putc(i);
                __putc((uint8_t)(va_arg(ap, int32_t) & 0xff));
                break;

            case 's':
                s = va_arg(ap, uint8_t *);
                if (s == NULL) s = (uint8_t *)"(null)";
                tmp = strlen((const char *)s);
                //if (precision && precision < tmp)  tmp = precision;
                length -= tmp;
                if (!(flags & LEFT_JUSTIFIED)) {
                    while (length > 0) {
                        length--;
                        __putc(' ');
                    }
                }
                while (tmp--)  {
                    __putc(*s++);
                }
                while (length-- > 0) {
                    __putc(' ');
                }
                break;

            case '%':
                __putc('%');
                break;

            default:
                //既定のフォーマットでなかったのでポインタを戻す
                while (*fmt != '%') fmt--;
                break;
        }

        fmt++;
    }
    return TRUE;
}

/*
1: 表示したい数値(正)
2: 表示する長さ
3: 符号 0,' ','+','-'
4: フラグ
*/
static bool_t put_integerD(bool_t (*__putc)(char), uint64_t n, int32_t length, int8_t sign, uint8_t flags) {
    char buf[26]; //20桁 + ,が6個
    uint8_t i = 0;
    uint8_t pad = ' ';

    //buf[]に表示内容の末尾から放り込む

    //64bitの割り算は重い..はず
    if ((n & 0xffffffff00000000) == 0) {
        uint32_t m = (uint32_t)n;
        do {
            buf[i++] = (m % 10) + '0';
            if( (flags & THOUSAND_GROUP) && (i & 3)==3) buf[i++] = ',';
        } while (m /= 10);
    } else {
        do {
            buf[i++] = (n % 10) + '0';
            if( (flags & THOUSAND_GROUP) && (i & 3)==3) buf[i++] = ',';
        } while (n /= 10);
    }

    length -= i;

    if (sign) {
        if (!__putc(sign)) return FALSE;
    }

    if (!(flags & LEFT_JUSTIFIED)) {
        if(flags & ZERO_PADDING) pad = '0';
        while (length > 0) {
            length--;
            if (!__putc(pad)) return FALSE;
        }
    }

    while (i > 0) {
        if (!__putc(buf[--i])) return FALSE;
    }
    while (length > 0) {
        length--;
        if (!__putc(pad)) return FALSE;
    }
    return TRUE;
}


static bool_t put_integerX(bool_t (*__putc)(char), uint64_t n, int32_t length, uint8_t flags) {
    static char *symbols_s = "0123456789abcdef";
    static char *symbols_c = "0123456789ABCDEF";

    char buf[16];
    uint8_t i = 0;
    uint8_t pad = ' ';
    char *symbols = (flags & CAPITAL_LETTER ? symbols_c : symbols_s);

    //buf[]に表示内容の末尾から放り込む

    do {
        buf[i++] = symbols[n & 15];
    } while (n >>= 4);

    length -= i;

    if (flags & ALTERNATIVE) {
        if (!__putc('0')) return FALSE;
        if (!__putc('x')) return FALSE;
    }

    if (!(flags & LEFT_JUSTIFIED)) {
        if(flags & ZERO_PADDING) pad = '0';
        while (length > 0) {
            length--;
            if (!__putc(pad)) return FALSE;
        }
    }

    while (i > 0) {
        if (!__putc(buf[--i])) return FALSE;
    }
    while (length > 0) {
        length--;
        if (!__putc(pad)) return FALSE;
    }
    return TRUE;
}

static bool_t put_integerO(bool_t (*__putc)(char), uint64_t n, int32_t length, uint8_t flags) {
    char buf[22];
    uint8_t i = 0;
    uint8_t pad = ' ';

    //buf[]に表示内容の末尾から放り込む

    do {
        buf[i++] = (n & 7) + '0';
    } while (n >>= 3);

    length -= i;

    if (flags & ALTERNATIVE) {
        if (!__putc('0')) return FALSE;
    }

    if (!(flags & LEFT_JUSTIFIED)) {
        if(flags & ZERO_PADDING) pad = '0';
        while (length > 0) {
            length--;
            if (!__putc(pad)) return FALSE;
        }
    }

    while (i > 0) {
        if (!__putc(buf[--i])) return FALSE;
    }
    while (length > 0) {
        length--;
        if (!__putc(pad)) return FALSE;
    }
    return TRUE;
}

static bool_t put_integerB(bool_t (*__putc)(char), uint64_t n, int32_t length, uint8_t flags) {
    char buf[64];
    uint8_t i = 0;
    uint8_t pad = ' ';

    //buf[]に表示内容の末尾から放り込む

    do {
        buf[i++] = (n & 1) + '0';
    } while (n >>= 1);

    length -= i;

    if (flags & ALTERNATIVE) {
        if (!__putc('0')) return FALSE;
        if (!__putc('b')) return FALSE;
    }

    if (!(flags & LEFT_JUSTIFIED)) {
        if(flags & ZERO_PADDING) pad = '0';
        while (length > 0) {
            length--;
            if (!__putc(pad)) return FALSE;
        }
    }

    while (i > 0) {
        if (!__putc(buf[--i])) return FALSE;
    }
    while (length > 0) {
        length--;
        if (!__putc(pad)) return FALSE;
    }
    return TRUE;
}
