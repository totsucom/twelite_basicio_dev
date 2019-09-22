
#include "basicio.h"
#include "string.h"

volatile uint32_t millisValue;
volatile uint32_t millisValueTick;


#ifdef USE_PBUTIL

static uint32_t u32PBDefined;       //ボタン登録されたDIO
static uint32_t u32PBPositive;      //押されたときにHになるボタン
static uint32_t u32PBCurIOStatus;   //現在のボタンのDIO状態
static uint8_t u8PBDelayCount[20];  //判定用ディレイカウンタ
static uint32_t u32PBPressed;       //押されたボタン
static uint32_t u32PBReleased;      //離されたボタン u32PBPressedとu32PBReleasedが同時にONにならない。いずれか直近のほうを保持

//指定したピンをプッシュボタンとする
//事前にdio_pinMode()でINPUTまたはINPUT_PULLUPに設定しておく
void pb_define(uint8_t pinNo, bool_t bPressToHi) {
    u32PBDefined |= (1UL << pinNo);
    uint32_t mask = (1UL << pinNo) ^ 0xfffff;
    if (bPressToHi) {
        u32PBPositive |= (1UL << pinNo);
    } else {
        u32PBPositive &= mask;
    }
    u32PBCurIOStatus = (u32PBCurIOStatus & mask) | ((u32AHI_DioReadInput() ^ u32PBPositive) & mask);
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

//プッシュボタンの状態をリセットする、つまり現在の状態をデフォルトとする。
void pb_reset() {
    u32PBCurIOStatus = u32AHI_DioReadInput() ^ u32PBPositive;
    u32PBPressed = 0;
    u32PBReleased = 0;
    memset(u8PBDelayCount, 0, sizeof(u8PBDelayCount));
}

#ifndef PB_JUDGE_COUNT
#define PB_JUDGE_COUNT 5
#endif

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
                        u32PBReleased &= b ^ 0xfffff;
                    } else {
                        //Released
                        u32PBReleased |= b;
                        u32PBPressed &= b ^ 0xfffff;
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


/*
 * スリープ
 */

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
割り込み起床は可能だが、起床に関する割り込みビットは取得できなくなる。

*/



/*
 * デジタルIO
 */

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


/*
 * デジタルIO(割り込み)
 */

//割り込みルーチンのポインタを保持
#ifndef MAX_DIO_INTERRUPT
#define MAX_DIO_INTERRUPT 2
#endif
void (*dioCallbackFunctions[MAX_DIO_INTERRUPT])(uint32_t);  //コールバック関数のポインタ
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
    for(i = 0; i < MAX_DIO_INTERRUPT; i++) {
        if (dioCallbackFunctions[i] == func) {
            //コールバック関数がすでに登録済みなので、ここを参照
            dioCallbackFuncIndices[pinNo] = i;
            break;
        } else if (freeIndex == 0xff && dioCallbackFunctions[i] == NULL) {
            //空きインデックスを記憶
            freeIndex = i;
        }
    }
    if (i == MAX_DIO_INTERRUPT) {
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
//事前にpinModeをINPUTに設定しておくこと（スリープ中はINPUT_PULLUPは使えないので自前で準備すること）
//この関数はsleep()の直前に呼び出してしてください。
bool_t dio_setWake(uint8_t pinNo, INTERRUPTIONEDGES mode) {
    if (pinNo > 19) return FALSE;
    if (mode != DISABLE && mode != RISING && mode != FALLING) return FALSE;

    (void)u32AHI_DioInterruptStatus(); // clear interrupt register
    if (mode != DISABLE) {
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
//bUseSecondPin Timer0 Timer1 Timer2 Timer3 Timer4
//   FALSEの場合  DIO10  DIO11  DIO12  DIO13  DIO17
//   TRUEの場合   DIO4   DIO5   DIO6   DIO7   DIO8
//timer_start()で開始します
bool_t timer_attachPWM(uint8_t timerNo, uint8_t prescale, uint16_t cycleCount, uint16_t pulseCount, bool_t bStartFromHi, bool_t bUseSecondPin) {
    if (timerNo > 4) return FALSE;
    //if (pulseCount == 0 || pulseCount >= cycleCount) return FALSE;

    vAHI_TimerSetLocation(timerNo, bUseSecondPin, FALSE);

    uint8_t b = 4 << timerNo;
    if ((timerFineGrainDIOControlValue & b) != 0) {
        //DIOを汎用からタイマー用に切り替える
        timerFineGrainDIOControlValue &= b ^ 255;
        vAHI_TimerFineGrainDIOControl(timerFineGrainDIOControlValue);
    }

    vAHI_TimerEnable(timerNo,
        prescale,
        FALSE, //bool_t bIntRiseEnable, 
        FALSE, //bool_t bIntPeriodEnable, 
        TRUE);//bool_t bOutputEnable);
    vAHI_TimerConfigureOutputs(timerNo,
        bStartFromHi, //TRUE:出力反転
        TRUE); //disable clock gation input
    sTimerApp[timerNo].u8Mode = 2; //PWM    
    sTimerApp[timerNo].bStartFromHi = bStartFromHi;
    sTimerApp[timerNo].u16HiCount = bStartFromHi ? pulseCount : cycleCount - pulseCount; //開始から変化までのカウント
    sTimerApp[timerNo].u16LoCount = cycleCount;             //開始から終了までのカウント=サイクル
    timerCallbackFunctions[timerNo] = NULL;

    return TRUE;
}

//timer_attachPWM()の簡易版。HzとDutyで指定 Hz:1～65536(0) Duty:0～32768
//bUseSecondPin Timer0 Timer1 Timer2 Timer3 Timer4
//   FALSEの場合  DIO10  DIO11  DIO12  DIO13  DIO17
//   TRUEの場合   DIO4   DIO5   DIO6   DIO7   DIO8
//timer_start()で開始します
bool_t timer_attachPWMByHzDuty(uint8_t timerNo, uint16_t hz, uint16_t duty, bool_t bStartFromHi, bool_t bUseSecondPin) {
    uint8_t prescale;
    uint16_t cycleCount;
    timerCalcParamFromHz(hz, &prescale, &cycleCount);

    uint16_t pulseCount = ((cycleCount != 0 ? (uint32_t)cycleCount : 65536) * (uint32_t)duty) >> 15;
    return timer_attachPWM(timerNo, prescale, cycleCount, pulseCount, bStartFromHi, bUseSecondPin);
}

//PWMのデューティー比を変更するためにパルスカウントを再設定する
bool_t timer_updatePWMPulseCount(uint8_t timerNo, uint16_t pulseCount) {
    if (timerNo > 4) return FALSE;
    if (sTimerApp[timerNo].u8Mode != 2) return FALSE; //not PWM
    //if (midCount == 0 || midCount >= sTimerApp[timerNo].u16LoCount) return FALSE;
    sTimerApp[timerNo].u16HiCount = sTimerApp[timerNo].bStartFromHi ? pulseCount : sTimerApp[timerNo].u16LoCount - pulseCount; //開始から変化までのカウント
    vAHI_TimerStartRepeat(timerNo, sTimerApp[timerNo].u16HiCount, sTimerApp[timerNo].u16LoCount);
    return TRUE;
}

//timer_updatePWMPulseCount()の簡易版。Duty:0～32768
bool_t timer_updatePWMDuty(uint8_t timerNo, uint16_t duty) {
    if (timerNo > 4 || duty > 65536) return FALSE;
    if (sTimerApp[timerNo].u8Mode != 2) return FALSE; //not PWM

    uint16_t cycleCount = sTimerApp[timerNo].u16LoCount;
    uint16_t pulseCount = ((cycleCount != 0 ? (uint32_t)cycleCount : 65536) * (uint32_t)duty) >> 15;

    sTimerApp[timerNo].u16HiCount = sTimerApp[timerNo].bStartFromHi ? pulseCount : sTimerApp[timerNo].u16LoCount - pulseCount; //開始から変化までのカウント
    vAHI_TimerStartRepeat(timerNo, sTimerApp[timerNo].u16HiCount, sTimerApp[timerNo].u16LoCount);
    return TRUE;
}

//timerNo=0..4, prescale=0..16, cycleCount=1..65536(0)
//周期 = (1 << prescale) * cycleCount / 16000000 [秒]
//startTimerで開始します
bool_t timer_attachCallback(uint8_t timerNo, uint8_t prescale, uint16_t cycleCount, void (*func)()) {
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
    return TRUE;
}

//timer_attachCallback()の簡易版。Hzで指定 1～65536(0)
bool_t timer_attachCallbackByHz(uint8_t timerNo, uint16_t hz, void (*func)()) {
    uint8_t prescale;
    uint16_t cycleCount;
    timerCalcParamFromHz(hz, &prescale, &cycleCount);
    return timer_attachCallback(timerNo, prescale, cycleCount, *func);
}

//疑似アナログ出力
//timerNo=0..4, power=0..65536(0)
//出力先DIO                   Timer0 Timer1 Timer2 Timer3 Timer4
//bUseSecondPin=FALSEの場合  DIO10  DIO11  DIO12  DIO13  DIO17
//bUseSecondPin=TRUEの場合   DIO4   DIO5   DIO6   DIO7   DIO8
bool_t timer_attachAnalogWrite(uint8_t timerNo, uint16_t power, bool_t bUseSecondPin) {
    if (timerNo > 4) return FALSE;

    vAHI_TimerSetLocation(timerNo, bUseSecondPin, FALSE);

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
bool_t timer_attachMicroCounter(uint8_t timerNo) {
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
//バッファの上位16ビットにLレベルの期間、下位に周期を示すカウンタ値が格納される
//1カウント値あたりの時間は 1 / (16000000 / (2^prescale)) 秒である
bool_t timer0_attachCapture(uint8_t prescale, uint32_t *pu32Buffer, uint16_t u16BufferLength, bool_t bUseSecondPin) {
    if (pu32Buffer == NULL || u16BufferLength == 0) return FALSE;

    pu16Timer0CapBuf = (uint16_t *)pu32Buffer;
    u16Timer0CapBufSize = u16BufferLength;
    u16Timer0CapCount = 0;
    bTimer0FirstCap = TRUE;
    sTimerApp[0].u8Mode = 5; //Capture

    vAHI_TimerSetLocation(0, bUseSecondPin, FALSE);

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
void timer0_attachCounter(uint8_t prescale, uint16_t count, bool_t bUseSecondPin, INTERRUPTIONEDGES mode, void (*func)()) {

    sTimerApp[0].u8Mode = 6; //Counter
    timerCallbackFunctions[0] = func;

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
}


#endif //USE_TIMER



/*
 * シリアル
 */

#ifdef USE_SERIAL

// FIFOキューや出力用の定義
tsFILE sUartStream0;
tsSerialPortSetup sUartPort0;
tsUartOpt sUartOpt0;

// 送信FIFOバッファ
#ifndef SERIAL_TX_BUFFER_SIZE
#define SERIAL_TX_BUFFER_SIZE 96
#endif
uint8_t au8SerialTxBuffer0[SERIAL_TX_BUFFER_SIZE]; 

// 受信FIFOバッファ
#ifndef SERIAL_RX_BUFFER_SIZE
#define SERIAL_RX_BUFFER_SIZE 32
#endif
uint8_t au8SerialRxBuffer0[SERIAL_RX_BUFFER_SIZE]; 

/*
bool_t serial_printf(const char* format, ...) {
    va_list args;
    va_start(args, format);

    SPRINTF_vRewind();
    vfPrintf(SPRINTF_Stream, format, args);
    va_end(args);
    return SERIAL_bTxString(E_AHI_UART_0, SPRINTF_pu8GetBuff());
}
*/
#endif //USE_SERIAL


#ifdef USE_SERIAL1

tsFILE sUartStream1;
tsSerialPortSetup sUartPort1;
tsUartOpt sUartOpt1;

// 送信FIFOバッファ
#ifndef SERIAL1_TX_BUFFER_SIZE
#define SERIAL1_TX_BUFFER_SIZE 96
#endif
uint8_t au8SerialTxBuffer1[SERIAL1_TX_BUFFER_SIZE]; 

// 受信FIFOバッファ
#ifndef SERIAL1_RX_BUFFER_SIZE
#define SERIAL1_RX_BUFFER_SIZE 32
#endif
uint8_t au8SerialRxBuffer1[SERIAL1_RX_BUFFER_SIZE]; 
#endif //USE_SERAIL1


#if defined(USE_SERIAL) || defined(USE_SERIAL1)
//Serial0,1共通関数を定義

//定数 SERIAL_BAUD_115200 などを渡すこと
bool_t serialx_init(uint8_t u8SerialPort, BAUDRATES baudRate, tsFILE *psUartStream, tsSerialPortSetup *psUartPort, tsUartOpt *psUartOpt) {
    if (u8SerialPort != E_AHI_UART_0 && u8SerialPort != E_AHI_UART_1) return FALSE;
    if ((baudRate & 0x80000000) == 0) return FALSE;

    psUartPort->pu8SerialRxQueueBuffer = au8SerialRxBuffer0;
    psUartPort->pu8SerialTxQueueBuffer = au8SerialTxBuffer0;
    psUartPort->u32BaudRate = baudRate;
    psUartPort->u16AHI_UART_RTS_LOW = 0xffff;
    psUartPort->u16AHI_UART_RTS_HIGH = 0xffff;
    psUartPort->u16SerialRxQueueSize = sizeof(au8SerialRxBuffer0);
    psUartPort->u16SerialTxQueueSize = sizeof(au8SerialTxBuffer0);
    psUartPort->u8SerialPort = u8SerialPort; //E_AHI_UART_0;
    psUartPort->u8RX_FIFO_LEVEL = E_AHI_UART_FIFO_LEVEL_1;

    psUartOpt->bHwFlowEnabled = FALSE; // TRUE, FALSE
	psUartOpt->bParityEnabled = FALSE; // TRUE, FALSE
	//psUartOpt->u8ParityType; // E_AHI_UART_EVEN_PARITY, E_AHI_UART_ODD_PARITY
	psUartOpt->u8StopBit = E_AHI_UART_1_STOP_BIT; // E_AHI_UART_1_STOP_BIT, E_AHI_UART_2_STOP_BIT
	psUartOpt->u8WordLen = 8; // 5-8 を指定 (E_AHI_UART_WORD_LEN_? は指定しない)

    vAHI_UartSetRTSCTS(u8SerialPort, FALSE);
    SERIAL_vInitEx(psUartPort, psUartOpt);

    psUartStream->bPutChar = SERIAL_bTxChar;
    psUartStream->u8Device = u8SerialPort; //E_AHI_UART_0;
    return TRUE;
}

//debugLevel=0..5 (0:デバッグ出力無し)
bool_t serialx_forDebug(tsFILE *psUartStream, uint8_t debugLevel) {
    if (debugLevel > 5) return FALSE;

    ToCoNet_vDebugInit(psUartStream);
	ToCoNet_vDebugLevel(debugLevel);
    return TRUE;
}

bool_t serialx_write(uint8_t u8SerialPort, uint8_t *pu8Data, uint8_t length)
{
    if (u8SerialPort != E_AHI_UART_0 && u8SerialPort != E_AHI_UART_1) return FALSE;

    while(length-- > 0)
    {
        if (!SERIAL_bTxChar(u8SerialPort, *pu8Data)) return FALSE;
        pu8Data++;
    }
    return TRUE;
}

int16_t serialx_readUntil(uint8_t u8SerialPort, uint8_t u8Terminate, uint8_t *pu8Buffer, uint16_t u16BufferLength) {
    int16_t len = 0;
    while (u16BufferLength > 0) {
        if (*pu8Buffer == '\0') break;
        pu8Buffer++;
        u16BufferLength--;
        len++;
    }
    if (u16BufferLength == 0) return -1; //Buffer not initialized. (Not found null terminate)

    while (u16BufferLength > 1 && !SERIAL_bRxQueueEmpty(u8SerialPort)) {
        int16_t c = SERIAL_i16RxChar(u8SerialPort);
        if (c < 0) return -1; //Error

        *pu8Buffer++ = c;
        u16BufferLength--;
        len++;

        if (c == u8Terminate) {
            *pu8Buffer = '\0';
            return len;
        }
    }
    *pu8Buffer = '\0';
    return (u16BufferLength == 1) ? len : 0;
}

#endif //USE_SERIAL || USE_SERIAL1



/*
 * ＡＤＣ
 */


//割り込みルーチンのポインタを保持
static void (*adcCallbackFunction)(uint16_t);

//attachAdcCallback()またはadc_attachCallbackWithTimerSampling()の設定を保持
static bool_t adcIsContinuous;
static bool_t adcIsRange2;
static bool_t adcIsExternalVRef;
static ADCSOURCES adcLastSource;        //adc_attachCallbackWithTimerSampling()実行時は0xff
//adc_attachCallbackWithTimerSampling()専用
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
//AD変換時間は (サンプリング数 x 3 + 14)クロック 
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

void adc_disable() {
    if (bAHI_APRegulatorEnabled()) {
        vAHI_ApConfigure(E_AHI_AP_REGULATOR_DISABLE,    //OFF
            E_AHI_AP_INT_DISABLE,                       //OFF
            ADC_SAMPLE_4,
            ADC_CLOCK_500KHZ,
            E_AHI_AP_INTREF);
    }
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
    case ADC_SOURCE_TEMP:
        range2 = FALSE;
        break;
    case ADC_SOURCE_VOLT:
        range2 = TRUE;
    }

    adcIsContinuous = continuous;
    adcIsRange2 = range2;
    adcLastSource = source;
    adcCallbackFunction = func;

    vAHI_AdcEnable(continuous, range2, source);
    vAHI_AdcStartSample(); // ADC開始
}

void adc_detachCallback()  {
    adcCallbackFunction = NULL;
    vAHI_AdcDisable();
}

static uint16_t adcConvertADCx(uint16_t src) {
    if (adcIsRange2) {
        //1023 = 2.470V, 2.470V * 1024/1023 = 2472
        if (!adcIsExternalVRef) {
            return (2472 * (int32_t)src) >> 10; //[mV]
        } else {
            return (adcExternalVRef_2048Kdiv1023 * (int32_t)src) >> 10;
        }
   } else {
        //1023 = 1.235V, 1.235V * 1024/1023 = 1236
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
    if (!adcIsExternalVRef) {
        resultValue = (int32_t)src * 1236;     //x1024[mV]
    } else {
        resultValue = (int32_t)src * adcExternalVRef_1024Kdiv1023;
    }
    resultValue -= 730 * 1024;              //x1024
    resultValue *= -771;                    //771=(1/1.66)*1280  x1024x1280
                                            //1280は最終結果をx10[℃]にしたいため
    return (resultValue >> 17) + 250; //x10[℃]
}

static uint16_t adcConvertVolt(uint16_t src) {
    //自身の電圧は2/3に分圧された値を測定しているので、1023のとき3.705V
    //1.235V * 2 * 1.5 = 3.705V
    //3709 = 3.705V / 1023 * 1000 * 1024

    if (!adcIsExternalVRef) {
        return ((int32_t)src * 3709) >> 10; //[mV]
    } else {
        return ((int32_t)src * adcExternalVRef_3072Kdiv1023) >> 10;
    }
}

#ifdef USE_TIMER
bool_t adc_attachCallbackWithTimerSampling(uint8_t timerNo, uint8_t prescale, uint16_t cycleCount,
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
    if (bitmap & ADC_SOURCE_BITMAP_5) dio_pinMode(2, INPUT);
    if (bitmap & ADC_SOURCE_BITMAP_6) dio_pinMode(3, INPUT);

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

bool_t adc_convertResults(ADCSOURCEBITMAP bitmap, uint16_t *pu16Buffer, uint16_t u16BufferSize) {
    if (bitmap == 0 || u16BufferSize == 0 || pu16Buffer == NULL) return FALSE;
    while(1) {
        if (bitmap & ADC_SOURCE_BITMAP_1) { *pu16Buffer = adcConvertADCx(*pu16Buffer); pu16Buffer++; if(--u16BufferSize == 0) break; }
        if (bitmap & ADC_SOURCE_BITMAP_2) { *pu16Buffer = adcConvertADCx(*pu16Buffer); pu16Buffer++; if(--u16BufferSize == 0) break; }
        if (bitmap & ADC_SOURCE_BITMAP_3) { *pu16Buffer = adcConvertADCx(*pu16Buffer); pu16Buffer++; if(--u16BufferSize == 0) break; }
        if (bitmap & ADC_SOURCE_BITMAP_4) { *pu16Buffer = adcConvertADCx(*pu16Buffer); pu16Buffer++; if(--u16BufferSize == 0) break; }
        if (bitmap & ADC_SOURCE_BITMAP_TEMP) { *pu16Buffer = adcConvertTemp(*pu16Buffer); pu16Buffer++; if(--u16BufferSize == 0) break; }
        if (bitmap & ADC_SOURCE_BITMAP_VOLT) { *pu16Buffer = adcConvertVolt(*pu16Buffer); pu16Buffer++; if(--u16BufferSize == 0) break; }
        if (bitmap & ADC_SOURCE_BITMAP_5) { *pu16Buffer = adcConvertADCx(*pu16Buffer); pu16Buffer++; if(--u16BufferSize == 0) break; }
        if (bitmap & ADC_SOURCE_BITMAP_6) { *pu16Buffer = adcConvertADCx(*pu16Buffer); pu16Buffer++; if(--u16BufferSize == 0) break; }
    }
    return TRUE;
}
#endif


/*
 *　コンパレータ
 */

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

//    bool_t turned_on = FALSE;
    if (!bAHI_APRegulatorEnabled()) {
        //設定には電源が必要
        vAHI_ApConfigure(E_AHI_AP_REGULATOR_ENABLE, // DISABLE にするとアナログ部の電源断
            FALSE,                      // 割り込みなし
            E_AHI_AP_SAMPLE_2,          // 意味なし サンプル数 2,4,6,8 が選択可能
            E_AHI_AP_CLOCKDIV_500KHZ,   // 意味なし 周波数 250K/500K/1M/2M
            FALSE);                     // 意味なし 内部基準電圧を使う

        while(!bAHI_APRegulatorEnabled()); // 安定するまで待つ（一瞬）
//        turned_on = TRUE;
    }

    compCallbackFunction = func;

    vAHI_ComparatorIntEnable(
        E_AHI_AP_COMPARATOR_1,  //uint8 u8Comparator,
        (func != NULL),         //bool_t bIntEnable, コールバックを呼ぶとき、ウェイク条件にするときはTRUE
        (mode == RISING));      //bool_t bRisingNotFalling);
/*
    if (turned_on) {
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



/*
 *　パルスカウンタ
 */

//割り込みルーチンのポインタを保持
static void (*pcCallbackFunctions[2])();


//u32AHI_PulseCounterStatus()の値を保持
static uint32_t pcCountStatus;

//パルスカウンタを開始します。入力ピンは事前にdio_pinMode()でINPUTまたはINPUT_PULLUPに設定しておいてください
//pcNo=0/1, mode=RISING/FALLING, count=何回目でコールバック関数を呼び出すかを指定
//bUseSecondPin PC0  PC1
//       FALSE  DIO1 DIO8
//       TRUE   DIO4 DIO5
//カウンタは０にクリアされますが、pc_start()を実行するまでパルスカウンタは開始されません。
//func=NULLが設定可能です。カウンタが到達したかどうかは割り込みまたはポーリングでpc_countReached()を調べることができます。
//カウンタが到達後も継続してカウントされるため必要に応じてpc_clear()を呼び出してください
bool_t pc_attachCallback(uint8_t pcNo, PCDEBOUNCEMODE debounce, uint16_t count, bool_t bUseSecondPin, INTERRUPTIONEDGES mode, void (*func)()) {
    if (pcNo > 1) return FALSE;
    if (mode != FALLING && mode != RISING) return FALSE;

    pcCallbackFunctions[pcNo] = func;

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
        func != NULL);      //bool_t bIntEnable);

    bAHI_SetPulseCounterRef(pcNo, (uint32_t)(count - 1));
    pc_clear(pcNo);
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

//パルスカウンタを起床条件に設定します
//pcNo=0/1, mode=RISING/FALLING, count=何回目で起床するかを指定
//bUseSecondPin PC0  PC1
//       FALSE  DIO1 DIO8
//       TRUE   DIO4 DIO5
//DIOはINPUTモードに設定されます。
//カウンタはクリアされ、カウントを開始します。そのため、この関数はsleep()の直前に実行してください。
//debounceにPC_DEBOUNCE_0_MAX100KHZ以外を使用する場合はスリープでオシレータを停止してはいけません。
bool_t pc_setWake(uint8_t pcNo, PCDEBOUNCEMODE debounce, uint16_t count, bool_t bUseSecondPin, INTERRUPTIONEDGES mode) {
    if (pcNo > 1) return FALSE;
    if (mode != FALLING && mode != RISING) return FALSE;

    dio_pinMode((pcNo == 0) ? (bUseSecondPin ? 4 : 1) : (bUseSecondPin ? 5 : 8), INPUT);

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
        TRUE);              //bool_t bIntEnable);

    bAHI_SetPulseCounterRef(pcNo, (uint32_t)(count - 1));
    pc_clear(pcNo);
    u32AHI_PulseCounterStatus();    //clear flag
    pc_start(pcNo);

    return TRUE;
}


//32ビットパルスカウンタを開始します。PC0とPC1を結合して使用するため、併用はできません。
//入力ピンは事前にdio_pinMode()でINPUTまたはINPUT_PULLUPに設定しておいてください
//pcNo=0/1, mode=RISING/FALLING, count=何回目でコールバック関数を呼び出すかを指定
//pinNo=1,4,5,8のいずれか
//カウンタは０にクリアされますが、pc32_start()を実行するまでパルスカウンタは開始されません。
//func=NULLが設定可能です。カウンタが到達したかどうかは割り込みまたはポーリングでpc32_countReached()を調べることができます。
//カウンタが到達後も継続してカウントされるため必要に応じてpc32_clear()を呼び出してください
bool_t pc32_attachCallback(PCDEBOUNCEMODE debounce, uint32_t count, uint8_t pinNo, INTERRUPTIONEDGES mode, void (*func)()) {
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

    pcCallbackFunctions[0] = func;

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
        func != NULL);      //bool_t bIntEnable);

    bAHI_SetPulseCounterRef(0, count - 1);
    pc32_clear();
    return TRUE;
}

//32ビットパルスカウンタのカウント値を読み出します
uint32_t pc32_read() {
    uint32_t value;
    if (!bAHI_Read32BitCounter(&value)) return 0xffffffff;
    return value;
}

//パルスカウンタを起床条件に設定します
//mode=RISING/FALLING, count=何回目で起床するかを指定
//pinNo=1,4,5,8のいずれか
//DIOはINPUTモードに設定されます。
//カウンタはクリアされ、カウントを開始します。そのため、この関数はsleep()の直前に実行してください。
//debounceにPC_DEBOUNCE_0_MAX100KHZ以外を使用する場合はスリープでオシレータを停止してはいけません。
bool_t pc32_setWake(PCDEBOUNCEMODE debounce, uint32_t count, uint8_t pinNo, INTERRUPTIONEDGES mode) {
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

    dio_pinMode(pinNo, INPUT);

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
        TRUE);              //bool_t bIntEnable);

    bAHI_SetPulseCounterRef(0, count - 1);
    pc32_clear();
    u32AHI_PulseCounterStatus();    //clear flag
    pc32_start();

    return TRUE;
}



/*
 * Ｉ２Ｃ
 */


I2CADDRESSINGMODE i2cAddressingMode;

//I2C初期化。標準のクロックは I2C_CLOCK_100KHZ
//bUseSecondPin  SCL   SDA
// = FALSE      DIO14 DIO15
// = TRUE       DIO16 DIO17
void i2c_enable(I2CCLOCKS clock, bool_t bUseSecondPin) {
    vAHI_SiSetLocation(bUseSecondPin);
    vAHI_SiMasterConfigure(TRUE, FALSE, clock);
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
    if (!i2c_readOnly(u16Address, pu8Data, u8Length)) return FALSE;
    return TRUE;
}

//I2Cで指定アドレスのコマンドから1バイトのデータを読み出します。失敗で-1
int16_t i2c_readByte(uint16_t u16Address, uint8_t u8Command) {
    if (!i2c_write(u16Address, u8Command, NULL, 0)) return -1;
    uint8_t data;
    if (!i2c_readOnly(u16Address, &data, 1)) return -1;
    return (int16_t)data;
}

//I2Cで指定アドレスにコマンドと1バイトのデータを書き込む
bool_t i2c_writeByte(uint16_t u16Address, uint8_t u8Command, uint8_t u8Data) {
    return i2c_write(u16Address, u8Command, &u8Data, 1);
}

//I2Cで指定アドレスからデータを1バイト読み出します。失敗で-1
int16_t i2c_readByteOnly(uint16_t u16Address) {
    uint8_t data;
    if (!i2c_readOnly(u16Address, &data, 1)) return -1;
    return (int16_t)data;
}


/*
 * ＳＰＩ
 */

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
void spi_selectSlave(int8_t slaveNo) {
    if (slaveNo >= 0 && slaveNo <= 2) {
        vAHI_SpiSelect(1 << slaveNo);
    } else {
        vAHI_SpiSelect(0);
    }
}

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
    if (u8Length == 0) return;

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



/*
 * 無線通信
 */

#ifdef USE_RADIO

//送信完了割り込みルーチンのポインタを保持
static void (*radioTxCallbackFunction)(uint8_t, bool_t);

//受信割り込みルーチンのポインタを保持
static void (*radioRxCallbackFunction)(uint32_t, uint8_t, uint8_t, uint8_t *, uint8_t, uint8_t);

//送信シーケンス番号を保持
static uint8_t u8RadioSeqNo;

//送信中のデータ数を保持
uint8_t u8NumRadioTx;


//無線送信完了割り込みルーチンを設定する
void radio_attachTxCallback(void (*func)(uint8_t u8CbId, bool_t bSuccess)) {
    radioTxCallbackFunction = func;
}

//無線受信割り込みルーチンを設定する
void radio_attachRxCallback(void (*func)(uint32_t u32SrcAddr, uint8_t u8CbId, uint8_t u8DataType, uint8_t *pu8Data, uint8_t u8Length, uint8_t u8Lqi)) {
    radioRxCallbackFunction = func;
}

//無線で特定の相手に送信する
//basicio_module.hで送信モジュールと同じAPP_ID,CHANNELに設定し、RX_ON_IDLE=TRUEとしたモジュールかつ、関数の引数でu32DistAddrに指定したモジュールが受信できる
//u32DestAddr=相手のモジュールアドレス。事前にSerial0_printf("%u", moduleAddress)等を実行してTWELITE毎のモジュールアドレスを知っておくとよい
//pu8Data=データ, u8Length=データ長さ(最大108バイト), u8DataType=データの簡易識別番号(0..7)
//簡易識別番号は受け取り側が何のデータか知るために使う。使用しない場合は値はなんでもよい
//関数はエラーで-1、送信開始で8bitの送信Id(u8CbId)を返す。これは送信完了コールバックで送信データの識別に使用される。
int16_t radio_write(uint32_t u32DestAddr, uint8_t *pu8Data, uint8_t u8Length, uint8_t u8DataType)
{
    if (u8Length > 108) return -1;

    u8RadioSeqNo++;

    tsTxDataApp tsTx;
    memset(&tsTx, 0, sizeof(tsTxDataApp));

    tsTx.u32SrcAddr = moduleAddress();              //送信元アドレス
    tsTx.u32DstAddr = u32DestAddr;                  //送信先アドレス

	tsTx.u8Cmd = u8DataType;                        //データ種別 (0..7)。データの簡易識別子。
	tsTx.u8Seq = u8RadioSeqNo; 		                //シーケンス番号(複数回送信時に、この番号を調べて重複受信を避ける)
	tsTx.u8Len = u8Length; 		                    //データ長
	tsTx.u8CbId = u8RadioSeqNo;	                    //送信識別ID。送信完了イベントの引数として渡され、送信イベントとの整合を取るために使用する
    memcpy(tsTx.auData, pu8Data, u8Length);

	tsTx.bAckReq = (u32DestAddr != TOCONET_MAC_ADDR_BROADCAST); //TRUE Ack付き送信を行う
#ifndef TX_RETRY
#define TX_RETRY 2
#endif
	tsTx.u8Retry = TX_RETRY;    		            //MACによるAck付き送信失敗時に、さらに再送する場合(ToCoNet再送)の再送回数

	//tsTx.u16ExtPan = 0;                           //0:外部PANへの送信ではない 1..0x0FFF: 外部PANへの送信 (上位4bitはリザーブ)

	//tsTx.u16DelayMax = 0;       //送信開始までのディレー(最大)[ms]。指定しない場合は 0 にし、指定する場合は Min 以上にすること。
	//tsTx.u16DelayMin = 0;       //送信開始までのディレー(最小)[ms]
	tsTx.u16RetryDur = 10;      //再送間隔[ms]。

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
int16_t radio_puts(uint32_t u32DestAddr, uint8_t *pu8String)
{
    uint16_t len = (uint16_t)strlen(pu8String);
    if (len > 108) return -1;

    u8RadioSeqNo++;

    tsTxDataApp tsTx;
    memset(&tsTx, 0, sizeof(tsTxDataApp));

    tsTx.u32SrcAddr = moduleAddress();              //送信元アドレス
    tsTx.u32DstAddr = u32DestAddr;                  //送信先アドレス

	tsTx.u8Cmd = 0;                                 //データ種別 (0..7)。データの簡易識別子。
	tsTx.u8Seq = u8RadioSeqNo; 		                //シーケンス番号(複数回送信時に、この番号を調べて重複受信を避ける)
	tsTx.u8Len = len; 		                        //データ長
	tsTx.u8CbId = u8RadioSeqNo;	                    //送信識別ID。送信完了イベントの引数として渡され、送信イベントとの整合を取るために使用する
    memcpy(tsTx.auData, pu8String, len);

	tsTx.bAckReq = (u32DestAddr != TOCONET_MAC_ADDR_BROADCAST); //TRUE Ack付き送信を行う
	tsTx.u8Retry = 2; 		                        //最大送信回数は3回になる

	//tsTx.u16ExtPan = 0;                           //0:外部PANへの送信ではない 1..0x0FFF: 外部PANへの送信 (上位4bitはリザーブ)

	//tsTx.u16DelayMax = 0;       //送信開始までのディレー(最大)[ms]。指定しない場合は 0 にし、指定する場合は Min 以上にすること。
	//tsTx.u16DelayMin = 0;       //送信開始までのディレー(最小)[ms]
	tsTx.u16RetryDur = 10;      //再送間隔[ms]。

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

//radio_write()のprintf版
bool_t radio_printf(uint32_t u32DestAddr, const char* format, va_list args) {
    SPRINTF_vRewind();
    vfPrintf(SPRINTF_Stream, format, args);
    va_end(args);
    return radio_puts(u32DestAddr, SPRINTF_pu8GetBuff());
}

#endif //USE_RADIO


/*
 * EEPROM
 */


/*
 * FRASHメモリ
 */

#ifdef USE_FLASH
#define FLASH_TYPE E_FL_CHIP_INTERNAL
#define FLASH_SECTOR_SIZE (32L* 1024L) // 32KB

#ifdef TWELITE_BLUE
#define FLASH_LAST_SECTOR   4   //BLUE
#else
#define FLASH_LAST_SECTOR   15  //RED
#endif


//sector=0..4(BLUE)/0..15(RED)。プログラムはセクタ0から書き込まれるので、使用していないセクタに書き込むこと
//flash_write()はビットを1から0にしか書き換えられないので、この関数により消去(ビットを1にする)した部分にしか書き込めない
bool_t flash_erase(uint8_t sector)
{
#ifdef FLASH_LAST_SECTOR
    if (sector > FLASH_LAST_SECTOR) return FALSE;
#endif

    if (!bAHI_FlashInit(FLASH_TYPE, NULL)) return FALSE;
    return bAHI_FlashEraseSector(sector);
}

//sector=0..4(BLUE)/0..15(RED)。プログラムはセクタ0から書き込まれるので、使用していないセクタに書き込むこと
//offsetはセクタ内オフセット値で、16の倍数とする
//pu8Dataの先頭に２バイトのリザーブ領域を設けておくこと。構造体の場合は uint8_t reserve[2]; とする。
//データ長を示すu16DataLengthにはリザーブ領域の2バイトを含む。1セクタは32KBであるが、このデータ領域がセクタ境界を越えてはならない
//事前にflash_erase()で領域をフォーマットしておくこと
bool_t flash_write(uint8_t sector, uint32_t offset, uint8_t *pu8Data, uint16_t u16DataLength)
{
    if ((offset & 15) != 0) return FALSE;
    if (offset + u16DataLength > FLASH_SECTOR_SIZE) return FALSE;
#ifdef FLASH_LAST_SECTOR
    if (sector > FLASH_LAST_SECTOR) return FALSE;
#endif

    offset += (uint32)sector * FLASH_SECTOR_SIZE;

    if (!bAHI_FlashInit(FLASH_TYPE, NULL)) return FALSE;

    *pu8Data = 0xE7;                                            //MAGIC_NUMBER
    *(pu8Data + 1) = u8CCITT8(pu8Data + 2, u16DataLength - 2);  //CRC8

    return bAHI_FullFlashProgram(offset, u16DataLength, pu8Data);
}

//MAGIC_NUMBERとCRC8の２つの方法でデータが書き込んだものと同じであることを確認している  
bool_t flash_read(uint8 sector, uint32 offset, uint8_t *pu8Data, uint16_t u16DataLength)
{
    if (offset + u16DataLength > FLASH_SECTOR_SIZE) return FALSE;

    offset += (uint32)sector * FLASH_SECTOR_SIZE;

    if (!bAHI_FlashInit(FLASH_TYPE, NULL)) return FALSE;
    if (!bAHI_FullFlashRead(offset, u16DataLength, pu8Data)) return FALSE;

    if (*pu8Data != 0xE7) return FALSE;                                             //MAGIC_NUMBER
    if (*(pu8Data + 1) != u8CCITT8(pu8Data + 2, u16DataLength - 2)) return FALSE;   //CRC8

    return TRUE;
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
        pb_update(); //プッシュボタンの状態を更新
#endif
        loop(EVENT_TICK_TIMER);
        break;
    case E_EVENT_TICK_SECOND:
        loop(EVENT_TICK_SECOND);
        break;
    }
}

#ifdef USE_RADIO
static uint32_t u32SrcAddrPrev;
static uint8_t u8seqPrev;
#endif

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

    memset(dioCallbackFunctions, 0, sizeof(dioCallbackFunctions));
    memset(dioCallbackFuncIndices, 0xff, sizeof(dioCallbackFuncIndices));

#ifdef USE_TIMER
    memset(timerCallbackFunctions, 0, sizeof(timerCallbackFunctions));
    memset(sTimerApp, 0, sizeof(sTimerApp));
    timerFineGrainDIOControlValue = 0xFF;
#endif

    adcCallbackFunction = NULL;
    compCallbackFunction = NULL;

    memset(pcCallbackFunctions, 0, sizeof(pcCallbackFunctions));
    pcCountStatus = 0;

    i2cAddressingMode = I2C_ADDRESS_7BIT;

#ifdef USE_RADIO
    radioTxCallbackFunction = NULL;
    radioRxCallbackFunction = NULL;
    u8RadioSeqNo = 0;
    u8NumRadioTx = 0;

    u32SrcAddrPrev = 0;
    u8seqPrev = 255;
#endif

    millisValue = 0;

#ifdef SPRINTF_H_
#ifndef SB_BUFFER_SIZE
#define SB_BUFFER_SIZE 128
#endif

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
#endif
}

void initAppContext()
{
#ifdef USE_RADIO
	sToCoNet_AppContext.u32AppId = APP_ID;      //!< 32bitのアプリケーションID。本IDでToCoNet同士の識別を行う。（必須設定項目）
	sToCoNet_AppContext.u8Channel = CHANNEL;    //!< モジュールのチャネル。NWK層の動作により変更される場合がある。(必須設定項目, Chマスクに存在するチャネルを指定すること)
	sToCoNet_AppContext.bRxOnIdle = RX_ON_IDLE; //!< TRUE:無線回路アイドル時も受信回路をオープンする。受信が必要な場合は必ずTRUEに設定する。(規定値は FALSE, Nwk層ではTRUE必須)
#else
	sToCoNet_AppContext.u8MacInitPending = TRUE; //!< TRUE:MAC 層の初期化をシステム始動時に行わない。無線部を使用せずに動作させる場合に設定します。
#endif
	//uint32 u32ChMask; 			            //!< 利用するチャネル群。NWK層やNeibourScanで利用する。(必須設定項目)
	//uint16 u16ShortAddress; 	                //!< モジュールのショートアドレス。指定しなければモジュールのシリアル番号から自動生成される。0xFFFFは指定出来ない。Nwk層利用時は指定しないこと。

#ifdef CPU_CLOCK
	sToCoNet_AppContext.u8CPUClk = CPU_CLOCK; 	//!< 通常稼働時のCPUクロック。3:32MHz, 2:16Mhz, 1:8Mhz, 0:4Mhz を指定する(規定値は 2)
#endif

#ifdef TX_POWER
	sToCoNet_AppContext.u8TxPower = TX_POWER; 	//!< モジュールの出力 3:最大 2: -11.5db 2: -23db 0:-34.5db となる (規定値は 3)
#endif
	//uint8 u8TxMacRetry; 		//!< MAC層の再送回数 7..0 を指定する。(規定値は3, Nwk層では１を推奨)

	//bool_t bPromiscuousMode; 	//!< テスト受信モード。通常は設定してはいけません (規定値は FALSE)
	//bool_t bSkipBootCalib;		//!< 始動時のキャリブレーション処理を省略する
	//uint8 u8Osc32Kmode; 		//!< 32K 水晶のモード (0x00: RC, 0x02: 32K水晶, 0x03: 32K発振器)
	//uint8 u8CCA_Retry; 			//!< CCA のリトライ回数 (通常は変更しない)
	//uint8 u8CCA_Level; 			//!< CCA アルゴリズムの開始レベル (通常は変更しない)

	//bool_t bNoAckMode;			//!< Ack を一切返さない。起動時の設定のみ反映され、起動後は変更できない。(通常は変更しない)
	//bool_t bRxExtPan;			//!< 他のPANからのパケットを受信する (1.0.8)

	sToCoNet_AppContext.u8RandMode = 3; //!< 乱数生成方法の指定。0:ハード 1:システム経過時間を元に生成 2:MT法 3:XorShift法 (32kOscモードで外部水晶が利用されたときは 0 の場合 XorShift 方を採用する)
                                //※ハードウェア乱数では同一ループ内では同じ値しか生成できない。MT法は高品質だがライセンス表記必要。    
#ifndef TICK_COUNT
#define TICK_COUNT  250
#endif
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
    //ToCoNet_REG_MOD_RAND_XOR_SHIFT();
    ToCoNet_vReg_mod_Rand_Xor_Shift();

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
    // 前回と同一の送信元＋シーケンス番号のパケットなら無視
    if (pRx->u32SrcAddr == u32SrcAddrPrev && pRx->u8Seq == u8seqPrev) {
        return;
    }
    u32SrcAddrPrev = pRx->u32SrcAddr;
    u8seqPrev = pRx->u8Seq;

    if (radioRxCallbackFunction != NULL) {
        //受信ルーチンを呼び出す
        (*radioRxCallbackFunction)(pRx->u32SrcAddr, pRx->u8Seq, pRx->u8Cmd, pRx->auData, pRx->u8Len, pRx->u8Lqi);
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

#ifdef USE_TIMER
    case E_AHI_DEVICE_TIMER0:
        //タイマー0割り込み処理ルーチンの呼び出し
        if (timerCallbackFunctions[0] != NULL) {
            (*timerCallbackFunctions[0])();
        }
        break;

    case E_AHI_DEVICE_TIMER1:
        //タイマー1割り込み処理ルーチンの呼び出し
        if (timerCallbackFunctions[1] != NULL) {
            (*timerCallbackFunctions[1])();
        }
        break;

    case E_AHI_DEVICE_TIMER2:
        //タイマー2割り込み処理ルーチンの呼び出し
        if (timerCallbackFunctions[2] != NULL) {
            (*timerCallbackFunctions[2])();
        }
        break;

    case E_AHI_DEVICE_TIMER3:
        //タイマー3割り込み処理ルーチンの呼び出し
        if (timerCallbackFunctions[3] != NULL) {
            (*timerCallbackFunctions[3])();
        }
        break;

    case E_AHI_DEVICE_TIMER4:
        //タイマー4割り込み処理ルーチンの呼び出し
        if (timerCallbackFunctions[4] != NULL) {
            (*timerCallbackFunctions[4])();
        }
        break;
#endif

    case E_AHI_DEVICE_SYSCTRL:
        _C {
            if (u32ItemBitmap & 0xfffff) {
                //DIO割り込み
                uint32_t bitmap = u32ItemBitmap;
                uint8_t pinNo;
                for(pinNo = 0; pinNo < 20; pinNo++) {
                    if (bitmap & 1) { //DIO番号に対応するビットが1になっている
                        uint8_t i = dioCallbackFuncIndices[pinNo];
                        if (i < MAX_DIO_INTERRUPT && dioCallbackFunctions[i] != NULL)
                            (*dioCallbackFunctions[i])(u32ItemBitmap);
                    }
                    bitmap >>= 1;
                    if (bitmap == 0) break;
                }
            }
            if ((u32ItemBitmap & E_AHI_SYSCTRL_COMP0_MASK) != 0 && compCallbackFunction != NULL) {
                //コンパレータ割り込み
                (*compCallbackFunction)();
            }
            if ((u32ItemBitmap & E_AHI_SYSCTRL_PC0_MASK) != 0 && pcCallbackFunctions[0] != NULL) {
                //パルスカウンタ0割り込み
                (*pcCallbackFunctions[0])();
            }
            if ((u32ItemBitmap & E_AHI_SYSCTRL_PC1_MASK) != 0 && pcCallbackFunctions[1] != NULL) {
                //パルスカウンタ1割り込み
                (*pcCallbackFunctions[1])();
            }


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
        //ADC(完了)割り込みルーチンの呼び出し
        switch(adcLastSource) {
            case 0xff:
                //adc_attachCallbackWithTimerSampling()割り込み
                if (!adcIsContinuous && --adcIntCountTillEnd == 0) {
                    vAHI_AdcDisableSampleBuffer();//完了時にこれをやっとかないと2回目が実行できない
                }
                (*((void (*)())adcCallbackFunction))();//パラメータ無し
                break;
            case ADC_SOURCE_VOLT:
                (*adcCallbackFunction)((int16_t)adcConvertVolt(u16AHI_AdcRead()));
                break;
            case ADC_SOURCE_TEMP:
                (*adcCallbackFunction)((int16_t)adcConvertTemp(u16AHI_AdcRead()));
                break;
            default:
                (*adcCallbackFunction)((int16_t)adcConvertADCx(u16AHI_AdcRead()));
                break;
        }
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
        if (sTimerApp[0].u8Mode == 4) { //Micro counter    
            sTimerApp[0].u16HiMicroSeconds++;
            return TRUE;
        }
        else if (sTimerApp[0].u8Mode == 5) { //Capture
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
                return TRUE;
            }
        }
        break;

    case E_AHI_DEVICE_TIMER1:
        if (sTimerApp[1].u8Mode == 4) { //Micro counter    
            sTimerApp[1].u16HiMicroSeconds++;
            return TRUE;
        }
        break;

    case E_AHI_DEVICE_TIMER2:
        if (sTimerApp[2].u8Mode == 4) { //Micro counter    
            sTimerApp[2].u16HiMicroSeconds++;
            return TRUE;
        }
        break;

    case E_AHI_DEVICE_TIMER3:
        if (sTimerApp[3].u8Mode == 4) { //Micro counter    
            sTimerApp[3].u16HiMicroSeconds++;
            return TRUE;
        }
        break;

    case E_AHI_DEVICE_TIMER4:
        if (sTimerApp[4].u8Mode == 4) { //Micro counter    
            sTimerApp[4].u16HiMicroSeconds++;
            return TRUE;
        }
        break;
#endif

    case E_AHI_DEVICE_TICK_TIMER:
        //u32AHI_TickTimerRead()がうまく値を返してくれない(仕様?)のでここでカウント
        millisValue += millisValueTick;

        //E_AHI_DEVICE_TICK_TIMERは例外的にTRUEを返してはいけない (固まる)
    }

	return FALSE;//FALSEによりcbToCoNet_vHwEvent()が呼ばれる
}

// メイン
void cbToCoNet_vMain(void)
{
}
