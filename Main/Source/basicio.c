
#include "basicio.h"
#include "string.h"

uint32_t millisValue;
uint32_t millisValueTick;

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
uint8_t dioCallbackPinNos[MAX_DIO_INTERRUPT];        //対応するピン番号0-19, 0xffで初期化

//pinNO=0..19 funcは引数を持たない関数 mode=RISING(立ち上がり)/FALLING(立ち下がり)/DISABLE
//一つのピンに一つの関数しか登録できない
bool_t dio_attachCallback(uint8_t pinNo, void (*func)(uint32_t u32DioBitmap), INTERRUPTIONEDGES mode) {
    if (mode == DISABLE) {
        return dio_detachCallback(pinNo);
    }

    if (pinNo > 19) return FALSE;

    uint8_t i, freeIndex = 0xff;
    for(i = 0; i < MAX_DIO_INTERRUPT; i++) {
        if (dioCallbackPinNos[i] == pinNo) {
            //処理ルーチンのポインタを上書き登録
            dioCallbackFunctions[i] = func;
            break;
        } else if (freeIndex == 0xff && dioCallbackPinNos[i] == 0xff) {
            //空きインデックスを記憶
            freeIndex = i;
        }
    }
    if (i == MAX_DIO_INTERRUPT) {
        if (freeIndex == 0xff) return FALSE;    //空きが無い

        //処理ルーチンのポインタを新規登録
        dioCallbackFunctions[freeIndex] = func;
        dioCallbackPinNos[freeIndex] = pinNo;
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
bool_t dio_detachCallback(uint8_t pinNo) {
    if (pinNo > 19) return FALSE;

    uint8_t i;
    for(i = 0; i < MAX_DIO_INTERRUPT; i++) {
        if (dioCallbackPinNos[i] == pinNo) break;
    }
    if (i == MAX_DIO_INTERRUPT) return FALSE; //登録されてない

    //処理ルーチンのポインタを削除
    dioCallbackFunctions[i] = NULL;
    dioCallbackPinNos[i] = 0xff;

    //割り込みを無効にする
    vAHI_DioInterruptEnable(0, 1UL << pinNo);
    return TRUE;
}

//DIOピンによるウェイクアップ pinNO=0..19 mode=RISING(立ち上がり)/FALLING(立ち下がり)/DISABLE
//事前にpinModeをINPUTに設定しておくこと（スリープ中はINPUT_PULLUPは使えないので自前で準備すること）
bool_t dio_setWake(uint8_t pinNo, INTERRUPTIONEDGES mode) {
    if (pinNo > 19) return FALSE;

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
tsTimerContext sTimerApp[5];

//vAHI_TimerFineGrainDIOControl()の値を保持（DIOピンを汎用かPWMで使うか）
uint8_t timerFineGrainDIOControlValue;

const uint8_t timerDeviceIndices[5] = {E_AHI_DEVICE_TIMER0,E_AHI_DEVICE_TIMER1,E_AHI_DEVICE_TIMER2,E_AHI_DEVICE_TIMER3,E_AHI_DEVICE_TIMER4};

//PWM
//timerNo = 0..4  duty = 0..1024
//Ct = 16000000 / (2^prescale) / hz が65535を超えないこと
//startTimerで開始します
bool_t timer_attachPWM(uint8_t timerNo, uint16_t hz, uint8_t prescale, uint16_t duty) {
    if (timerNo > 4) return FALSE;

    uint8_t b = 4 << timerNo;
    if ((timerFineGrainDIOControlValue & b) != 0) {
        //DIOを汎用からタイマー用に切り替える
        timerFineGrainDIOControlValue &= b ^ 255;
        vAHI_TimerFineGrainDIOControl(timerFineGrainDIOControlValue);
    }

    memset(&sTimerApp[timerNo], 0, sizeof(tsTimerContext));
    sTimerApp[timerNo].u8Device = timerDeviceIndices[timerNo];
    sTimerApp[timerNo].u16Hz = hz;
    sTimerApp[timerNo].u8PreScale = prescale;
    sTimerApp[timerNo].u16duty = duty;
    sTimerApp[timerNo].bPWMout = TRUE;
    sTimerApp[timerNo].bDisableInt = TRUE; // no interrupt (for CPU save)
    vTimerConfig(&sTimerApp[timerNo]);
    return TRUE;
}

//timerNo = 0..4
//Ct = 16000000 / (2^prescale) / hz が65535を超えないこと
//startTimerで開始します
bool_t timer_attachCallback(uint8_t timerNo, uint16_t hz, uint8_t prescale, void (*func)()) {
    if (timerNo > 4) return FALSE;

    uint8_t b = 4 << timerNo;
    if ((timerFineGrainDIOControlValue & b) == 0) {
        //DIOをタイマー用から汎用に切り替える
        timerFineGrainDIOControlValue |= b;
        vAHI_TimerFineGrainDIOControl(timerFineGrainDIOControlValue);
    }

    memset(&sTimerApp[timerNo], 0, sizeof(tsTimerContext));
    sTimerApp[timerNo].u8Device = timerDeviceIndices[timerNo];
    sTimerApp[timerNo].u16Hz = hz;
    sTimerApp[timerNo].u8PreScale = prescale;

    timerCallbackFunctions[timerNo] = func;

    vTimerConfig(&sTimerApp[timerNo]); // initialize
    return TRUE;
}

//timerNo = 0..4
bool_t timer_detachCallback(uint8_t timerNo) {
    if (timerNo > 4) return FALSE;
    
    vTimerStop(&sTimerApp[timerNo]);
    vTimerDisable(&sTimerApp[timerNo]);

    timerCallbackFunctions[timerNo] = NULL;

    if (sTimerApp[timerNo].bPWMout == TRUE) {
        //PWMに使用した後はDIOをタイマー用から汎用に切り替える
        timerFineGrainDIOControlValue |= 4 << timerNo;
        vAHI_TimerFineGrainDIOControl(timerFineGrainDIOControlValue);
    }
    return TRUE;
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
static void (*adcCallbackFunction)(uint16,int16_t);

//attachAdcCallback()の設定を保持
static bool_t adcIsContinuous;
static bool_t adcIsRange2;
static ADCSOURCES adcLastSource;

//sample = サンプリング数, clock = ADCモジュールのクロック(500KHZが推奨)
//AD変換時間は (サンプリング数 x 3 + 14)クロック 
void adc_enable(ADCSAMPLES sample, ADCCLOCKS clock) {
    if (!bAHI_APRegulatorEnabled()) {
        //アナログ部の電源投入
        vAHI_ApConfigure(E_AHI_AP_REGULATOR_ENABLE, // DISABLE にするとアナログ部の電源断
            E_AHI_AP_INT_ENABLE,    // 割り込み
            sample,                 // サンプル数 2,4,6,8 が選択可能
            clock,                  // 周波数 250K/500K/1M/2M
            E_AHI_AP_INTREF);

        while(!bAHI_APRegulatorEnabled()); // 安定するまで待つ（一瞬）
    }
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
void adc_attachCallback(bool_t continuous, bool_t range2, ADCSOURCES source, void (*func)(uint16_t rawData, int16_t adcResult)) {

    switch (source) {
    case ADC_SOURCE_3:
        dio_pinMode(0, INPUT);
        break;
    case ADC_SOURCE_4:
        dio_pinMode(1, INPUT);
        break;
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

    //if (adcIsContinuous) {
    //    //コンティニュアスモードの場合は停止
    //    vAHI_AdcDisable();
    //}
}


/*
 * Ｉ２Ｃ
 */

static bool_t bSMBusWait(void)
{
	while(bAHI_SiMasterPollTransferInProgress()); /* busy wait */
	if (bAHI_SiMasterPollArbitrationLost() | bAHI_SiMasterCheckRxNack())	{
		/* release bus & abort */
		vAHI_SiMasterSetCmdReg(E_AHI_SI_NO_START_BIT,
						 E_AHI_SI_STOP_BIT,
						 E_AHI_SI_NO_SLAVE_READ,
						 E_AHI_SI_SLAVE_WRITE,
						 E_AHI_SI_SEND_ACK,
						 E_AHI_SI_NO_IRQ_ACK);
        return FALSE;
	}
	return TRUE;
}

//I2Cで指定アドレスにコマンドとデータを書き込む。データが無いときはpu8Data=NULL,u8Length=0とする
bool_t i2c_write(uint8_t u8Address, uint8_t u8Command, const uint8* pu8Data, uint8_t u8Length)
{
	bool_t bCommandSent = FALSE;
	/* Send address with write bit set */
	vAHI_SiMasterWriteSlaveAddr(u8Address, E_AHI_SI_SLAVE_RW_SET);
	vAHI_SiMasterSetCmdReg(E_AHI_SI_START_BIT,
					 E_AHI_SI_NO_STOP_BIT,
					 E_AHI_SI_NO_SLAVE_READ,
					 E_AHI_SI_SLAVE_WRITE,
					 E_AHI_SI_SEND_ACK,
					 E_AHI_SI_NO_IRQ_ACK);//) return FALSE;
	if(!bSMBusWait()) return(FALSE);
	while(bCommandSent == FALSE || u8Length > 0){
		if(!bCommandSent){
			/* Send command byte */
			vAHI_SiMasterWriteData8(u8Command);
			bCommandSent = TRUE;
		} else {
			u8Length--;
			/* Send data byte */
			vAHI_SiMasterWriteData8(*pu8Data++);
		}
		/*
		 * If its the last byte to be sent, send with
		 * stop sequence set
		 */
		if(u8Length == 0){
			vAHI_SiMasterSetCmdReg(E_AHI_SI_NO_START_BIT,
							 E_AHI_SI_STOP_BIT,
							 E_AHI_SI_NO_SLAVE_READ,
							 E_AHI_SI_SLAVE_WRITE,
							 E_AHI_SI_SEND_ACK,
							 E_AHI_SI_NO_IRQ_ACK);//) return FALSE;

		} else {
			vAHI_SiMasterSetCmdReg(E_AHI_SI_NO_START_BIT,
							 E_AHI_SI_NO_STOP_BIT,
							 E_AHI_SI_NO_SLAVE_READ,
							 E_AHI_SI_SLAVE_WRITE,
							 E_AHI_SI_SEND_ACK,
							 E_AHI_SI_NO_IRQ_ACK);//) return FALSE;
		}
		if(!bSMBusWait()) return(FALSE);
	}
	return(TRUE);
}

//I2Cで指定アドレスからデータを読み出します
bool_t i2c_read(uint8_t u8Address, uint8* pu8Data, uint8_t u8Length)
{
	/* Send address with write bit set */
	vAHI_SiMasterWriteSlaveAddr(u8Address, !E_AHI_SI_SLAVE_RW_SET);
	vAHI_SiMasterSetCmdReg(E_AHI_SI_START_BIT,
					 E_AHI_SI_NO_STOP_BIT,
					 E_AHI_SI_NO_SLAVE_READ,
					 E_AHI_SI_SLAVE_WRITE,
					 E_AHI_SI_SEND_ACK,
					 E_AHI_SI_NO_IRQ_ACK);//) return FALSE;
	if(!bSMBusWait()) return(FALSE);
	while(u8Length > 0){
		u8Length--;
		/*
		 * If its the last byte to be sent, send with
		 * stop sequence set
		 */
		if(u8Length == 0){
			vAHI_SiMasterSetCmdReg(E_AHI_SI_NO_START_BIT,
							 E_AHI_SI_STOP_BIT,
							 E_AHI_SI_SLAVE_READ,
							 E_AHI_SI_NO_SLAVE_WRITE,
							 E_AHI_SI_SEND_NACK,
							 E_AHI_SI_NO_IRQ_ACK);//) return FALSE;
		} else {
			vAHI_SiMasterSetCmdReg(E_AHI_SI_NO_START_BIT,
							 E_AHI_SI_NO_STOP_BIT,
							 E_AHI_SI_SLAVE_READ,
							 E_AHI_SI_NO_SLAVE_WRITE,
							 E_AHI_SI_SEND_ACK,
							 E_AHI_SI_NO_IRQ_ACK);//) return FALSE;
		}
		while(bAHI_SiMasterPollTransferInProgress()); /* busy wait */
		*pu8Data++ = u8AHI_SiMasterReadData8();
	}
	return(TRUE);
}

//I2Cで指定アドレスのコマンドからデータを読み出します
bool_t i2c_commandRead(uint8_t u8Address, uint8_t u8Command, uint8* pu8Data, uint8_t u8Length) {
    if (!i2c_write(u8Address, u8Command, NULL, 0)) return FALSE;
    if (!i2c_read(u8Address, pu8Data, u8Length)) return FALSE;
    return TRUE;
}

//I2Cで指定アドレスにコマンドと1バイトのデータを書き込む
bool_t i2c_writeByte(uint8_t u8Address, uint8_t u8Command, uint8_t u8Data) {
    return i2c_write(u8Address, u8Command, &u8Data, 1);
}

//I2Cで指定アドレスからデータを1バイト読み出します。失敗で-1
int16_t i2c_readByte(uint8_t u8Address) {
    uint8_t data;
    if (!i2c_read(u8Address, &data, 1)) return -1;
    return (int16_t)data;
}


/*
 * ＳＰＩ
 */

//SPIマスターを有効にする
//u8NumSlaves=1..3
//使用するピン CLK:DO0(ｼﾙｸC), MISO(in):DO1(ｼﾙｸI), MOSI(out):DIO18
//最初のスレーブはSS(CSB)にDIO19を使うが、1つのスレーブの場合はスレーブのSS(CSB)をGNDに落として常時選択でもOK
bool_t spi_enable(uint8_t u8NumSlaves, bool_t bLsbFirst, SPIMODES u8Mode, SPICLOCKS u8Divider) {
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
        bLsbFirst,                  //bool_t bLsbFirst,
        polarity,                   //bool_t bPolarity,
        phase,                      //bool_t bPhase,
        u8Divider,                  //uint8 u8ClockDivider,
        E_AHI_SPIM_INT_DISABLE,     //bool_t bInterruptEnable,
        E_AHI_SPIM_AUTOSLAVE_DSABL  //bool_t bAutoSlaveSelect
    );
    return TRUE;
}

//slaveNo=1または2のSSピンを変更する。spi_selectSave()に適用
//TRUEでslaveNo=1または2のSSピンが、DIO0->DIO14, DIO1->DIO15に変更できる
bool_t spi_selectPin(uint8_t slaveNo, bool_t bSecondPin) {
    if (slaveNo != 1 && slaveNo != 2) return FALSE;
    vAHI_SpiSelSetLocation(slaveNo, bSecondPin);
    return TRUE;
}

//SPIスレーブ 0..2を選択。その他の値で無選択となる
//具体的には0..2で次のDIO(SSピン)がLになる DIO19,DIO0*,DIO1*  *spi_selectPin()で変更可能
void spi_selectSlave(int8_t slaveNo) {
    if (slaveNo >= 0 && slaveNo <= 2) {
        vAHI_SpiSelect(slaveNo + 1);
    } else {
        vAHI_SpiSelect(0);
    }
}

bool_t spi_write(uint32_t u32Data, uint8_t u8BitLength) {
    if (u8BitLength == 0 || u8BitLength > 32) return FALSE;
    vAHI_SpiStartTransfer(u8BitLength - 1, u32Data);
    while(bAHI_SpiPollBusy());
    return TRUE;
}

void spi_writeByte(uint8_t u8Data) {
    vAHI_SpiStartTransfer(7, u8Data);
    while(bAHI_SpiPollBusy());
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
uint16_t radio_printf(uint32_t u32DestAddr, const char* format, va_list args) {
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
extern void setup(bool_t warmWake, uint32_t dioWakeStatus);
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
    memset(dioCallbackFunctions, 0, sizeof(dioCallbackFunctions));
    memset(dioCallbackPinNos, 0xff, sizeof(dioCallbackPinNos));

#ifdef USE_TIMER
    memset(timerCallbackFunctions, 0, sizeof(timerCallbackFunctions));
    memset(sTimerApp, 0, sizeof(sTimerApp));
    timerFineGrainDIOControlValue = 0xFF;
#endif

    adcCallbackFunction = NULL;

#ifdef USE_RADIO
    radioTxCallbackFunction = NULL;
    radioRxCallbackFunction = NULL;
    u8RadioSeqNo = 0;
    u8NumRadioTx = 0;

    u32SrcAddrPrev = 0;
    u8seqPrev = 255;
#endif

    millisValue = 0;
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

// 電源オンによるスタート
void cbAppColdStart(bool_t bAfterAhiInit)
{
    static uint32_t dioWakeStatus;

	if (!bAfterAhiInit) {

        //起動原因を調査
        if (u8AHI_WakeTimerFiredStatus()) {
            dioWakeStatus = 0;
        } else {
            dioWakeStatus = u32AHI_DioWakeStatus();
        }

        //モジュールを登録
        regMod();
	} else {
        initAppContext();

        //変数や構造体を初期化
        resetVars();

#ifdef SPRINTF_H_
        SPRINTF_vInit128(); //vfPrintf()用
#endif

        //イベントハンドラを登録
        ToCoNet_Event_Register_State_Machine(vProcessEvCore);

        //ユーザーの初期化ルーチンを呼び出す
        setup(FALSE, dioWakeStatus);

#ifdef USE_RADIO
        // MAC 層開始
        ToCoNet_vMacStart();
#endif
	}
}

// スリープからの復帰
void cbAppWarmStart(bool_t bAfterAhiInit)
{
    static uint32_t dioWakeStatus;

	if (!bAfterAhiInit) {

        //起動原因を調査
        if (u8AHI_WakeTimerFiredStatus()) {
            dioWakeStatus = 0;
        } else {
            dioWakeStatus = u32AHI_DioWakeStatus();
        }

        //モジュールを登録
        regMod();
	} else {

        //変数や構造体を初期化
        resetVars();

        //イベントハンドラを登録
        ToCoNet_Event_Register_State_Machine(vProcessEvCore);

        //ユーザーの初期化ルーチンを呼び出す
        setup(TRUE, dioWakeStatus);

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
        //DIO割り込み処理ルーチンの呼び出し
        _C {
            u32ItemBitmap &= 0xfffff;
            if (u32ItemBitmap) {
                uint32_t b = 1;
                uint8_t pinNo;
                for(pinNo = 0; pinNo < 20; pinNo++) {
                    if (u32ItemBitmap & b) { //DIO番号に対応するビットが1になっている
                        uint8_t i;
                        for(i = 0; i < MAX_DIO_INTERRUPT; i++) {
                            if (dioCallbackPinNos[i] == pinNo && dioCallbackFunctions[i] != NULL) {
                                (*dioCallbackFunctions[i])(u32ItemBitmap);
                                break;
                            }                       
                        }
                    }
                    b <<= 1;
                }
            }
        }
        break;

    case E_AHI_DEVICE_ANALOGUE:
        //ADC(完了)割り込みルーチンの呼び出し
        _C {
            if (adcCallbackFunction != NULL) {

                //ADC値の読み出しと単位変換

                uint16_t adcValue = u16AHI_AdcRead(); // 値の読み出し
                int32_t resultValue;

                if (adcLastSource == ADC_SOURCE_VOLT) {
                    //自身の電圧は2/3に分圧された値を測定しているので、1023のとき3.705V
                    //3709 = 3.705V / 1023 * 1000 * 1024
                    resultValue = ((int32_t)adcValue * 3709) >> 10; //[mV]

                } else if (adcLastSource == ADC_SOURCE_TEMP) {
                    //温度センサーのスペック
                    //730mV@25℃, -1.66mV/℃
                    resultValue = (int32_t)adcValue * 1236;     //x1024[mV]
                    resultValue -= 730 * 1024;              //x1024
                    resultValue *= -771;                    //771=(1/1.66)*1280  x1024x1280
                    resultValue = (resultValue >> 17) + 250; //x10[℃]

                } else {
                    if (adcIsRange2) {
                        //1023 = 2.470V, 2.470V * 1024/1023 = 2472
                        resultValue = (2472 * (int32_t)adcValue) >> 10; //[mV]

                    } else {
                        //1023 = 1.235V, 1.235V * 1024/1023 = 1236
                        resultValue = (1236 * (int32_t)adcValue) >> 10; //[mV]
                    }
                }

                //ユーザー処理ルーチンを呼び出す
                (*adcCallbackFunction)(adcValue, (int16_t)resultValue);
            }
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

    if (u32DeviceId == E_AHI_DEVICE_TICK_TIMER) {
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
