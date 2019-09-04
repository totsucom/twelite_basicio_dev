

/* タイマー／ＰＷＭ 機能 */
//#define USE_TIMER


/* シリアル0 機能 */
#define USE_SERIAL  //シリアル0を使用する

/* シリアル0 オプション */
//#define SERIAL_TX_BUFFER_SIZE 96 //送信バッファの大きさ(規定値:96)
//#define SERIAL_RX_BUFFER_SIZE 32 //受信バッファの大きさ(規定値:32)



/* シリアル1 機能 */
//#define USE_SERIAL1 //シリアル1を使用する（シリアル0とシリアル1は併用できます）

/* シリアル1 オプション */
//#define SERIAL1_TX_BUFFER_SIZE 96 //送信バッファの大きさ(規定値:96)
//#define SERIAL1_RX_BUFFER_SIZE 32 //受信バッファの大きさ(規定値:32)



/* 無線通信 機能 */
#define USE_RADIO   //無線通信機能を使う

/* 無線通信 設定（USE_RADIO宣言時は必須）*/
#define APP_ID      0x67720103  //無線通信するうえで、そのグループは同じ値APP_IDを設定すること
                                //有効なAPP_IDの範囲。0xHHHHLLLLの場合、HHHH,LLL共に0x0001～0x7FFF
#define CHANNEL     18          //無線通信するうえで、そのグループは同じ値CHANNELを設定すること
#define RX_ON_IDLE  TRUE        //無線受信を行う場合はTRUEにする

/* 無線通信 オプション */
//#define TX_POWER    3         //無線送信出力の設定。 3:最大 2: -11.5db 2: -23db 0:-34.5db となる (規定値は 3)

//送信キューのサイズを決定します。SMALLは送信用で 3ヶ、MIDは6ヶ、BIGは20ヶのキューを確保します。
//パケット分割を行うような一度に多くのパケットを連続的に送信する場合はBIGを指定します。
//１ヶあたり約128バイトのメモリを消費し、未定義時は MID となります。
#define ToCoNet_USE_MOD_TXRXQUEUE_SMALL
//#define ToCoNet_USE_MOD_TXRXQUEUE_MID
//#define ToCoNet_USE_MOD_TXRXQUEUE_BIG


/* EEPROM 機能 */
//#define USE_EEPROM


/* FLASHメモリ 機能 */
//#define USE_FLASH


/* システム 設定 */

/*
 * 通常稼働時のCPUクロック。3:32MHz, 2:16Mhz, 1:8Mhz, 0:4Mhz を指定する(規定値は 2:16MHz)
 * 16MHz以外のクロックを設定すると通信やタイマー、PWM、millisなどすべてのタイミングが変わってくると思われる
 * ※basicioでは変更を考慮していません
 */
//#define CPU_CLOCK   2   //=16[MHz]

/*
 * loop()のEVENT_TICK_TIMERイベント周期を決定
 * 規定値は250=4ms, 1000で割り切れる値にすること。事実上 1000, 500, 250, 200, 100 のみ
 */
//#define	TICK_COUNT  250 // 1000/250=4[ms]

