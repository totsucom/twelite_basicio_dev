/*
 * basicio.h
 * バージョン 1.00
 * 2019/10/07 by T.Maeoka
 */



/*
 * 機能のON/OFF
 */
#define USE_DIO           //ＤＩＯ関数を使用する
//#define USE_TIMER         //タイマー系（タイマー、ＰＷＭなど）関数を使用する
//#define USE_SERIAL        //シリアル0関数を使用する
//#define USE_SERIAL1       //シリアル1関数を使用する
//#define USE_ADC           //ADC関数を使用する
//#define USE_COMP          //コンパレータ関数を使用する
//#define USE_PC            //パルスカウンタ関数を使用する
//#define USE_I2C           //I2C通信を使用する
//#define USE_SPI           //SPI通信を使用する
//#define USE_RADIO         //無線通信を使用する(送受信)
//#define USE_RADIO_TXONLY  //無線通信を使用する(送信のみ)
//#define USE_EEPROM        //EEPROMを使用する
//#define USE_FLASH         //フラッシュメモリを使用する
//#define USE_PBUTIL        //プッシュボタン補助関数 pb_XXX を使用する
//#define USE_SBUTIL        //文字列バッファ補助関数 sb_XXX を使用する





/* プッシュボタン補助関数 オプション */
//#define PB_JUDGE_COUNT 5          //チャタリング防止のためのディレイ。時間は、TICK_TIMER(通常4ms) x PB_JUDGE_COUNT (規定値:5 つまり20ms)


/* 文字列バッファ補助関数 オプション */
//#define SB_BUFFER_SIZE  128       //sb_printf()等で使用するバッファサイズを指定します 32/64/128/256/512/1024のいずれかです。（規定値: 128）


/* デジタルIO オプション */
//#define MAX_DIO_INTERRUPT 2       //DIO割り込みルーチンが登録できる最大のピン数(最大20(DIOピンは20本))(規定値:2)


/* シリアル0 オプション */
//#define SERIAL_HW_FLOW_CONTROL   //シリアル0にハードウェアフロー制御(CTS,RTS)を使用する(内部的にTimer4を使用)
//#define SERIAL_TX_BUFFER_SIZE 96 //送信バッファの大きさ(規定値:96)
//#define SERIAL_RX_BUFFER_SIZE 32 //受信バッファの大きさ(規定値:32)


/* シリアル1 オプション */
//#define SERIAL1_TX_BUFFER_SIZE 96 //送信バッファの大きさ(規定値:96)
//#define SERIAL1_RX_BUFFER_SIZE 32 //受信バッファの大きさ(規定値:32)


/* 無線通信 設定（USE_RADIO宣言時は必須）*/
#define APP_ID      0x67720103      //無線通信するうえで、そのグループは同じ値APP_IDを設定すること
                                    //有効なAPP_IDの範囲。0xHHHHLLLLの場合、HHHH,LLL共に0x0001～0x7FFF
#define CHANNEL     18              //無線通信するうえで、そのグループは同じ値CHANNELを設定すること

/* 無線通信 オプション */
//#define TX_POWER    3             //無線送信出力の設定。 3:最大 2: -11.5db 1: -23db 0:-34.5db となる (規定値:3)
//#define TX_RETRY    2             //相手にデータが届かない場合、リトライ送信する回数。0～7 (規定値:2)
                                    //ブロードキャスト送信の場合は常に(TX_RETRY+1)回の送信が実行される

//送信キューのサイズを決定します。SMALLは送信用で 3ヶ、MIDは6ヶ、BIGは20ヶのキューを確保します。
//パケット分割を行うような一度に多くのパケットを連続的に送信する場合はBIGを指定します。
//１ヶあたり約128バイトのメモリを消費し、未定義時は MID となります。
//#define ToCoNet_USE_MOD_TXRXQUEUE_SMALL
//#define ToCoNet_USE_MOD_TXRXQUEUE_MID //デフォ
//#define ToCoNet_USE_MOD_TXRXQUEUE_BIG


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

