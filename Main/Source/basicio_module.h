/*
 * basicio.h
 * バージョン 3.1
 * 2020/3/5 totsucom
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
//#define USE_I2C           //I2C(マスター)通信を使用する
//#define USE_I2CS          //I2C(スレーブ)通信を使用する
//#define USE_SPI           //SPI(マスター)通信を使用する
//#define USE_RADIO         //無線通信を使用する(送受信)
//#define USE_EEPROM        //EEPROMを使用する
//#define USE_FLASH         //フラッシュメモリを使用する
//#define USE_PBUTIL        //プッシュボタン補助関数 pb_XXX を使用する
//#define USE_SBUTIL        //文字列バッファ補助関数 sb_XXX を使用する




/* プッシュボタン補助関数 オプション */
//#define PB_JUDGE_COUNT 5          //チャタリング防止のためのディレイ。時間は、TICK_TIMER(通常4ms) x PB_JUDGE_COUNT 。1以上を設定(規定値:5 つまり20ms)


/* 文字列バッファ補助関数 オプション */
//#define SB_BUFFER_SIZE  128       //sb_XXX()で使用するバッファサイズを指定します。(規定値: 128）
                                    //終端'\0'のため、実際には SB_BUFFER_SIZE+1 のバッファが確保されます

/* デジタルIO オプション */
//#define MAX_DIO_INTERRUPT_FUNCS 5 //DIO割り込みルーチンが登録できる最大数。1～20(規定値:5)


/* シリアル0 オプション */
//#define SERIAL_TX_BUFFER_SIZE 96 //送信バッファの大きさ。16～2047(規定値:96)
//#define SERIAL_RX_BUFFER_SIZE 32 //受信バッファの大きさ。16～2047(規定値:32)


/* シリアル1 オプション */
//#define SERIAL1_TX_BUFFER_SIZE 96 //送信バッファの大きさ。16～2047(規定値:96)
//#define SERIAL1_RX_BUFFER_SIZE 32 //受信バッファの大きさ。16～2047(規定値:32)


/* I2Cスレーブ オプション */
//#define I2CS_MW_BUFFER_SIZE   20  //マスターの書き込むデータを受けるバッファ。1～255(規定値:20)
//#define I2CS_MR_DATA_NOT_READY 0  //マスターの読み込みが行われるときにデータが準備されなかった場合に返す値。0～255(規定値:0)
//#define I2CS_MR_OUT_OF_RANGE  255 //マスターの読み込みが行われるとき、準備したデータの範囲外を参照した場合に返す値。0～255(規定値:255)


/* 無線通信 オプション */

//送信キューのサイズを決定します。SMALLは送信用で 3ヶ、MIDは6ヶ、BIGは20ヶのキューを確保します。
//パケット分割を行うような一度に多くのパケットを連続的に送信する場合はBIGを指定します。
//１ヶあたり約128バイトのメモリを消費し、未定義時は MID となります。
//#define ToCoNet_USE_MOD_TXRXQUEUE_SMALL
#define ToCoNet_USE_MOD_TXRXQUEUE_MID //デフォ
//#define ToCoNet_USE_MOD_TXRXQUEUE_BIG


/* システム 設定 */

/*
 * 通常稼働時のCPUクロック。3:32MHz, 2:16Mhz, 1:8Mhz, 0:4Mhz を指定する(規定値は 2:16MHz)
 * ※basicioでは変更を考慮していません
 */
//#define CPU_CLOCK   2   //=16[MHz]

/*
 * loop()のEVENT_TICK_TIMERイベント周期を決定
 * 規定値は250=4ms, 1000を割り切れる値にすること。事実上 1ms:1000, 2ms:500, 4ms:250, 5ms:200, 10ms:100 のみ
 */
//#define	TICK_COUNT  250 // 4ms

