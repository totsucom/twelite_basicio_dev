#include "basicio.h"

#define SEND_ADDR 0x810e2e97 //送信先モジュールアドレス（moduleAddress()値）

int count = 0;

//送信完了で呼ばれる
void radioSent(uint8_t u8CbId, bool_t bSuccess) {
    //送信結果がわかる
    serial_printf("\r\nsent id=%d %s", (int)u8CbId, bSuccess ? "success" : "failue");

    dio_write(DIO_MONOSTICK_LED, LOW);
    count = 20;
}

void setup(bool warmWake, uint32 dioWakeStatus) {
    //シリアル0を初期化する
    serial_init(SERIAL_BAUD_115200);

    //送信完了コールバックを登録
    radio_attachTxCallback(radioSent);

    dio_pinMode(DIO_MONOSTICK_LED, OUTPUT);
    dio_write(DIO_MONOSTICK_LED, HIGH);
}

void loop(EVENTS event) {
    static int n = 0;

    if (event == EVENT_START_UP) {
        serial_printf("\r\nModule address: %u", moduleAddress());
    }
    else if (event == EVENT_TICK_TIMER) {
        if (count > 0) {
            if (--count == 0) {
                dio_write(DIO_MONOSTICK_LED, HIGH);
            }
        }
    }
    else if (event == EVENT_TICK_SECOND) {
        if (--n <= 0) {
            //相手に乱数文字列を送信
            uint16_t id = radio_printf(SEND_ADDR, "%u", rand());
            //送信関数からidを取得できるので、コールバック関数で結果をトレースできる
            serial_printf("\r\nsend id=%d", (int)id);

            n = 3;
        }
    }
}

