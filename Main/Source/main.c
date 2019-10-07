/*
 * basicio.h でLチカ
 */

#include "basicio.h"

#define LED 16

// 起動時や起床時に呼ばれる関数
void setup(bool_t warmWake, uint32_t bitmapWakeStatus) {
    dio_pinMode(LED, OUTPUT);
}

// setup()後、３種類のイベントで呼ばれるループ関数
void loop(EVENTS event) {
    static bool_t b = TRUE;

    if (event == EVENT_START_UP) {
        // 最初に呼ばれる

    } else if (event == EVENT_TICK_SECOND) {
        // 1秒毎に呼ばれる

        // 1秒毎に点滅を繰り返す
        // モノスティックの場合はLOW(FALSE)で点灯
        dio_write(LED, b);
        b = !b;
    } else if (event == EVENT_TICK_TIMER) {
        // 4ミリ秒毎(デフォルト)に呼ばれる

    }
}
