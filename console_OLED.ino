/*----------------------------

    console_OLED

    2026 by Kenichi Iwata

----------------------------*/

/*----------------------------

 ATtiny85のクロックを16MHzにすること

 Arduino IDE > TOOL > CLOCK > Internal 16MHz

----------------------------*/

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <SoftwareSerial.h>
#include <U8x8lib.h>

#define OLED_SDA 0   // PB0=pin5
#define OLED_SCL 2   // PB2=pin7
#define RX_PIN  1   // PB1=pin6 USB-シリアル変換モジュールのTXへ
#define TX_PIN  4   // PB4=pin3 USB-シリアル変換モジュールのRXへ(接続しない)
#define SDA_PIN 5   // pin5=PB0
#define SCL_PIN 7   // pin7=PB2

SoftwareSerial mySerial(RX_PIN, TX_PIN);

U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(
  OLED_SCL,
  OLED_SDA,
  U8X8_PIN_NONE
);

#define LINE_BUF_SIZE 16
#define ROW 3
#define STATE_NORMAL 0
#define STATE_X 3
#define STATE_Y 4
#define STATE_I 5

uint8_t cols = u8x8.getCols();
uint8_t rows = u8x8.getRows();

char windmill[] = {'-', '\\', '|', '/'};
uint8_t cell = 0;
bool spin = false;
volatile bool eventTriggered = false;

uint8_t state = STATE_NORMAL;
uint8_t pos_x = 0, pos_y = 0;

char lineBuf[LINE_BUF_SIZE + 1];
uint8_t linePos = 0;


void setup() {
  mySerial.begin(4800);

  u8x8.begin();
  u8x8.setBusClock(400000);
  u8x8.setPowerSave(0);
  u8x8.setFont(u8x8_font_torussansbold8_r);

  cli(); // 割り込み禁止

  // Timer1停止
  TCCR1 = 0;
  GTCCR = 0;
  TCNT1 = 0;

  // CTCモード
  TCCR1 |= _BV(CTC1);

  // OCR1A一致で割り込み
  OCR1A = 60;

  // TOP値
  OCR1C = 60;

  // 比較一致A割り込み許可
  TIMSK |= _BV(OCIE1A);

  // プリスケーラ 16384
  TCCR1 |= _BV(CS13)
        |  _BV(CS12)
        |  _BV(CS11)
        |  _BV(CS10);

  sei(); // 割り込み許可
}


void loop() {
  while (mySerial.available()) {
    uint8_t c = mySerial.read();
    
    if (state == STATE_X) {
      pos_x = c % cols;
      state = STATE_Y;
    } else if (state == STATE_Y) {
      pos_y = c % rows;
      state = STATE_I;
    } else if (state == STATE_I) {
      u8x8.setInverseFont(c & 1);
      state = STATE_NORMAL;
    } else {
      if (c >= 0x20) {
        if (linePos < LINE_BUF_SIZE) {
          lineBuf[linePos++] = c;
        }
      } else {
        if (c == 0x01) {
          spin = true; // 風車を表示する
        } else if (c == 0x02) {
          spin = false; // 風車を非表示にする
          u8x8.drawGlyph(0, 0, '*');
        } else if (c == '\n') {
          uint8_t i;

          lineBuf[linePos] = '\0';
          u8x8.drawString(pos_x, pos_y, lineBuf);
          
          for (i = linePos; i < cols; i++) {
            u8x8.drawGlyph(i, pos_y, ' '); // pos_x 以降を空白で埋める
          }

          linePos = pos_x = 0;
          u8x8.setInverseFont(0);

        } else if (c == 0x0c) {
          u8x8.clear(); // 画面を消去する
        } else {
          state = c;
        }
      }
    }
  }
  
  // タイマー割り込みで風車を回転させる
  if (!mySerial.available() && eventTriggered) {
    eventTriggered = false;

    if (spin) {
      u8x8.drawGlyph(0, 0, windmill[cell++]);
      cell &= 3;
    }
  }
}


// タイマー1の比較一致A 割り込みサービスルーチン (ISR)
// 250msごとに自動で呼び出されます
ISR(TIMER1_COMPA_vect) {
  eventTriggered = true; // メインループに処理を促すフラグを立てる
}


