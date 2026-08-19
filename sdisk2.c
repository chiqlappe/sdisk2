/*------------------------------------

  SDISK II LCD Farmware (1 of 3)
  
  2010.11.11 by Koichi Nishida

------------------------------------*/

/*
hardware information:

use ATMEGA328P AVR.
connect 27MHz (overclock...) crystal to the AVR.
3.3V power.

fuse setting :
  LOW: 11011110
  HIGH: default

connection:
  D0: DO (SD card)
  D1: USART TX to the LCD unit
  D2: WRITE REQUEST (APPLE II disk IF, pull up with 10K ohm)
  D3: EJECT switch (LOW if SD card is inserted)
  D4: DI (SD card)
  D5: CLK (SD card)
  D6: yellow LED (through 330 ohm, on when HIGH)
  D7: CS (SD card)
  B0: PHASE-0 (APPLE II disk IF)
  B1: PHASE-1 (APPLE II disk IF)
  B2: PHASE-2 (APPLE II disk IF)
  B3: PHASE-3 (APPLE II disk IF)
  B4: red LED (through 330 ohm, on when HIGH)
  B5: ENTER switch (LOW when pushed)
  B6-B7: connect to the crystal
  C0: DRIVE ENABLE (APPLE II disk IF)
  C1: READ PULSE (APPLE II disk IF through 74HC125 3state)
  C2: WRITE (APPLE II disk IF)
  C3: WRITE PROTECT (APPLE II disk IF through 74HC125 3state)
  C4: UP switch (LOW when pushed)
  C5: DOWN switch (LOW when pushed)
  C6: NC
  
  Note that the enable input of the 3state buffer 74HC125,
  should be connected with DRIVE ENABLE.
*/

/*
This is a part of the firmware for DISK II emulator by Nishida Radio.

Copyright (C) 2010 Koichi NISHIDA
email to Koichi NISHIDA: tulip-house@msf.biglobe.ne.jp

This program is free software; you can redistribute it and/or modify it
under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>.
*/



/*------------------------------------

    Modified version 1.1.0

    2026.08.19 by Kenichi Iwata

    Based on SDISK II LCD Firmware
    by Koichi Nishida.

------------------------------------*/

/*
hardware information:

use ATMEGA328P AVR.
connect 27MHz (overclock...) crystal to the AVR.
5V power.

fuse setting :
  HIGH: default
  LOW: 11101111

  CKDIV8  1
  CKOUT   1
  SUT1    0
  SUT0    0
  CKSEL3  1
  CKSEL2  1
  CKSEL1  1
  CKSEL0  1

connection:
  B0: PHASE-0 (APPLE II disk IF)
  B1: PHASE-1 (APPLE II disk IF)
  B2: PHASE-2 (APPLE II disk IF)
  B3: PHASE-3 (APPLE II disk IF)
  B4: red LED (through 330 ohm, on when HIGH)
  B5: ENTER switch (LOW when pushed)
  B6-B7: connect to the crystal

  C0: DRIVE ENABLE (APPLE II disk IF)
  C1: READ PULSE (APPLE II disk IF through 74HC125 3state)
  C2: WRITE (APPLE II disk IF)
  C3: WRITE PROTECT (APPLE II disk IF through 74HC125 3state)
  C4: UP switch (LOW when pushed)
  C5: DOWN switch (LOW when pushed)
  C6: *CHANGED* RESET switch (LOW when pushed)
  
  D0: DO (SD card)
  D1: USART TX to the LCD unit
  D2: WRITE REQUEST (APPLE II disk IF, pull up with 10K ohm) INT0
  D3: EJECT switch (LOW if SD card is inserted)
  D4: DI (SD card)
  D5: CLK (SD card)
  D6: yellow LED (through 330 ohm, on when HIGH)
  D7: ~CS (SD card)

  Note that the enable input of the 3state buffer 74LS125,
  should be connected with DRIVE ENABLE.
*/

/*
Modified by Kenichi Iwata, 2026.

Major modifications include:
- FAT16 and long filename support
- SD card access improvements
- memory usage optimizations
- display-related modifications
- hardware support changes

See the Git history for details.

SPDX-License-Identifier: GPL-3.0-only
*/


#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#define WAIT 1

// バッファ数(オリジナル版から変更)
#define BUF_COUNT 4

// バッファサイズ
#define BUF_SIZE 350

#define FAT_DSK_ELEMS 18

#define FAT_NIC_ELEMS 35

#define nop() __asm__ __volatile__ ("nop")

// ATtiny85へのシリアル送信用
#define USART_BAUD 4800

// 文字列の格納にFLASHを使用する命令のエイリアス
#define consoleFlash(s) console_P(PSTR(s))

// SDカードの基本ブロック長
#define DEFAULT_BLOCK_SIZE 512

// FAT16 ディレクトリエントリのサイズ
#define DIR_ENTRY_SIZE 32

// FAT16 セクタサイズ
#define SECTOR_SIZE 512

// FAT16 FAT数
#define NUM_FATS 2

// FAT16 FATエントリサイズ
#define FAT_ENTRY_SIZE 2

// FAT16 ルートディレクトリに含まれるディレクトリエントリの数
#define ROOT_ENTRY_COUNT 512

// FAT16 ファイル名ボディサイズ
#define DIR_BODY_SIZE 8

// FAT16 ファイル名拡張子サイズ
#define DIR_EXT_SIZE 3

// FAT16 ファイル名(ボディ+拡張子)サイズ
#define DIR_NAME_SIZE DIR_BODY_SIZE+DIR_EXT_SIZE

// FAT16 ディレクトリエントリ構造体の短いファイル名のボディと拡張子へのオフセット値
#define DIR_NAME_OFST 0

// FAT16 ディレクトリエントリ構造体のファイルアトリビュートへのオフセット値
#define DIR_ATRB_OFST 11

// FAT16 ディレクトリエントリ構造体のファイルが最後に変更された時刻へのオフセット値
#define DIR_WRT_TIME_OFST 22

// FAT16 ディレクトリエントリ構造体のファイルデータの先頭クラスタ番号の下位16ビットへのオフセット値
#define DIR_1ST_CLST_OFST 26

// FAT16 LFNの最大長(プログラム仕様) 本体(16)+拡張子(3)+ドット(1)
#define DIR_LFN_SIZE 20

// DSK トラック総数
#define DSK_MAX_TRACKS 35

// DSK 物理トラック総数(クォーター・トラック)
#define DSK_MAX_PH_TRACKS DSK_MAX_TRACKS * 4

// DSK セクタサイズ
#define DSK_SECTOR_SIZE 256

// DSK 1トラックあたりのセクタ数
#define DSK_SCTR_PER_TRK 16

// GCR 下位2ビット格納領域
#define GCR_LBIT_SIZE 86

// GCR 上位6ビット格納領域
#define GCR_HBIT_SIZE 256

// GCR アドレスマーカー
#define GCR_ADRS_MARKER 0x96

// GCR データマーカー
#define GCR_DATA_MARKER 0xad

// GCR データエリア全体のサイズ(プロローグ、エンコード済みデータ、チェックサム、エピローグを含む)
#define GCR_DATA_AREA_SIZE 349

// SDカードエラー
#define SDERR_NONE     0
#define SDERR_CMD24    1
#define SDERR_DATA     2
#define SDERR_BUSY     3
#define SDERR_EJECT    4
#define SDERR_NIC      5
#define SDERR_INIT     6

// ロングファイルネーム関連
#define ATTR_LFN 0x0f
#define LFN_LAST_ENTRY 0x40

// コンソール関連
#define CENTER_Y 4

// メニュー関連
#define SHORT_PUSH 100
#define LONG_PUSH 1000000
#define MENU_ROWS 7
#define MENU_CENTER 3


// C prototypes

// cancel read
void cancelRead(void);
// write a byte data to the SD card
void writeByteSlow(unsigned char c);
void writeByteFast(unsigned char c);
// read data from the SD card
unsigned char readByteSlow(void);
unsigned char readByteFast(void);
// wait until finish a command
unsigned char waitFinish(void);
// issue SD card command slowly without getting response
void cmd_(unsigned char cmd, unsigned long adr);
// issue SD card command fast and wait normal response
unsigned char cmdFast(unsigned char cmd, unsigned long adr);
// get command response slowly from the SD card
unsigned char getRespSlow(void);
// get command response fast from the SD card
//unsigned char getRespFast(void);
// issue command 17 and get ready for reading
void cmd17Fast(unsigned long adr);
// get a SFN from a directory entry
void getSFN(unsigned short entryNo, char *name);
// find a file whose extension is targExt, and whose name is targName if withName is true
int findExt(const char *targExt, unsigned char *protectOut, char *targName, unsigned char withName);
// prepare the FAT table on memory
void prepareFat(unsigned short entryNo, unsigned short *fat, unsigned short len, unsigned char fatNum, unsigned char fatElemNum);
// duplicate FAT for FAT16
void duplicateFat(void);
// write to the SD cart one by one
void writeSD(unsigned long adr, unsigned char *data, unsigned short len);
// create a NIC image file
int createFile(char *name, char *ext, char *lfn, unsigned short sectNum);
// translate a DSK image into a NIC image
void dsk2Nic(void);
// make file name list
unsigned short makeFileNameList(unsigned short *list, char *targExt);
// choose a NIC file from a NIC file name list
unsigned char chooseANicFile(void *tempBuff, unsigned char btfExists, char *filebase);
// initialization called from check_eject
unsigned char init(void);
//
void prepareFiles(unsigned char choose);
// called when the SD card is inserted or removed
void check_eject(void);
// buffer clear
void buffClear(void);
// output a character on OLED through USART
void outCharUsart(unsigned char c);
// write data back to a NIC image 
void writeBack(void);
void writeBackSub(void);
void writeBackSub2(unsigned char bn, unsigned char sc, unsigned char track);

// assembler functions
// see sub.S file
void wait5(unsigned short time);


// SDカード情報
unsigned char sdHighCapacity;  // 0:SDSC(byte addressing), 1:SDHC/SDXC(block addressing)

unsigned long bpbAddr, rootAddr;
unsigned long fatAddr;            // the beginning of FAT
unsigned short fileFatTop;
unsigned long userAddr;           // the beginning of user data
unsigned short fatNic[FAT_NIC_ELEMS];
unsigned char prevFatNumDsk, prevFatNumNic;
unsigned short nicEntryNo, dskEntryNo, btfEntryNo;
unsigned short maxCluster;        // FAT探索用

unsigned char sectorsPerCluster;  // BPB+13 1クラスタあたりのセクタ数
unsigned short reservedSectors;   // BPB+14~15  予約セクタ数
//unsigned char numFats;          // BPB+16     定数 NUM_FATS で代用する
//unsigned short rootEntryCount;  // BPB+17~18  定数 ROOT_ENTRY_COUNT で代用する
unsigned long totalSectors;       // BPB+19~20 または BPB+32~35 の総セクタ数
unsigned short sectorsPerFat;     // BPB+22~23  FAT 1個あたりのセクタ数
unsigned char sectorsPerCluster2;

// SDカードエラー
volatile unsigned char sdError;   // エラーコード格納用
volatile unsigned char sdWriteError; // SD書き込みエラー発生フラグ

// DISK II status
unsigned char ph_track;   // 0~139 物理トラック(1トラックを4分割したクォータートラック)の番号 140=35*4
unsigned char readPulse;  // sub.S で使用
unsigned short bitbyte;   // sub.S で使用 0~(8*512-1) = 4095
unsigned char sector;     // sub.S で使用 0~15
unsigned char prepare;    // sub.S で使用 次のセクタ準備待ち状態なら 1 になる
unsigned char mounted;    // NICファイルがマウントされたら 1 になる
unsigned char magState;
unsigned char protect;    // ディスクがプロテクト状態なら bit3 が立つ
unsigned char formatting;
const unsigned char volume = 0xfe; // デフォルトのボリューム番号(254)

// write data buffer
unsigned char writeData[BUF_COUNT][BUF_SIZE];         // 汎用バッファ (4*350=1400バイト)
unsigned char sectors[BUF_COUNT], tracks[BUF_COUNT];  // バッファに書き込まれているデータに対応するセクタ番号とトラック番号
unsigned char buffNum;
unsigned char *writePtr;  // sub.S で使用

// ステッピングモーター移動量テーブル
// 符号付き表現では {0,-1,-2,-3,0,3,2,1}
const uint8_t stepper_table[8] PROGMEM = {0b00000000, 0b11111111, 0b11111110, 0b11111101,
                                          0b00000000, 0b00000011, 0b00000010, 0b00000001};


// GCR変換テーブル
const uint8_t encTable[] PROGMEM = {
  0x96,0x97,0x9A,0x9B,0x9D,0x9E,0x9F,0xA6,
  0xA7,0xAB,0xAC,0xAD,0xAE,0xAF,0xB2,0xB3,
  0xB4,0xB5,0xB6,0xB7,0xB9,0xBA,0xBB,0xBC,
  0xBD,0xBE,0xBF,0xCB,0xCD,0xCE,0xCF,0xD3,
  0xD6,0xD7,0xD9,0xDA,0xDB,0xDC,0xDD,0xDE,
  0xDF,0xE5,0xE6,0xE7,0xE9,0xEA,0xEB,0xEC,
  0xED,0xEE,0xEF,0xF2,0xF3,0xF4,0xF5,0xF6,
  0xF7,0xF9,0xFA,0xFB,0xFC,0xFD,0xFE,0xFF
};

// 論理セクタ番号(ソフトセクタ番号)を物理セクタ番号に変換するテーブル "2 descending skew"
const uint8_t physicalSector[] PROGMEM = {0, 13, 11, 9, 7, 5, 3, 1, 14, 12, 10, 8, 6, 4, 2, 15};

// for bit flip
const uint8_t FlipBit[]  PROGMEM = {0,  2,  1,  3};
const uint8_t FlipBit1[] PROGMEM = {0,  2,  1,  3}; // 0b------00, 0b------10, 0b------01, 0b------11
const uint8_t FlipBit2[] PROGMEM = {0,  8,  4, 12}; // 0b----00--, 0b----10--, 0b----01--, 0b----11--
const uint8_t FlipBit3[] PROGMEM = {0, 32, 16, 48}; // 0b--00----, 0b--10----, 0b--01----, 0b--11----

// LFN文字の格納位置
static const unsigned char lfnCharOffset[13] = {
   1,  3,  5,  7,  9,
  14, 16, 18, 20, 22, 24,
  28, 30
};


// 追加された関数
static void console(const char *str);
static void console_P(PGM_P str);
static void consoleDec(unsigned char val);
static void seekSD(unsigned long adr);
static void discard(unsigned short num);
static void readRootEntry(unsigned short entryNo, unsigned char *entry);
static unsigned short readLFNChar(const unsigned char *entry, unsigned char index);
static unsigned char getLFN(unsigned short entryNo,char *name,unsigned char nameSize);
static void removeExtension(char *name);
static void dispFileName(unsigned short entryNo);
static void allDsk2Nic(void);
static void busy(unsigned char mode);
static unsigned char lfnChecksum(const unsigned char *sfn);
static void changeLfnExt(char *lfn);

static unsigned short getMaxCluster(
  unsigned long totalSectors,
  unsigned short reservedSectors,
  unsigned short sectorsPerFat,
  unsigned char sectorsPerCluster);

static unsigned char cmd24Fast(unsigned long adr);
static void writeSector(unsigned long adr, unsigned char *buf);
static unsigned char isTargetFile(unsigned char *entry, const char *ext);
static void finishWrite(void);
static void informCardError(const unsigned char err);
static void locate(const uint8_t x, const uint8_t y, const uint8_t inv);
static void cls(void);

//static void consoleHex(unsigned char val);
//static void consoleHex16(unsigned short val);
//static void consoleHex32(unsigned long val);

/**
 * @brief キャラクタコードをUARTで送信する
 *  UCSR0A : USART制御・ステータスレジスタ0A
 *  UDRE0  : USART送信データレジスタ空きフラグ
 *  UDR0   : USART送受信バッファ
 * @param c UARTで送信するキャラクタコード
 */
void outCharUsart(unsigned char c)
{
  while (!(UCSR0A & (1 << UDRE0))) {
    nop(); // 送信バッファが空になり次のデータを書き込める状態になるまで待つ
  }
  UDR0 = c; // 送信バッファにcのデータを書き込んで送信開始
}


/**
 * @brief 汎用バッファを0クリアする
 * バッファ関連のsectorsとtracksを初期化する
 */
void buffClear(void)
{
  unsigned char i;
  unsigned short j;
  
  for (i = 0; i < BUF_COUNT; i++) {
    for (j = 0; j < BUF_SIZE; j++) {
      writeData[i][j] = 0;
    }
  }

  for (i = 0; i < BUF_COUNT; i++) {
    sectors[i] = tracks[i] = 0xff; // 0xffは未使用を示す
  }
}


/**
 * @brief SDカードからのデータ受信を中止する
 */
void cancelRead(void)
{
  unsigned short i;
  if (bitbyte < (402 * 8)) {
    PORTD = 0b00010000; // CLK=L
    for (i = bitbyte; i < (514 * 8); i++) { // 514 = DEFAULT_BLOCK_SIZE + CRC
      if (bit_is_set(PIND, 3)) return;
      PORTD = 0b00110000; // CLK=H
      PORTD = 0b00010000; // CLK=L
    }
    bitbyte = 402 * 8;
  }
}


/**
 * @brief SDカードにSPI形式で1バイトを低速送信する
 * @param c バイトデータ
 * @note コマンド送信開始からレスポンス、データ転送の間はCSをアサート(Low)に保持しておく
 */
void writeByteSlow(unsigned char c)
{
  unsigned char d;
  for (d = 0b10000000; d; d >>= 1) {
    if (c & d) {
      PORTD = 0b00010000; // DI=H, CLK=L, CS=L
      wait5(WAIT);
      PORTD = 0b00110000; // DI=H, CLK=H, CS=L
    } else {
      PORTD = 0b00000000; // DI=L, CLK=L, CS=L
      wait5(WAIT);
      PORTD = 0b00100000; // DI=L, CLK=H, CS=L
    }
    wait5(WAIT);
  }
  PORTD = 0b00000000; // DI=L, CLK=L, CS=L
}


/**
 * @brief SDカードにSPI形式で1バイトを送信する
 *  MSBからLSBにかけて8ビット分をSDへ書き込む
 * @param c バイトデータ
 */
void writeByteFast(unsigned char c)
{
  unsigned char d;
  for (d = 0b10000000; d; d >>= 1) {
    if (c & d) {
      PORTD = 0b00010000; // DI=H, CLK=L, CS=L
      PORTD = 0b00110000; // DI=H, CLK=H, CS=L
    } else {
      PORTD = 0b00000000; // DI=L, CLK=L, CS=L
      PORTD = 0b00100000; // DI=L, CLK=H, CS=L
    }
  }
  PORTD = 0b00000000; // DI=L, CLK=L, CS=L
}


/**
 * @brief SDカードからSPI形式で1バイトを低速受信する
 * @return 受信した1バイト
 */
unsigned char readByteSlow(void)
{
  unsigned char c = 0;
  volatile unsigned char i;

  PORTD = 0b00010000; // DI=H, CLK=L, CS=L
  wait5(WAIT);
  for (i = 0; i != 8; i++) {
    PORTD = 0b00110000; // DI=H, CLK=H, CS=L
    wait5(WAIT);
    c = ((c << 1) | (PIND & 1));
    PORTD = 0b00010000; // DI=H, CLK=L, CS=L
    wait5(WAIT);
  }
  return c;
}


/**
 * @brief SDカードからSPI形式で1バイトを高速受信する
 * @return 受信した1バイト
 */
unsigned char readByteFast(void)
{
  unsigned char c = 0;
  volatile unsigned char i;

  PORTD = 0b00010000; // DI=H, CLK=L
  for (i = 0; i != 8; i++) {
    PORTD = 0b00110000; // DI=H, CLK=H
    c = ((c << 1) | (PIND & 1));
    PORTD = 0b00010000; // DI=H, CLK=L
  }
  return c;
}


/**
 * @brief ポーリングしてビジーフラグが解除されるのを待つ
 * @retval 1 ビジーフラグが解除された
 * @retval 0 タイムアウトした、またはSDカードが挿入されていない場合
 * @note 永久ループ防止のタイムアウトを実装
 */
unsigned char waitFinish(void)
{
  unsigned short timeout = 0xffff;

  while (timeout--) {
    if (bit_is_set(PIND, 3)) {
      return 0;
    }
    if (readByteFast() == 0xff) {
      return 1;
    }
  }
  return 0;
}


/**
 * @brief SDカードの初期化処理で必要なSPIコマンドを低速送信する
 * @param cmd SPIコマンド
 * @param adr SPIコマンドの引数
 * @note CMD0とCMD8には専用のCRC値を送信する
 *  R1レスポンス受信は関数の外で行う
 */
void cmd_(unsigned char cmd, unsigned long adr)
{
  unsigned char crc;
  
  // CMD8:0x87、CMD0およびその他:0x95
  // SPIモード移行後はCRCチェックされないため、
  // CMD0以外では0x95のままで問題ない
  crc = (cmd == 8) ? 0x87 : 0x95;
  
  writeByteSlow(0xff);                // コマンド前のダミークロック
  writeByteSlow(0x40 + cmd);          // Index部 bit7,6は0,1であること
  writeByteSlow(adr >> 24);           // Argument1
  writeByteSlow((adr >> 16) & 0xff);  // Argument2
  writeByteSlow((adr >> 8) & 0xff);   // Argument3
  writeByteSlow(adr & 0xff);          // Argument4
  writeByteSlow(crc); // CRC値はSPIモードに移行したら無視される。ただし省略はできない
  
  // ここで0xffを送信してレスポンスを読み捨てない
  // R1はgetRespSlow()で取得する
}

/**
 * @brief SPIコマンドを送信してR1レスポンスを受信する
 * @param cmd SPIコマンド
 * @param adr SPIコマンドの引数
 * @retval 0x80 R1レスポンス
 * @retval 0xff タイムアウトした場合
 */
unsigned char cmdFast(unsigned char cmd, unsigned long adr)
{
  unsigned char res;
  unsigned short timeout;
  
  writeByteFast(0xff);
  writeByteFast(0x40 | cmd);
  writeByteFast((unsigned char)(adr >> 24));
  writeByteFast((unsigned char)(adr >> 16));
  writeByteFast((unsigned char)(adr >> 8));
  writeByteFast((unsigned char)adr);
  
  writeByteFast(0x95); // CRC

  for (timeout = 0; timeout < 1000; timeout++) {
    res = readByteFast(); // レスポンスを受信
    if ((res & 0x80) == 0) {
      return res; // MSBが0ならR1レスポンスとみなす
    }
  }
  return 0xff; // R1レスポンスがタイムアウトしたら0xffを返す
}


/**
 * @brief R1レスポンスを低速受信する
 * @retval 0x80 R1レスポンス
 * @retval 0xff タイムアウトした場合
 * @note ACMD41によるSD初期化中に実行
 */
unsigned char getRespSlow(void)
{
  unsigned char res;
  unsigned short timeout;
  
  for (timeout = 0; timeout < 1000; timeout++) {
    res = readByteSlow(); // レスポンスを低速受信
    if ((res & 0x80) == 0) {
      return res; // MSBが0ならR1レスポンスとみなす
    }
  }
  return 0xff; // R1レスポンスがタイムアウトしたら0xffを返す
}


/**
 * @brief CMD17を送信してデータトークンを待つ
 * @param adr バイトアドレス
 * @note SDHC/SDXCでは512バイト単位のブロックアドレスへ変換する
 */
void cmd17Fast(unsigned long adr)
{
  unsigned char ch;

  if (sdHighCapacity) {
    adr >>= 9; // byte address / 512 -> block address
  }

  cmdFast(17, adr);

  do {
    ch = readByteFast();
  } while (ch != 0xfe);
}


/**
 * @brief CMD24を送信してシングルブロック書き込みを開始する
 * @param adr バイトアドレス
 * @return CMD24のR1レスポンス
 * @note SDHC/SDXCでは512バイト単位のブロックアドレスへ変換する
 */
static unsigned char cmd24Fast(unsigned long adr)
{
  if (sdHighCapacity) {
    adr >>= 9; // byte address / 512 -> block address
  }

  return cmdFast(24, adr);
}



/**
 * @brief ルートディレクトリエントリ番号から短いファイル名(SFN)を取得する
 * @param entryNo ルートディレクトリエントリ番号
 * @param *name 短いファイル名(SFN)
 */
__attribute__((noinline, noclone))
void getSFN(unsigned short entryNo, char *name)
{
  unsigned char i;

  seekSD(rootAddr + entryNo * DIR_ENTRY_SIZE);
  for (i = 0; i < DIR_BODY_SIZE; i++) {
    *(name++) = (char)readByteFast();
  }
  discard(DIR_BODY_SIZE);
}


/**
 * @brief ルートディレクトリ内から指定した拡張子を持つファイルを探してそのルートディレクトリエントリ番号を返す
 *  withNameが1の場合はファイル名本体の一致も条件になる
 *  同名のファイルが複数ある場合はタイムスタンプが最も新しいファイルが選ばれる
 * @param *targExt 拡張子名
 * @param *protectOut 書き込み禁止状態の格納先ポインタ
 * @param *targName ファイル名本体
 * @param withName ファイル名本体の一致も条件の場合は1
 * @retval 0x000 ~ 0x1ff  一致したファイルのルートディレクトリエントリ番号
 * @retval 0x200          一致するファイルが見つからなかった、またはSDカードが挿入されていない場合
 */
__attribute__((noinline, noclone))
int findExt(const char *targExt, unsigned char *protectOut, char *targName, unsigned char withName)
{
  unsigned short i, j;
  unsigned short max_file = ROOT_ENTRY_COUNT;
  unsigned short max_time = 0, max_date = 0;
  unsigned short tm, dt;
  unsigned char name[DIR_BODY_SIZE];
  unsigned char entry[DIR_ENTRY_SIZE];
  
  // ルートディレクトリエントリを先頭から走査する
  for (i = 0; i < ROOT_ENTRY_COUNT; i++) {
    if (bit_is_set(PIND, 3)) {
      return ROOT_ENTRY_COUNT;
    }
    readRootEntry(i, entry); // ルートディレクトリエントリ番号iのエントリ32バイトをentryに読み出す
    
    // 拡張子が一致した場合
    if (isTargetFile(entry, targExt)) {
      if (withName) { // withNameが1ならファイル名本体とも比較する
        for (j = 0; j < DIR_BODY_SIZE; j++) {
          name[j] = entry[j];
        }
        
        if (memcmp(name, targName, DIR_BODY_SIZE) != 0) {
          continue; // 本体が一致しなかったら残りの処理をスキップする
        }
      }
      
      // タイムスタンプを求める
      tm = entry[DIR_WRT_TIME_OFST + 0] | (unsigned short)entry[DIR_WRT_TIME_OFST + 1] << 8;
      dt = entry[DIR_WRT_TIME_OFST + 2] | (unsigned short)entry[DIR_WRT_TIME_OFST + 3] << 8;
      
      // 最も新しいタイムスタンプを持つファイルが見つかった場合
      if ((dt > max_date) || ((dt == max_date) && (tm >= max_time))) {
        max_time = tm;
        max_date = dt;
        max_file = i; // 最も新しいタイムスタンプを持つファイルのディレクトリエントリ番号
        
        // 書き込み禁止状態の格納先ポインタが与えられている場合は、ファイルの書き込み禁止状態をセットする
        if (protectOut) {
          *protectOut = (entry[DIR_ATRB_OFST] & 1) << 3;
        }
        
        // 拡張子が一致するファイルが見つかり、targNameがポインタでかつ、withNameが 0 の場合は、ファイル名の本体をtargNameにコピーして返す
        if ((targName != 0) && !withName) {
          memcpy(targName, entry, DIR_BODY_SIZE);
        }
      }
    }
  }
  return max_file; // 該当する拡張子を持つファイルが見つからなかった場合は 0x200 を返す
}


/**
 * @brief FATテーブルをメモリに用意する
 * @param entryNo ルートディレクトリエントリ番号
 * @param *fat
 * @param len
 * @param fatNum
 * @param fatElemNum
 * @note Flashメモリ節約のため定数伝播専用版を作らないようにコンパイラに指示 -> 870バイト削減
 */
__attribute__((noinline, noclone))
void prepareFat(unsigned short entryNo, unsigned short *fat, unsigned short len, unsigned char fatNum, unsigned char fatElemNum)
{
  unsigned short ft, i;
  unsigned char fn;

  if (bit_is_set(PIND, 3)) {
    return;
  }
  
  // 先頭クラスタ番号を求める
  seekSD(rootAddr + entryNo * DIR_ENTRY_SIZE + DIR_1ST_CLST_OFST);
  ft = readByteFast();
  ft += (unsigned short)readByteFast() << 8; // 先頭クラスタ番号の下位16ビット
  discard(2);
  
  if (fatNum == 0) {
    fat[0] = ft;
  }
  
  for (i = 0; i < len; i++) {
    fn = (i + 1) / fatElemNum; // fatElemNum={18(DSK), 35(NIC)}
    
    seekSD((unsigned long)fatAddr + (unsigned long)ft * FAT_ENTRY_SIZE); // createFile()にも同じ記述あり
    ft = readByteFast();
    ft += (unsigned short)readByteFast() << 8; // ft = FATエントリの値(次のクラスタ番号)
    discard(2);
    
    if (fn == fatNum) {
      fat[(i + 1) % fatElemNum] = ft;
    }
    
    if ((ft > 0xfff6) || (fn > fatNum)) { // ft:0xfff7=不良クラスタ, 0xfff8~0xffff=使用中クラスタのチェーン終端
      break;
    }
  }
}


/**
 * @brief SDカードの指定されたアドレスにバイトデータを書き込む
 * @param adr   書き込み先のアドレス
 * @param *data 書き込むバイトデータ
 * @param len   書き込むバイトデータのサイズ
 * @note バッファ領域を跨いだデータをチェックしていない
 */
void writeSD(unsigned long adr, unsigned char *data, unsigned short len)
{
  unsigned int i;
  unsigned char *buf = &writeData[0][0];

  if (bit_is_set(PIND, 3)) {
    return;
  }

  // ブロック全体(512バイト)を汎用バッファ buf に取り込む
  seekSD(adr & 0xfffffe00); // adrの下位9ビットを0にマスクすることで境界を512バイト(SDカードの基本ブロック長)にする
  for (i = 0; i < DEFAULT_BLOCK_SIZE; i++) {
    buf[i] = readByteFast();
  }
  discard(DEFAULT_BLOCK_SIZE);
  
  // バッファの一部をdataで置き換える
  memcpy(&(buf[adr & 0x1ff]), data, len); // adrの下位9ビット(0~511)をインデックスに使用している
  writeSector(adr & 0xfffffe00, buf); // 書き換えられたバッファをSDに書き戻す
}


/**
 * @brief SDカードのFAT1をFAT2へ複製する
*/
void duplicateFat(void)
{
  unsigned short i, j;
  unsigned long adr = fatAddr;
  unsigned char *buf = &writeData[0][0];

  if (bit_is_set(PIND, 3)) {
    return;
  }

  for (j = 0; (j < sectorsPerFat) && !sdWriteError; j++) { // ループ中に sdWriteError が立ったら強制的にループを抜ける
    seekSD(adr);
    for (i = 0; i < SECTOR_SIZE; i++) {
      buf[i] = readByteFast(); // セクタ単位でFATデータをバッファに取り込む
    }
    discard(SECTOR_SIZE);
    
    writeSector(adr + (unsigned long)sectorsPerFat * SECTOR_SIZE, buf); // sdWriteErrorが返る
    adr += SECTOR_SIZE; // FATアドレスを次のセクタへ移動する
  }
}


/**
 * @brief DSKイメージをNICイメージに変換する
 */
void dsk2Nic(void)
{
  unsigned char trk, logic_sector;
  unsigned short i;
  unsigned char *dst = (&writeData[0][0] + SECTOR_SIZE); // NICファイル用バッファ dst[0~511]
  unsigned short *fatDsk = (unsigned short *)(&writeData[0][0] + 1024);
  
  prevFatNumNic = prevFatNumDsk = 0xff;
  
  // Gap1 (22)
  for (i = 0; i < 0x16; i++) {
    dst[i] = 0xff; // 0xffを22個セットする
  }

  // シンクバイト (12)
  dst[0x16] = 0x03;
  dst[0x17] = 0xfc;
  dst[0x18] = 0xff;
  dst[0x19] = 0x3f;
  dst[0x1a] = 0xcf;
  dst[0x1b] = 0xf3;
  dst[0x1c] = 0xfc;
  dst[0x1d] = 0xff;
  dst[0x1e] = 0x3f;
  dst[0x1f] = 0xcf;
  dst[0x20] = 0xf3;
  dst[0x21] = 0xfc;
  
  // アドレス プロローグ (3)
  dst[0x22] = 0xd5;
  dst[0x23] = 0xaa;
  dst[0x24] = GCR_ADRS_MARKER;
  
  // dst[0x25 ~ 0x2c] アドレスフィールド (8) : volume (2), track (2), sector (2), checksum (2) 4-and-4エンコード
  
  // アドレス エピローグ (3)
  dst[0x2d] = 0xde;
  dst[0x2e] = 0xaa;
  dst[0x2f] = 0xeb;
  
  // Gap2 (5)
  for (i = 0x30; i < 0x35; i++) {
    dst[i] = 0xff;
  }
  
  // データ プロローグ (3)
  dst[0x35] = 0xd5;
  dst[0x36] = 0xaa;
  dst[0x37] = GCR_DATA_MARKER;
  
  // dst[0x38 ~ 0x18d] エンコード済みデータ (342) : 6-and-2エンコード
  // dst[0x18e] チェックサム (1) 
  
  // データ エピローグ (3)
  dst[0x18f] = 0xde;
  dst[0x190] = 0xaa;
  dst[0x191] = 0xeb;
  
  // Gap3 (14)
  for (i = 0x192; i < 0x1a0; i++) {
    dst[i] = 0xff;
  }
  
  // 残りを 0 で埋める (96) 
  // これで512バイト
  for (i = 0x1a0; i < 512; i++) {
    dst[i] = 0x00;
  }
  
  for (trk = 0; trk < DSK_MAX_TRACKS; trk++) {
    PORTB ^= (1 << PB4); // 赤LED点滅
    
    for (logic_sector = 0; logic_sector < DSK_SCTR_PER_TRK; logic_sector++) {
      unsigned char *src;
      unsigned short ph_sector = (unsigned short)pgm_read_byte_near(physicalSector + logic_sector);

      if (bit_is_set(PIND, 3)) {
        return;
      }

      if ((logic_sector & 1) == 0) { // = {0,2,4,..,14}
        unsigned short long_sector = trk * 8 + (logic_sector / 2);
        unsigned short long_cluster = (long_sector >> sectorsPerCluster2);
        unsigned char fatNum = long_cluster / FAT_DSK_ELEMS;
        unsigned short ft;

        if (fatNum != prevFatNumDsk) {
          prevFatNumDsk = fatNum;
          prepareFat(dskEntryNo, fatDsk, ((280 + sectorsPerCluster - 1) >> sectorsPerCluster2), fatNum, FAT_DSK_ELEMS);
        }
        
        ft = fatDsk[long_cluster % FAT_DSK_ELEMS];
        
        seekSD(userAddr + (((unsigned long)(ft - 2UL) << sectorsPerCluster2) + (long_sector & (sectorsPerCluster - 1))) * 512);
        for (i = 0; i < 512; i++) {
          if (bit_is_set(PIND, 3)) {
            return;
          }
          *(&writeData[0][0] + i) = readByteFast(); // 汎用バッファに512バイト読み込む
        }
        discard(512);
        src = &writeData[0][0];
      } else { // logic_sector = {1,3,5,...,15}
        src = (&writeData[0][0] + DSK_SECTOR_SIZE); // 偶数セクタの時に読み込まれたバッファ領域を指定する
      }
      
      {
        unsigned char c, x, ox = 0;
        
        // トラック番号やセクタ番号は 4-and-4エンコード
        dst[0x25] = ((volume >> 1) | 0xaa); // ボリューム番号
        dst[0x26] = (volume | 0xaa);
        
        dst[0x27] = ((trk >> 1) | 0xaa); // トラック番号
        dst[0x28] = (trk | 0xaa);
        
        dst[0x29] = ((ph_sector >> 1) | 0xaa); // セクタ番号
        dst[0x2a] = (ph_sector | 0xaa);
        
        c = (volume ^ trk ^ ph_sector); // アドレスフィールドのXORチェックサム
        dst[0x2b] = ((c >> 1) | 0xaa);
        dst[0x2c] = (c | 0xaa);
        
        // 各バイトの下位2bitを3バイト分まとめる(=256/3=85.33=86) dst[56~141]
        for (i = 0; i < GCR_LBIT_SIZE; i++) { // 86
          x = (pgm_read_byte_near(FlipBit1 + (src[i] & 3)) |
            pgm_read_byte_near(FlipBit2 + (src[i + GCR_LBIT_SIZE] & 3)) |
            ((i <= 83) ? pgm_read_byte_near(FlipBit3 + (src[i + GCR_LBIT_SIZE*2] & 3)) : 0)); // 83+86+86=255
          
          dst[i + 0x38] = pgm_read_byte_near(encTable + (x ^ ox)); // チェックサム用に直前の値とのXORを取る
          ox = x;
        }
        
        // 各バイトの上位6bit(256) dst[142~397]
        for (i = 0; i < GCR_HBIT_SIZE; i++) { // 256
          x = (src[i] >> 2);
          dst[i + 0x8e] = pgm_read_byte_near(encTable + (x ^ ox)); // チェックサム用に直前の値とのXORを取る
          ox = x;
        }
        dst[0x18e] = pgm_read_byte_near(encTable + ox); // 連鎖XORチェックサム dst[398]
      }
      
      {
        unsigned short long_sector = (unsigned short)trk * DSK_SCTR_PER_TRK + ph_sector;
        unsigned short long_cluster = (long_sector >> sectorsPerCluster2);
        unsigned char fatNum = long_cluster / FAT_NIC_ELEMS;
        unsigned short ft;
      
        if (fatNum != prevFatNumNic) {
          prevFatNumNic = fatNum;
          prepareFat(nicEntryNo, fatNic, ((560 + sectorsPerCluster - 1) >> sectorsPerCluster2), fatNum, FAT_NIC_ELEMS);
        }
        
        ft = fatNic[long_cluster % FAT_NIC_ELEMS];

        PORTD = 0b10000000; // CS=H
        PORTD = 0b00000000; // CS=L
    
        // CMD24 シングルブロック書き込み
        if (cmd24Fast(userAddr + ((((unsigned long)ft - 2UL) << sectorsPerCluster2) + (long_sector & (sectorsPerCluster - 1))) * DEFAULT_BLOCK_SIZE) != 0) {
          /* エラー処理 */
          sdError = SDERR_CMD24;
          sdWriteError = 1;
          protect |= 0x08;
          
          PORTD = 0b10000000;
          PORTD = 0b00000000;
      
          return;
        }
        
        writeByteFast(0xff);
        writeByteFast(0xfe);
        
        for (i = 0; i < SECTOR_SIZE; i++) {
          if (bit_is_set(PIND, 3)) {
            return;
          }
          writeByteFast(dst[i]);
        }
        finishWrite();
        
        PORTD = 0b10000000; // CS=H
        PORTD = 0b00000000; // CS=L
      }
    }
  }
  buffClear();
  PORTB &= ~(1 << PB4); // 赤LED消灯
}


/**
 * @brief 指定された拡張子を持つファイルのルートディレクトリエントリ番号リストを作成する
 *  listはファイル名でソートされたルートディレクトリエントリ番号を要素に持つ配列
 * @param *list 汎用バッファのポインタ
 * @param *targExt 拡張子
 * @return 対象ファイル数
 * @retval 0 ~ 512: 一致するファイルの総数
 * @retval 0        SDカードが挿入されていない場合
 * @note オリジナル版ではSDカードが挿入されていない場合に 512 を返している
 */
unsigned short makeFileNameList(unsigned short *list, char *targExt)
{
  unsigned short i, j, k, entryNum = 0;
  char name1[DIR_BODY_SIZE], name2[DIR_BODY_SIZE]; // ファイル名ソート用
  unsigned char entry[DIR_ENTRY_SIZE];
  
  for (i = 0; i < ROOT_ENTRY_COUNT; i++) {
    if (bit_is_set(PIND, 3)) {
      return 0; // SDカードが挿入されていなければ 0 を返す
    }
  
    readRootEntry(i, entry);
    if (isTargetFile(entry, targExt)) {
      list[entryNum++] = i;
    }
  }
  
  // listをファイル名でソートする
  if (entryNum > 1) {
    for (i = 0; i <= (entryNum - 2); i++) {
      for (j = 1; j <= (entryNum - i - 1); j++) {
        getSFN(list[j], name1);
        getSFN(list[j - 1], name2);
        
        if (memcmp(name1, name2, DIR_BODY_SIZE) < 0) {
          k = list[j];
          list[j] = list[j - 1];
          list[j - 1] = k;
        }
      }
    }
  }
  return entryNum;
}


/**
 * @brief マウントするNICファイルをユーザーに選択させる
 *  BTFファイルが存在すれば初期カーソル位置をそのNICファイルにする
 * @param *tempBuff 汎用バッファのポインタ
 * @param btfExists BTFファイルが存在すれば 1
 * @param *filebase BTFファイル名の本体
 *  ファイルが選択された場合はそのファイル名
 * @retval 1 ファイルが選択された
 * @retval 0 SDカードが取り出された、または NICファイルが見つからなかった場合
 */
unsigned char chooseANicFile(void *tempBuff, unsigned char btfExists, char *filebase)
{
  unsigned short *list = (unsigned short *)tempBuff; // void * からほかのオブジェクトポインタ型への変換は自動的に行われるためキャストは省略できる(AI)
  unsigned short num = makeFileNameList(list, "NIC"); // listにNICファイルのリストを作成する
  char name[DIR_BODY_SIZE]; // ファイル名本体
  short cur = 0, prevCur = -1; // カーソル位置
  unsigned long i;
  
  if (num > 0) { // NICファイルが存在する場合
    
    PORTD |= (1 << PB6); // 黄LED点灯
    
    if (btfExists) { // BTFファイルが存在する場合、listから一致するファイルを探してカーソルを合わせる
      for (i = 0; i < num; i++) {
        getSFN(list[i], name);
        if (memcmp(name, filebase, DIR_BODY_SIZE) == 0) {
          cur = i; // filebaseと一致するNICファイルのインデックスをカーソルにセット
          break;
        }
      }
    }
    
    busy(0); // 選択中は風車を止める
    
    // メニューからNICファイルを選択するためのループ
    while (1) {
      if (bit_is_set(PIND, 3)) {
        PORTD &= ~(1 << PB6); // 黄LED消灯
        return 0;
      }
      
      if (bit_is_clear(PINC, 4)) { // UPボタンが押された
        i = 0;
        while (bit_is_clear(PINC, 4)) {
          i++;
          if (i == LONG_PUSH) {
            break;
          }
        }
        
        if (i > SHORT_PUSH) {
          if (i == LONG_PUSH) {
            cur -= MENU_ROWS; // 1ページ戻る
            if (cur < 0) {
              cur = 0;
            }
          } else {
            cur--;
            if (cur < 0) {
              cur += num;
            }
          }
        }
      } else if (bit_is_clear(PINC, 5)) { // DOWNボタンが押された
        
        i = 0;
        while (bit_is_clear(PINC, 5)) {
          i++;
          if (i == LONG_PUSH) {
            break;
          }
        }
        
        if (i > SHORT_PUSH) {
          if (i == LONG_PUSH) {
            cur += MENU_ROWS;
            if (cur >= num) {
              cur = num -1;
            }
          } else {
            cur++;
            if (cur >= num) {
              cur -= num;
            }
          }
        }
      } else if (bit_is_clear(PINB, 5)) { // ENTERボタンが押された
        
        i = 0;
        while (bit_is_clear(PINB, 5)) {
          i++;
          if (i == LONG_PUSH) {
            break;
          }
        }
        
        if (i > SHORT_PUSH) {
          break;
        }
      }
      
      // カーソル移動があればファイル名表示を更新する
      if (prevCur != cur) {
        prevCur = cur;
        
        char inv;
        short ser, i;
        
        for (i = 0; i < MENU_ROWS; i++) {
          inv = (i == MENU_CENTER) ? 1 : 0;
          locate(0, i + 1, inv);
          ser = cur - MENU_CENTER + i;
          
          if (ser >= 0 && ser < num) {
            consoleDec(ser + 1); // 通し番号は 1~99
            outCharUsart(' ');
            dispFileName(list[ser]);
            _delay_ms(100); // バッファ詰まりを防止するため
          } else {
            outCharUsart('\n');
            _delay_ms(100); // バッファ詰まりを防止するため
          }
        }
      }
    }
    
    getSFN(list[cur], name);
    memcpy(filebase, name, DIR_BODY_SIZE); // filebase に name をコピーする
    
    cls();
    locate(0, CENTER_Y, 1);
    dispFileName(list[cur]); // OLEDに選択されたファイル名を表示する
    
    PORTD &= ~(1 << PB6); // 黄LED消灯
    
    return 1;
  } else {
    return 0; // NICファイルが存在しない場合
  }
}

/**
 * @brief SDカードをSPIモードに初期化してFAT16関連パラメータを取得する
 * @note check_ejectから呼ばれる
 */
unsigned char init(void)
{
  unsigned char ch;
  unsigned char i;
  unsigned char sdV2;
  unsigned char r7_2, r7_3;
  unsigned char ocr0;
  unsigned short retry;
  char str[5];

  protect = 0;
  mounted = 0;
  sdHighCapacity = 0;

  /*
   * SDカードをSPIモードに初期化する
   * DI,CSをHにして74クロック以上送信
   */
  PORTD = 0b10000000; // CS=H

  for (i = 0; i != 200; i++) {
    PORTD = 0b10110000; // CS=H, DI=H, CLK=H
    wait5(WAIT);

    PORTD = 0b10010000; // CS=H, DI=H, CLK=L
    wait5(WAIT);
  }


  /*
   * CMD0
   * SPIモードへ移行
   */
  PORTD = 0b00000000; // CS=L

  cmd_(0, 0);
  ch = getRespSlow();

  if (ch != 0x01) {
    PORTD = 0b10000000;
    return 0;
  }

  /*
   * CMD8
   *
   * VHS           = 0x01
   * Check Pattern = 0xaa
   */
  cmd_(8, 0x000001aaUL);
  ch = getRespSlow();

  if (ch == 0x01) {

    /*
     * SD Version 2以降
     *
     * R7:
     *   [31:24]
     *   [23:16]
     *   [15:8]  VHS
     *   [7:0]   Check Pattern
     */

    readByteSlow();        // R7[31:24]
    readByteSlow();        // R7[23:16]
    r7_2 = readByteSlow(); // R7[15:8]
    r7_3 = readByteSlow(); // R7[7:0]

    if ((r7_2 != 0x01) || (r7_3 != 0xaa)) {
      PORTD = 0b10000000;
      return 0;
    }
    
    sdV2 = 1;

  } else if (ch == 0x05) {

    /*
     * 旧SDカード
     *
     * 0x05 =
     * In Idle State + Illegal Command
     *
     * CMD8未対応
     */
    sdV2 = 0;
    
  } else {

    // 想定外のCMD8レスポンス
    PORTD = 0b10000000;
    return 0;
  }

  /*
   * ACMD41
   *
   * SD v2 : HCS=1
   * SD v1 : HCS=0
   */
  retry = 0xffff;

  while (retry--) {

    if (bit_is_set(PIND, 3)) {
      PORTD = 0b10000000;
      return 0;
    }

    // CMD55
    cmd_(55, 0);
    ch = getRespSlow();

    if ((ch != 0x01) && (ch != 0x00)) {
      PORTD = 0b10000000;
      return 0;
    }

    // ACMD41
    if (sdV2) {
      cmd_(41, 0x40000000UL); // HCS=1
    } else {
      cmd_(41, 0);            // 旧SDSC
    }

    ch = getRespSlow();

    if (ch == 0x00) {
      break;
    }

    if (ch != 0x01) {
      PORTD = 0b10000000;
      return 0;
    }
  }

  // ACMD41タイムアウト
  if (ch != 0x00) {
    PORTD = 0b10000000;
    return 0;
  }
  
  /*
   * CMD58
   *
   * SD Version 2の場合はOCRを読み、
   * CCSからアドレス方式を判定する
   */
  if (sdV2) {

    cmd_(58, 0);
    ch = getRespSlow();

    if (ch != 0x00) {
      PORTD = 0b10000000;
      return 0;
    }

    // OCR[31:24]
    ocr0 = readByteSlow();

    // OCR[23:0]
    readByteSlow();
    readByteSlow();
    readByteSlow();

    // bit31: Power Up Status
    if (!(ocr0 & 0x80)) {
      PORTD = 0b10000000;
      return 0;
    }

    // bit30: CCS
    if (ocr0 & 0x40) {
      sdHighCapacity = 1;
    } else {
      sdHighCapacity = 0;
    }

  } else {

    // 旧SDカードはbyte addressing
    sdHighCapacity = 0;
  }

  /*
   * 初期化完了
   */
  PORTD = 0b10000000; // CS=H
  
  /*
  // SDSCのみブロックサイズを512バイトに設定
  if (!sdHighCapacity) {
    cmdFast(16, DEFAULT_BLOCK_SIZE);
  }
  */
  
  /*
   * FAT16初期化処理
   */

  // SD初期化が完了したら送受信はウェイトなし
  seekSD(54);

  for (i = 0; i < 5; i++) {
    str[i] = readByteFast();
  }

  discard(5);
  
  // BPBアドレスを求める
  if ((str[0] == 'F') && (str[1] == 'A') && (str[2] == 'T') && (str[3] == '1') && (str[4] == '6')) {
    bpbAddr = 0; // BS_FilSysTypeが"FAT16"の場合
  } else {
    seekSD(0x1c6); // 第1パーティションの開始セクタ番号(4バイト)
    bpbAddr = readByteFast();
    bpbAddr += (unsigned long)readByteFast() << 8;  // * 0x100;
    bpbAddr += (unsigned long)readByteFast() << 16; // * 0x10000;
    bpbAddr += (unsigned long)readByteFast() << 24; // * 0x1000000;
    bpbAddr *= SECTOR_SIZE; // 第1パーティションの開始セクタ番号にセクタサイズを掛けてbpbAddrとする
    discard(4);
  }
  
  if (bit_is_set(PIND, 3)) {
    return 0;
  }
  
  // BPBのパラメータとFAT開始アドレスを求める
  {
    volatile unsigned char k;
    
    seekSD(bpbAddr + 13);

    // BPB+13
    sectorsPerCluster = k = readByteFast();
    sectorsPerCluster2 = 0;

    while (k != 1) {
      sectorsPerCluster2++; // = 2の累乗であるsectorsPerClusterの指数。セクタ番号をクラスタ番号に変換するビットシフト時に使用される
      k >>= 1;
    }
  
    // BPB+14~15
    reservedSectors = readByteFast();
    reservedSectors += (unsigned short)readByteFast() << 8;
    
    // BPB+16
    readByteFast(); // NUM_FATS を使用する
    
    // BPB+17~18
    readByteFast(); // ROOT_ENTRY_COUNT を使用する
    readByteFast();
    
    // BPB+19~20
    totalSectors = readByteFast();
    totalSectors += (unsigned short)readByteFast() << 8;
    
    // BPB+21
    readByteFast(); // Media は読み捨てる
    
    // BPB+22~23
    sectorsPerFat = readByteFast();
    sectorsPerFat += (unsigned short)readByteFast() << 8;
    
    discard(11);
    
    // BPB_TotSec16が0ならBPB_TotSec32を使用する
    if (totalSectors == 0) {
      seekSD(bpbAddr + 32);
      totalSectors = readByteFast();
      totalSectors += (unsigned long)readByteFast() << 8;
      totalSectors += (unsigned long)readByteFast() << 16;
      totalSectors += (unsigned long)readByteFast() << 24;
      discard(4);
    }
    
    // FAT領域はブートセクタやBPBを含む予約領域が終わるセクタから始まる
    fatAddr  = bpbAddr  + (unsigned long)SECTOR_SIZE * reservedSectors;
    rootAddr = fatAddr  + ((unsigned long)sectorsPerFat * NUM_FATS * SECTOR_SIZE);
    userAddr = rootAddr + (unsigned long)ROOT_ENTRY_COUNT * DIR_ENTRY_SIZE;
    
    // FAT探索用
    maxCluster = getMaxCluster(totalSectors, reservedSectors, sectorsPerFat, sectorsPerCluster);
  }
  
  return 1;
}


/**
 * @brief BTFファイルやNICファイルを準備する
 * @param choose NICファイルをメニュー選択する場合は 1 にする
 */
void prepareFiles(unsigned char choose)
{
  char filebase[DIR_BODY_SIZE], btfbase[DIR_BODY_SIZE];
  unsigned char btfExists, choosen;
  
  cls();
  locate(0, CENTER_Y, 0);
  consoleFlash("Searching...");
  busy(1);
  
  // BTFファイルを探す
  btfEntryNo = findExt("BTF", 0, btfbase, 0); // "BTF"を拡張子に持つファイルが見つかったら、btfbaseにファイル名の本体を返す
  btfExists = (btfEntryNo != ROOT_ENTRY_COUNT);

  // NICファイルリストからNICファイルを選択する
  // NICファイルが無ければ 0 、選択されたら 1 が返る。btfbaseには選択されたファイル名本体が入る
  choosen = (choose) ? chooseANicFile(&writeData[0][0], btfExists, btfbase) : 0;
  
  // BTFファイルが存在するか、あるいはNICファイルが選択された場合、filebase に btfbase をコピーする
  if (btfExists || choosen) {
    memcpy(filebase, btfbase, DIR_BODY_SIZE);
  }
  
  busy(1);
  
  // NICファイルを探す
  // プロテクト状態を取得する
  nicEntryNo = findExt("NIC", &protect, filebase, btfExists || choosen);
  
  // NICファイルが見つからなかった時
  if (nicEntryNo == ROOT_ENTRY_COUNT) {
    locate(0, CENTER_Y, 0);
    consoleFlash("*** NO FILE ***");
    busy(0);
    
    while (bit_is_clear(PIND, 3)) {
      nop(); // Ejectされるまで待つ
    }
    return;
  }
  
  
  /*
   * BTFの作成・更新
   *
   * SDSC/SDHCともにバイトアドレスで位置を計算し、
   * writeSector() -> cmd24Fast() でSDHCのブロックアドレスへ変換する。
   */

  // BTFファイルが無ければ作成する
  if (!btfExists) {
    createFile(filebase, "BTF", NULL, 0);

    // 書き込みエラーが発生した場合は以降の処理を行わない
    if (sdWriteError) {
      return;
    }

    btfEntryNo = findExt("BTF", 0, filebase, 1);
    btfExists = (btfEntryNo != ROOT_ENTRY_COUNT);

  // 既存BTFと選択されたNICファイル名が異なる場合だけ更新する
  } else if (choosen ||
             (memcmp(filebase, btfbase, DIR_BODY_SIZE) != 0)) {

    writeSD(
      rootAddr + btfEntryNo * DIR_ENTRY_SIZE,
      (unsigned char *)filebase,
      DIR_BODY_SIZE);

    if (sdWriteError) {
      return;
    }
  }

  // マウントされたNICファイル名を表示
  locate(0, CENTER_Y, 1);
  dispFileName(nicEntryNo);
  
  prevFatNumNic = 0xff;
  prevFatNumDsk = 0xff;
  bitbyte = 0;
  readPulse = 0;
  magState = 0;
  ph_track = 0;
  sector = 0;
  buffNum = 0;
  formatting = 0;
  writePtr = &(writeData[buffNum][0]);
  
  buffClear();
  
  mounted = 1; // NICファイルがマウントされた状態
  prepare = 1;
}


/**
 * @brief SDカードの挿入・排出チェック
 */
void check_eject(void)
{
  unsigned long i;
  static char ejected; // SDカードが取り外されていたら 1、挿入されたら 0 になる
  
  if (bit_is_set(PIND, 3)) { // SDカードが取り外されている
    
    for (i = 0; i != 0x50000; i++) {
      if (bit_is_clear(PIND, 3)) {
        return; // チャタリング対策
      }
    }
    
    if (!ejected) {
      ejected = 1;
      
      cls();
      locate(0, CENTER_Y, 0);
      consoleFlash("*** NO CARD ***");
      busy(0);
    
      TIMSK0 &= ~(1 << TOIE0); // タイマー0 オーバーフロー割り込み禁止
      EIMSK  &= ~(1 << INT0); // 外部割り込み0の禁止
      
      mounted = 0;
      prepare = 0;
      
      // 以前のエラー状態を解除する
      sdWriteError = 0;
      sdError = SDERR_NONE;
    }
  } else { // SDカードが挿入されている
    
    ejected = 0;
    
    if (bit_is_clear(PINB, 5)) { // Enterボタンが押された
      i = 0;
      while (bit_is_clear(PINB, 5)) {
        i++;
        if (i == LONG_PUSH) {
          break;
        }
      }
      
      if (i > SHORT_PUSH) {
        PORTD &= ~(1 << PB6); // 黄LED消灯
        
        cli(); // 割り込み禁止
        
        busy(1);
        if (init()) { // SDカード設定
          prepareFiles(1); // NICファイルをユーザーに選択させる
        } else {
          sdError = SDERR_INIT;
        }
        busy(0);
        
        
        if (mounted) {
        
          // 読み出し用タイマーは有効
          TIMSK0 |= (1 << TOIE0);
        
          // Apple IIからの書き込み要求を許可
          // SDHCのアドレス変換はcmd24Fast()で行う
          EIMSK |= (1 << INT0);
        }
        
        sei(); // 割り込み許可
      }
    } else if (bit_is_clear(PINC, 4) && bit_is_clear(PINC, 5)) { // UPとDOWNボタンが同時に押された
      
      for (i = 0; i < 0x50000; i++) {
        if (bit_is_set(PINC, 4) || bit_is_set(PINC, 5)) {
          return;
        }
      }
      cli(); // 割込み禁止
      
      busy(1);
      if (init()) {
        // SDSC/SDHCのどちらでもDSK -> NIC変換を実行する
        // SDHCのCMD24アドレス変換はcmd24Fast()で行う
        allDsk2Nic();
      } else {
        sdError = SDERR_INIT;
      }
      busy(0);
      
      sei(); // 割り込み許可
    } else if (!mounted) { // NICファイルがマウントされていない -> SDカードが新たに挿入された
      
      for (i = 0; i < 0x50000; i++) {
        if (bit_is_set(PIND, 3)) {
          return; // 挿入信号がチャタリングを起こしていたら中止する
        }
      }
      cli(); // 割込み禁止
      
      busy(1);
      if (init()) {
        prepareFiles(0); // BTFファイルが無ければ作成する
      } else {
        sdError = SDERR_INIT;
      }
      busy(0);
      
      
      if (mounted) {
      
        // 読み出し用タイマーは有効
        TIMSK0 |= (1 << TOIE0);
      
        // Apple IIからの書き込み要求を許可
        // SDHCのアドレス変換はcmd24Fast()で行う
        EIMSK |= (1 << INT0);
      }
      
      sei(); // 割り込み許可
    }
  }
}


/**
 * @brief ステッピングモーターの状態を監視し物理トラック番号やセクタ番号などを更新する
 * 各種変数、GPIO、UART、割り込みの初期化
 * SDカードの挿入・取り出し監視
 * 使用済みバッファをSDカードに書き出す
 */
int main(void)
{
  static unsigned char oldStp = 0, stp; // ステッピングモーター入力
  const unsigned short baud = (F_CPU / (16UL * USART_BAUD)) - 1;
  unsigned char ofs;

  // GPIO設定
  // 0=入力,1=出力
  DDRB  = 0b00010000;
  DDRC  = 0b00001010;
  DDRD  = 0b11110010;

  PORTB = 0b00000000;
  PORTC = 0b00000010; // READパルス オン
  PORTD = 0b00000000;

  // USART設定
  UBRR0H = (unsigned char)(baud >> 8); // シリアル通信の通信速度を設定するための上位8ビットレジスタ。UBRR0Lとペアで使用し合計16ビットで分周比を指定する
  UBRR0L = (unsigned char)baud;
  UCSR0B = _BV(TXEN0); // 送信を許可
  UCSR0C = (1 << USBS0) | (3 << UCSZ00); // ストップビット長=2ビット, データビット長=8ビット

  // タイマー割り込み
  OCR0A = 0; // タイマーのカウント動作を停止
  TCCR0A = 0; // タイマー0を「通常のカウントアップモード」にし、ピン出力機能（PWMなど）をすべて切断する
  TCCR0B = 1; // クロック分周なし

  // INT0割り込み
  EICRA = 0b00000010; // 「INT0（外部割り込み0）の立ち下がりエッジ」で割り込みを発生させる設定

  mounted = 0; // マウントされていない
  prepare = 1;
  
  sector = 0;
  readPulse = 0;
  protect = 0;
  bitbyte = 0;
  magState = 0;
  ph_track = 0;
  buffNum = 0;
  formatting = 0;
  writePtr = &(writeData[buffNum][0]);
  
  sdWriteError = 0;
  sdError = SDERR_NONE;
  
  while (1) {
    check_eject(); // SDカードの取り出しをチェックする
    
    // SDエラー処理
    if (sdError) {
      informCardError(sdError); // SDカードが取り出されるまで待つ
      continue; // whileループの先頭へ戻し check_eject() を実行させる
    }
    
    if (bit_is_set(PINC, 0)) { // ドライブ無効状態
      PORTB &= ~(1 << PB4); // 赤LED消灯
      busy(0);
    } else { // ドライブ有効状態
      PORTB |= (1 << PB4); // 赤LED点灯
      busy(1);
      
      stp = (PINB & 0b00001111); // ステッピングモーターの状態(PH0~PH3)
      
      // ステッピングモーターの状態が変化した
      if (stp != oldStp) {
        oldStp = stp;
        
        // ステッピングモーターの位相を調べる
        ofs = 
        ((stp == 0b00001000) ? 2:           // PH3
        ((stp == 0b00000100) ? 4:           // PH2
        ((stp == 0b00000010) ? 6:           // PH1
        ((stp == 0b00000001) ? 0: 0xff)))); // PH0
        
        // ステッピングモーターの状態からトラックの移動量を求め、物理トラック番号を更新する
        if (ofs != 0xff) {
          ofs = ((ofs + ph_track) & 0b00000111); // ofsに物理トラック番号を加算し下位3ビットを取得する。 ofs={0~7}
          ph_track += pgm_read_byte_near(stepper_table + ofs);
          
          if (ph_track > 196) {
            ph_track = 0; // ph_trackが"負の値"になったら0にする
          }
          
          if (ph_track > DSK_MAX_PH_TRACKS-1) {
            ph_track = DSK_MAX_PH_TRACKS-1; // ph_track={0~139}
          }
        }
      }
      
      if (mounted && prepare) {
        cli(); // 割り込みを止めてアセンブリ変数を設定する(prepare, sector, bitbyte)
        sector = ((sector + 1) & 0xf); // sectorを次に進める sector={0~15}
        
        {
          unsigned char trk = (ph_track >> 2); // クォータートラックの物理トラック番号を論理トラック番号に変換する(1/4)
          unsigned short long_sector = (unsigned short)trk * DSK_SCTR_PER_TRK + sector; // トラック0からの通算セクタ番号
          unsigned short long_cluster = (long_sector >> sectorsPerCluster2);
          unsigned char fatNum = long_cluster / FAT_NIC_ELEMS;
          unsigned short ft;

          if (fatNum != prevFatNumNic) { // fatNumが前回から変化したら次のFAT領域をメモリに読み込む
            prevFatNumNic = fatNum;
            prepareFat(nicEntryNo, fatNic, ((560 + sectorsPerCluster - 1) >> sectorsPerCluster2), fatNum, FAT_NIC_ELEMS);
          }
          
          ft = fatNic[long_cluster % FAT_NIC_ELEMS]; // インデックスは 0~34
          
          {
            unsigned char i;
            for (i = 0; i < BUF_COUNT; i++) {
              if ((sectors[i] == sector) && (tracks[i] == trk)) { // バッファ上に現在のセクタ＆トラックの未保存データがあればSDカードに書き出す
                writeBackSub();
                break; // 重要！
              }
            }
          }
          
          // 現在の通算セクタ番号のアドレスまでread位置を進める
          seekSD(userAddr + ((((unsigned long)ft - 2UL) << sectorsPerCluster2) + (long_sector & (sectorsPerCluster - 1))) * DEFAULT_BLOCK_SIZE);
          
          bitbyte = 0;
          prepare = 0;
        }
        sei(); // 割り込み許可
      }
    }
  }
}


/**
 * @brief SDカードへバッファを書き出す
 * @note sub.Sから呼び出される
 */
void writeBack(void)
{
  static unsigned char sec; // staticなのでプログラム開始時に 0 に初期化される
  
  if (bit_is_set(PIND,3)) {
    return;
  }
  
  if (sdWriteError) {
    return; // 書き込みエラーが発生している場合は書き込みを禁止する
  }
  
  if (writeData[buffNum][2] == GCR_DATA_MARKER) {
    if (!formatting) { // formattingが 0 の時
      sectors[buffNum] = sector;
      tracks[buffNum]  = (ph_track >> 2); // 論理トラック番号は物理トラック番号の1/4
      sector = ((((sector == 0xf) || (sector == 0xd)) ? (sector+2) : (sector+1)) & 0xf);
      
      if (buffNum == (BUF_COUNT - 1)) {
        cancelRead();
        writeBackSub();
        prepare = 1;
      } else {
        buffNum++;
        writePtr = &(writeData[buffNum][0]);
      }
    } else { // formattingが 1 の時
      sector = sec;
      formatting = 0;
      
      if (sec == (DSK_SCTR_PER_TRK - 1)) {
        cancelRead();
        prepare = 1;
      }
    }
  }
  
  // 上の処理でbuffNumが進んでいる場合があるため、新しい現在バッファのアドレスフィールドも確認する
  if (writeData[buffNum][2] == GCR_ADRS_MARKER) {
    sec = (((writeData[buffNum][7] & 0x55) << 1) | (writeData[buffNum][8] & 0x55)); // セクタ番号を更新する
    formatting = 1;
  }
}


/**
 * @brief バッファ上のセクタデータをすべてSDカードに書き出し、バッファ関連変数を初期化する
 */
void writeBackSub(void)
{
  unsigned char i;

  if (bit_is_set(PIND, 3)) {
    return;
  }
  
  if (sdWriteError) {
    return; // 書き込みエラーが発生している場合は書き込みを禁止する
  }
  
  for (i = 0; (i < BUF_COUNT) && !sdWriteError; i++) { // ループ中に sdWriteError が立ったら強制的にループを抜ける
    if (sectors[i] != 0xff) {
      writeBackSub2(i, sectors[i], tracks[i]); // バッファ上のセクタデータをSDカードに書き出す。処理中にsdWriteErrorが更新される
      sectors[i] = 0xff; // SDに書き出し終わったsectorsとtracksに 0xff をセットする
      tracks[i] = 0xff;
      writeData[i][2] = 0; // アドレスヘッダの3バイト目を無効なマーカー(0)にすることでwriteBack()の処理対象から外している(ループ中の割り込み対策？)
    }
  }
  
  // バッファ関連の変数を初期化する
  buffClear();
  buffNum = 0;
  writePtr = &writeData[0][0];
}


/**
 * @brief バッファ上のセクタデータをSDカードへ書き出す
 * @param bn    バッファ番号
 * @param sc    セクタ番号
 * @param track トラック番号
 */
void writeBackSub2(unsigned char bn, unsigned char sc, unsigned char track)
{
  unsigned char c;
  unsigned short i;
  unsigned short long_sector = (unsigned short)track * DSK_SCTR_PER_TRK + sc;
  unsigned short long_cluster = (long_sector >> sectorsPerCluster2);
  unsigned char fatNum = long_cluster / FAT_NIC_ELEMS;
  unsigned short ft;

  if (bit_is_set(PIND, 3)) {
    return;
  }
  
  if (fatNum != prevFatNumNic) {
    prevFatNumNic = fatNum;
    prepareFat(nicEntryNo, fatNic, ((560 + sectorsPerCluster - 1) >> sectorsPerCluster2), fatNum, FAT_NIC_ELEMS);
  }
  
  PORTD = 0b10000000; // CS=H
  PORTD = 0b00000000; // CS=L
  
  ft = fatNic[long_cluster % FAT_NIC_ELEMS];
  
  if (cmd24Fast(userAddr + ((((unsigned long)ft - 2UL) << sectorsPerCluster2) + (long_sector & (sectorsPerCluster - 1))) * DEFAULT_BLOCK_SIZE) != 0) {
    /* エラー処理 */
    sdError = SDERR_CMD24;
    sdWriteError = 1;
    protect |= 0x08; // SDカードへ書き込めないようにプロテクトビットを立てる
  
    PORTD = 0b10000000; // CS=H
    PORTD = 0b00000000; // CS=L
  
    return;
  }
  
  writeByteFast(0xff);
  writeByteFast(0xfe);
  
  for (i = 0; i < 22; i++) { // 0xff を22回書き込む
    writeByteFast(0xff);
  }
  
  // sync header 12
  writeByteFast(0x03);
  writeByteFast(0xfc);
  writeByteFast(0xff);
  writeByteFast(0x3f);
  writeByteFast(0xcf);
  writeByteFast(0xf3);
  writeByteFast(0xfc);
  writeByteFast(0xff);
  writeByteFast(0x3f);
  writeByteFast(0xcf);
  writeByteFast(0xf3);
  writeByteFast(0xfc);

  // address header
  writeByteFast(0xd5); // address prolog (0xd5 0xaa 0x96)
  writeByteFast(0xAA);
  writeByteFast(0x96);
  
  writeByteFast((volume>>1)|0xaa); // 4x4enc disk volume number
  writeByteFast(volume|0xaa);
  
  writeByteFast((track>>1)|0xaa); // 4x4enc track number
  writeByteFast(track|0xaa);
  
  writeByteFast((sc>>1)|0xaa); // 4x4enc sector number
  writeByteFast(sc|0xaa);
  
  c = (volume^track^sc); // 4x4enc address prolog checksum
  writeByteFast((c>>1)|0xaa);
  writeByteFast(c|0xaa);
  
  writeByteFast(0xde); // address epilog (0xde 0xaa 0xeb)
  writeByteFast(0xAA);
  writeByteFast(0xeb);

  // sync header
  writeByteFast(0xff);
  writeByteFast(0xff);
  writeByteFast(0xff);
  writeByteFast(0xff);
  writeByteFast(0xff); // ここまでで 53バイト

  // data
  for (i = 0; i < GCR_DATA_AREA_SIZE; i++) { // データエリアを出力 349
    writeByteFast(writeData[bn][i]);
  }
  
  for (i = 0; i < 14; i++) { // 0xffを14回出力
    writeByteFast(0xff);
  }
  
  for (i = 0; i < 96; i++) { // 0x00を96回出力 合計 512バイト
    writeByteFast(0x00);
  }
  finishWrite();
  
  PORTD = 0b10000000; // CS=H
  PORTD = 0b00000000; // CS=L
}



/**
 * @brief OLED画面を消去してヘッダーを表示する
 */
static void cls(void) {
  outCharUsart(12); // 画面消去コード
  locate(0, 0, 0);
  consoleFlash("* SDISK 2 BOOK *"); // ヘッダーを常に最上行に表示する
}


/**
 * @brief OLED画面のカーソル位置と反転指定をセットする
 * @param x X座標
 * @param y Y座標
 * @param inv 1で白黒反転表示。改行で反転は解除される
 */
static void locate(const uint8_t x, const uint8_t y, const uint8_t inv)
{
  outCharUsart(3);
  outCharUsart(x);
  outCharUsart(y);
  outCharUsart(inv);
}


/**
 * @brief 文字列をカーソル位置に出力する
 * @param *str 表示する文字列ポインタ
 * @note 終端の'\0'の位置には'\n'が出力される。'\n'の処理方法は受信側のプログラムによる
 */
static void console(const char *str)
{
  if (str == NULL) {
    return;
  }
  
  while (*str != '\0') {
    outCharUsart((unsigned char)*str);
    str++;
  }
  outCharUsart('\n');
}


/**
 * @brief フラッシュROM上の文字列をカーソル位置に出力する
 * @note consoleFlashから呼ばれる
 */
static void console_P(PGM_P str)
{
  char c;
  
  while ((c = pgm_read_byte(str++)) != '\0') {
    outCharUsart(c);
  }
  outCharUsart('\n');
}


/**
 * @brief OLED画面の風車表示を切り替える
 * @param mode 1でアクティブ
 */
static void busy(unsigned char mode)
{
  static unsigned char last = 255;
  
  if (mode == last) {
    return;
  }
  last = mode;
  if (mode) {
    outCharUsart(1); // 風車 表示(回転)
  } else {
    outCharUsart(2); // 風車 消去
  }
}


/**
 * @brief 2桁の10進数文字列をカーソル位置に表示する
 * 10未満は先頭に0が表示される
 * 100の桁は表示されない
 * @param val 数値
 */
static void consoleDec(unsigned char val)
{
  static const char dec[] = "0123456789";
  val %= 100;
  outCharUsart(dec[val / 10]);
  outCharUsart(dec[val % 10]);
}


/**
 * @brief SDカードのアドレスを指定位置まで進める
 * @param adr バイトアドレス
 */
static void seekSD(unsigned long adr)
{
  unsigned short i;
  unsigned short adr_l = adr & (DEFAULT_BLOCK_SIZE - 1);
  unsigned long adr_h = adr - adr_l;

  // 新しいコマンドを開始
  PORTD = 0b10000000; // CS=H
  PORTD = 0b00000000; // CS=L
  
  
  // SDSCのみブロックサイズを512バイトに設定
  if (!sdHighCapacity) {
    cmdFast(16, DEFAULT_BLOCK_SIZE);
  }
  
  
  // cmd17Fast()内部でSDHCならblock addressへ変換する
  cmd17Fast(adr_h);

  // 512バイト境界から目的位置まで読み進める
  for (i = 0; i < adr_l; i++) {
    readByteFast();
  }
}


/**
 * @brief CMD17で開始したシングルブロックリードモードを終了させる
 * @param num すでに受信したバイトデータ数
 */
static void discard(unsigned short num)
{
  unsigned short i;
  
  for (i = 0; i < (DEFAULT_BLOCK_SIZE + 2) - num; i++) { // CRCの2バイトを加えている
    readByteFast(); // モードを終了させるために必要な残りのデータを読み捨てる
  }
}


/**
 * @brief entryNo番目のルートディレクトリエントリの内容をentryに取り込む
 * @param entryNo ルートディレクトリエントリの番号
 * @param *entry 取り込み先へのポインタ
 */
__attribute__((noinline, noclone))
static void readRootEntry(unsigned short entryNo, unsigned char *entry)
{
  unsigned short i;
  
  seekSD(rootAddr + entryNo * DIR_ENTRY_SIZE);
  for (i = 0; i < DIR_ENTRY_SIZE; i++) {
    entry[i] = readByteFast();
  }
  discard(DIR_ENTRY_SIZE);
}


/**
 * @brief LFNエントリ内のindex番目のUTF-16LE文字を取得する
 * @param *entry
 * @param index
 * @return LFNエントリ内のindex番目のUTF-16LE文字
 */
static unsigned short readLFNChar(const unsigned char *entry, unsigned char index)
{
  unsigned char p;
  
  p = lfnCharOffset[index];
  return (unsigned short)entry[p] | ((unsigned short)entry[p + 1] << 8);
}


/**
 * @brief エントリ番号からASCII形式のLFNを取得する
 * @param entryNo
 * @param *name
 * @param nameSize
 * @retval 0 LFNなし または LFNの文字数が nameSize-1 を超過した
 * @retval 1 LFN取得成功
 * @note 最後に name[nameSize-1] へ '\0' がセットされるため、有効な文字数は nameSize-1 であることに注意
 * @note ファイル名として取得したい文字数に +1 した数をnameのサイズとすること
 */
static unsigned char getLFN(unsigned short entryNo, char *name, unsigned char nameSize)
{
  unsigned char entry[DIR_ENTRY_SIZE];
  unsigned char order;
  unsigned char i;
  unsigned short pos;
  unsigned short ch;

  if ((name == 0) || (nameSize == 0)) {
    return 0;
  }
  name[0] = '\0';

  if (entryNo == 0) {
    return 0;
  }

  while (entryNo != 0) {
    entryNo--;
    readRootEntry(entryNo, entry);

    // LFNエントリでなければ終了
    if (entry[11] != ATTR_LFN) {
      name[0] = '\0';
      return 0;
    }

    // 削除済みエントリ
    if (entry[0] == 0xe5) {
      name[0] = '\0';
      return 0;
    }
    
    order = entry[0] & 0x1f;
    if ((order == 0) || (order > 20)) {
      name[0] = '\0';
      return 0;
    }

    for (i = 0; i < 13; i++) {
      ch = readLFNChar(entry, i);

      // 文字列終端
      if (ch == 0x0000) {
        continue;
      }

      // 未使用領域
      if (ch == 0xffff) {
        continue;
      }

      pos = (unsigned short)(order - 1) * 13 + i;
      if (pos >= (unsigned short)(nameSize - 1)) {
        name[0] = '\0'; // 文字数が nameSize-1 になったら 0 を返す
        return 0;
      }

      // ASCIIならそのまま格納
      // ASCII以外は '?' に変換する
      if (ch <= 0x007f) {
        name[pos] = (char)ch;
      } else {
        name[pos] = '?';
      }
      
      name[pos + 1] = '\0';
    }

    // LFN列の先頭エントリに到達
    if (entry[0] & LFN_LAST_ENTRY) {
      name[nameSize - 1] = '\0';
      return 1;
    }
    
  }
  name[0] = '\0';
  return 0;
}


/**
 * @brief *nameの最後の'.'を'\0'に置き換える
 */
static void removeExtension(char *name)
{
  char *p;
  char *dot = 0;

  for (p = name; *p != '\0'; p++) {
    if (*p == '.') {
      dot = p;
    }
  }

  if (dot != 0) {
    *dot = '\0';
  }
}


/**
 * @brief 指定されたルートディレクトリエントリのファイル名をコンソールに改行付きで表示する
 * LFNが無ければSFNを表示する
 */
static void dispFileName(unsigned short entryNo)
{
  char buf[DIR_LFN_SIZE + 1];
  
  if (getLFN(entryNo, buf, sizeof(buf))) { // bufの終端は'\0'であること
    removeExtension(buf); // 最後の'.'を'\0'に置き換える
  } else {
    getSFN(entryNo, buf);
    buf[DIR_BODY_SIZE] = '\0';
  }
  console(buf);
}


/**
 * @brief ルートディレクトリのDSKファイルをすべてNIC形式に変換する
 */
static void allDsk2Nic(void)
{
  unsigned short i, j;
  unsigned char entry[DIR_ENTRY_SIZE], d;
  char name[DIR_BODY_SIZE], ext[DIR_EXT_SIZE];
  char lfn[DIR_LFN_SIZE + 1]; // データサイズには終端の'\0'を含む
  
  cls();
  locate(0, CENTER_Y, 0);
  consoleFlash("Converting...");
  
  for (i = 0; i < ROOT_ENTRY_COUNT; i++) {
    if (bit_is_set(PIND, 3)) {
      return;
    }
    
    readRootEntry(i, entry); // ルートディレクトリエントリ32バイトをentryに読み出す
    d = entry[0]; // エントリの先頭バイト
    
    if ((d == 0x00) || (d == 0x05) || (d == 0x2e) || (d == 0xe5)) {
      continue;
    }
    
    if (!(((d >= 'A') && (d <= 'Z')) || ((d >= '0') && (d <= '9')))) {
      continue;
    }
    
    // d = 属性バイト
    d = entry[DIR_ATRB_OFST];
    
    if (d & 0x1e) {
      continue; // 隠しファイル,システムファイル,ボリュームラベル,サブディレクトリのいずれか
    }
    
    if (d == 0xf) {
      continue; // 長いファイル名用の特殊エントリ
    }
    
    // ファイル名の拡張子が "DSK" か？
    for (j = 0; j < DIR_EXT_SIZE; j++) {
      ext[j] = entry[DIR_BODY_SIZE + j];
    }
    
    if (memcmp(ext, "DSK", DIR_EXT_SIZE) != 0) {
      continue; // 拡張子が "DSK" でなければスキップする
    }
    
    // そのファイルのNICファイルが存在するか？
    for (j = 0; j < DIR_BODY_SIZE; j++) {
      name[j] = entry[j];
    }
    
    if (findExt("NIC", 0, name, 1) < ROOT_ENTRY_COUNT) {
      continue; // 拡張子が"NIC"でファイル名の本体がnameと一致したら変換済みなのでスキップする
    }
    
    // NICファイルが存在しないDSKファイルを見つけた
    
    dskEntryNo = findExt("DSK", 0, name, 1); // DSKファイルのディレクトリエントリ番号を求める
    
    if (dskEntryNo == ROOT_ENTRY_COUNT) {
      sdError = SDERR_NIC; // 取得に失敗した場合
      return;
    }
    
    if (!getLFN(dskEntryNo, lfn, sizeof(lfn))) { // DSKファイルのLFNをlfnに取得する。LFNがない場合、またはLFNの長さがDIR_LFN_SIZEを超過した場合は lfn=0
      sdError = SDERR_NIC; // 取得に失敗した場合
      return;
    }
    
    changeLfnExt(lfn); // 拡張子を"NIC"に変更する
    
    if (!createFile(name, "NIC", lfn, 560)) {
      // SD書き込みエラーが既に設定されている場合は、そのエラーコードを保持する
      if (sdError == SDERR_NONE) {
        sdError = SDERR_NIC;
      }
      return;
    }
    
    // createFile()内のSD書き込みでエラーが発生した場合は後続処理を行わない
    if (sdWriteError) {
      return;
    }
    
    nicEntryNo = findExt("NIC", 0, name, 1); // NICファイルのディレクトリエントリ番号を求める
    
    if (nicEntryNo == ROOT_ENTRY_COUNT) {
      sdError = SDERR_NIC; // NICファイルのディレクトリエントリ番号が取得できなかった場合
      return;
    }
    
    locate(0, CENTER_Y, 0);
    dispFileName(dskEntryNo); // 変換中のファイル名を表示
    
    dsk2Nic(); // 変換処理
    
    // 変換中にSD書き込みエラーが発生した場合は次のファイルへ進まない
    if (sdWriteError || (sdError != SDERR_NONE)) {
      return;
    }
    
    locate(0, CENTER_Y, 0);
    consoleFlash("Next...");
  }
}


/**
 * @brief LFNに対応したファイルイメージを作成する
 * @param *name SFN本体(8文字、スペース埋め)
 * @param *ext SFN拡張子(3文字、スペース埋め)
 * @param *lfn LFN文字列(0終端、最大 DIR_LFN_SIZE 文字)
 * 0または空文字列の場合はLFNを作成しない
 * @sectNum 確保するセクタ数
 * 0の場合はセクタを確保しない
 * @retval 1 作成成功
 * @retval 0 作成失敗
 */
int createFile(char *name, char *ext, char *lfn, unsigned short sectNum)
{
  unsigned long adr;
  unsigned short ft, entryNum, clusterNum, fatValue, target;
  unsigned short firstCluster;
  unsigned char firstByte;
  unsigned char dirEntry[DIR_ENTRY_SIZE];
  unsigned char lfnEntry[DIR_ENTRY_SIZE];
  unsigned char lfnLen, lfnCount, checksum;
  unsigned char freeCount;
  unsigned char entryIndex, order;
  unsigned char start, copyLen;
  unsigned char charNum, offset;

  if (bit_is_set(PIND, 3)) {
    return 0;
  }
  
  // SFNエントリをRAM上に作成する。この時点ではSDへ書き込まない
  memset(dirEntry, 0, DIR_ENTRY_SIZE);
  memcpy(dirEntry, (unsigned char *)name, DIR_BODY_SIZE);
  memcpy(dirEntry + DIR_BODY_SIZE, (unsigned char *)ext, DIR_EXT_SIZE);
  *(unsigned long *)(dirEntry + 28) = (unsigned long)sectNum * SECTOR_SIZE;

  // LFNの文字数と必要エントリ数を求める
  lfnLen = 0;
  lfnCount = 0;
  checksum = 0;

  if (lfn != 0) {
    while ((lfnLen < DIR_LFN_SIZE) && (lfn[lfnLen] != '\0')) {
      lfnLen++;
    }
    // DIR_LFN_SIZE を超えていたら失敗
    if (lfn[lfnLen] != '\0') {
      return 0;
    }

    if (lfnLen != 0) {
      lfnCount = (lfnLen + 12) / 13;
      checksum = lfnChecksum(dirEntry);
    }
  }

  /*
   * ─────────────────────────────
   * 第1段階
   * ルートディレクトリに必要な連続空きが
   * あることを確認する
   * ─────────────────────────────
   */
  freeCount = 0;

  for (entryNum = 0;
       entryNum < ROOT_ENTRY_COUNT;
       entryNum++) {

    seekSD(
      rootAddr
      + (unsigned long)entryNum * DIR_ENTRY_SIZE
      + DIR_NAME_OFST);

    firstByte = readByteFast();
    discard(1);

    if ((firstByte == 0xe5) || (firstByte == 0x00)) {
      freeCount++;
      //LFNエントリ数 + SFN 1個の連続空きが見つかった
      if (freeCount == (unsigned char)(lfnCount + 1)) {
        entryNum -= lfnCount;
        break;
      }
    } else {
      freeCount = 0;
    }
  }
  
  if (entryNum == ROOT_ENTRY_COUNT) {
    return 0;
  }

  // ファイルに必要なクラスタ数を求める
  target = (sectNum + sectorsPerCluster - 1) >> sectorsPerCluster2;

  /*
   * ─────────────────────────────
   * 第2段階
   * FATに必要数の空きクラスタがあるか確認する
   *
   * このパスではFATを書き換えない
   * ─────────────────────────────
   */
  if (target != 0) {
    clusterNum = 0;

    for (ft = 2;
         (ft <= maxCluster) && (clusterNum < target);
         ft++) {

      seekSD(fatAddr + (unsigned long)ft * FAT_ENTRY_SIZE);
      fatValue = readByteFast();
      fatValue += (unsigned short)readByteFast() << 8;
      discard(2);

      if (fatValue == 0) {
        clusterNum++;
      }
    }

    // 必要なクラスタ数が確保できない場合、SDを何も変更せずに失敗する
    if (clusterNum < target) {
      return 0;
    }

    /*
     * ─────────────────────────────
     * 第3段階
     * FATをもう一度走査して、
     * 実際にクラスタチェーンを作成する
     * ─────────────────────────────
     */
    clusterNum = 0;
    firstCluster = 0;
    adr = 0;

    for (ft = 2;
         (ft <= maxCluster) && (clusterNum < target);
         ft++) {

      seekSD(fatAddr + (unsigned long)ft * FAT_ENTRY_SIZE);
      fatValue = readByteFast();
      fatValue += (unsigned short)readByteFast() << 8;
      discard(2);

      if (fatValue == 0) {
        clusterNum++;

        // 最初に見つかったクラスタ番号を保存する
        if (firstCluster == 0) {
          firstCluster = ft;
        } else {
          // 直前のクラスタから現在のクラスタへFATチェーンを接続する
          fatValue = ft;
          writeSD(adr, (unsigned char *)&fatValue, FAT_ENTRY_SIZE);
        }

        // 現在のクラスタのFATエントリアドレスを保存
        adr = fatAddr + (unsigned long)ft * FAT_ENTRY_SIZE;
      }
    }
    // 最後のクラスタを0xffffで終端する
    fatValue = 0xffff;

    writeSD(adr, (unsigned char *)&fatValue, FAT_ENTRY_SIZE);

    /*
     * SFNエントリに先頭クラスタ番号を設定する
     *
     * まだRAM上のdirEntryを書き換えるだけなので
     * SDのディレクトリには公開されていない
     */
    *(unsigned short *)(dirEntry + DIR_1ST_CLST_OFST) = firstCluster;

    // FAT1の変更をFAT2へ複製する
    duplicateFat();

    // FAT更新中に書き込みエラーが発生した場合は
    // ディレクトリエントリを公開せずに失敗として戻る
    if (sdWriteError) {
      return 0;
    }
  }

  /*
   * target == 0の場合は
   * dirEntryを0で初期化しているため、
   * 先頭クラスタ番号は0のままになる。
   * FATも変更しない。
   */

  /*
   * ─────────────────────────────
   * 第4段階
   * FATチェーンの作成が完了してから
   * LFN/SFNをルートディレクトリへ書き込む
   * ─────────────────────────────
   */

  // LFNエントリを書き込む
  for (entryIndex = 0;
       entryIndex < lfnCount;
       entryIndex++) {

    // ディレクトリ上では大きいOrderからSFN方向へ並ぶ
    order = lfnCount - entryIndex;

    /*
     * Order=1 → 文字0～12
     * Order=2 → 文字13～23
     */
    start = (order - 1) * 13;
    copyLen = lfnLen - start;

    if (copyLen > 13) {
      copyLen = 13;
    }

    memset(lfnEntry, 0xff, DIR_ENTRY_SIZE);

    // 最大OrderのエントリにLAST_LONG_ENTRY(0x40)を設定する
    lfnEntry[0] = order;

    if (order == lfnCount) {
      lfnEntry[0] |= 0x40;
    }

    // LFN固定項目
    lfnEntry[11] = 0x0f;
    lfnEntry[12] = 0x00;
    lfnEntry[13] = checksum;
    lfnEntry[26] = 0x00;
    lfnEntry[27] = 0x00;

    // ASCII → UTF-16LE
    for (charNum = 0;
         charNum < copyLen;
         charNum++) {

      offset = lfnCharOffset[charNum];
      lfnEntry[offset] = (unsigned char)lfn[start + charNum];
      lfnEntry[offset + 1] = 0x00;
      
    }

    // 13文字未満の場合は0x0000で終端する。残りはmemset済みの0xffff
    if (copyLen < 13) {
      offset = lfnCharOffset[copyLen];
      lfnEntry[offset] = 0x00;
      lfnEntry[offset + 1] = 0x00;
    }

    writeSD(rootAddr + (unsigned long)(entryNum + entryIndex) * DIR_ENTRY_SIZE, lfnEntry, DIR_ENTRY_SIZE);

    if (sdWriteError) {
      return 0;
    }
  }

  // LFNの直後にSFNを書き込む
  entryNum += lfnCount;

  writeSD(rootAddr + (unsigned long)entryNum * DIR_ENTRY_SIZE, dirEntry, DIR_ENTRY_SIZE);

  if (sdWriteError) {
    return 0;
  }

  return 1;
}


/**
 * @brief SFN(8.3形式)からLFNエントリ用チェックサムを計算する
 * @param *sfn ファイル名本体8バイト+拡張子3バイト
 * @return 1バイトのチェックサム
 */
static unsigned char lfnChecksum(const unsigned char *sfn)
{
  unsigned char checksum;
  unsigned char i;

  checksum = 0;
  for (i = 0; i < DIR_NAME_SIZE; i++) {
    checksum = ((checksum & 1) ? 0x80 : 0x00)
      + (checksum >> 1)
      + sfn[i];
  }
  return checksum;
}


/**
 * @brief LFNの拡張子を"NIC"に変更する
 * '.' が見つからなければ何もしない
 * @param *lfn 
 * @note 引数のlfnには3文字の拡張子"DSK"が存在することを前提としているため、チェックを省略している
 */
static void changeLfnExt(char *lfn)
{
  char *p;
  char *dot;

  dot = 0;
  for (p = lfn; *p; p++) {
    if (*p == '.') {
      dot = p;
    }
  }

  if (dot == 0) {
    return;
  }

  dot[1] = 'N';
  dot[2] = 'I';
  dot[3] = 'C';
  dot[4] = '\0';
}


/**
 * @brief FAT16で使用可能な最大クラスタ番号を求める
 * @param totalSectors パーティション全体のセクタ数
 * @param reservedSectors 予約セクタ数
 * @param sectorsPerFat FAT1個あたりのセクタ数
 * @param sectorsPerCluster 1クラスタあたりのセクタ数
 * @return 使用可能な最大クラスタ番号
 */
static unsigned short getMaxCluster(
  unsigned long totalSectors,
  unsigned short reservedSectors,
  unsigned short sectorsPerFat,
  unsigned char sectorsPerCluster)
{
  unsigned long rootSectors;
  unsigned long dataSectors;
  unsigned long clusterCount;

  rootSectors =
    ((unsigned long)ROOT_ENTRY_COUNT * DIR_ENTRY_SIZE
    + SECTOR_SIZE - 1) / SECTOR_SIZE;

  dataSectors =
    totalSectors
    - reservedSectors
    - (unsigned long)NUM_FATS * sectorsPerFat
    - rootSectors;

  clusterCount = dataSectors / sectorsPerCluster;

  /* クラスタ番号は2から始まるので最大番号は+1 */
  if (clusterCount >= 0xffef) {
    return 0xffef;
  }

  return (unsigned short)(clusterCount + 1);
}


/**
 * @brief セクタデータをSDに書き込む
 * @param adr
 * @param *buf
 */
static void writeSector(unsigned long adr, unsigned char *buf)
{
  unsigned short i;
  
  if (sdWriteError) { // エラー後もSDへ書き続けるとファイルシステムの破損範囲を広げる可能性があるので、エラー後の書き込みを止める
    return;
  }
  
  PORTD = 0b10000000; // CS=H
  PORTD = 0b00000000; // CS=L
  
  // CMD24 シングルブロック書き込み
  //sdError = SDERR_NONE;
  if (cmd24Fast(adr) != 0) {
    /* エラー処理 */
    sdError = SDERR_CMD24;
    sdWriteError = 1;
    protect |= 0x08;
 
    PORTD = 0b10000000;
    PORTD = 0b00000000;

    return;
  }
  
  writeByteFast(0xff); // コマンドレスポンスとデータパケットの間は1バイト以上空ける(0xFFを送信) (elm-chan)
  writeByteFast(0xfe); // CMD24用のデータトークン データパケット送信はデータトークン、データブロック、CRCで構成される
  
  for (i = 0; i < SECTOR_SIZE; i++) { // データブロック 512バイト
    if (bit_is_set(PIND, 3)) {
      return;
    }
    writeByteFast(buf[i]);
  }
  
  finishWrite();
  
  PORTD = 0b10000000; // CS=H
  PORTD = 0b00000000; // CS=L
}


/**
 * @brief 拡張子が一致する有効なファイルかチェックする
 * @param *entry ディレクトリエントリ
 * @param *ext 拡張子
 * @retval 1 目的のファイルである
 * @retval 0 目的のファイルではない
 * @note __attribute__((noinline, noclone)) を指定するとFlash使用量が 600バイト程度増えてしまった
 */
static unsigned char isTargetFile(unsigned char *entry, const char *ext)
{
  unsigned char d;

  d = entry[0];
  if (!(((d >= 'A') && (d <= 'Z')) || ((d >= '0') && (d <= '9')))) {
    return 0;
  }

  if (entry[DIR_ATRB_OFST] & 0x1e) {
    return 0;
  }

  if (memcmp(entry + DIR_BODY_SIZE, ext, DIR_EXT_SIZE) != 0) {
    return 0;
  }

  return 1;
}


/**
 * @brief データブロック(512バイト)をSDへ送信した後の処理
 * CRC送信->データレスポンス受信->ビジーフラグポーリング
 * @note sdErrorにエラーコード、sdWriteErrorにSD書き込みエラーフラグが入る
 */
static void finishWrite(void)
{
  unsigned char response;

  writeByteFast(0xff); // CRC
  writeByteFast(0xff);

  response = readByteFast(); // 送った512バイトをSDカードが正常に受理したらデータレスポンスは 0x05 になる
  if ((response & 0x1f) != 0x05) {
    sdError = SDERR_DATA; // 送った512バイトをSDカードが受理しなかった場合
  }
  
  if (!waitFinish()) { // データレスポンスの後はビジーフラグが出力されるので、ポーリングしながら書き込み動作終了を待つ(elm-chan)
    if (bit_is_set(PIND, 3)) {
      sdError = SDERR_EJECT; // ポーリング中にEjectされた場合
    } else {
      sdError = SDERR_BUSY; // ビジー状態でタイムアウトした場合
    }
  }
  
  if (sdError != SDERR_NONE) { // エラー発生したらフラグを立ててディスクをプロテクト状態にする
    sdWriteError = 1;
    protect |= 0x08;
  }
}


/**
 * @brief エラーメッセージを表示してSDカードが取り出されるまで待つ
 * @param *mes
 */
static void informCardError(const unsigned char err) {
  
  EIMSK &= ~(1 << INT0); // Apple IIからの書き込み要求を禁止

  if (bit_is_clear(PIND, 3)) {
    cls();
    locate(0, CENTER_Y, 0);
    
    if (err == SDERR_CMD24) {
      consoleFlash("*CMD24 ERR*");
    } else if (err == SDERR_DATA) {
      consoleFlash("*DATA ERR*");
    } else if (err == SDERR_BUSY) {
      consoleFlash("*BUSY ERR*");
    } else if (err == SDERR_EJECT) {
      consoleFlash("*EJECT ERR*");
    } else if (err == SDERR_INIT) {
      consoleFlash("*CARD ERR*");
    } else {
      consoleFlash("*UNKNOWN ERR*");
    }
    
    PORTB |= (1 << PB4); // 赤LED点灯
    PORTD &= ~(1 << PB6); // 黄LED消灯
    
    while (bit_is_clear(PIND, 3)) { // SDカードが抜かれるまで待つ
      PORTB ^= (1 << PB4); // 赤LED点滅
      PORTD ^= (1 << PB6); // 黄LED点滅
      _delay_ms(100);
    }
    
    PORTB &= ~(1 << PB4); // 赤LED消灯
    PORTD &= ~(1 << PB6); // 黄LED消灯
  }
}


/*
static void consoleHex(unsigned char val)
{
  static const char hex[] = "0123456789ABCDEF";
  outCharUsart(hex[(val >> 4) & 0x0F]);
  outCharUsart(hex[val & 0x0F]);
  
}


static void consoleHex16(unsigned short val)
{
  
  consoleHex((unsigned char)(val >> 8));
  consoleHex((unsigned char)val);
  
}


static void consoleHex32(unsigned long val)
{
  
  consoleHex16((unsigned short)(val >> 16));
  consoleHex16((unsigned short)val);
  
}
*/
