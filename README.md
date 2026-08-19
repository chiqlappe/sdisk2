# SDISK II LCD Modified Firmware

![基板イメージ](https://github.com/chiqlappe/sdisk2/blob/main/image/IMG_3473.JPG)
Apple II 用 DISK II エミュレータ「SDISK II LCD」の改造版ファームウェアです。

このプログラムは、西田ラヂオ Koichi Nishida 氏による
SDISK II LCD Firmware をベースに改造したものです。

オリジナルの SDISK II LCD については以下を参照してください。

- 西田ラヂオ SDISK II LCD
  [https://tulip-house.ddo.jp/digital/SDISK2LCD/index.html](https://tulip-house.ddo.jp/digital/SDISK2LCD/index.html)

## 概要

SDISK II は、Apple II の DISK II Interface Card に接続し、
SDカード上のディスクイメージをフロッピーディスクとして使用する
DISK II エミュレータです。

本リポジトリでは、オリジナルの SDISK II LCD Firmware に対して、
ハードウェアおよびファームウェアの変更を行っています。

## 主な変更点

オリジナル版からの主な変更点は以下の通りです。

- 5V単一電源で動作するハードウェアへの対応
- FAT16 Long File Name (LFN) 対応
- SDカードアクセス処理の改良
- SRAM / Flash 使用量の最適化
- NICファイル選択機能の変更
- DSKからNICへの変換処理の変更（上下ボタン同時押しで一括変換を開始）
- ATtiny85 + SSD1306 OLEDによる表示
- SDSC / SDHC対応 (FAT16フォーマット限定)
- その他、ハードウェア変更に伴う修正

詳細な変更内容についてはソースコードおよびGitの履歴を参照してください。

## ハードウェア

メインコントローラ:

- ATmega328P
- クロック: 27 MHz

表示コントローラ:

- ATtiny85
- クロック: 16 MHz
- SSD1306 128x64 OLED
- ATmega328Pとの通信: USART 4800 bps

メインファームウェアは `sdisk2.c` および `sub.S`、
OLED表示用ファームウェアは `console_OLED.ino` です。

## SDカード

SDカードは FAT16 でフォーマットされたものを使用します。

オリジナルの SDISK II は32MB～2GBのSDカードを想定して設計されています。
SDカードによっては互換性やアクセス速度に差がある可能性があります。

## ディスクイメージ

主に以下のファイルを使用します。

- `.DSK` : Apple II DOS 3.3 ディスクイメージ
- `.NIC` : SDISK II 内部で使用するディスクイメージ
- `.BTF` : マウントするイメージの管理に使用

この改造版では FAT16 Long File Name (LFN) に対応しています。

## ビルド

メインファームウェアは AVR-GCC を使用してビルドします。

ターゲットMCU:

    ATmega328P

クロック:

    F_CPU = 27000000

リポジトリに含まれる `Makefile` を使用して、

    make

を実行するとファームウェアをビルドできます。

## OLEDファームウェア

`console_OLED.ino` は ATtiny85 用のOLED表示ファームウェアです。

Arduino IDEでビルドする場合はATtiny85のクロックを
Internal 16 MHzに設定してください。

使用ライブラリ:

- U8x8lib
- SoftwareSerial

## 注意

ATmega328Pは27MHzで動作させているため、通常の仕様を超えた
オーバークロック動作になります。

また、SDカードへの書き込みを行うため、
重要なディスクイメージについては事前にバックアップを作成してください。

本ソフトウェアおよびハードウェアの使用は自己責任で行ってください。

## Original Author

Original SDISK II firmware:

Copyright (C) 2010 Koichi NISHIDA

Nishida Radio / 西田ラヂオ

https://tulip-house.ddo.jp/digital/SDISK2/

## Modifications

Modified by Kenichi Iwata, 2026.

## License

This project is licensed under the GNU General Public License version 3
(GPL-3.0-only).

The original SDISK II firmware by Koichi Nishida is also distributed
under the GNU General Public License version 3.

See the `LICENSE` file for details.
