# Raspberry Pi Pico WH NTP Stratum 1 Server

Raspberry Pi Pico WH と GPS モジュールを使用して構築する、高精度な NTP Stratum 1 サーバです。  
FreeRTOSを使った開発の練習として作ったものです。GPS の PPS (Pulse Per Second) 信号に同期したマイクロ秒精度の時刻配信を実現しています。

## 特徴
- **Stratum 1**: GPS 衛星からの時刻情報を直接参照します。
- **PPS 同期**: 1秒周期のパルス信号により、マイコン内部時計のドリフトを補正します。
- **Wi-Fi 設定**: 起動時にシリアルポートからインタラクティブに Wi-Fi 設定が可能です。
- **自動接続**: 設定した SSID/パスワードはフラッシュメモリに保存され、次回以降自動接続されます。
- **Web 監視**: ブラウザから GPS の捕捉状態やシステムメトリクスをリアルタイムに確認できます。

## 必要なハードウェア
- Raspberry Pi Pico WH
- GPS モジュール (NEO-6M / NEO-8M 等、PPS 出力があるもの)
- ジャンパーワイヤ

### 配線
| GPS側ピン | Pico WH 側 (GPIO) | Pico 物理ピン | 役割 |
| :--- | :--- | :--- | :--- |
| **VCC** | **3V3(OUT)** | 36番ピン | 電源 (3.3V) |
| **GND** | **GND** | 38番ピン | グランド |
| **TX** | **GP1 (UART0 RX)** | 2番ピン | NMEAデータ受信 |
| **PPS** | **GP2** | 4番ピン | 1秒パルス信号 |

## 開発環境の構築

### 1. ツールチェーンのインストール
- **ARM GNU Toolchain**: [Arm GNU Toolchain](https://developer.arm.com/downloads/-/gnu-rm) から、`arm-none-eabi-gcc` をインストールし、システムパスを通してください。
- **CMake**: [cmake.org](https://cmake.org/download/) からインストールしてください。
- **Ninja** (推奨) または Make: [Ninja Build](https://github.com/ninja-build/ninja/releases) 等をインストールしてください。

### 2. リポジトリの準備
このプロジェクトをクローンし、依存する SDK とカーネルをプロジェクト直下に配置します。

```bash
# プロジェクトのクローン
git clone https://github.com/tetsutalow/pico-ntp-server.git
cd pico-ntp-server

# Pico SDK のクローンとサブモジュールの初期化
git clone --depth 1 https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init --recursive
cd ..

# FreeRTOS Kernel のクローン
git clone --depth 1 https://github.com/FreeRTOS/FreeRTOS-Kernel.git
```

## ビルド手順

```bash
mkdir build
cd build

# PICO_SDK_PATH を指定して CMake を実行
# Windows (PowerShell) の例:
$env:PICO_SDK_PATH = "..\pico-sdk"
cmake -G "Ninja" ..

# ビルド実行
ninja
```

ビルドが成功すると `build/ntp_server.uf2` が生成されます。

## 使い方
1. Pico WH の BOOTSEL ボタンを押しながら PC にUSB接続するとマスストレージとして認識されますので、そのドライブに`ntp_server.uf2` をコピーします。
2. コピーが終わるとシステムがブートします。USBがCOMデバイスとして認識されるのでシリアル端末（Tera Term, PuTTY 等）を 115200bps で開きます。
3. 画面の指示に従い Wi-Fi をスキャン・選択し、パスワードを入力します（10秒待てば前回設定に自動接続しますので2回目以降は不要です）。
4. 接続に成功すると、IP アドレスが表示されます。
5. ブラウザで `http://<表示されたIPアドレス>/` にアクセスすると、デバッグ画面が表示されます。
6. GPS が Fix し、UTC 秒が取得されると NTP サーバが有効になります。

### 時刻同期の確認
外部の NTP クライアントから動作を確認できます。
```bash
ntpdate -q <PicoのIPアドレス>
```



## 免責事項
このプロジェクトは実験的なものであり、ミッションクリティカルな用途での精度を保証するものではありません。このコードは無保証であり、作者は利用によって生じたいかなる事故も補償しません。

## バージョン履歴
2026/3/12 v0.1 公開 ほぼGemini CLI一発生成版
