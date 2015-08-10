## 概要

R-Mobile A1 の CEU に接続されたカメラから取り込んだ映像を  
H.264 Encoder で圧縮し、これを RTP パケットとして送信するアプリケーションです。

※ H.264 Encoder のパラメータ調整や、フレームレートの調整の機能は
   実装されていません。
  - これらの調整を行う場合は、`ceu2rtp.c` のマクロ定義を修正し、
    再コンパイルを行ってください。

[Armadillo-810][1], [Armadillo-840][2] で動作することを確認しています。

本アプリケーションは [GStreamer 1.0][3] を使って書かれています。  
GStreamer パイプラインは以下の通りです。

```
appsrc ! h264parse ! rtph264pay ! udpsink
```

`appsrc` は、カメラから映像を取り込む処理とエンコード処理の2つを行っています。  
`appsrc` のコードを読まれる場合は、このファイル末尾の "開発者の方へ" を参照してから、
読むことをおすすめします。

その他のエレメントについては [Overview of available plug-ins][4] を参照してください。

[1]: http://armadillo.atmark-techno.com/armadillo-810
[2]: http://armadillo.atmark-techno.com/armadillo-840
[3]: http://gstreamer.freedesktop.org/
[4]: http://gstreamer.freedesktop.org/documentation/plugins.html

## コンパイル方法

### 必要なもの
  - GStreamer1.0 の dev パッケージ
    - Debian package version `1.0.8-1~bpo70+1` で動作確認済み

###  コンパイル
  - cross-compile
```sh
make
```
  - native-compile
```sh
make KERNEL_DIR=/path/to/linux-3.4-at/include CROSS_COMPILE=
```

  - 任意のカーネルヘッダファイルを使用したい場合
```sh
make KERNEL_DIR=/path/to/linux-3.4-at/include
```

## 実行方法

### 環境

- ハードウェア
  - Armadillo-810 または Armadillo-840
  - 標準カメラモジュール (OP-A810-CAM01-00) または FullHD対応カメラモジュール (KBCR-S02TXG)

- ソフトウェア
  - カーネル
    - CEU のデバイスドライバが DMABUF に対応していること
      - linux-3.4_at15 以降
    - KBCR-S02TXG を使用する場合は専用のデバイスドライバが入っていること
  - ユーザーランド
    - AVコーデックミドルウェア (ACM) が動作すること
      - エンコーダーが有効化されていること

### コマンド例

- Armadillo-810 + FullHD対応カメラモジュール (KBCR-S02TXG) で実行する
```sh
./ceu2rtp
```
※ オプション無しで実行すると localhost (127.0.0.1) に送信されます。  
※ デフォルトで使用するポートは 5004 番です。

- 送信先の IP アドレスおよび、ポート番号を指定する
```sh
./ceu2rtp --ip 192.168.0.5 --port 1234
```

- Armadillo-810 + 標準カメラモジュール (OP-A810-CAM01-00) で実行する
```sh
./ceu2rtp --input-width 640 --input-height 480 --output-width 640 --output-height 480
```

- Armadillo-840 で実行する
  1. ACM のエンコーダーを有効化します
```sh
echo encoder > /sys/devices/platform/acm.0/codec
```
  2. CEU および、H.264 Encoder の デバイスファイルを適切に指定して実行します
```sh
./ceu2rtp -c /dev/video0 -e /dev/video1
```

- アプリケーションの終了させるには `Ctrl-C` を押下してください。

- PC (linux) で受信する際には以下のコマンドを参考にしてください。
```sh
gst-launch-1.0 \
rtpbin name=rtpbin \
udpsrc caps="application/x-rtp,\
             media=video,clock-rate=90000,\
             encoding-name=H264" \
       port=5004 ! rtpbin.recv_rtp_sink_0 \
rtpbin. ! rtph264depay ! avdec_h264 \
! videoconvert ! fpsdisplaysink max-lateness=-1 sync=false
```

### オプション一覧

| long option     | short | 機能                             |
|-----------------|-------|----------------------------------|
| --ip            | -i    | IP アドレス                      |
| --port          | -p    | ポート番号                       |
| --camera        | -c    | CEU のデバイスファイル           |
| --encoder       | -e    | H.264 Encoder のデバイスファイル |
| --input-width   |       | CEU 入力解像度 横                |
| --input-height  |       | CEU 入力解像度 縦                |
| --output-width  |       | Encoder 出力解像度 横            |
| --output-height |       | Encoder 出力解像度 縦            |

## 開発者の方へ

### AppSrc で実装している処理の概要

`appsrc` の処理は `v4l2src` と `acmh264enc` を繋ぎあわせた場合と **ほぼ同等** です。

しかし、`v4l2src` と `acmh264enc` は 2015/07/16 現在、
DMA 転送に対応していません。  
そのため、カメラから高解像度の映像を取得した時、
メモリコピーがボトルネックとなりフレームレートが下がります。

このボトルネックを解消するために、`appsrc` で R-Mobile A1 の CEU から取り込んだ映像を  
H.264 Encoder に DMABUF で転送する処理を実装しています。

### AppSrc Capabilities

```
Pad Templates:
  SINK template: 'sink'
    Availability: Always
    Capabilities:
      video/x-h264
              alignment: au
          stream-format: byte-stream
```

### 構造体階層
```
C2rPipeline
  |-- GstPipeline
  `-- C2rAppsrc
        |-- GstAppSrc
        |-- C2rAppsrcSize
        |-- C2rCamera
        |     |-- C2rDevice
        |     `-- C2rV4l2Buf : CAPTURE 用
        |-- C2rEncoder
        |     |-- C2rDevice
        |     |-- C2rV4l2Buf : OUTPUT 用
        |     `-- C2rV4l2Buf : CAPTURE 用
        |-- C2rTask          : camera  task
        |-- C2rTask          : output  task
        `-- C2rTask          : capture task
```

### Object について

+ C2rPipeline 
  - `main()` が扱う Object
    - C2rPipeline を Play すると Streaming thread が動く
  - メンバ
    - Gstreamer の Pipeline
    - C2rAppSrc (GstAppSrc の Wrapper)
+ C2rAppsrc
  - AppSrc のすべてを含んだ Object
  - メンバ
    - GstAppSrc
    - Devices (Camera, Encoder)
    - Tasks
    - Size (I/O Width, Height)
  - `c2r_appsrc_new()` した後に、各 GstTask を `gst_task_start()` で
    動かす必要がある
+ C2rDevice
  - C2rEncoder と C2rCamera の親 Object
  - デバイスファイル名と fd だけを管理する
+ C2rTask
  - GstTask の Helper object
  - `c2r_task_setup (C2rTask)` で `gst_task_start()` 出来る状態にする
+ C2rV4l2Buf
  - v4l2_buffer の管理を行う Object
  - DMA 転送に必要な v4l2_exportbuffer もこの Object が持つ
  - `C2rIndexQueue` というメンバにより、現在使用 (Enqueue) 可能な
    index を管理している

