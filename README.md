# ROSでダンシングライダーを制御

## 前提

### ハードウェア構成

#### PC

1. Ubuntu/ROSが動作するPC
1. ゲームパッド

    ラジコン操作のUIとして、アナログスティックを利用。

    今回は、 [エレコム JC-U3912TBK](https://www.amazon.co.jp/dp/B01NCIYC3F) を利用。

#### Wi-Fiアクセスポイント

1. Wi-Fiルータ

    PCとラジコンを接続するために必要。PCをアクセスポイントにする、スマホを使うなどでも可。

#### ラジコン

1. ダンシングライダー本体

    [タミヤ ダンシングライダー](https://www.amazon.co.jp/dp/B07CZHQX75) の組み立て済みのものを利用。
    
    プロポ、受信機などもついてるけど、今回は利用しない。

1. ESC

    PWMでモータを制御、および、電源をマイコンボードに供給するために利用。

    今回は、[GoolRC](https://www.amazon.co.jp/dp/B01IT4YOV4) が安く買えたので利用。

1. マイコンボード

    ESP32頭囲の自作ボードを使ったけど、電源さえうまく確保できるのであれば、ESP32-DevKitCやM5Stackなどでも対応可。

##### オプション

1. バッテリ

    ラジコンの電源として [タミヤ LFバッテリー LF1100-6.6V](https://www.amazon.co.jp/dp/B002FL4PNK) を利用。
    
    単三電池4本でも可。

1. ヘッドライトLED

    [抵抗内蔵５ｍｍ白色ＬＥＤ（１２Ｖ用）](http://akizukidenshi.com/catalog/g/gI-06254/) を利用。

    ラジコン側の初期化完了時に点灯するように利用。

### ソフトウェアについて

#### PC

1. Ubuntu18.04
1. ROS melodic

    ROS melodicを採用した理由は特になし。サンプルを動かすときになんとなくそうなったのでそのまま。

    利用したパッケージは次の通り。

    1. joy
    1. rosserial
    1. rosserial_arduino

#### Arduino

1. Arduino IDE
1. arduino-esp32をインストール

    インストールは、 [arduino-esp32/debian_ubuntu.md at master · espressif/arduino-esp32](https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/debian_ubuntu.md) を参照。

##### rosserial-arduino

ROS melodicだとArduinoで使えるROSライブラリが作られてないので、自前で作る必要がある。

```
// Arduino IDEが参照するライブラリディレクトリに移動
$ cd <Arduino Sketch root>/libraries
// ライブラリ生成を実行
$ rosrun rosserial_arduino make_libraries.py .
```

参照: [rosserial を用いてROSとArduinoの間の通信を行う - うごくものづくりのために](http://makemove.hatenablog.com/entry/2015/04/05/181604)

## ビルド

### PC

1. ROSワークスペースディレクトリ内のソースディレクトリ内に、``src/ros`` ディレクトリをコピーする。
1. ROSワークスペースディレクトリで ``catkin_make`` を実行する。

### Adrudino

1. Arduino IDEで ``src/Arduino/DancingRider.ino`` を開く。
1. 利用するESP32に対して書き込みを実行。

## 実行

1. PCで ``ros_dancing_rider`` を ``roslaunch`` を実行

    ```
    $ roslaunch ros_dancing_rider ros_dancing_rider.launch
    ```

1. ESP32の電源を投入。
