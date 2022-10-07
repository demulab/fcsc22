# sawyer_ir
PCに接続したIRMagicianからjsonファイルの赤外線信号を送信するパッケージ。

Sawyerから自動開閉棚に向けて開閉の指示を出すのに用いる。
## 基本となる使用方法
roscoreを起動。

- roscore

新しくターミナルを開き、rosrunでirmagician_controll.pyを起動

- rosrun sawyer_ir irmagician_controll.py

新しくターミナルを開き、rosrunでirmagician_switch.pyを起動。起動後、a ~ fの文字を入力すると指令がpublishされる

- rosrun sawyer_ir irmagician_switch.py

## 使用にあたる注意点
### irmcli.pyの場所
sawyer_irではirmagician_controll.pyからIRMagicianにどのjsonファイルを用いて赤外線信号を出すか指示を送っているが、その際にsubprocessのcallを用いてirmcli.pyを起動している。

このとき、irmagician_controll.pyの中のsubprocess.callでirmcli.py及びjsonファイルの場所が適切に表記されているか注意する。場所の指定は/homeから始まる。

### Ir.msgのインポート
sawyer_irではトピック通信にIr.msgというカスタムmsgファイルを用いている。Ir.msgにはString型のirsignalが定義されており、これをスクリプトにインポートしたあとirsiginalに特定の文字列（例：high_shelf_open）を代入し、それをpublishすることでirmagician_controllが指示の判断を行える。

もしirmagician_switch.pyを用いらず、別のスクリプトからPublisherを使ってトピックを送るためにはスクリプトの上の方に、以下の一文が必要

from sawyer_ir.msg import Ir

publishの仕方などはirmagician_switch.pyを参考にする

### jsonファイル
現在jsonは一番上の棚が展開・格納する信号しかない。信号増やす場合は、IRMagicianを使って信号を学習、保存する必要がある。

IRMagicianを接続後、学習用のコマンドを入力。irmcli.pyの場所は適宜変更
- python /home/ユーザ名/path/to/irmcli.py -c

信号をjsonファイルに出力。jsonファイル
- python /home/ユーザ名/path/to/irmcli.py -s -f ファイル名.json

### udevの設定
sawyer_irではIRMagicianのデバイス名をttyACM_IRMAGICIANに変更している。使用する際には、udevの設定をする必要がある。
