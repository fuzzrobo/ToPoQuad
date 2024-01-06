# ToPoQuad

## 現状実装されている機能
 - Dynamixelの自動検出 [dynamixel_handler/dynamixel_handler_node]
 - Dynamixelとの通信 [dynamixel_handler/dynamixel_handler_node]
 - Dynamixelのエラーの検出とノードの再起動によるエラークリア [dynamixel_handler/dynamixel_handler_node]
 - 脚への角度指令をDynamixelへの角度指令への変換 [topoquad_master/leg_node]
 - 足先の位置をDynamixelへの角度指令へ変換 [topoquad_master/leg_node]
    - 足先位置はBody座標系から見たもの．
    - 単純な3LinkのIKを解いている．
 - 首への角度指令をDynamixelへの角度指令へ変換 [topoquad_master/neck_node]
 - ４脚歩容のサンプル [topoquad_control/leg_sample_control.py]
 - 首のパンチルト機構による物体のトラッキング [topoquad_control/neck_tracking_target, detect_target_color]
    - realsenseの画像から特定の色の位置を検出
    - 画角の中心に物体が来るようにパンチルト角を制御

## 起動方法

### まとめてroslaunch

```
$ roslaunch topoquad_control tracking_target_color_with_sample_walk.launch
```
下記の構成でnodeをまとめて起動する．

```
{topoquad_control}/tracking_target_color_with_sample_walk.launch
   ┣ leg_sample_control.py
   ┣ neck_tracking_target.py
   ┣ detect_target_color.py
   ┗ {topoquad_master}/launch/spider_test.launch
        ┣ leg_node
        ┣ neck_node
        ┗ {dynamixel_handler}/launch/dynamixel_handler.launch
             ┗ dynamixel_handler_node
```
デフォルトだと"Dynamixelとの通信を司るノード"のusb deviceの値が`DEVICE=/dev/ttyUSB0`になっているので，適当に変更すること．
`dynamixel_handler.launch`を複製して，`DEVICE`をラズパイ用に変更した`dynamixel_raspi.launch`を作成するとよいかと思われる．

### 個別にrosrun

#### 1 roscore
```
$ roscore
```

#### 2 Dynamixelとの通信を司るノード
[こちらを参照](https://github.com/ROBOTIS-JAPAN-GIT/DynamixelHandler-ros1/tree/main)

#### 3-1 脚への制御指令を受け付けるノード
```
$ rosrun topoquad_master leg_node
```
ロボットの関節にどのIDのdynamixelがどんな向きでついているかを知っているノード．
脚の関節角の指令をsubして，dynamixelへの角度指令に直してpubしている．

#### 3-1 首への制御指令を受け付けるノード
```
$ rosrun topoquad_master neck_node
```
機能は同上．

#### 4-1 脚への制御指令を出力するノード
```
$ rosrun topoquad_control leg_sample_control.py
```
ロボットの制御を行うためのノード．
脚の関節角の指令をpubし続ける．
サンプルなので歩容は適当．

#### 4-2 首への制御指令を出力するノード
```
$ rosrun topoquad_control neck_tracking_target.py
```
ロボットの制御を行うためのノード．
首の関節角の指令をpubし続ける．
下記nodeからpubされる`/target_position/ratio`トピックの値を`(0,0)`にするように首(pantilt機構に取り付けられたカメラ)を動かす．
制御は適当なpid制御．

```
$ rosrun topoquad_control detect_target_color.py
```
カメラ画像`/camera/color/image_raw`から適当な色(デフォルトは赤)を検出して，その位置の画角に対する割合を`/target_position/ratio`トピックとしてpubする．

## 便利なエイリアスの設定

#### 特定の姿勢をワンコマンドで指令できるようにする．
```
alias pose1="rostopic pub /spider/cmd/leg_angle topoquad_master/QuadRobotCmdLegAngle \
\"
angles_FR: [0, 1.0, 0.52]
angles_FL: [0, 1.0, 0.52]
angles_BR: [0, 1.0, 0.52]
angles_BL: [0, 1.0, 0.52]
\" -1" 
```
これを./bashrcなりに書いておく．
```
$ pose1
```
とすれば，指定した姿勢になるようにros topicがpubされる．


## トピックについて
各pkgのReadMeを参照．
（まだ controlは書けてないので，直接launch or src読んでください，すいません．）

## dynamixel id map

#### 脚
topoquad_master pkg の leg_node が 持っている情報.
 - 後右 :  4  3  2
 - 前右 : 14 13 12
 - 前左 : 24 23 22
 - 後左 : 34 33 32
　　（根元 <--> 足先）

 launchから書き換え可能．

#### 首(optional)
topoquad_master pkg の neck_node が 持っている情報.
 - Pan : 43
 - Tilt : 42

 launchから書き換え可能．

## メモ
dynamixel_handlerは将来リポジトリごと独立させた．

