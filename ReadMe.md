# ToPoQuad

## セットアップ手順

### 1. ロボット側コンピュータの環境設定 (Raspberry Pi 4b)

#### 1.1. Raspberry Pi 4b のセットアップについて

#### 1.2. ROS 2 Humble のインストール

[ROS 公式のインストールガイド](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)に従って，ROS 2 Humbleをインストールします．

まず，Ubuntu Universe リポジトリが有効になっていることを確認します．
```bash
$ sudo apt install -y software-properties-common
$ sudo add-apt-repository universe
```

ROS 2 Humbleをインストールします．
```bash
$ sudo apt update && sudo apt -y install curl gnupg lsb-release
$ sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
$ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
$ sudo apt update
$ sudo apt install -y ros-humble-desktop
$ echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
$ sudo apt install -y python3-colcon-common-extensions python3-pip
```

#### 1.3. ToPoQuad用 ROS 2 パッケージのインストール

```bash
cd ~/ros2_ws/src
git clone --recursive https://github.com/ROBOTIS-JAPAN-GIT/ToPoQuad.git -b humble-devel
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
```

### 2. リモートPCの環境設定

#### 2.1. リモートPCのセットアップについて
ロボットを完全自律で動かす場合を除き、リモートPC側にもROS 2環境やドライバをインストールする必要があります。

#### 2.2. ROS 2 Humbleのインストール

- ROS 2 がインストールされている場合

    ワークスペースを作成します．
    ```bash
    $ mkdir -p ~/turtlebot3_ws/src
    $ cd ~/turtlebot3_ws && colcon build --symlink-install && . install/setup.bash
    ```
    ワークスペースやROS_DOMAINを設定します．
    ```bash
    $ echo '. ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
    $ echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
    $ source ~/.bashrc
    ```


- ROS 2 がインストールされていない場合
    <details>
  
    <summary><a href="#12-ros-2-humble-のインストール">1.2. ROS 2 Humble のインストール</a>と同様です．</summary>
    <a href="[#12-ros-2-humble-のインストール](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html))">ROS 公式のインストールガイド</a>に従って，ROS 2 Humbleをインストールします．
    まず，Ubuntu Universe リポジトリが有効になっていることを確認します．
    ```bash
    $ sudo apt install -y software-properties-common
    $ sudo add-apt-repository universe
    ```

    ROS 2 Humbleをインストールします．
    ```bash
    $ sudo apt update && sudo apt -y install curl gnupg lsb-release
    $ sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    $ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    $ sudo apt update
    $ sudo apt install -y ros-humble-desktop
    $ echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    $ source ~/.bashrc
    $ sudo apt install -y python3-colcon-common-extensions python3-pip
    ```

    ワークスペースを作成します．
    ```bash
    $ mkdir -p ~/turtlebot3_ws/src
    $ cd ~/turtlebot3_ws && colcon build --symlink-install && . install/setup.bash
    ```
    ワークスペースやROS_DOMAINを設定します．
    ```bash
    $ echo '. ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
    $ echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
    $ source ~/.bashrc
    ```
    </details>

## 3. 実機での動かし方

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


## dynamixel id map

### Leg/脚
topoquad_master pkg の leg_node が 持っている情報.
 - 後右 :  4  3  2
 - 前右 : 14 13 12
 - 前左 : 24 23 22
 - 後左 : 34 33 32
　　（根元 <--> 足先）

 launchから書き換え可能．

### Neck/首 (optional)
topoquad_master pkg の neck_node が 持っている情報.
 - Pan : 43
 - Tilt : 42

 launchから書き換え可能．

