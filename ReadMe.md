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
git clone --recursive git@github.com:fuzzrobo/ToPoQuad.git -b humble-devel
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
```

#### 1.4 その他のROS 2パッケージのインストール
```bash
sudo apt install ros-humble-plotjuggler
sudo apt install ros-humble-teleop-twist-keyboard 
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

### 3.1. セットアップ
RasPiにssh接続して以下のコマンドを実行
```bash
ros2 launch topoquad_master spider_test.launch.py # ファイル名は要修正
```

### 3.2. 自律でサンプル歩容を試す
RasPiにssh接続して以下のコマンドを実行
```bash
ros2 run topoquad_control walk_node --ros-args --remap __ns:=/ns # launch ファイルにまとめたい
```

### 3.3. テレオペで動かす

 - キーボードで動かす場合
リモートPCで以下のコマンドをそれぞれ別のターミナルで実行
```bash
ros2 run topoquad_control keyboard_node --ros-args --remap __ns:=/ns
```

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap __ns:=/ns
```
２つ目のターミナルにカーソルを合わせた状態で適切なキーを押せばロボットが動く

### 3.4. ロボットの歩容を確認する
リモートPCで以下のコマンドを実行
```bash
ros2 run plotjuggler plotjuggler -l ~/ros2_ws/src/ToPoQuad/topoquad.xml
```
出てくるウィンドウでyesを選択。
Select ROS message というwindowでは /ns/legs/point, /ns/legs/state/goal, /ns/legs/state/present, の3つを選択してOK.




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

