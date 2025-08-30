# ToPoQuad

## セットアップ手順

### 1. ロボット側の環境設定 (Raspberry Pi 4b と OpenCR)

#### 1.1. Raspberry Pi 4b のセットアップについて

ラズパイにUbuntuがクリーンインストールされている前提で説明します．

#### 1.2. ROS 2 Humble のインストール

[ROS 公式のインストールガイド](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)に従って，ROS 2 Humbleをインストールします．

まず，Ubuntu Universe リポジトリが有効になっていることを確認します．
```bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe
```

ROS 2 Humbleをインストールします．
```bash
sudo apt update && sudo apt -y install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install -y python3-colcon-common-extensions python3-pip
```

ワークスペースを作成します．
```bash
mkdir -p ~/topoquad_ws/src
cd ~/topoquad_ws && colcon build --symlink-install && . install/setup.bash
```

ワークスペースを設定します．
```bash
echo '. ~/topoquad_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

#### 1.3. ToPoQuad用 ROS 2 パッケージのインストール

```bash
cd ~/topoquad_ws/src
git clone --recursive git@github.com:fuzzrobo/ToPoQuad.git -b humble-devel
cd ~/topoquad_ws && colcon build --symlink-install && source install/setup.bash
```

`$ git clone ...`でエラーが出る場合は，以下を実行して git をインストールしてください．
```bash
sudo apt update
sudo apt install -y git
```
インストールが完了したら，再度上記の `git clone` 以降のコマンドを実行してください．


#### 1.4. ttyACM0へのアクセス権を付与

```bash
wget https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/99-opencr-cdc.rules
sudo cp ./99-opencr-cdc.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
rm ./99-opencr-cdc.rules
```

#### 1.5. OpenCRのセットアップ

OpenCRをにファームウェアを書き込み，dynamixelとの通信とOpenCR内のIMUデータをRaspberry Piに送信できるようにします．

Arduino IDEを用いて，`topoquad_master/opencr/usb_topdxl_imu/usb_to_imu.ino`をOpenCRボードに書き込みます．

Arduino IDEのインストール方法やOpenCRのセットアップ方法については，以下を参照してください．  
[Open CR v1.0 公式サイト](https://emanual.robotis.com/docs/en/parts/controller/opencr10/#arduino-ide)

#### 1.6. SSH接続の設定

Raspberry Piをホストとして，ssh接続できるように設定します．

```bash
sudo apt update
sudo apt install -y openssh-server
sudo reboot
```

再起動後，以下のコマンドでraspberry piのIPアドレスを確認し，メモします．
```bash
hostname -I
```

### 2. リモートPCの環境設定

#### 2.1. リモートPCのセットアップについて
ロボットを完全自律で動かす場合を除き、リモートPC側にもROS 2環境やドライバをインストールする必要があります。

#### 2.2. ROS 2 環境のセットアップ

ROS 2 がインストールされている前提で説明します．

ワークスペースを作成します．
```bash
mkdir -p ~/topoquad_ws/src
cd ~/topoquad_ws && colcon build --symlink-install && . install/setup.bash
```

ワークスペースを設定します．
```bash
echo '. ~/topoquad_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

<details>

<summary> ROS 2 がインストールされていない場合は <a href="#12-ros-2-humble-のインストール">1.2. ROS 2 Humble のインストール</a>と同様に導入してください．</summary>
<a href="[#12-ros-2-humble-のインストール](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html))">ROS 公式のインストールガイド</a>に従って，ROS 2 Humbleをインストールします．
まず，Ubuntu Universe リポジトリが有効になっていることを確認します．

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe
```

ROS 2 Humbleをインストールします．
```bash
sudo apt update && sudo apt -y install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install -y python3-colcon-common-extensions python3-pip
```

ワークスペースを作成します．
```bash
mkdir -p ~/topoquad_ws/src
cd ~/topoquad_ws && colcon build --symlink-install && . install/setup.bash
```

ワークスペースを設定します．
```bash
echo '. ~/topoquad_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```
</details>

#### 2.3. ToPoQuad用 ROS 2 パッケージのインストール

```bash
cd ~/topoquad_ws/src
git clone --recursive git@github.com:fuzzrobo/ToPoQuad.git -b humble-devel
cd ~/topoquad_ws && colcon build --symlink-install && source install/setup.bash
```

`$ git clone ...`でエラーが出る場合は，以下を実行して git をインストールしてください．
```bash
sudo apt update
sudo apt install -y git
```
インストールが完了したら，再度上記の `git clone` 以降のコマンドを実行してください．

#### 2.4. その他のROS 2パッケージのインストール
```bash
sudo apt install ros-humble-plotjuggler
sudo apt install ros-humble-teleop-twist-keyboard 
```

#### 2.5. Raspberry PiへのSSH接続の確認

[1.6. SSH接続の設定](#16-ssh接続の設定)で確認したRaspberry PiのIPアドレスを用いて，以下のコマンドでssh接続できることを確認します．
```bash
ssh your_username@<Raspberry_PiのIPアドレス>
```

## 3. 実機での動かし方

### 3.1. セットアップ
RasPiにssh接続して以下のコマンドを実行
```bash
ros2 launch topoquad_bringup hardware.launch.py
```

### 3.2. 自律でサンプル歩容を試す
RasPiにssh接続して以下のコマンドを実行
```bash
# 新しいターミナルで実行
ros2 launch topoquad_control sample_walk.launch.py
```

### 3.3. テレオペで動かす

 - コントローラで動かす場合
```bash
# yet
```

 - キーボードで動かす場合
リモートPCで以下のコマンドをそれぞれ別のターミナルで実行
```bash
# 新しいターミナルで実行
ros2 run topoquad_control keyboard_node --ros-args --remap __ns:=/topoquad
```

```bash
# 新しいターミナルで実行
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap __ns:=/topoquad
```
２つ目のターミナルにカーソルを合わせた状態で適切なキーを押せばロボットが動く

### 3.4. ロボットの歩容を確認する
リモートPCで以下のコマンドを実行
```bash
# 新しいターミナルで実行
ros2 run plotjuggler plotjuggler -l ~/topoquad_ws/src/ToPoQuad/plot_config.xml
```
出てくるウィンドウでyesを選択。
Select ROS message という window では `/topoquad/legs/command`, `/topoquad/legs/state/goal`, `/topoquad/legs/state/present`, の3つを選択してOK.

## その他

### Dynamixel の Baudrate を変えたくなったら

始めに，Raspberry Pi 内の `/topoquad_master/config/dynamixel_unify_baudrate.yaml` の `target_baudrate` を変更する．
```yml
/**:
    ros__parameters:
    #  指定可能なボーレート
        # 9600    
        # 57600   
        # 115200  
        # 1000000 
        # 2000000 
        # 3000000 
        # 4000000 
    # 通信機器の設定
        device_name: /dev/ttyACM0 # 通信するデバイス名
        target_baudrate: 1000000 # 通信速度
        latency_timer: 1 # 通信のインターバル
    # 探索するサーボの設定
        min_id: 0
        max_id: 40
        min_search_baudrate: 57600
        max_search_baudrate: 4000000
```

保存した後，Raspberry Pi で以下を実行する．

```bash
ros2 launch topoquad_master dynamixel_unify_baudrate.launch.py
```

接続しているすべてのDynamixelを探索して，ボーレートを統一する．