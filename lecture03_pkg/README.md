# Lecture 3: TurtleBot3，SLAM，Navigation2

## 1. 準備

### 1.1 演習資料と演習パッケージのダウンロード

下記のリンクにアクセスして資料と演習パッケージをダウンロードしてください．

- 資料のダウンロード：[handout_ros2_lecture_03_ja.pdf](https://drive.google.com/uc?export=download&id=1xSSBobktiKBlyYBVCmupPuYNo209aTap)
- 演習パッケージのダウンロード：[lecture03_pkg.tar.gz](https://drive.google.com/uc?export=download&id=1w0DlbitBY_b90GVlRmUftmcIoED5rH99)

### 1.2 README.mdとPythonスクリプトの移動

下記にしたがって，ダウンロードした演習パッケージを移動してください．

- ダウンロードした演習パッケージ(`lecture03_pkg.tar.gz`)を解凍してください．
- 解凍した演習パッケージ(`lecture03_pkg`)を`~/ros2_lecture_ws/src/7_lectures`に移動してください．

### 1.3 パッケージのビルド

下記のコマンドを実行してください．

- 端末1
  - 仮想環境の起動等(<font color="Yellow">実行済みの場合はスキップ</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - パッケージをビルド

    ```Bash
    colcon build --symlink-install
    ```

ビルドが終了したら，仮想環境を終了してください（`Ctrl+D`）．

## 2. TurtleBot3（実機）

### 2.1 キーボード入力でTurteBot3を制御

#### 2.1.1 実行方法

- 端末1（ 仮想環境を起動せずに実行してください．）
  - 時刻同期

    ```Bash
    turtlebot3_mode
    ```

- 端末2
  - TurtleBot3にリモートアクセス

    ```Bash
    ssh -YC turtle@192.168.11.2
    ```

    パスワード：turtlebot
    <font color="Yellow">※パスワードは入力しても表示されません．</font>

  - システムの起動

    ```Bash
    ros2 launch ros2_lecture bringup.launch.py
    ```

- 端末3
  - 仮想環境の起動等(<font color="Yellow">実行済みの場合はスキップ</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - ROS_DOMAIN_IDとTurtleBot3の環境変数を設定

    ```Bash
    . 4a_turtlebot3_settings.sh
    ```

  - teleop_keyboardノードを実行

    ```Bash
    ros2 run turlebot3_teleop teleop_keyboard
    ```

    a, w, s, x, dキーを押してTurtleBot3を操作してみましょう

### 2.2 キーボード入力でTurteBot3を制御

TurtleBot3を前進，停止させるサンプルコード

#### 2.2.1 実行方法（テレオペ）

- 端末1（<font color="Yellow">実行済みの場合はスキップ</font>）
  - 時刻同期

    ```Bash
    turtlebot3_mode
    ```

- 端末2（<font color="Yellow">実行済みの場合はスキップ</font>）
  - TurtleBot3にリモートアクセス

    ```Bash
    ssh -YC turtle@192.168.11.2
    ```

    パスワード：turtlebot
    <font color="Yellow">※パスワードは入力しても表示されません．</font>

  - システムの起動

    ```Bash
    ros2 launch ros2_lecture bringup.launch.py
    ```

- 端末3
  - 仮想環境の起動等(<font color="Yellow">実行済みの場合はスキップ</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - ROS_DOMAIN_IDとTurtleBot3の環境変数を設定

    ```Bash
    . 4a_turtlebot3_settings.sh
    ```

  - turtlebot_controllerノードを実行(前進)

    ```Bash
    source install/setup.bash
    ```

    ```Bash
    ros2 run lecture03_pkg turtlebot_controller
    ```

  - 一定時間動作させてから`CTRL`+`C`キーでノードを停止

  - turtlebot_stopノードを実行(停止)

    ```Bash
    ros2 run lecture03_pkg turtlebot_stop
    ```

### 2.3 演習1

その場で旋回させるプログラム

#### 2.3.1 実行方法

- 端末1（<font color="Yellow">実行済みの場合はスキップ</font>）
  - 時刻同期

    ```Bash
    turtlebot3_mode
    ```

- 端末2（<font color="Yellow">実行済みの場合はスキップ</font>）
  - TurtleBot3にリモートアクセス

    ```Bash
    ssh -YC turtle@192.168.11.2
    ```

    パスワード：turtlebot
    <font color="Yellow">※パスワードは入力しても表示されません．</font>

  - システムの起動

    ```Bash
    ros2 launch ros2_lecture bringup.launch.py
    ```

- 端末3
  - 仮想環境の起動等(<font color="Yellow">実行済みの場合はスキップ</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - ROS_DOMAIN_IDとTurtleBot3の環境変数を設定

    ```Bash
    . 4a_turtlebot3_settings.sh
    ```

  - turtlebot_exercise1ノードを実行

    ```Bash
    source install/setup.bash
    ```

    ```Bash
    ros2 run lecture03_pkg turtlebot_exercise1
    ```

### 2.4 演習2

30cm前進して時計回りに90度旋回する動作を繰り返すプログラム

#### 2.4.1 実行方法

- 端末1（<font color="Yellow">実行済みの場合はスキップ</font>）
  - 時刻同期

    ```Bash
    turtlebot3_mode
    ```

- 端末2（<font color="Yellow">実行済みの場合はスキップ</font>）
  - TurtleBot3にリモートアクセス

    ```Bash
    ssh -YC turtle@192.168.11.2
    ```

    パスワード：turtlebot
    <font color="Yellow">※パスワードは入力しても表示されません．</font>

  - システムの起動

    ```Bash
    ros2 launch ros2_lecture bringup.launch.py
    ```

- 端末3
  - 仮想環境の起動等(<font color="Yellow">実行済みの場合はスキップ</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - ROS_DOMAIN_IDとTurtleBot3の環境変数を設定

    ```Bash
    . 4a_turtlebot3_settings.sh
    ```

  - turtlebot_exercise2ノードを実行

    ```Bash
    source install/setup.bash
    ```

    ```Bash
    ros2 run lecture03_pkg turtlebot_exercise2
    ```

## 3. Simultaneous Localization And Mapping（SLAM）

### 3.1 SLAMの実行

- 端末1（<font color="Yellow">実行済みの場合はスキップ</font>）
  - 時刻同期

    ```Bash
    turtlebot3_mode
    ```

- 端末2（<font color="Yellow">実行済みの場合はスキップ</font>）
  - TurtleBot3にリモートアクセス

    ```Bash
    ssh -YC turtle@192.168.11.2
    ```

    パスワード：turtlebot
    <font color="Yellow">※パスワードは入力しても表示されません．</font>

  - システムの起動

    ```Bash
    ros2 launch ros2_lecture bringup.launch.py
    ```

- 端末3
  - 仮想環境の起動等(<font color="Yellow">実行済みの場合はスキップ</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - ROS_DOMAIN_IDとTurtleBot3の環境変数を設定

    ```Bash
    . 4a_turtlebot3_settings.sh
    ```

  - teleop_keyboardノードを実行

    ```Bash
    ros2 run turlebot3_teleop teleop_keyboard
    ```

- 端末4
  - 仮想環境の起動等(<font color="Yellow">実行済みの場合はスキップ</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - ROS_DOMAIN_IDとTurtleBot3の環境変数を設定

    ```Bash
    . 4a_turtlebot3_settings.sh
    ```

  - SLAMを実行

    ```Bash
    ros2 launch turtlebot3_cartographer cartographer.launch.py
    ```

- 端末5
  - 仮想環境の起動等(<font color="Yellow">実行済みの場合はスキップ</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - ROS_DOMAIN_IDとTurtleBot3の環境変数を設定

    ```Bash
    . 4a_turtlebot3_settings.sh
    ```

  - 作成した環境地図を保存

    ```Bash
    source install/setup.bash
    ```

    ```Bash
    ros2 run nav2_map_server map_saver_cli –f ~/ros2_lecture_ws/map
    ```

### 3.2 SLAMとナビゲーションの実行

- 端末1（<font color="Yellow">実行済みの場合はスキップ</font>）
  - 時刻同期

    ```Bash
    turtlebot3_mode
    ```

- 端末2（<font color="Yellow">実行済みの場合はスキップ</font>）
  - TurtleBot3にリモートアクセス

    ```Bash
    ssh -YC turtle@192.168.11.2
    ```

    パスワード：turtlebot
    <font color="Yellow">※パスワードは入力しても表示されません．</font>

  - システムの起動

    ```Bash
    ros2 launch ros2_lecture bringup.launch.py
    ```

- 端末3
  - 仮想環境の起動等(<font color="Yellow">実行済みの場合はスキップ</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - ROS_DOMAIN_IDとTurtleBot3の環境変数を設定

    ```Bash
    . 4a_turtlebot3_settings.sh
    ```

  - navigation2システムを起動

    ```Bash
    ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/ros2_lecture_ws/map.yaml
    ```

    - Rviz2の画面上の「2D Pose Estimate」をクリック
    - ロボットの初期位置をクリックしたまま，ロボットの向きにあわせてドラックしてください．
    - Rviz2の画面上の「Navigation2 Goal」をクリック
    - 移動させる目標位置をクリックしたまま，目標地点でのロボットの向きにドラックしてください．

### 3.3 環境地図におけるロボットの位置と姿勢の表示

- 端末1（<font color="Yellow">実行済みの場合はスキップ</font>）
  - 時刻同期

    ```Bash
    turtlebot3_mode
    ```

- 端末2（<font color="Yellow">実行済みの場合はスキップ</font>）
  - TurtleBot3にリモートアクセス

    ```Bash
    ssh -YC turtle@192.168.11.2
    ```

    パスワード：turtlebot
    <font color="Yellow">※パスワードは入力しても表示されません．</font>

  - システムの起動

    ```Bash
    ros2 launch ros2_lecture bringup.launch.py
    ```

- 端末3（<font color="Yellow">実行済みの場合はスキップ</font>）
  - 仮想環境の起動等(<font color="Yellow">実行済みの場合はスキップ</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - ROS_DOMAIN_IDとTurtleBot3の環境変数を設定

    ```Bash
    . 4a_turtlebot3_settings.sh
    ```

  - navigation2システムを起動

    ```Bash
    ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/ros2_lecture_ws/map.yaml
    ```

    - Rviz2の画面上の「2D Pose Estimate」をクリック
    - ロボットの初期位置をクリックしたまま，ロボットの向きにあわせてドラックしてください．

- 端末4
  - 仮想環境の起動等(<font color="Yellow">実行済みの場合はスキップ</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - ROS_DOMAIN_IDとTurtleBot3の環境変数を設定

    ```Bash
    . 4a_turtlebot3_settings.sh
    ```

  - show_poseノードの実行

    ```Bash
    . ros2 run lecture03_pkg show_pose
    ```

    - 最低3か所でロボットの位置と姿勢をメモしてください．後ほど使用します．

### 3.3 自作ノードでナビゲーションを動かす

先程メモした目的地の位置から1つ選んで指定してナビゲーションを実行(`NavigateToPose`

#### 3.3.1 実行方法

- 端末1（<font color="Yellow">実行済みの場合はスキップ</font>）
  - 時刻同期

    ```Bash
    turtlebot3_mode
    ```

- 端末2（<font color="Yellow">実行済みの場合はスキップ</font>）
  - TurtleBot3にリモートアクセス

    ```Bash
    ssh -YC turtle@192.168.11.2
    ```

    パスワード：turtlebot\
    <font color="Yellow">※パスワードは入力しても表示されません．</font>

  - システムの起動

    ```Bash
    ros2 launch ros2_lecture bringup.launch.py
    ```

- 端末3（<font color="Yellow">実行済みの場合はスキップ</font>）
  - 仮想環境の起動等(<font color="Yellow">実行済みの場合はスキップ</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - ROS_DOMAIN_IDとTurtleBot3の環境変数を設定

    ```Bash
    . 4a_turtlebot3_settings.sh
    ```

  - navigation2システムを起動

    ```Bash
    ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/ros2_lecture_ws/map.yaml
    ```

    - Rviz2の画面上の「2D Pose Estimate」をクリック
    - ロボットの初期位置をクリックしたまま，ロボットの向きにあわせてドラックしてください．

- 端末4
  - 仮想環境の起動等(<font color="Yellow">実行済みの場合はスキップ</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - ROS_DOMAIN_IDとTurtleBot3の環境変数を設定

    ```Bash
    . 4a_turtlebot3_settings.sh
    ```

  - show_poseノードの実行

    ```Bash
    . ros2 run lecture03_pkg navigation_sample1
    ```

### 3.4 複数指定でナビゲーションを動かす

先程メモしたすべての目的地の位置を指定してナビゲーションを実行(`FollowWaypoints`)

#### 3.4.1 実行方法

- 端末1（<font color="Yellow">実行済みの場合はスキップ</font>）
  - 時刻同期（ホスト側）

    ```Bash
    turtlebot3_mode
    ```

- 端末2（<font color="Yellow">実行済みの場合はスキップ</font>）
  - TurtleBot3にリモートアクセス（ホスト側）

    ```Bash
    ssh -YC turtle@192.168.11.2
    ```

    パスワード：turtlebot
    <font color="Yellow">※パスワードは入力しても表示されません．</font>

  - システムの起動

    ```Bash
    ros2 launch ros2_lecture bringup.launch.py
    ```

- 端末3（<font color="Yellow">実行済みの場合はスキップ</font>）
  - 仮想環境の起動等(<font color="Yellow">実行済みの場合はスキップ</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - ROS_DOMAIN_IDとTurtleBot3の環境変数を設定

    ```Bash
    . 4a_turtlebot3_settings.sh
    ```

  - navigation2システムを起動

    ```Bash
    ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/ros2_lecture_ws/map.yaml
    ```

    - Rviz2の画面上の「2D Pose Estimate」をクリック
    - ロボットの初期位置をクリックしたまま，ロボットの向きにあわせてドラックしてください．

- 端末4
  - 仮想環境の起動等(<font color="Yellow">実行済みの場合はスキップ</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - ROS_DOMAIN_IDとTurtleBot3の環境変数を設定

    ```Bash
    . 4a_turtlebot3_settings.sh
    ```

  - show_poseノードの実行

    ```Bash
    . ros2 run lecture03_pkg navigation_sample2
    ```

