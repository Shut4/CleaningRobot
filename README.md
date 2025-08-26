# Lecture 4: State Machine，roslaunch2

## 1. 準備

### 1.1 演習資料と演習パッケージのダウンロード

下記のリンクにアクセスして資料と演習パッケージをダウンロードしてください．

- 資料のダウンロード：[handout_ros2_lecture_04_ja.pdf](https://drive.google.com/uc?export=download&id=1wW6KGdHoVjEAapLMGMNCM9ei3cf2qwAO)
- 演習パッケージのダウンロード：[lecture04_pkg.tar.gz](https://drive.google.com/uc?export=download&id=18crysb3YTTMJGa45pQBkvg4E9HhRCpHy)

### 1.2 パッケージの作成

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

  - パッケージの作成

    ```Bash
    cd src/7_lectures/lecture04
    ```

    ```Bash
    ros2 pkg create lecture04_pkg --build-type ament_python
    ```

### 1.2 README.mdとPythonスクリプトの移動

下記にしたがって，ダウンロードした演習パッケージを移動してください．

- ダウンロードした演習パッケージ(`lecture04_pkg.tar.gz`)を解凍してください．
- 解凍した演習パッケージ(`lecture04_pkg`)を`~/ros2_lecture_ws/src/7_lectures`に移動してください．

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

## 2. State Machine（YASMIN: Yet Another State MachINe）

### 2.1 YASMIN

- [arXiv論文](https://arxiv.org/pdf/2205.13284)
- [Githubリポジトリ](https://github.com/uleroboticsgroup/yasmin.git)

### 2.2 ステートマシンのサンプル1を動かす

#### 2.2.1 実行方法

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

  - Yasmin Viewerノードの起動

    ```Bash
    ros2 run yasmin_viewer yasmin_viewer_node
    ```

    - [http://localhost:5000/](http://localhost:5000/)にアクセス

- 端末2
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

  - sm_sample1ノードの実行

    ```Bash
    source install/setup.bash
    ```

    ```Bash
    ros2 run lecture04_pkg sm_sample1
    ```

### 2.3 演習課題（sm_exercise）

sm_sample1のステートマシンに`HOGE`ステートを追加してみましょう．
sm_exercise.pyを編集してください．

- 返り値”outcome3” → ステート“FOO”に遷移
- 返り値”outcome4” → ステート“BAR”に遷移

### 2.3.1 実行方法

- 端末1(<font color="Yellow">実行済みの場合はスキップ</font>)
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

  - Yasmin Viewerノードの起動

    ```Bash
    ros2 run yasmin_viewer yasmin_viewer_node
    ```

    - [http://localhost:5000/](http://localhost:5000/)にアクセス

- 端末2
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

  - sm_sample1ノードの実行

    ```Bash
    source install/setup.bash
    ```

    ```Bash
    ros2 run lecture04_pkg sm_exercise
    ```

### 2.4 ステートマシンのサンプル2を動かす

#### 2.4.1 実行方法

- 端末1(<font color="Yellow">実行済みの場合はスキップ</font>)
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

  - Yasmin Viewerノードの起動

    ```Bash
    ros2 run yasmin_viewer yasmin_viewer_node
    ```

    - [http://localhost:5000/](http://localhost:5000/)にアクセス（Layoutを`grid`に設定し，全画面表示にしてください）

- 端末2
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

  - sm_sample1ノードの実行

    ```Bash
    source install/setup.bash
    ```

    ```Bash
    ros2 run lecture04_pkg sm_sample2
    ```

## 3. Launch

このセクションでは，lecture01_pkgとTurtleBot3本体が必要です．

### 3.1 Launchファイルのサンプルを動かす

launchファイルで第1回目のtalker，listenerノードのサンプルを実行してみましょう．

#### 3.1.1 実行方法

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

  - launch_sample1.launch.pyの起動

    ```Bash
    source install/setup.bash
    ```

    ```Bash
    ros2 launch lecture04_pkg launch_sample1.launch.py
    ```

## 3.2 TurtleBot3を用いたLaunchファイルのサンプルを動かす

<font color="Yellow">ここからはTurtleBot3を使用します．準備してください．</font> \
TURTLEBOT3に搭載されているカメラに赤色のものを写し，ロボットを操作してみましょう．

#### 3.2.1 実行方法

- 端末1
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
    ※パスワードは入力しても表示されません．

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

  - competition.launch.pyの起動

    ```Bash
    source install/setup.bash
    ```

    ```Bash
    . 4a_turtlebot3_settings.sh​
    ```

    ```Bash
    ros2 launch lecture04_pkg launch_sample2.launch.py
    ```

    http://localhost:5000/ にアクセス

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

  - sm_mainノードを起動

    ```Bash
    source install/setup.bash
    ```

    ```Bash
    . 4a_turtlebot3_settings.sh​
    ```

    ```Bash
    ros2 run lecture04_pkg sm_main
    ```

    Enterをクリックしてステートマシンを開始してください．
