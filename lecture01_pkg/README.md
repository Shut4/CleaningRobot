# Lecture 1: Publisher, Subscriber

## 1. Preparation

### 1.1 Downloading Exercise Materials and Package

Please access the links below to download the materials and exercise package.

- Download materials: [handout_ros2_lecture_01_en.pdf](https://drive.google.com/uc?export=download&id=1Tol7oZnOaYqb-nkcjqUQ5W61BstePc11)
- Download exercise package: [lecture01_pkg.tar.gz](https://drive.google.com/uc?export=download&id=1vwazHIHHW8yEUYk0wtt1ohbdnPaywFv7)

### 1.2 Moving README.md and Python Scripts

Follow the instructions below to move the downloaded exercise package.

- Extract the downloaded exercise package (`lecture01_pkg.tar.gz`).
- Move the extracted exercise package (`lecture01_pkg`) to `~/ros2_lecture_ws/src/7_lectures`.

### 1.3 Building the Package

Execute the following commands.

- Terminal 1
  - Launch virtual environment, etc. (<font color="Yellow">Skip if already done</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - Build the package

    ```Bash
    colcon build --symlink-install
    ```

## 2. Sample (Publisher, Subscriber)

Let's run a program that publishes String type messages and a program that subscribes to them.

### 2.1 Execution Method

- Terminal 1
  - Launch virtual environment, etc.

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - Run the listener node

    ```Bash
    source install/setup.bash
    ```

    ```Bash
    ros2 run lecture01_pkg listener
    ```

- Terminal 2
  - Launch virtual environment, etc.

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - Run the talker node

    ```Bash
    source install/setup.bash
    ```

    ```Bash
    ros2 run lecture01_pkg talker
    ```

## 3. Exercise 1

Improve the sample code and create a program that publishes Int64 type and a program that subscribes to it according to the following conditions.

- Create talker_exercise1.py
  - Import the message type "std_msgs/Int64"
  - Publisher settings
    - Topic Name: /chatter/int64
    - Message Type: Int64
    - Queue Size: 10
- Create listener_exercise1.py
  - Import the message type "std_msgs/msg/Int64"
  - Subscriber settings
    - Topic Name: /chatter/int64
    - Message Type: Int64
    - Callback Function: listener_callback
- Define node information in setup.py

### 3.1 Execution Method

- Terminal 1
  - Launch virtual environment, etc. (<font color="Yellow">Skip if already done</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - Run the listener_exercise1 node

    ```Bash
    source install/setup.bash
    ```

    ```Bash
    ros2 run lecture01_pkg listener_exercise1
    ```

- Terminal 2
  - Launch virtual environment, etc. (<font color="Yellow">Skip if already done</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - Run the talker_exercise1 node

    ```Bash
    source install/setup.bash
    ```

    ```Bash
    ros2 run lecture01_pkg talker_exercise1
    ```

## 4. Exercise 2

Improve the sample code and create a program that publishes Twist type messages (messages for handling velocity) and a program that subscribes to them according to the following conditions. Also, set appropriate random values for the message values.

<span style="color: yellow; ">Hint: You can check the information of Twist type messages by executing `ros2 interface show geometry_msgs/msg/Twist` in the terminal.</span>

- Create talker_exercise2.py
  - Import the random library
  - Import the message type "geometry_msgs/msg/Twist"
  - Publisher settings
    - Topic Name: /cmd_vel
    - Message Type: Twist
    - Queue Size: 10
  - Set appropriate random values for each message value
- Create listener_exercise2.py
  - Import the message type "geometry_msgs/msg/Twist"
  - Subscriber settings
    - Topic Name: /cmd_vel
    - Message Type: Twist
    - Callback Function: listener_callback
- Define nodes in setup.py

### 4.1 Execution Method

- Terminal 1
  - Launch virtual environment, etc. (<font color="Yellow">Skip if already done</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - Run the listener_exercise2 node

    ```Bash
    source install/setup.bash
    ```

    ```Bash
    ros2 run lecture01_pkg listener_exercise2
    ```

- Terminal 2
  - Launch virtual environment, etc. (<font color="Yellow">Skip if already done</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - Run the talker_exercise2 node

    ```Bash
    source install/setup.bash
    ```

    ```Bash
    ros2 run lecture01_pkg talker_exercise2
    ```
