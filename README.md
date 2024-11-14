# Teleop Manager

最終的な速度指令を決定するモジュール

動作確認：Ubuntu20.04＋ROS1

## Usage

### Dependencies
joy-con  `sudo apt install ros-noetic-joy`

### Install
`git clone https://github.com/mtakasago/teleop_manager.git`

### Start
`roslaunch teleop_manager teleop_manager.launch`

テスト用(turtlesim を用いた手動操作の確認)： 

`roslaunch teleop_manager turtle_test.launch`

## Description
### Publish
geometry_msgs/Twist型：`/cmd_vel`

### Modes
**manual mode**

- 入り方：**Yボタン**
- **L1**を押しながら**左スティック**で操作する

**auto mode 1**

- 入り方：**Aボタン**
- 自律移動モード1

**auto mode 2**

- 入り方：**Bボタン**
- 自律移動モード2

**stop mode**

- 入り方：**Xボタン**
- 0指令を送り続ける

**combination mode　(TODO)**

- 入り方：**◯ボタン**
- 自律移動モード1と2をいい具合に混ぜる
