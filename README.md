# Teleop Manager

最終的な速度指令を決定するモジュール

動作確認：Ubuntu22.04＋ROS2 Humble

## Usage

### Dependencies
joy-con  `sudo apt install ros-humble-joy`

### Install
`git clone https://github.com/mtakasago/teleop_manager.git -b humble-devel`

### Start
`ros2 launch teleop_manager teleop_manager.launch.xml`

## Description
### Publish
geometry_msgs/Twist型：`/cmd_vel`

### Params
- max_x : x軸方向の最高速度 \[m/s]（manual mode）
- max_y : y軸方向の最高速度 \[m/s]（manual mode）
- max_auto : 自律走行時の最大速度 \[m/s]（x,y共通）
- maw_yawrate : 旋回の最大速度 \[m/s]

### Modes
**manual mode**

- 入り方：**Yボタン**
- **L1**を押しながら**左スティック**で直進と旋回・**右スティック**で真横移動

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
