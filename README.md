# linear_motor_ros2_driver
設計製作特論用
ラズパイ5でROS2を使ってリニアアクチュエータを制御するノード
## install
```
sudo apt update
sudo apt install libgpiod-dev
cd ~/ros2_ws/src
git clone https://github.com/kokiikeda6/linear_motor_ros2_driver.git
cd ~/ros2_ws
colcon build
```

## execute
1. リニアアクチュエータ起動
```
ros2 run linear_motor_controller linear_motor_controller_node
```
2. サービス送信 (縮む)
```
ros2 service call /action_command linear_motor_msgs/srv/Act "{action: "up"}"
```
2. サービス送信 (伸びる)
```
ros2 service call /action_command linear_motor_msgs/srv/Act "{action: "down"}"
```