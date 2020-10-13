# ros2serial
  
## Features

ROS2とシリアルデバイスのインターフェースです。
 
## Requirement
 
* ROS2 Foxy Fitzroy
  
## Usage

```bash
ros2 run ros2serial main 
```
 
## Note
 

ポート、ボーレート、ノード名を指定する方法

```bash
ros2 run ros2serial main -p /dev/ttyACM0 -b 115200 -n serial_interface
```
