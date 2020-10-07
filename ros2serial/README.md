# ROS2SERIAL

## はじめに

ROS2でシリアル通信を実現します。
例えば、ArduinoをROS2コミュニティに参加させることができます。

## 使い方

Arduinoを例に挙げます。

1. ArduinoとPC間でシリアル通信を行っておく.

2. 以下のコマンドで、ROS2ノードを起動させる．


引数の詳細は、ros2serial/ros2serial.pyのargparseを見てください。

## トラブルシューティング

- ROS2ノード起動時のエラー

    - `ValueError`

        Will be raised when parameter are out of range, e.g. baud rate, data bits.

    - `SerialException `

        In case the device can not be found or can not be configured.


