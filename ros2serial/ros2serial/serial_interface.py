import argparse
import serial

import rclpy
from rclpy.node import Node


# TODO:String固定
# TODO:Arduinoからのメッセージは垂れ流し=> TOPIC (String)
# TODO:Arduinoにおくるメッセージはserviceにするのがベター => SERVICE (いろいろ受け取れるように。)
# １つのことを着実にやるやつ。シェルの概念。by大原くん
# arduinoの型の種類的にこのぐらいあれば十分かと。

from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import Bool

parser = argparse.ArgumentParser(description='Explanation of this program!')

parser.add_argument('port', type=str, help='Device name such as /dev/ttyACM0') 
parser.add_argument('baurate', type=int, help='Rate such as 9600 or 115200 etc.')    
parser.add_argument('node_name', type=str, default=None, help='Name of the this node')    
parser.add_argument('-pn', '--pub_topic_name', type=str, default=None, help='Name of the topic for message got from arduino')  
parser.add_argument('-pt', '--pub_topic_type', type=str, default=None, help='Type of the topic for message got from arduino, e.g. String, Float32, Int32 or Bool')  
parser.add_argument('-sn', '--sub_topic_name', type=str, default=None, help='Name of the topic for message sent to arduino')    
parser.add_argument('-st', '--sub_topic_type', type=str, default=None, help='Type of the topic for message sent to arduino, e.g. String, Float32, Int32 or Bool')  

args = parser.parse_args()

class ROS2Serial(Node):
    def __init__(self,
                 ser : serial.Serial,
                 node_name : str,
                 pub_topic_name : str,
                 pub_topic_type : str,
                 sub_topic_name : str,
                 sub_topic_type : str,):
        super().__init__(node_name)

        self.ser = ser
        self.pub_topic_name = pub_topic_name
        self.sub_topic_name = sub_topic_name
        self.pub_topic_type = eval(pub_topic_type) if pub_topic_type is not None else None
        self.sub_topic_type = eval(sub_topic_type) if sub_topic_type is not None else None

        # Arduino     ===> ROS2 Topic
        if self.pub_topic_type is None:
            pass
        else:
            self.arduino_to_topic = self.create_publisher(
                                        self.pub_topic_type,
                                        self.pub_topic_name,
                                        10)
            timer_period = 0.01 # 100Hz TODO: This value should be set automatically.
            self.timer = self.create_timer(timer_period, self.arduino_to_topic_callback)

        # ROS2 Topic  ===> Arduino
        if self.sub_topic_type is None:
            pass
        else:
            self.topic_to_arduino = self.create_subscription(
                                            self.sub_topic_type,
                                            self.sub_topic_name,
                                            self.topic_to_arduino_callback,
                                            10)

    def arduino_to_topic_callback(self):
        msg = self.pub_topic_type()
        #FIXME:展開
        try:
            received_data = self.ser.readline().decode()
            msg_python_type = self.type_ros2_to_python(self.pub_topic_type)
            msg.data = msg_python_type(received_data)
            self.arduino_to_topic.publish(msg)
            self.get_logger().info(f'From Arduino : {msg.data}')
        except ValueError as e:
            self.get_logger().info(f"{e}")
    

    def topic_to_arduino_callback(self, msg):
        self.get_logger().info(f'To Arduino : {msg.data}')
        self.ser.write(str(msg.data).encode())

    def type_ros2_to_python(self, ros2_type):
        if ros2_type == String:
            return str
        elif ros2_type == Int32:
            return int
        elif ros2_type == Bool:
            return bool
        elif ros2_type == Float32:
            return float

def main():
    
    print(f'Waiting for port : {args.port}, baurate : {args.baurate}')
    ser = serial.Serial(args.port, args.baurate, timeout=None) # timeout=None : read呼び出し時、受信できるまでずっと待つ。

    rclpy.init()
    ros2serial = ROS2Serial(
        ser = ser,
        node_name=args.node_name,
        pub_topic_name=args.pub_topic_name,
        pub_topic_type=args.pub_topic_type,
        sub_topic_name=args.sub_topic_name,
        sub_topic_type=args.sub_topic_type)

    rclpy.spin(ros2serial)
    rclpy.shutdown()
    ser.close()

if __name__=='__main__':
    main()
