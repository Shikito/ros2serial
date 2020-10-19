import sys
import argparse
import serial

import rclpy
from rclpy.node import Node

from ros2serial_interfaces.msg import ROS2SerialMsg
from ros2serial_interfaces.srv import ROS2SerialSrv
from ros2serial.ros2serial_utils.node_utils import create_thread

class SerialInterface(Node):
    def __init__(self,
                 ser : serial.Serial,
                 node_name : str,
                ):
        super().__init__(node_name)
        self.ser = ser
        self.pub_serial_msg = self.create_publisher(ROS2SerialMsg, f'{node_name}/serial_msg', 10)
        self.read_thread = create_thread(0, self.read_from_port_callback)
        self.read_thread.start()
        self.write_srv = self.create_service(ROS2SerialSrv, f'{node_name}/write', self.write_to_port_callback)

    def read_from_port_callback(self):
        msg = ROS2SerialMsg()
        try:
            received = self.ser.readline()
            msg.data = received.decode()
            self.pub_serial_msg.publish(msg)
            # self.get_logger().info(f'From Arduino : {msg.data}')
        except ValueError as e:
            self.get_logger().info(f"{e}")
    
    def write_to_port_callback(self, request, response):
        # self.get_logger().info(f'To Arduino : {request.data}')
        written_bytes_num = self.ser.write(request.data.encode())
        if written_bytes_num > 0:
            response.success = True
        else:
            response.success = False
        return response

def main(argv=sys.argv):

    parser = argparse.ArgumentParser(description='ROS2SERIAL')
    parser.add_argument('-p', '--port', type=str, default='/dev/ttyACM0', help='Device name such as /dev/ttyACM0') 
    parser.add_argument('-b', '--baurate', type=int, default=115200, help='Rate such as 9600 or 115200 etc.')
    parser.add_argument('-n', '--node_name', type=str, default='serial_interface', help='Name of this node')
    args = parser.parse_args()

    print(f'Waiting for port : {args.port}, baurate : {args.baurate}')
    ser = serial.Serial(args.port, args.baurate, timeout=None)

    rclpy.init()
    ros2serial = SerialInterface(
        ser=ser,
        node_name=args.node_name)
    rclpy.spin(ros2serial)
    
    rclpy.shutdown()
    ser.close()

if __name__=='__main__':
    main()
