import argparse
import serial

import rclpy
from rclpy.node import Node

from ros2serial_interfaces.msg import ROS2SerialMsg
from ros2serial_interfaces.srv import ROS2SerialSrv

parser = argparse.ArgumentParser(description='Explanation of this program!')

parser.add_argument('port', type=str, help='Device name such as /dev/ttyACM0') 
parser.add_argument('baurate', type=int, help='Rate such as 9600 or 115200 etc.')    
parser.add_argument('-pr', '--p2r_tpc_name', type=str, default=None, help='Name of the topic for data from port to ros community')  
parser.add_argument('-rp', '--r2p_srv_name', type=str, default=None, help='Name of the service for data from ros community to port')
parser.add_argument('-prt', '--pr_time_period', type=float, default=0.01, help='Time period of reading data from port')
args = parser.parse_args()

class SerialInterface(Node):
    def __init__(self,
                 ser : serial.Serial,
                 p2r_tpc_name : str,
                 r2p_srv_name : str,
                 p2r_time_period : float,
                ):
        super().__init__("serial_interface")
        self.ser = ser

        if p2r_tpc_name is not None:
            self.arduino_to_topic = self.create_publisher(ROS2SerialMsg, p2r_tpc_name, 10)
            self.timer = self.create_timer(p2r_time_period, self.read_from_port_callback)

        if r2p_srv_name is not None:
            self.srv = self.create_service(ROS2SerialSrv, r2p_srv_name, self.write_to_port_callback)

    def read_from_port_callback(self):
        msg = ROS2SerialMsg()
        try:
            received = self.ser.readline()
            received = received.decode()
            msg.data = str(received)
            self.arduino_to_topic.publish(msg)
            self.get_logger().info(f'From Arduino : {msg.data}')
        except ValueError as e:
            self.get_logger().info(f"{e}")
    
    def write_to_port_callback(self, request, response):
        self.get_logger().info(f'To Arduino : {request.data}')
        written_bytes_num = self.ser.write(request.data.encode())
        if written_bytes_num > 0:
            response.success = True
        else:
            response.success = False
        return response

def main():

    print(f'Waiting for port : {args.port}, baurate : {args.baurate}')
    ser = serial.Serial(args.port, args.baurate, timeout=None)

    rclpy.init()
    ros2serial = SerialInterface(
        ser = ser,
        p2r_tpc_name=args.p2r_tpc_name,
        r2p_srv_name=args.r2p_srv_name,
        p2r_time_period=args.p2r_time_period)
    rclpy.spin(ros2serial)
    
    rclpy.shutdown()
    ser.close()

if __name__=='__main__':
    main()
