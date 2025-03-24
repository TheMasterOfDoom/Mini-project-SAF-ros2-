from plc_msgs.msg import CarierData
import rclpy
from rclpy.node import Node


class Dumber(Node):
    def __init__(self):
        super().__init__('dumber')
        self.subscription = self.create_subscription(CarierData, 'plc_out', self.listener_callback, 20)

    def listener_callback(self, msg):
        with open("dumber.txt", "a") as f:
            f.write(f"{msg.carrierid},{msg.stationid},{msg.readtime}\n")


def main():
    rclpy.init()
    dumber = Dumber()
    rclpy.spin(dumber)
    dumber.destroy_node()
    rclpy.shutdown()
