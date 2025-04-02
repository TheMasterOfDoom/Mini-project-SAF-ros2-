from ament_index_python.packages import get_package_share_directory
import os

from plc_msgs.srv import CarierProcess

import rclpy
from rclpy.node import Node


class CalcService(Node):

    def __init__(self):
        super().__init__('prosses_calc')
        path = os.path.join(get_package_share_directory("plcbind"), 'data', 'table.csv')
        self.table = make_table(path)
        self.srv = self.create_service(CarierProcess, 'calc', self.calc_callback)

    def calc_callback(self, request, response):
        response.procsestime = self.table[request.carrierid-1][request.stationid-1]
        self.get_logger().info('Incoming request: cid: %d sid: %d' % (request.carrierid, request.stationid))
        self.get_logger().info('Sending response: %d' % response.procsestime)

        return response

def make_table(filename):
    with open(filename) as file:
        return [[int(value) for value in line.split(";")[1:]] for line in file.read().strip().split("\n")[1:]]

def main():
    rclpy.init()

    minimal_service = CalcService()

    rclpy.spin(minimal_service)

    minimal_service.destroy_node()

    rclpy.shutdown()



if __name__ == '__main__':
    main()
