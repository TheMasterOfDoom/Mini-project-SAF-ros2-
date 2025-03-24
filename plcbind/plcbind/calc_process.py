

from plc_msgs.srv import CarierProcess

import rclpy
from rclpy.node import Node
import csv


class CalcService(Node):

    def __init__(self):
        super().__init__('prosses_calc')
        self.table = make_table()
        self.srv = self.create_service(CarierProcess, 'calc', self.calc_callback)

    def calc_callback(self, request, response):
        response.procsestime = self.table[request.carrierid-1][request.stationid-1]
        self.get_logger().info('Incoming request: cid: %d sid: %d' % (request.carrierid, request.stationid))
        self.get_logger().info('Sending response: %d' % response.procsestime)

        return response


def make_table():
    tab = []
    tab.append([2648,2479,5140,4951,2057,4593,4263,4559,1464,2885,5325,2307,4260,1618,1001,4649])
    tab.append([3979,1681,1624,5551,1125,1692,4428,4998,1619,3407,5835,4592,2100,5428,2973,5626])
    tab.append([5042,1838,3994,3388,1646,1846,3744,2415,2942,4440,3399,5797,1475,3236,1837,2674])
    tab.append([1557,2545,1667,5634,2067,1359,1788,2611,3553,4436,5583,5012,1368,1296,5001,2320])
    tab.append([3399,5236,5755,4934,5933,1967,2961,4352,3598,2886,2953,3815,3397,1542,4112,5927])
    tab.append([4235,2548,1515,5424,3316,5656,5737,5720,2742,2148,3777,1821,4146,2774,4947,2149])
    tab.append([5072,2415,1229,3374,1534,3682,4115,3006,4212,5470,1820,1990,1118,3710,2365,4766])
    tab.append([4490,2628,1385,2491,2416,2113,2143,2084,2600,2994,1846,3287,4573,4230,2106,2011])
    tab.append([4465,2542,5855,5609,1068,3637,1357,4459,5146,5445,3403,1497,4591,1910,3591,3566])
    tab.append([5071,4906,5356,4064,4926,4232,4446,4471,5338,3710,1997,2760,3602,4620,5214,4644])
    tab.append([2876,5700,1451,3529,5505,5550,1954,4170,3318,3246,2946,5155,1313,5437,3136,4398])
    tab.append([1615,1614,4399,2319,4950,3015,3076,1256,5163,2314,4100,3854,3639,4127,4429,1509])
    tab.append([1086,2009,5779,2760,3768,2541,4343,4712,4507,5389,5265,2605,3666,4989,2987,3305])
    tab.append([2213,2612,4025,4462,3469,2624,3537,4387,1757,4123,3754,3393,3826,2529,5871,4013])
    tab.append([4254,2044,3136,3944,1150,2433,1080,5894,5757,1583,3055,1047,1881,1820,1281,5020])
    tab.append([1208,3706,1658,4891,5881,4820,5390,4352,4922,5817,2343,3639,4982,2111,2023,1248])
    return tab

def main():
    rclpy.init()

    minimal_service = CalcService()

    rclpy.spin(minimal_service)

    minimal_service.destroy_node()

    rclpy.shutdown()



if __name__ == '__main__':
    main()
