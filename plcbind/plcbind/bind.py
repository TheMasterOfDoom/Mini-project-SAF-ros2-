#from time import sleep
from plc_msgs.srv import CarierProcess
from plc_msgs.msg import CarierData
import rclpy
from rclpy.node import Node
import socket
import xml.etree.ElementTree as ET 



class PLC_Bind(Node):
    def __init__(self):
        super().__init__('plcbind')
        self.publisher = self.create_publisher(CarierData, 'plc_out', 20)
        self.nextmsg = ""
        self.sock = socket.socket()

        self.cli = self.create_client(CarierProcess, 'calc')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = CarierProcess.Request()
        self.dumb = CarierData()


    def listener_callback(self, msg):
        self.nextmsg = msg.data
    
 
    def handle_msg(self, msg):
        msg = msg.replace('DT#', '').replace('STPLC_', '').strip().replace(msg.split(">")[-1], '')
        self.get_logger().info(msg)
        tree = ET.fromstring(msg)
        cid = int(tree.find('cID').text)
        sid = int(tree.find('sID').text)
        readtime = tree.find('dt').text
        self.req.carrierid = cid
        self.req.stationid = sid
        self.req.readtime = readtime
        self.dumb.carrierid = cid
        self.dumb.stationid = sid
        self.dumb.readtime = readtime
        self.publisher.publish(self.dumb)
        self.get_logger().info('xml: %s' % msg)
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.nextmsg = str(response.procsestime).replace('None', '')
        self.get_logger().info('response: %d' % response.procsestime)
    

    def server(self):
        raw = ""
        self.sock.bind(("0.0.0.0",6969))
        self.sock.listen(2)
        self.conn, addr = self.sock.accept()
        self.get_logger().info('connected:' + str(addr))
        self.sock.settimeout(0.2)
        while True:
            rclpy.spin_once(self,timeout_sec=0.2)

            if self.nextmsg != "":
                self.get_logger().info('next: %s' % str(len(self.nextmsg)))
                self.conn.sendall(self.nextmsg.encode())
                self.nextmsg = ""

            try:
                raw = self.conn.recv(1024).decode()

                
            except:
                pass
            if not raw or raw == "":
                continue

            time = self.handle_msg(raw)
            self.conn.sendall(str(time).encode())

    def __del__(self):
        self.sock.close()




def main():
    rclpy.init()

    bot = PLC_Bind()

    bot.server()

    bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
