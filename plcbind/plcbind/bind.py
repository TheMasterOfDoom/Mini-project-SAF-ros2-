#from time import sleep
from socket import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import xml.dom.minidom 



class PLC_Bind(Node):
    def __init__(self):
        super().__init__('plcbind')
        self.publisher = self.create_publisher(String, 'plc_out', 20)
        self.subscription = self.create_subscription(String, 'plc_in', self.listener_callback, 20)
        self.nextmsg = ""
        self.sock = socket.socket()


    def listener_callback(self, msg):
        self.nextmsg = msg.data
    
 
    def handle_msg(self, msg):
        print(msg)
        self.publisher.publish(String(data=msg))

    def server(self):
        raw = ""
        self.sock.bind(("0.0.0.0",6968))
        self.sock.listen(2)
        self.conn, addr = self.sock.accept()
        print(addr)
        self.sock.settimeout(0.5)
        while True:
            rclpy.spin_once(self,timeout_sec=0.5)

            try:
                raw = self.conn.recv(1024).decode()
                temp = xml.dom.minidom.parseString(raw) 
                new_xml = temp.toprettyxml() 
                print(new_xml) 
            except:
                pass
            if not raw or raw == "":
                continue

            self.conn.sendall("2000".encode())
            self.handle_msg(raw)




def main(args=None):
    rclpy.init(args=args)

    bot = PLC_Bind()

    bot.server()

if __name__ == '__main__':
    main()
