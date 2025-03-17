from time import sleep
from socket import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket


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
        self.publisher.publish(String(data=msg))

    def server(self):
        self.sock.bind(("0.0.0.0",6969))
        self.sock.listen(2)
        self.conn, addr = self.sock.accept()
        self.sock.settimeout(0.1)
        while True:
            rclpy.spin_once(self)

            if self.nextmsg != "":
                self.conn.sendall(self.nextmsg.encode())
                self.nextmsg=""

            try:
                raw = self.conn.recv(1024).decode()
            except:
                pass
            if not raw:
                continue
            
            self.handle_msg(raw)




def main(args=None):
    rclpy.init(args=args)

    bot = PLC_Bind()

    bot.tcp.server()

if __name__ == '__main__':
    main()
