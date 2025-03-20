import time
import socket

class PLCStub:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        time.sleep(1)  # Wait for the server to start
        
        try:
            self.sock.connect((self.ip, self.port))
            print(f"Connected to PLC at {self.ip}:{self.port}")
        except socket.error as e:
            print(f"Failed to connect to {self.ip}:{self.port} - {e}")
            self.sock = None  # Avoid using an invalid socket

    def send_carrier(self, cid, sid):
        if self.sock:
            try:
                message = f"<PLC><sid>{sid}</sid><cid>{cid}</cid><dt>1</dt></PLC>"
                self.sock.sendall(message.encode('utf-8'))
                print(f"Sent: {message}")
            except socket.error as e:
                print(f"Failed to send data: {e}")

    def close(self):
        if self.sock:
            self.sock.close()
            print("Connection closed")

def main():
    plc = PLCStub("localhost", 6969)
    
    if plc.sock:  # Only proceed if connection was successful
        try:
            while True:
                for i in range(1,17):
                    for j in range(1,17):
                        plc.send_carrier(i, j)
                        time.sleep(0.5)
        except KeyboardInterrupt:
            print("\nInterrupted by user.")
        finally:
            plc.close()

if __name__ == "__main__":
    main()


