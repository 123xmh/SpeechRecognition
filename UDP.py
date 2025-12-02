import socket
import threading

class UDPComm:
    def __init__(self, target_ip, target_port, ack_port):
        self.target_ip = target_ip
        self.target_port = target_port
        self.ack_port = ack_port
        self.ack_received = False
        
        # 创建发送socket
        self.send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # 创建接收socket（用于ACK）
        self.recv_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_socket.bind(('0.0.0.0', ack_port))
        self.recv_socket.settimeout(0.1)  # 设置超时
        
        # 启动ACK监听线程
        self.listener_thread = threading.Thread(target=self._listen_for_ack, daemon=True)
        self.listener_thread.start()
    
    def _listen_for_ack(self):
        while True:
            try:
                data, addr = self.recv_socket.recvfrom(1024)
                if data.decode() == "ACK":
                    self.ack_received = True
            except socket.timeout:
                pass
            except Exception as e:
                print(f"ACK监听错误: {e}")
    
    def send_data(self, data):
        try:
            # 直接发送原始字节数据
            self.send_socket.sendto(data, (self.target_ip, self.target_port))
            return True
        except Exception as e:
            print(f"发送错误: {e}")
            return False
    
    def receive_ack(self):
        """接收ACK确认包"""
        try:
            data, addr = self.recv_socket.recvfrom(1024)
            return data
        except socket.timeout:
            return None
        except Exception as e:
            print(f"接收ACK错误: {e}")
            return None
    
    def close(self):
        self.send_socket.close()
        self.recv_socket.close()