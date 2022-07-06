import socket
import struct

port=5000
addr="127.0.0.1"


s = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 20) # Change TTL (=20) to suit
s.setblocking(0)
s.bind((addr,port))
s.settimeout(1)


def recv(buf_size=1024):
    data,addr = s.recvfrom(4)
    data = struct.unpack('f', data)
    return data

while(True):
    try:
        print(recv())
    except socket.timeout:
        print('timeout')
        
    
    
