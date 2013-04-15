import socket

server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server.bind(('', 2121))
data, addr = server.recvfrom(1024)
print 'addr: ', addr
print 'data: ', data
