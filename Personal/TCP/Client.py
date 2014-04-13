# Echo client program
# Test speed of communication

NUM_MSG = 10000
msg = "1"*1000

import socket
from time import clock

# HOST = '10.0.0.112'         # Self
HOST = '10.0.0.91'         # Spider01
PORT = 50007                # The same port as used by the server
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

print "Message being sent is: {}.".format(msg)

t1 = clock()
for _ in range(NUM_MSG):
    s.sendall(msg)
    msgIn = s.recv(1024)
    if msg != msgIn:
        raise Exception("Messages do not match!")
print "Messages rate was {} per second.".format(NUM_MSG/(clock() - t1))


 
# s.sendall('Hello, world')
# data = s.recv(1024)
# s.close()
# print 'Received', repr(data)

