# Echo client program
# Test speed of communication using a simple protobuf message

NUM_MSG = 1000

# import os
# os.system("python Server.py")

# import subprocess
# subprocess.Popen("python Server.py")

import socket
from time import clock
from random import random
import velocity_pb2

def setRandVel(velMsg):
    velMsg.linear = random()
    velMsg.angular = random()
    return velMsg

# HOST = '10.0.0.112'         # Self
HOST = '10.0.0.91'         # Spider01
PORT = 50007                # The same port as used by the server
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
 
t1 = clock()
for _ in range(NUM_MSG):
    velMsg = velocity_pb2.VelMsg()
    setRandVel(velMsg)
    s.sendall(velMsg.SerializeToString())
    
    msgIn = velocity_pb2.VelMsg()
    msgIn.ParseFromString(s.recv(4096))
    
    if velMsg.linear != msgIn.linear or velMsg.angular != msgIn.angular:
        errorMsg = "Messages do not match!\n Sent: {}, {}\n Got: {}, {}".format(
            velMsg.linear, velMsg.angular, msgIn.linear, msgIn.angular)
        raise Exception(errorMsg)
    
print "Messages rate was {} per second.".format(NUM_MSG/(clock() - t1))


 
# s.sendall('Hello, world')
# data = s.recv(1024)
# s.close()
# print 'Received', repr(data)

print "Done"
