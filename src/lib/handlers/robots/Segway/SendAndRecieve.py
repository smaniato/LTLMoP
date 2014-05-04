
import LtlmopCsharpMessages_pb2
from struct import pack, unpack
import socket

class MessageStream:
    """ Messages are passed in the following format:
    Length - 4 byte int little-endian
    Message - Length number of bytes
    """
    
    def __init__(self, ip, port):
        """ Take an ip and a port and opens a socket
        
        @param ip: A string with the ip address
        @param port: An int with the port number
        """
        self.clientSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.clientSocket.connect((ip, port))
        
    def getMessage(self):
        """ Returns a string with the next message or an empty string
        if the port has been closed.
        """
        lengthInBytes = self.clientSocket.recv(4)
        length = unpack('<i', lengthInBytes)[0]
        return self.clientSocket.recv(length)
    
    def sendMessage(self, message):
        """ Sends a message to the client.
        
        @param message: A string with the desired payload
        """
        length = len(message)
        fullMessage = pack('<i', length) + message
        self.clientSocket.sendall(fullMessage)
        
        
if __name__ == "__main__":
    """ A Quick test that sends a protobuf message and waits for it to
    be echo'd back.
    """  
    HOST = '10.0.0.91'          # Spider01
    PORT = 50007                # The same port as used by the server
    linearV = 4
    angularV = 3.1
    
    print "Creating stream"
    stream = MessageStream(HOST, PORT)
    
    message = LtlmopCsharpMessages_pb2.VelocityMessage()
    message.linear = linearV
    message.angular = angularV
    
    print "Sending"
    stream.sendMessage(message.SerializeToString())
    
    print "Receiving"
    message.ParseFromString(stream.getMessage())
    
    if message.linear != linearV or message.angular != angularV:
        print "Error: Messages did not match!"
    else:
        print "Message was echo'd back successfully!"
    
    
    
    