import serial
import time
import sys

"""
    Used for debugging, add comport as command line argument
    eg. python testServo.py COM6
    
    Sends out direct servo commands to Johnny 5
"""
class testServo:

    def __init__(self):
        
        #comPort = "/dev/tty.usbserial-A600eIiI"
        try:
            comPort = sys.argv[1]
            self.johnny5Serial = serial.Serial(port = comPort, baudrate = 115200)
        except:
            print("Couldn't connect to Johnny 5")
            sys.exit(-1)

        self.johnny5Serial.write('#14 P1500\r')
        self.johnny5Serial.write('#15 P1500\r')

        self.johnny5Serial.close()

if __name__ == '__main__':
    j5=testServo()
