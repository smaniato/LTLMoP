import serial
import time

class test:

    def __init__(self):
        
        #comPort = "/dev/tty.usbserial-A600eIiI"
        try:
            self.johnny5Serial = serial.Serial(port = '/dev/tty.usbserial-A600eIiI', baudrate = 115200)
        except:
            print ("(INIT) ERROR: Couldn't connect to Johnny 5")
            exit(-1)

        self.johnny5Serial.write('#14 P1500\r')
        self.johnny5Serial.write('#15 P1500\r')


    
        self.johnny5Serial.close()

if __name__ == '__main__':
    j5=test()
