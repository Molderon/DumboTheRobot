import zmq
import time
from rpi_ws281x import *
import argparse
import os

# LED strip configuration:
LED_COUNT      = 12     
LED_PIN        = 12       
LED_FREQ_HZ    = 800000  
LED_DMA        = 10      
LED_BRIGHTNESS = 200     
LED_INVERT     = False   
LED_CHANNEL    = 0       

class LED:
    def __init__(self):
        self.LED_COUNT      = 16      # 
        self.LED_PIN        = 12      #  
        self.LED_FREQ_HZ    = 800000  # 
        self.LED_DMA        = 10      # 
        self.LED_BRIGHTNESS = 255     # 
        self.LED_INVERT     = False   #  
        self.LED_CHANNEL    = 0       # 
        parser = argparse.ArgumentParser()
        parser.add_argument('-c', '--clear', action='store_true', help='clear the display on exit')
        args = parser.parse_args()

        self.strip = Adafruit_NeoPixel(self.LED_COUNT, self.LED_PIN, self.LED_FREQ_HZ, self.LED_DMA, self.LED_INVERT, self.LED_BRIGHTNESS, self.LED_CHANNEL)

        self.strip.begin()



    def colorWipe(self, R, G, B):
        color = Color(R,G,B)
        for i in range(self.strip.numPixels()):
            self.strip.setPixelColor(i, color)
            self.strip.show()


def networking():
    port = "5555"
    init_system_addr = "tcp://127.0.0.1:" + port
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind(init_system_addr)
    Condition = socket.recv()
    return str(Condition)


# the bug led still lives.. kill it!
if __name__ == '__main__':
    Condition = networking()
    led = LED()
    led.colorWipe(0,0,0)
    
    if Condition == 'GREEN':
        led.colorWipe(0,255,0)  # green
    elif Condition == 'BLUE':
        led.colorWipe(0, 0, 255) #Blue
    elif Condition == 'RED':
        led.colorWipe(255, 0, 0)  #RED
    else:
        print("\n\n[System]::LED module has failed\n")
        os.system("sudo python3 robotLight.py")
        exit()
