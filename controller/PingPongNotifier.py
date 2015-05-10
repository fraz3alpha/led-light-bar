__author__ = "andrewtaylor2@gmail.com"

# A basic bit of python to send some data to a set of WS2812
#  LEDs attached via an Arduino (over a [USB] serial port)

import serial
from time import sleep
import time
import argparse

class PingPongNotifier(object):

  leds = {}

  RED = (0xFF, 0x00, 0x00)
  GREEN = (0x00, 0xFF, 0x00)
  BLUE = (0x00, 0x00, 0xFF)
  BLACK = (0x0,0x0,0x0)
  WHITE = (0xFF, 0xFF, 0xFF)

  last_refresh = -1

  def __init__(self, serial_port, baud_rate, number_of_leds):
    self.ser = serial.Serial(serial_port, baud_rate, timeout=1)
    # Wait for the Arduino to settle down, in case it resets
    sleep(2)
    self.number_of_leds = number_of_leds
    for i in range(self.number_of_leds):
      self.leds[i] = self.BLACK

  def refresh(self):
    output_buffer = []
    for i in reversed(range(0, self.number_of_leds)):
      c = self.leds[i]
      f = 0
#      if c == self.RED:
#        f = 1

      #print (c)
      for byte in c:
        output_buffer.append(chr(byte))
      output_buffer.append(chr(f))

    output = "".join(output_buffer)
    #print ("Output length: %d" % (len(output)))
    # print (output)
    self.ser.write(output)
    if self.last_refresh != 0:
      fps = int(1.0/(time.time() - self.last_refresh))
      print ("FPS: %d" % (fps))
    self.last_refresh = time.time()

  def set_led(self, led_number, colour):
    self.leds[led_number] = colour
 
  def set_all(self, colour):
    for i in range(self.number_of_leds):
      self.leds[i] = colour

parser = argparse.ArgumentParser(description='Controls an attached LED light bar')
parser.add_argument('--port', dest='port', default="/dev/ttyUSB0", help='The serial port to open (defaults to /dev/ttyUSB0)')
parser.add_argument('--baudrate', dest='baudrate', default="9600", help='The serial baudrate (defaults to 9600)')
args = parser.parse_args()

print ("Configuration:")
print ("port:     %s" % (args.port))
print ("baudrate: %s" % (args.baudrate))

LEDS = 8
ppn = PingPongNotifier(args.port, args.baudrate, LEDS)

for idx in range(LEDS):
  ppn.set_led(idx, ppn.BLACK)
#ppn.set_led(LEDS-1, ppn.GREEN)
ppn.refresh()

colour_loop = (ppn.RED, ppn.GREEN, ppn.BLUE, ppn.WHITE)
counter = 0
while True:
  #ppn.set_led(0, colour_loop[counter % 4])
  for i in range(LEDS):
    ppn.set_led(i, ppn.BLACK)
  ppn.refresh() 
  sleep(0.05)
  for i in range(LEDS):
    ppn.set_led(i, ppn.WHITE)
  ppn.refresh() 
  sleep(0.05)
  #counter += 1
  #sleep(1)
  break;

while True:
  ppn.set_all(ppn.BLACK)
  for i in range(0,2):
    ppn.set_led(i, ppn.RED)
  ppn.refresh()
  sleep(0.25)

  ppn.set_all(ppn.BLACK)
  for i in range(3,5):
    ppn.set_led(i, ppn.GREEN)
  ppn.refresh()
  sleep(0.25)

  ppn.set_all(ppn.BLACK)
  for i in range(6,8):
    ppn.set_led(i, ppn.BLUE)
  ppn.refresh()
  sleep(0.25)

ser.close()             # close port
exit()
