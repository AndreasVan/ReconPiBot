#!/usr/bin/env python
# coding: Latin-1

import time
import os
import RPi.GPIO as GPIO
import pylcdlib
from subprocess import *
from time import sleep, strftime
from datetime import datetime
import Adafruit_DHT

sensor = Adafruit_DHT.DHT22
pin = 5

lcd = pylcdlib.lcd(0x27,1)
lcd.lcd_clear
lcd.lcd_write(0x80);
lcd.lcd_puts(" Loading PiBot ...  ",1) #display on line 1

humidity, temperature = Adafruit_DHT.read_retry(sensor, pin)

cmd = "ip addr show wlan0 | grep inet | awk '{print $2}' | cut -d/ -f1"

def run_cmd(cmd):
        p = Popen(cmd, shell=True, stdout=PIPE)
        output = p.communicate()[0]
        return output

#lcd = pylcdlib.lcd(0x27,1)
#lcd.lcd_clear
#lcd.lcd_write(0x80);
#lcd.lcd_puts(" Loading PiBot ...  ",1) #display on line 1
#time.sleep(3)
lcd.lcd_puts(" *  PiBot Online  * ",1) #display on line 1
lcd.lcd_write(0xC0);
ipaddr = run_cmd(cmd)
lcd.lcd_puts(" Waiting for IP ... ",2) #display on line 2
time.sleep(3)
lcd.lcd_puts('IP %s' % ( ipaddr ),2)  #display on line 2
#lcd.lcd_puts("    Version 3.27    ",2) #display on line 2
lcd.lcd_write(0xA0);
lcd.lcd_puts(" Reading temp ... ",3) #display on line 3
time.sleep(3)
lcd.lcd_puts('Tmp:{0:0.1f}*C Hum:{1:0.1f}%'.format(temperature, humidity),3) #display on line 3
#lcd.lcd_puts(" MainMenu: Function ",3) #display on line 3
lcd.lcd_write(0xE0);
lcd.lcd_puts(" 1.Web 2.Joy 3.Ball",4) #display on line 4
lcd.lcd_backlight(1)
#lcd.lcdCursorBlink(false)
#lcd.blink(False)
time.sleep(3)

#pins where the switch is connected
buttonPin3 = 13
buttonPin2 = 19
buttonPin1 = 26
buttonPin4 = 6

GPIO.setmode(GPIO.BCM)
GPIO.setup(buttonPin1,GPIO.IN)
GPIO.setup(buttonPin2,GPIO.IN)
GPIO.setup(buttonPin3,GPIO.IN)
GPIO.setup(buttonPin4,GPIO.IN)

while True:
  #assuming script1 
  if not (GPIO.input(buttonPin1)):
    #this is the script that will be called (as root)
    os.system("sudo python /home/pi/pibot/pisonoweb00.py")
   #assuming script2 
  if not (GPIO.input(buttonPin2)):
    #this is the script that will be called (as root)
    os.system("python /home/pi/pibot/joy00.py")

 #assuming script3 
  if not (GPIO.input(buttonPin3)):
    #this is the script that will be called (as root)
    os.system("python /home/pi/pibot/joyball00.py")

 #assuming script4
 # if  (GPIO.input(buttonPin4)):
 #   this is the script that will be called (as root)
 #   os.system("sudo init 0")

GPIO.cleanup()
