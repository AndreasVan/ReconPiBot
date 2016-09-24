#!/usr/bin/env python
# coding: Latin-1

# Import library functions we need
import PicoBorgRev
import time
import sys
import threading
import SocketServer
import picamera
import picamera.array
import cv2
import UltraBorg
import datetime
import RPi.GPIO as GPIO
import pylcdlib
import os
from subprocess import *
from time import sleep, strftime
from datetime import datetime
from envirophat import light, weather, motion, analog, leds
import Adafruit_DHT

sensor = Adafruit_DHT.DHT22
pin = 5

baro = weather.pressure()

lcd = pylcdlib.lcd(0x27,1)
lcd.lcd_clear
lcd.lcd_write(0x80);
lcd.lcd_puts(" Loading PiBot ... ",1) #display on line 1

leds.on()
time.sleep(0.1)
leds.off()

humidity, temperature = Adafruit_DHT.read_retry(sensor, pin)

cmd = "ip addr show wlan0 | grep inet | awk '{print $2}' | cut -d/ -f1"

def run_cmd(cmd):
        p = Popen(cmd, shell=True, stdout=PIPE)
        output = p.communicate()[0]
        return output

leds.on()
time.sleep(0.1)
leds.off()
time.sleep(0.1)
leds.on()
time.sleep(0.1)
leds.off()
time.sleep(0.1)
leds.on()
time.sleep(0.1)
leds.off()


lcd.lcd_puts(" *  PiBot Online  * ",1) #display on line 1
lcd.lcd_write(0xC0);
ipaddr = run_cmd(cmd)
lcd.lcd_puts('IP %s' % ( ipaddr ),2)  #display on line 2
lcd.lcd_write(0xA0);
lcd.lcd_puts('Tmp:{0:0.1f}*C Hum:{1:0.1f}%'.format(temperature, humidity),3) #display on line 3
lcd.lcd_write(0xE0);
lcd.lcd_puts('Baro:{0:0.2f} hPa  '.format(baro/100),4) #display on line 4
#lcd.lcd_puts("Ultrasonic & IRMode",4) #display on line 4
lcd.lcd_backlight(1)

#IR Distance Sensor
GPIOIR = 17 

# Settings Light
GPIO.setmode(GPIO.BCM)
GPIO.setup(21, GPIO.OUT) # white LED
GPIO.setup(20, GPIO.OUT) # white LED 
GPIO.setup(16, GPIO.OUT) # red LED
GPIO.setup(GPIOIR, GPIO.IN)
GPIO.output(16, False)
time.sleep(0.3)
GPIO.output(16, True)
time.sleep(0.3)
GPIO.output(16, False)
time.sleep(0.3)
GPIO.output(16, True)
time.sleep(0.3)
GPIO.output(16, False)

#pins where the switch is connected
global buttonPin1
buttonPin1 = 6
GPIO.setup(buttonPin1,GPIO.IN)

# Settings for the web-page
webPort = 80                            # Port number for the web-page, 80 is what web-pages normally use
imageWidth = 480                        # Width of the captured image in pixels
imageHeight = 360                       # Height of the captured image in pixels
frameRate = 20                          # Number of images to capture per second
displayRate = 2                         # Number of images to request per second
photoDirectory = '/home/pi'             # Directory to save photos to

# Movement mode constants
MANUAL_MODE = 0                         # User controlled movement
SEMI_AUTO_MODE = 1                      # Semi-automatic movement
AUTO_MODE = 2                           # Fully automatic movement

# Global values
global PBR
global lastFrame
global lockFrame
global camera
global processor
global running
global watchdog
global movementMode
running = True
movementMode = MANUAL_MODE

# Setup the UltraBorg
global UB
UB = UltraBorg.UltraBorg()              # Create a new UltraBorg object
UB.Init()                               # Set the board up (checks the board is connected)

# Setup the PicoBorg Reverse
PBR = PicoBorgRev.PicoBorgRev()
#PBR.i2cAddress = 0x44                  # Uncomment and change the value if you have changed the board address
PBR.Init()
if not PBR.foundChip:
    boards = PicoBorgRev.ScanForPicoBorgReverse()
    if len(boards) == 0:
        print 'No PicoBorg Reverse found, check you are attached :)'
    else:
        print 'No PicoBorg Reverse at address %02X, but we did find boards:' % (PBR.i2cAddress)
        for board in boards:
            print '    %02X (%d)' % (board, board)
        print 'If you need to change the IÂ²C address change the setup line so it is correct, e.g.'
        print 'PBR.i2cAddress = 0x%02X' % (boards[0])
    sys.exit()

#PBR.SetEpoIgnore(True)                 # Uncomment to disable EPO latch, needed if you do not have a switch / jumper
PBR.SetCommsFailsafe(False)             # Disable the communications failsafe
PBR.ResetEpo()

# Power settings
voltageIn = 1.2 * 12                    # Total battery voltage to the PicoBorg Reverse
voltageOut = 12.0                       # Maximum motor voltage

# Setup the power limits
if voltageOut > voltageIn:
    maxPower = 1.0
else:
    maxPower = voltageOut / float(voltageIn)


# Timeout thread
class Watchdog(threading.Thread):
    def __init__(self):
        super(Watchdog, self).__init__()
        self.event = threading.Event()
        self.terminated = False
        self.start()
        self.timestamp = time.time()
        
    def run(self):
        timedOut = True
        # This method runs in a separate thread
        while not self.terminated:
            # Wait for a network event to be flagged for up to one second
            if timedOut:
                if self.event.wait(1):
                    # Connection
                    print 'Reconnected...'
                    timedOut = False
                    self.event.clear()
            else:
                if self.event.wait(1):
                    self.event.clear()
                else:
                    # Timed out
                    print 'Timed out...'
                    timedOut = True
                    PBR.MotorsOff()

# Image stream processing thread
class StreamProcessor(threading.Thread):
    def __init__(self):
        super(StreamProcessor, self).__init__()
        self.stream = picamera.array.PiRGBArray(camera)
        self.event = threading.Event()
        self.terminated = False
        self.start()
        self.begin = 0

    def run(self):
        global lastFrame
        global lockFrame
        # This method runs in a separate thread
        while not self.terminated:
            # Wait for an image to be written to the stream
            if self.event.wait(1):
                try:
                    # Read the image and save globally
                    self.stream.seek(0)
                    flippedArray = cv2.flip(self.stream.array, -1) # Flips X and Y
                    retval, thisFrame = cv2.imencode('.jpg', flippedArray)
                    del flippedArray
                    lockFrame.acquire()
                    lastFrame = thisFrame
                    lockFrame.release()
                finally:
                    # Reset the stream and event
                    self.stream.seek(0)
                    self.stream.truncate()
                    self.event.clear()

# Image capture thread
class ImageCapture(threading.Thread):
    def __init__(self):
        super(ImageCapture, self).__init__()
        self.start()

    def run(self):
        global camera
        global processor
        print 'Start the stream using the video port'
        camera.capture_sequence(self.TriggerStream(), format='bgr', use_video_port=True)
        print 'Terminating camera processing...'
        processor.terminated = True
        processor.join()
        print 'Processing terminated.'

    # Stream delegation loop
    def TriggerStream(self):
        global running
        global buttonPin1
        while running:
            if processor.event.is_set():
                time.sleep(0.01)
            else:
                yield processor.stream
                processor.event.set()
            # Check for button press
            if (GPIO.input(buttonPin1)):
                #this is the script that will be called (as root)
                PBR.MotorsOff()
                GPIO.output(21, False)
                GPIO.output(20, False)
                time.sleep(0.5)
                GPIO.output(16, True)
                time.sleep(0.5)
                GPIO.output(16, False)
                time.sleep(0.5)
                GPIO.output(16, True)
                time.sleep(0.5)
                GPIO.output(16, False)
                time.sleep(0.5)
                GPIO.output(16, True)
                time.sleep(0.5)
                GPIO.output(16, False)
                time.sleep(0.5)
                GPIO.output(16, True)
                lcd.lcd_clear
                lcd.lcd_write(0xE0);
                lcd.lcd_puts(" * PiBot Offline! * ",4) #display on line 1
                time.sleep(0.3)
                lcd.lcd_backlight(0)
                time.sleep(0.1)
                os.system("sudo init 0")

# Automatic movement thread
class AutoMovement(threading.Thread):
    def __init__(self):
        super(AutoMovement, self).__init__()
        self.terminated = False
        self.start()

    def run(self):
        global movementMode
        # This method runs in a separate thread
        while not self.terminated:
            # See which mode we are in
            if movementMode == MANUAL_MODE:
		# User movement, wait a second before checking again
		#print 'Manual mode
                time.sleep(1.0)

            elif movementMode == SEMI_AUTO_MODE:
                # Semi-automatic movement mode, checks twice per second
                # Ultrasonic distance readings semi auto mode
		#print 'Semi auto mode'

                # Get the readings from ultra sensors
      		#print 'Reading distance'
                distance1 = int(UB.GetRawDistance1())
                distance2 = int(UB.GetRawDistance2())
                distance3 = int(UB.GetRawDistance3())
                #distance4 = int(UB.GetRawDistance4())                
                # Set critical allowed distance to object in front right
                if distance1 <= 100:
		    PBR.MotorsOff()
                elif distance1 <= 200:
                    driveRight = 0.3 * maxPower
                    driveLeft = 0.3 * maxPower
                    PBR.SetMotor1(driveRight)
                    PBR.SetMotor2(-driveLeft)

                # Set critical allowed distance to object in front left
                if distance2 <= 100:
                    PBR.MotorsOff()
                elif distance2 <= 200:
                    driveRight = 0.3 * maxPower
                    driveLeft = 0.3 * maxPower
                    PBR.SetMotor1(driveRight)
                    PBR.SetMotor2(-driveLeft)

                #Set critical allowed distance to front
                #if distance4 <= 80:
                #    PBR.MotorsOff()
                #else:

                #Set critical allowed distance to object left
                #if distance3 <= 50:
                #    PBR.MotorsOff() 
                #else:

                #Set critical allowed distance to object rear
                if distance3 <= 90: 
                    PBR.MotorsOff()
                #elif distance3 <= 200:
                #    driveRight = 0.3 * maxPower
                #    driveLeft = 0.3 * maxPower
                #    PBR.SetMotor1(-driveRight)
                #    PBR.SetMotor2(driveLeft) 

                # Wait for 1/2 of a second before reading again
                time.sleep(0.1)

            elif movementMode == AUTO_MODE:
                # Automatic movement mode, updates five times per second

                # Get the readings from ultra sensors
                distance1 = int(UB.GetRawDistance1())
                distance2 = int(UB.GetRawDistance2())
                distance4 = int(UB.GetRawDistance4())
                distance5 = not GPIO.input(GPIOIR)

                if distance5:
                    print 'Panic IR detect'
                    driveRight = 0.3 * maxPower
                    driveLeft = 0.3 * maxPower
                    PBR.SetMotor1(-driveRight)
                    PBR.SetMotor2(driveLeft)
                    time.sleep(2)
                    driveRight = 0.6 * maxPower
                    driveLeft = 0.6 * maxPower
                    PBR.SetMotor1(-driveRight)
                    PBR.SetMotor2(-driveLeft)
                    time.sleep(0.5)

                elif distance1 <= 315:
                    print 'Obstacle on left'
                    driveRight = 0.6 * maxPower
                    driveLeft = 0.6 * maxPower
                    PBR.SetMotor1(-driveRight)
                    PBR.SetMotor2(-driveLeft) 
                    time.sleep(0.5)

                elif distance2 <= 315:
                    print 'Obstacle on right'
                    driveRight = 0.6 * maxPower
                    driveLeft = 0.6 * maxPower
                    PBR.SetMotor1(driveRight)
                    PBR.SetMotor2(driveLeft)
                    time.sleep(0.5)

                elif distance4 <= 330:
                    print 'Obstacle in front'
                    driveRight = 0.6 * maxPower
                    driveLeft = 0.6 * maxPower
                    PBR.SetMotor1(-driveRight)
                    PBR.SetMotor2(-driveLeft)
                    time.sleep(0.5)

                elif distance1 and distance2 <= 330:
                    print 'Obstacles around'
                    driveRight = 0.3 * maxPower
                    driveLeft = 0.3 * maxPower
                    PBR.SetMotor1(-driveRight)
                    PBR.SetMotor2(driveLeft)
                    time.sleep(3)
                    driveRight = 0.6 * maxPower
                    driveLeft = 0.6 * maxPower
                    PBR.SetMotor1(driveRight)
                    PBR.SetMotor2(driveLeft) 
                    time.sleep(2)

                elif distance4 <=400:
                    driveRight = 0.35 * maxPower
                    driveLeft = 0.35 * maxPower
                    PBR.SetMotor1(driveRight)
                    PBR.SetMotor2(-driveLeft)

                else:
                    driveRight = 0.5 * maxPower
                    driveLeft = 0.5 * maxPower
                    PBR.SetMotor1(driveRight)
                    PBR.SetMotor2(-driveLeft)

               
                # Wait for 1/5 of a second before reading again
                time.sleep(0.0001)
            else: 
                # Unexpected, print an error and wait a second before trying again
                print 'Unexpected movement mode %d' % (movementMode)
                time.sleep(0.01)


# Class used to implement the web server
class WebServer(SocketServer.BaseRequestHandler):
    def handle(self):
        global PBR
        global lastFrame
        global watchdog
        global movementMode
        # Get the HTTP request data
        reqData = self.request.recv(1024).strip()
        reqData = reqData.split('\n')
        # Get the URL requested
        getPath = ''
        for line in reqData:
            if line.startswith('GET'):
                parts = line.split(' ')
                getPath = parts[1]
                break

        watchdog.event.set()

        if getPath.startswith('/distances-once'):
            # Ultrasonic distance readings
            # Get the readings
            distance1 = int(UB.GetRawDistance1())
            distance2 = int(UB.GetRawDistance2())
            distance3 = int(UB.GetRawDistance3())
            distance4 = int(UB.GetRawDistance4())

            # Read and display time
            localtime = time.asctime( time.localtime(time.time()) )

            # Read and display Enviro pHat  
            north = 228.24
            corr_heading = (motion.heading() - north) % 360
            baro = weather.pressure()
            tmp = weather.temperature()
            lux = light.light() 
            air = weather.pressure()- weather.temperature()
#            acc = motion.accelerometer() 
 
            # On time 
            start_time = time.clock()
            t_time = time.time() 
            s_time = start_time/60/60            

            # Get the WiFi signal strength using iwconfig
            wifiQuery = os.popen('iwconfig wlan0|grep Link|cut -d"=" -f2|cut -d" " -f1')
            wifiResult = wifiQuery.readlines()
            wifiQuery.close()
            # See if we are connected
            if len(wifiResult) == 0:
                # Not connected
                wifiStrength = 'Not connected'
            else:
                # Connected, the value we get here will be a division, e.g. 5/20
                wifiResult = wifiResult[0] # Get the text
                wifiResult = wifiResult[:-1] # Remove the '\n' symbol from the end
                wifiValues = wifiResult.split('/')
                if len(wifiValues) == 2:
                    # Get the percentage from the values
                    wifiPercent = (float(wifiValues[0]) / float(wifiValues[1])) * 100.0
                    wifiStrength = '%.0f %%' % (wifiPercent)
                else:
                    # Unexpected, show the returned text instead
                    wifiStrength = wifiResult
            # Send our value here (currently prints the text for testing)
            #print wifiStrength

            # Build a table for the values
            httpText = '<html><body><center>'
            httpText += '%s' % (localtime)
            httpText += '<table border="0" style="width:60%"><center><tr>'
            httpText += '<td width="25%%"><center>Online: {0:0.2f}</center></td>'.format(s_time)
            httpText += '<td width="25%%"><center>Wifi: %.0f%%</center></td>' % (wifiPercent)
            httpText += '<td width="25%%"><center>Light: %04d </center></td>' % (lux)
            httpText += '<td width="25%%"><center>Compass: %03d</center></td>' %(corr_heading)
            httpText += '</tr><tr>'
            httpText += '<td width="25%%"><center>Temp: {0:0.1f}°C</center></td>'.format(tmp)
            httpText += '<td width="25%%"><center>Hum: {0:0.1f}%</center></td>' .format(humidity)
            httpText += '<td width="25%%"><center>AQI: {0:0.1f}</center></td>' .format(air/1000/4)
            httpText += '<td width="25%%"><center>Baro: {0:0.2f} hPa</center></td>'.format(baro/100) 
            httpText += '</tr><tr>'
            if distance1 == 0:
                httpText += '<td width="25%"><center>None</center></td>'
            else:
                httpText += '<td width="25%%"><center>Left: %04d</center></td>' % (distance1/10)
	    if distance4 == 0:
                httpText += '<td width="25%"><center>None</center></td>'
            else:
                httpText += '<td width="25%%"><center>Front: %04d</center></td>' % (distance4/10)
            if distance3 == 0:
                httpText += '<td width="25%"><center>None</center></td>'
            else:
                httpText += '<td width="25%%"><center>Rear: %04d</center></td>' % (distance3/10)
            if distance2 == 0:
                httpText += '<td width="25%"><center>None</center></td>'
            else:
                httpText += '<td width="25%%"><center>Right: %04d</center></td>' % (distance2/10)
            httpText += '</tr><tr>'
            httpText += '</tr></table>'

            #httpText += '%s' % (localtime)
            #httpText += ' Online : {0:0.2f}' .format(s_time)
            #httpText += '  Temp: {0:0.1f}°C   -   Hum: {1:0.1f}%  '.format(tmp, humidity)
            #httpText += '   -   Baro: {0:0.2f} hPa '.format(baro/100)
            #httpText += '  -  Wifi : %.0f%%' % (wifiPercent)
            #httpText += ' %s' % (acc/1000)
            httpText += '</center></body></html>'
            self.send(httpText) 

        elif getPath.startswith('/semiAuto'):
              # Toggle Auto mode
              if movementMode == SEMI_AUTO_MODE:
                  # We are in semi-auto mode, turn it off
                  movementMode = MANUAL_MODE
                  httpText = '<html><body><center>'
                  httpText += 'Speeds: 0 %, 0 %'
                  httpText += '</center></body></html>'
                  self.send(httpText)
                  PBR.MotorsOff()
              else:
                  # We are not in semi-auto mode, turn it on
                  movementMode = SEMI_AUTO_MODE
                  httpText = '<html><body><center>'
                  httpText += 'Semi Mode'
                  httpText += '</center></body></html>'
                  self.send(httpText)

        elif getPath.startswith('/Auto'):
              # Toggle Auto mode
              if movementMode == AUTO_MODE:
                  # We are in auto mode, turn it off
                  movementMode = MANUAL_MODE
                  httpText = '<html><body><center>'
                  httpText += 'Speeds: 0 %, 0 %'
                  httpText += '</center></body></html>'
                  self.send(httpText)
                  PBR.MotorsOff()
              else:
                  # We are not in auto mode, turn it on
                  movementMode = AUTO_MODE
                  httpText = '<html><body><center>'
                  httpText += 'Auto Mode'
                  httpText += '</center></body></html>'
                  self.send(httpText)

        elif getPath.startswith('/cam.jpg'):
            # Camera snapshot
            lockFrame.acquire()
            sendFrame = lastFrame
            lockFrame.release()
            if sendFrame != None:
                self.send(sendFrame.tostring())

        elif getPath.startswith('/Lighton'):
            # Lights on
            GPIO.output(20, True)
            GPIO.output(21, True)
  
        elif getPath.startswith('/Lightoff'):
            # Lights off
            GPIO.output(20, False)
            GPIO.output(21, False)  

        elif getPath.startswith('/off'):
            # Turn the drives off and switch to manual mode
            movementMode = MANUAL_MODE
            httpText = '<html><body><center>'
            httpText += 'Speeds: 0 %, 0 %'
            httpText += '</center></body></html>'
            self.send(httpText)
            PBR.MotorsOff()

        elif getPath.startswith('/set/'):
            # Motor power setting: /set/driveLeft/driveRight
            parts = getPath.split('/')
            # Get the power levels
            if len(parts) >= 4:
                try:
                    driveLeft = float(parts[2])
                    driveRight = float(parts[3])
                except:
                    # Bad values
                    driveRight = 0.0
                    driveLeft = 0.0
            else:
                # Bad request
                driveRight = 0.0
                driveLeft = 0.0
            # Ensure settings are within limits
            if driveRight < -1:
                driveRight = -1
            elif driveRight > 1:
                driveRight = 1
            if driveLeft < -1:
                driveLeft = -1
            elif driveLeft > 1:
                driveLeft = 1
                
            # Report the current settings
            percentLeft = driveLeft * 100.0;
            percentRight = driveRight * 100.0;
            httpText = '<html><body><center>'
            if movementMode == MANUAL_MODE:
                httpText += 'Speeds: %.0f %%, %.0f %%' % (percentLeft, percentRight)
            elif movementMode == SEMI_AUTO_MODE:
                httpText += 'Semi: %.0f %%, %.0f %%' % (percentLeft, percentRight)
            elif movementMode == AUTO_MODE:
                percentLeft = PBR.GetMotor2() * 100.0;
                percentRight = PBR.GetMotor1() * 100.0;
                httpText += 'Auto: %.0f %%, %.0f %%' % (percentLeft, percentRight)
            httpText += '</center></body></html>'
            self.send(httpText)
            
            # Set the outputs as long as we are not in auto mode
            if movementMode != AUTO_MODE:
                driveLeft *= maxPower
                driveRight *= maxPower
                PBR.SetMotor1(driveRight)
                PBR.SetMotor2(-driveLeft)

        elif getPath.startswith('/photo'):
            # Save camera photo
            lockFrame.acquire()
            captureFrame = lastFrame
            lockFrame.release()
            httpText = '<html><body><center>'
            if captureFrame != None:
                photoName = '%s/Photo %s.jpg' % (photoDirectory, datetime.datetime.utcnow())
                try:
                    photoFile = open(photoName, 'wb')
                    photoFile.write(captureFrame)
                    photoFile.close()
                    httpText += 'Photo saved to %s' % (photoName)
                except:
                    httpText += 'Failed to take photo!'
            else:
                httpText += 'Failed to take photo!'
            httpText += '</center></body></html>'
            self.send(httpText)                    

        elif getPath == '/':
            # Main page, click buttons to move and to stop
            httpText = '<html>\n'
            httpText += '<head>\n'
            httpText += '<script language="JavaScript"><!--\n'
            httpText += 'function Drive(left, right) {\n'
            httpText += ' var iframe = document.getElementById("setDrive");\n'
            httpText += ' var slider = document.getElementById("speed");\n'
            httpText += ' left *= speed.value / 100.0;'
            httpText += ' right *= speed.value / 100.0;'
            httpText += ' iframe.src = "/set/" + left + "/" + right;\n'
            httpText += '}\n'
            httpText += 'function Lightoff() {\n'
            httpText += ' var iframe = document.getElementById("setDrive");\n'
            httpText += ' iframe.src = "/Lightoff";\n'
            httpText += '}\n'
            httpText += 'function Lighton() {\n'
            httpText += ' var iframe = document.getElementById("setDrive");\n'
            httpText += ' iframe.src = "/Lighton";\n'
            httpText += '}\n'
            httpText += 'function Off() {\n'
            httpText += ' var iframe = document.getElementById("setDrive");\n'
            httpText += ' iframe.src = "/off";\n'
            httpText += '}\n'
            httpText += 'function Photo() {\n'
            httpText += ' var iframe = document.getElementById("setDrive");\n'
            httpText += ' iframe.src = "/photo";\n'
            httpText += '}\n'
            httpText += 'function semiAuto() {\n'
            httpText += ' var iframe = document.getElementById("setDrive");\n'
            httpText += ' iframe.src = "/semiAuto";\n'
            httpText += '}\n'
            httpText += 'function Auto() {\n'
            httpText += ' var iframe = document.getElementById("setDrive");\n'
            httpText += ' iframe.src = "/Auto";\n'
            httpText += '}\n'
            httpText += '//--></script>\n'
            httpText += '</head>\n'
            httpText += '<body>\n'
            httpText += '<iframe src="/stream" width="100%" height="500" frameborder="0"></iframe>\n'
            httpText += '<iframe id="setDrive" src="/off" width="100%" height="50" frameborder="0"></iframe>\n'
            httpText += '<center>\n'
            httpText += '<button onclick="Drive(-1,1)" style="width:200px;height:30px;"><b>Spin Left</b></button>\n'
            httpText += '<button onclick="Drive(1,1)" style="width:200px;height:30px;"><b>Forward</b></button>\n'
            httpText += '<button onclick="Drive(1,-1)" style="width:200px;height:30px;"><b>Spin Right</b></button>\n'
            httpText += '<br /><br />\n'
            httpText += '<button onclick="Drive(0,1)" style="width:200px;height:30px;"><b>Turn Left</b></button>\n'
            httpText += '<button onclick="Off()" style="width:200px;height:30px;"><b>Stop</b></button>\n'
            httpText += '<button onclick="Drive(1,0)" style="width:200px;height:30px;"><b>Turn Right</b></button>\n'
            httpText += '<br /><br />\n'
            httpText += '<button onclick="semiAuto(1)" style="width:200px;height:30px;"><b>Semi Auto</b></button>\n'
            httpText += '<button onclick="Drive(-1,-1)" style="width:200px;height:30px;"><b>Reverse</b></button>\n'
            httpText += '<button onclick="Auto(1)" style="width:200px;height:30px;"><b>Auto Mode</b></button>\n'
            httpText += '<br /><br />\n'
            httpText += '<button onclick="Lighton()" style="width:200px;height:30px;"><b>Light On</b></button>\n'
            httpText += '<button onclick="Photo()" style="width:200px;height:30px;"><b>Save Photo</b></button>\n'
            httpText += '<button onclick="Lightoff()" style="width:200px;height:30px;"><b>Light Off</b></button>\n'
            httpText += '<br /><br />\n'
            httpText += '<input id="speed" type="range" min="0" max="100" value="30" style="width:600px" />\n'
            httpText += '<br /><center> - Real-time data - </centre><br />\n'
            httpText += '<iframe src="/distances" width="100%" height="200" frameborder="0"></iframe>\n'            
            httpText += '</center>\n'
            httpText += '</body>\n'
            httpText += '</html>\n'
            self.send(httpText)
            
        elif getPath == '/stream':
            # Streaming frame, set a delayed refresh
            displayDelay = int(1000 / displayRate)
            httpText = '<html>\n'
            httpText += '<head>\n'
            httpText += '<script language="JavaScript"><!--\n'
            httpText += 'function refreshImage() {\n'
            httpText += ' if (!document.images) return;\n'
            httpText += ' document.images["rpicam"].src = "cam.jpg?" + Math.random();\n'
            httpText += ' setTimeout("refreshImage()", %d);\n' % (displayDelay)
            httpText += '}\n'
            httpText += '//--></script>\n'
            httpText += '</head>\n'
            httpText += '<body onLoad="setTimeout(\'refreshImage()\', %d)">\n' % (displayDelay)
            httpText += '<center><img src="/cam.jpg" style="width:640;height:480;" name="rpicam" /></center>\n'
            httpText += '</body>\n'
            httpText += '</html>\n'
            self.send(httpText)
            
        elif getPath == '/distances':
            # Repeated reading of the ultrasonic distances, set a delayed refresh
            # We use AJAX to avoid screen refreshes caused by refreshing a frame
            displayDelay = int(1000 / displayRate)
            httpText = '<html>\n'
            httpText += '<head>\n'
            httpText += '<script language="JavaScript"><!--\n'
            httpText += 'function readDistances() {\n'
            httpText += ' var xmlhttp;\n'
            httpText += ' if (window.XMLHttpRequest) {\n'
            httpText += '  // code for IE7+, Firefox, Chrome, Opera, Safari\n'
            httpText += '  xmlhttp = new XMLHttpRequest();\n'
            httpText += ' } else {\n'
            httpText += '  // code for IE6, IE5\n'
            httpText += '  xmlhttp = new ActiveXObject("Microsoft.XMLHTTP");\n'
            httpText += ' }\n'
            httpText += ' xmlhttp.onreadystatechange = function() {\n'
            httpText += '  var div = document.getElementById("readDistances");\n'
            httpText += '  var DONE = 4;\n'
            httpText += '  var OK = 200;\n'
            httpText += '  if (xmlhttp.readyState == DONE) {\n'
            httpText += '   if (xmlhttp.status == OK) {\n'
            httpText += '    div.innerHTML = xmlhttp.responseText;\n'
            httpText += '   } else {\n'
            httpText += '    div.innerHTML = "<center>Failed reading data (not running?)</center>";\n'
            httpText += '   }\n'
            httpText += '  }\n'
            httpText += ' }\n'
            httpText += ' xmlhttp.open("GET","distances-once",true);\n'
            httpText += ' xmlhttp.send();\n'
            httpText += ' setTimeout("readDistances()", %d);\n' % (displayDelay)
            httpText += '}\n'
            httpText += '//--></script>\n'
            httpText += '</head>\n'
            httpText += '<body>\n'
            httpText += '<body onLoad="setTimeout(\'readDistances()\', %d)">\n' % (displayDelay)
            httpText += '<div id="readDistances"><center>Waiting for first distance reading...</center></div>\n'
            httpText += '</body>\n'
            httpText += '</html>\n'
            self.send(httpText)

        else:
            # Unexpected page
            self.send('Path : "%s"' % (getPath))

    def send(self, content):
        self.request.sendall('HTTP/1.0 200 OK\n\n%s' % (content))


# Create the image buffer frame
lastFrame = None
lockFrame = threading.Lock()

# Startup sequence
print 'Setup camera'
camera = picamera.PiCamera()
camera.resolution = (imageWidth, imageHeight)
camera.framerate = frameRate

print 'Setup the stream processing thread'
processor = StreamProcessor()

print 'Wait ...'
time.sleep(2)
captureThread = ImageCapture()

print 'Setup the watchdog'
watchdog = Watchdog()

print 'Setup the automatic movement'
autoMovement = AutoMovement()

# Run the web server until we are told to close
httpServer = SocketServer.TCPServer(("0.0.0.0", webPort), WebServer)
try:
    print 'Press CTRL+C to terminate the web-server'
    while running:
        httpServer.handle_request()
except KeyboardInterrupt:
    # CTRL+C exit
    print '\nUser shutdown'
finally:
    # Turn the motors off under all scenarios
    PBR.MotorsOff()
    print 'Motors off'
# Tell each thread to stop, and wait for them to end
running = False
captureThread.join()
processor.terminated = True
watchdog.terminated = True
autoMovement.terminated = True
processor.join()
watchdog.join()
autoMovement.join()
del camera
PBR.SetLed(True)
GPIO.output(21, False)
GPIO.output(20, False)
time.sleep(0.5)
GPIO.output(16, True)
time.sleep(0.5)
GPIO.output(16, False)
time.sleep(0.5)
GPIO.output(16, True)
time.sleep(0.5)
GPIO.output(16, False)
time.sleep(0.5)
GPIO.output(16, True)
time.sleep(0.5)
GPIO.output(16, False)
time.sleep(0.5)
GPIO.output(16, True)
leds.on()
time.sleep(0.1)
leds.off()
time.sleep(0.1)
leds.on()
time.sleep(0.1)
leds.off()
print 'Web-server terminated.'
lcd = pylcdlib.lcd(0x27,1)
lcd.lcd_clear
lcd.lcd_write(0x01);
lcd.lcd_puts("                    ",1) #display on line 1
lcd.lcd_write(0x02);
lcd.lcd_puts("   PibBot Offline   ",2) #display on line 2
lcd.lcd_write(0x03);
lcd.lcd_puts("   Please Restart   ",3) #display on line 3
lcd.lcd_write(0xE0);
lcd.lcd_puts("                   ",4) #display on line 4
lcd.lcd_backlight(1)
