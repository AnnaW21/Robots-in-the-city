import RPi.GPIO as GPIO
import cv2
import atexit
from i2c_itg3205 import *
from i2c_adxl345 import *
import websocket
import json
import math
from threading import Thread


cars = []
yourCarPosition = []

def on_message(ws, mess):
    jmes = json.loads(mess)
    if jmes["action"] == "SendAllCoords":
        cars.append({carName:jmes["data"]["carName"], position: jmes["data"]["position"] })
    elif jmes["action"] == "SendPos":
        global yourCarPosition
        yourCarPosition = jmes["position"]

def on_close(ws):
    ws.close()

def on_open(ws):
    print("Connection open")


ws2 = websocket.WebSocketApp("ws://localhost:8765", on_message=on_message, on_close=on_close, on_open=on_open)
new_p2 = Thread(target=ws2.run_forever)
new_p2.start()
sleep(2)
itg3205 = i2c_itg3205(1)
adxl345 = i2c_adxl345(1)
    
# Set the type of GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Motor drive interface definition
ENA = 13  # //L298 Enable A
servo = 12  #
IN1 = 19  # //Motor interface 1
IN2 = 16  # //Motor interface 2
nul_ang = 38
nul = nul_ang/18 + 2

DELTA = 0.05
DELTA_ANGLE = 2
SPEED = 50
RADIUS_ROTATION = 0.9
MAX_ANGLE = 38

# Set the type of GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
# Motor initialized to LOW
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(IN2, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(servo, GPIO.OUT)

pwmA = GPIO.PWM(ENA, 100)
pwmA.start(0)
# pwmServo.start(nul)

# Sensor distance
ECHO = 24
TRIG = 18
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup(TRIG, GPIO.OUT, initial=GPIO.LOW)

cam = cv2.VideoCapture(2)


def close():
    print("end", flush=True)
    MotorStop()
    cam.release()
    cv2.destroyAllWindows()


atexit.register(close)


def GetPosition():
    x = yourCarPosition[0]
    y = yourCarPosition[1]

    return x, y


def GetGiro():
    x = yourCarPosition[2]
    y = yourCarPosition[3]
    z = yourCarPosition[4]
    return [x, y, z]


def GetAxiler():
    print(adxl345)
    sleep(1)
    return adxl345


def GetImage():
    '''
      Return picture from video camera
   '''
    ret, image = cam.read()
    if ret:
        return ret, image
    else:
        return ret, False


def GetDistance():
    '''
      Determine the distance to the nearest obstacle in centimeters. Return distance and error.
      If everything is OK: distance, "OK"
      If the response waiting time is exceeded: 0, "Timeout"
      If the distance exceeds 3 meters or is equal to 0: distance, "Out of reach"
   '''

    StartTime = time()
    StopTime = time()
    GPIO.output(TRIG, False)
    sleep(0.1)
    GPIO.output(TRIG, True)
    sleep(0.00001)
    GPIO.output(TRIG, False)

    while GPIO.input(ECHO) == 0:
        StartTime = time()

    while GPIO.input(ECHO) == 1:
        StopTime = time()

    pulseDuration = (StopTime - StartTime)
    distance = pulseDuration * 17000
    if pulseDuration >= 0.01746:
        return 0, 'Time out'
    elif distance > 300 or distance == 0:
        return distance, 'Out of range'

    return distance, 'Ok'


#detection angle between direction movement and target direction 
def detectionAngle(x2, y2):
    x0 = GetPosition[0]
    y0 = GetPosition[1]
    
    MotorForward(SPEED)
    sleep(1)
    
    x1 = GetPosition[0]
    y1 = GetPosition[1]
    
    scalPr = (x1 - x0) * (x2 - x0) + (y1 - y0) * (y2 - y0)
    len1 = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
    len2 = ((x2 - x0) ** 2 + (y2 - y0) ** 2) ** 0.5
    angle = math.degrees(math.acos(scalPr / (len1 * len2)))
    
    return angle


#movement by straight line
def movementStraight(xEnd, yEnd, xNextEnd, yNextEnd):
    xStart, yStart = GetPosition()
    
    if abs(yEnd) <= yStart + DELTA:
        if xEnd > xStart:
            while abs(xEnd - RADIUS_ROTATION) > xStart + DELTA:
                MotorForward(SPEED)
                sleep(0.5)
            if yNextEnd > yEnd:
                rotation_90("LEFT", xNextEnd, yNextEnd)
            elif yNextEnd <= yEnd:
                rotation_90("RIGHT", xNextEnd, yNextEnd)
        elif xEnd <= xStart:
            while abs(xEnd + RADIUS_ROTATION) > xStart + DELTA:
                MotorForward(SPEED)
                sleep(0.5)
            if yNextEnd > yEnd:
                rotation_90("RIGHT", xNextEnd, yNextEnd)
            elif yNextEnd <= yEnd:
                rotation_90("LEFT", xNextEnd, yNextEnd)
        MotorStop()
            
    if abs(xEnd) <= xStart + DELTA:
        if yEnd > yStart:
            while abs(yEnd - RADIUS_ROTATION) > yStart + DELTA:
                MotorForward(SPEED)
                sleep(0.5)
            if xNextEnd > xEnd:
                rotation_90("RIGHT")
            elif xNextEnd <= xEnd:
                rotation_90("LEFT")
        elif yEnd <= yStart:
            while abs(yEnd + RADIUS_ROTATION) > yStart + DELTA:
                MotorForward(SPEED)
                sleep(0.5)
            if xNextEnd > xEnd:
                rotation_90("LEFT")
            elif xNextEnd <= xEnd:
                rotation_90("RIGHT")
        MotorStop()


#rotation while DETECTION_ANGLE != 0
def rotationCalib(angle, x2, y2):
    set_angle(angle)
    sleep(1)
    while abs(detectionAngle(x2, y2)) > DELTA_ANGLE:
        MotorForward(SPEED)
        

#turning on curter 90
def rotation_90(dir_rot,xNextEnd, yNextEnd):
    if dir_rot == "LEFT":
        set_angle(-MAX_ANGLE)
    elif dir_rot == "RIGHT":
        set_angle(MAX_ANGLE)
    while abs(detectionAngle(xNextEnd, yNextEnd)) >= DELTA_ANGLE:
        MotorForward(SPEED)
    

#movement before coord (x2, y2) from (x0, y0)
def movementByCoord(x2, y2):
    while 1:
        angle = detectionAngle(x2, y2)
        if abs(angle) <= DELTA_ANGLE:
            movementStraight(x2, y2)
            break
        else:
            rotationCalib(angle, x2, y2)

    

# Motor control
def MotorForward(speed):
    """
        The car stops. The car will continue to stand until another movement is called.
    """
    print('motor forward')
    pwmA.ChangeDutyCycle(speed)
    GPIO.output(IN1, True)
    GPIO.output(IN2, False)


def MotorBackward(speed):
    print('motor backward')
    pwmA.ChangeDutyCycle(speed)
    GPIO.output(IN1, False)
    GPIO.output(IN2, True)


def MotorStop():
    print('motor stop')
    pwmA.ChangeDutyCycle(0)
    GPIO.output(IN1, True)
    GPIO.output(IN2, True)


def set_angle(angle):
    pwmServo = GPIO.PWM(servo, 50)
    pwmServo.start(nul)
    duty = angle/18 + 2
    pwmServo.ChangeDutyCycle(duty)
    sleep(0.2)
    pwmServo.stop()


#main


data = GetPosition()
if data:
    x = data[0]
    y = data[1]
    print("x = ", x, flush=True)
    print("y = ", y, flush=True)
else:
    print("none position!", flush=True)
"""
MotorForward(50)
sleep(0.8)
data = GetGiro()
if data:
    x = data[0]
    y = data[1]
    z = data[2]
    print("ddX = ", x, flush=True)
    print("ddY = ", y, flush=True)
    print("ddZ = ", z, flush=True)
else:
    print("none data of giro!", flush=True)
"""


movementByCoord(3.5, 0.125)
sleep(1)
movementByCoord(5, 0.125)
sleep(1)
movementByCoord(5, 4.825)
sleep(1)
movementByCoord(0.125, 4.825)
sleep(1)
movementByCoord(0.125, 2.75)
sleep(1)

sleep(0.4)

"""
i = 0
while i < 5:
    ret, frames = GetImage()
    if ret:
        print("camera ok", flush=True)
        frames = frames[:, :, 0]
    else:
        print("camera no", flush=True)
    i += 1

set_angle(0)
sleep(2)
set_angle(nul)
sleep(2)
set_angle(72)
sleep(2)
set_angle(0)
sleep(2)
MotorStop()
dist, status = GetDistance()
print("Status - ", status, flush=True)
print("Distance =  ", dist, flush=True)

print("program done!", flush=True)
close()
sleep(1)
"""






"""
Our coordinates:
    
    First point: (3.5, 0.75)
    
    Second point: (3.5, 0.125)
    
    Third point: (5, 0.125)
    
    Fourth point: (5, 4.825)
    
    Fifth point: (0.125, 4.825)
    
    Sixth point: (0.125, 2.75)
"""