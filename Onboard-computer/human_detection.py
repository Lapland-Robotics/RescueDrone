#!/usr/bin/env python
#importing modules
from threading import Timer
import threading
import Jetson.GPIO as GPIO
import time
from uvctypes import *
from asyncore import ExitNow
import cv2
import numpy as np
import jetson.inference
import jetson.utils
import rospy
from sar_drone.msg import directions
from sar_drone.msg import status
try:
  from queue import Queue
except ImportError:
  from Queue import Queue

import sys
import signal

#Command defines so it is easier to change the commands without going through the whole code
# StartIR = 0 #Bodytemp detection starts now
BodytempD = 110 #bodytemp detected
BodyTempND = 111 #bodytemp not detected
BodyTempDS = 125 #bodytemp detection succeeded
#HumanDs = 4 #human detection starts
HumanBL = 119 #Human (framed in) bottom left
HumanBC = 114 #Human (framed in) bottom center
HumanBR = 220 #Human (framed in) bottom right
HumanCL = 115 #Human (framed in) center left
# HumanCC = 9 #Human (framed in) center center
HumanCR = 116 #Human (framed in) center right
HumanTL = 117 #Human (framed in) top left
HumanTC = 113 #Human (framed in) top center
HumanTR = 118 #Human (framed in) top right
# HumanNF = 14 #Human not in frame
HumanRNS = 111 #human recognition not succeeded, back to bodytemp detection
HumanRS = 112 #human recognition succeeded
# SGPS = 17 #Send GPS
StartHumanDetect = 10
StopHumanDetect1 = 8
StopHumanDetect2 = 9
NextStep = 5

#global var

AIDone = False #30 sec timer for human recognition
personDetected = False #Variable to check if a human has been found inside the time of the AIDone
needToRun = False #Command to rerun the program after it has found a person
needData = False #Command to receive from the drone in order to determine if it wants data
#display = jetson.utils.glDisplay()

def signal_handler(signal, frame):
  print("got sigint interupt")
  sys.exit(0)

def TimerCallback():
  global AIDone
  AIDone = True

def talker(tmp):
	global needData
	needData = False
	pub_val = directions()
	pub_val.ID = 3
	pub_val.Command = tmp
	rospy.loginfo(pub_val)
	pub.publish(pub_val)

def talkerPRIO(tmp):
	pub_val = directions()
	pub_val.ID = 3
	pub_val.Command = tmp
	rospy.loginfo(pub_val)
	pub_prio.publish(pub_val)

def callback(data):
  global needToRun
  global needData

  callbackData = data.Status
  rospy.loginfo("%d", callbackData)
  if callbackData == StartHumanDetect:
    #needToRun = True
    servo(8)
  if callbackData == NextStep:
    needData = True
  if (callbackData == StopHumanDetect1) or (callbackData == StopHumanDetect2):
    needToRun = False
    servo(2.5)
 
def listener():
	rospy.Subscriber("status", status, callback)
  
BUF_SIZE = 2
q = Queue(BUF_SIZE)

def py_frame_callback(frame, userptr):
  array_pointer = cast(frame.contents.data, POINTER(c_uint16 * (frame.contents.width * frame.contents.height)))
  data = np.frombuffer(array_pointer.contents, dtype=np.dtype(np.uint16)).reshape(frame.contents.height, frame.contents.width) # no copy
  # data = np.fromiter(frame.contents.data, dtype=np.dtype(np.uint8), count=frame.contents.data_bytes).reshape(frame.contents.height, frame.contents.width, 2)

  if frame.contents.data_bytes != (2 * frame.contents.width * frame.contents.height):
    return
  if not q.full():
    q.put(data)

PTR_PY_FRAME_CALLBACK = CFUNCTYPE(None, POINTER(uvc_frame), c_void_p)(py_frame_callback)

#def ktof(val):
  #return (1.8 * ktoc(val) + 32.0)
#def ktoc(val):
  #return (val - 27315) / 100.0

def raw_to_8bit(data):
  cv2.normalize(data, data, 0, 65535, cv2.NORM_MINMAX)
  np.right_shift(data, 8, data)
  return cv2.cvtColor(np.uint8(data), cv2.COLOR_GRAY2RGB)

#def display_temperature(img, val_k, loc, color):
  val = ktof(val_k)
  cv2.putText(img,"{0:.1f} degF".format(val), loc, cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2)
  x, y = loc
  cv2.line(img, (x - 2, y), (x + 2, y), color, 1)
  cv2.line(img, (x, y - 2), (x, y + 2), color, 1)

def servo(angle):
  servo = 32 #GPIO pin
  start = 4 #4 is 45 degr
  stop = 8 #8 is stop
  eyelevel = 2.5 #2.5 is 90 degr

  GPIO.setmode(GPIO.BOARD)
  GPIO.setup(servo, GPIO.OUT)
  p=GPIO.PWM(servo,50) # 50hz frequency
  p.start(2.5) # starting duty cycle ( it set the servo to 0 degree )
  if angle == start:
    p.ChangeDutyCycle(start)
    time.sleep(2)
  elif angle == stop:
    p.ChangeDutyCycle(stop)
    time.sleep(2)
  elif angle == eyelevel:
    p.ChangeDutyCycle(eyelevel)
    time.sleep(2)

def ircam():
  ctx = POINTER(uvc_context)()
  dev = POINTER(uvc_device)()
  devh = POINTER(uvc_device_handle)()
  ctrl = uvc_stream_ctrl()
  counterC = 0
  counterN = 0
  global AIDone
  global personDetected
  global needToRun

  servo(2.5)

  res = libuvc.uvc_init(byref(ctx), 0)
  if res < 0:
    print("uvc_init error")
    exit(1)

  try:
    res = libuvc.uvc_find_device(ctx, byref(dev), PT_USB_VID, PT_USB_PID, 0)
    if res < 0:
      print("uvc_find_device error")
      exit(1)

    try:
      res = libuvc.uvc_open(dev, byref(devh))
      if res < 0:
        print("uvc_open error 1")
        exit(1)

      #print("device opened!")
      #print_device_info(devh)
      #print_device_formats(devh)

      frame_formats = uvc_get_frame_formats_by_guid(devh, VS_FMT_GUID_Y16)
      if len(frame_formats) == 0:
        print("device does not support Y16")
        exit(1)

      libuvc.uvc_get_stream_ctrl_format_size(devh, byref(ctrl), UVC_FRAME_FORMAT_Y16,
        frame_formats[0].wWidth, frame_formats[0].wHeight, int(1e7 / frame_formats[0].dwDefaultFrameInterval)
      )

      res = libuvc.uvc_start_streaming(devh, byref(ctrl), PTR_PY_FRAME_CALLBACK, None, 0)
      if res < 0:
        print("uvc_start_streaming failed: {0}".format(res))
        exit(1)

      print("Body temperature detection starts now.")
      # talker(StartIR) #Bodytemp detection starts now


      sendTempDone = False

      try:
        while True:
          if needToRun:
            data = q.get(True, 500)
            if data is None:
              break
              
            x = 0
            y = 0
            counter = 0
            dataa = data
            while x < 120:
              while y < 160:
                if dataa[x,y] > 28300: #Pixel more than 30 degrees Celcius
                  if dataa[x,y] < 31355: #Pixel less than 40 degrees Celcius
                    counter = counter + 1
                y = y + 1
              y = 0
              x = x + 1

            if counter >= 20: #If there is more than 200 out of 19200 pixels that has a value between 20 and 40 degrees Celcius, print bodytemp
              counterN = 0
              counterC = counterC + 1
              if (counterC % 5 == 0) and (not sendTempDone):
                talkerPRIO(BodytempD) #bodytemp in frame detected
                sendTempDone = True
              x = 0
              y = 0
            else:
              counterC = 0
              counterN = counterN + 1
              if (counterN % 5 == 0) and (sendTempDone):
                talkerPRIO(BodyTempND) #no bodytemp in frame detected
                sendTempDone = False
              x = 0
              y = 0

            #needData == False #resetting this for the next time the drone wants to receive data
            counter = 0 #Reset counter for next frame
            data = cv2.resize(data[:,:], (640, 480))
            img = raw_to_8bit(data)
            #cv2.imshow('Body temperature detection', img)
            #cv2.waitKey(1)
              
            if (counterC >= 100):
              talkerPRIO(BodyTempDS) #bodytemp detection succeeded
              AI()
              if AIDone and not personDetected:
                x = 0
                y = 0
                counter = 0
                dataa = 0
                counterC = 0
                counterN = 0
                AIDone = False
                personDetected = False
                sendTempDone = False
                talkerPRIO(HumanRNS) #human recognition not succeeded, back to bodytemp detection
                print("Human recognition not succeeded, back to bodytemp detection")
              elif personDetected:
                talkerPRIO(HumanRS) #human recognition succeeded
                print("Human recognition succeeded")
                #servo(8)
                time.sleep(3)
                GPIO.cleanup()
                x = 0
                y = 0
                counter = 0
                dataa = 0
                counterC = 0
                counterN = 0
                AIDone = False
                personDetected = False
                needToRun = False
                sendTempDone = False
          
          elif (not needToRun):
            time.sleep(3)
            
        #cv2.destroyAllWindows()
      finally:
        libuvc.uvc_stop_streaming(devh)
    finally:
      libuvc.uvc_unref_device(dev)
  finally:
    libuvc.uvc_exit(ctx)

def AI():
  #loading the detection model with a default treshold of 50 percent 
  #Decreasing the treshold will result in detecting more objects while increasing the treshold will result in detecting less objects
  #Use SSD-MobileNet-V2 for object detection, it has 91 different objects
  net = jetson.inference.detectNet("pednet", threshold=0.5)

  #Importing the camera stream
  #camera = jetson.utils.gstCamera(2560, 720, "/dev/video0") #ZED camera
  global display
  camera = jetson.utils.gstCamera(1920, 1080, "0") #RPI Cam
  detects = 0
  #talker(HumanDs) #human detection starts
  print("human detection starts now.")
  global AIDone
  global needData
  global personDetected

  timer = threading.Timer(60.0, TimerCallback)
  timer.start()

  servo(4)

  while not(AIDone) and not personDetected:
    img, width, height = camera.CaptureRGBA()
    detections = net.Detect(img, width, height)
    #display.RenderOnce(img, width, height)
    #display.SetTitle("Search and Rescue Drone Advanced Detection System | Framerate {:.0f} FPS".format(net.GetNetworkFPS()))
    for detect in detections:
      ID = detect.ClassID
      item = net.GetClassDesc(ID)
      center = detect.Center
      if (0 < center[0] < 641) and (720 < center[1] < 1081): #bottom left
        if needData == True:
          talker(HumanBL)
        detects = detects + 1
      elif (640 < center[0] < 1281) and (720 < center[1] < 1081): #bottom mid
        if needData == True:
          talker(HumanBC)
        detects = detects + 1
      elif (1280 < center[0] < 1921) and (720 < center[1] < 1081): #bottom right
        if needData == True:
          talker(HumanBR)
        detects = detects + 1
      elif (0 < center[0] < 641) and (360 < center[1] < 721): #mid left
        if needData == True:
          talker(HumanCL)
        detects = detects + 1
      elif (640 < center[0] < 1281) and (360 < center[1] < 721): #mid
        #if needData == True:
          #talker(HumanCC)
        detects = detects + 1
      elif (1280 < center[0] < 1921) and (360 < center[1] < 721): #mid right
        if needData == True:
          talker(HumanCR)
        detects = detects + 1
      elif (0 < center[0] < 641) and (0 < center[1] < 361): #top left
        if needData == True:
          talker(HumanTL)
        detects = detects + 1
      elif (640 < center[0] < 1281) and (0 < center[1] < 361): #top mid
        if needData == True:
          talker(HumanTC)
        detects = detects + 1
      elif (1280 < center[0] < 1921) and (0 < center[1] < 361): #top right
        if needData == True:
          talker(HumanTR)
        detects = detects + 1
      else:
        #talker(HumanNF) #no human, detection resets
        detects = 0

      #print(detects)
      if detects == 250: 
        personDetected = True

  servo(8)

if __name__ == '__main__':
  rospy.init_node('hunman_detect', anonymous=False)
  pub = rospy.Publisher('directions', directions, queue_size=10)
  pub_prio = rospy.Publisher('directionsPRIO', directions, queue_size=10)

  signal.signal(signal.SIGINT, signal_handler)

  listener()
  while True:
    ircam()
