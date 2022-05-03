#!/usr/bin/env python
#importing modules
from threading import Timer
import threading
import Jetson.GPIO as GPIO
import time
#from multiprocessing.connection import wait
from uvctypes import *
from asyncore import ExitNow
import cv2
import numpy as np
import jetson.inference
import jetson.utils
import rospy
from humandetection.msg import Num
try:
  from queue import Queue
except ImportError:
  from Queue import Queue

callbackData = 0
AIDone = False

def TimerCallback():
  global AIDone
  AIDone = True

def talker(tmp):
	pub = rospy.Publisher('chatter', Num, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	pub_val = Num()
	pub_val.ID = 3
	pub_val.Command = tmp
	rospy.loginfo(pub_val)
	pub.publish(pub_val)

def callback(data):
  global callbackData
  callbackData=data.Command
  rospy.loginfo("%d", callbackData)
 
def listener():
	rospy.init_node('talker', anonymous=True)
	rospy.Subscriber("chatter", Num, callback)
# spin() simply keeps python from exiting until this node is stopped
#	rospy.spin()
  
BUF_SIZE = 2
q = Queue(BUF_SIZE)

def py_frame_callback(frame, userptr):

  array_pointer = cast(frame.contents.data, POINTER(c_uint16 * (frame.contents.width * frame.contents.height)))
  data = np.frombuffer(array_pointer.contents, dtype=np.dtype(np.uint16)).reshape(frame.contents.height, frame.contents.width) # no copy

  # data = np.fromiter(
  #   frame.contents.data, dtype=np.dtype(np.uint8), count=frame.contents.data_bytes
  # ).reshape(
  #   frame.contents.height, frame.contents.width, 2
  # ) # copy

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

      print("device opened!")

      print_device_info(devh)
      print_device_formats(devh)

      frame_formats = uvc_get_frame_formats_by_guid(devh, VS_FMT_GUID_Y16)
      if len(frame_formats) == 0:
        print("device does not support Y16")
        exit(1)

      libuvc.uvc_get_stream_ctrl_format_size(devh, byref(ctrl), UVC_FRAME_FORMAT_Y16,
        frame_formats[0].wWidth, frame_formats[0].wHeight, int(1e7 / frame_formats[0].dwDefaultFrameInterval)
      )

      counterC = 0

      res = libuvc.uvc_start_streaming(devh, byref(ctrl), PTR_PY_FRAME_CALLBACK, None, 0)
      if res < 0:
        print("uvc_start_streaming failed: {0}".format(res))
        exit(1)

      print("Body temperature detection starts now.")
      talker(0) #Bodytemp detection starts now
      biggestC = 0

      try:
        while True:
          data = q.get(True, 500)
          if data is None:
            break
          
          x = 0
          y = 0
          counter = 0
          dataa = data
          #print(dataa)
          while x < 120:
            while y < 160:
              if dataa[x,y] > 29015: #Pixel more than 20 degrees Celcius
                if dataa[x,y] < 30055: #Pixel less than 40 degrees Celcius
                  counter = counter + 1
              y = y + 1
            y = 0
            x = x + 1

          #print(counter)
          if counter >= 450: #If there is more than 200 out of 19200 pixels that has a value between 20 and 40 degrees Celcius, print bodytemp
            counterC = counterC + 1
            biggestC = biggestC + 1
            if counterC % 5 == 0:
              talker(1) #bodytemp in frame detected
            x = 0
            y = 0
          elif counter < 450:
            counterC = 0
            biggestC = biggestC + 1
            if counterC % 5 == 0:
              talker(2) #no bodytemp in frame detected
            x = 0
            y = 0

          counter = 0 #Reset counter for next frame
          data = cv2.resize(data[:,:], (640, 480))
          img = raw_to_8bit(data)
          #cv2.imshow('Body temperature detection', img)
          #cv2.waitKey(1)
            
          if (counterC >= 100):
            talker(3) #bodytemp detection succeeded
            break

        cv2.destroyAllWindows()
      finally:
        libuvc.uvc_stop_streaming(devh)
    finally:
      libuvc.uvc_unref_device(dev)
  finally:
    libuvc.uvc_exit(ctx)


def AI():
  #loading the detection model with a default treshold of 50 percent 
  #Decreasing the treshold will result in detecting more objects
  #Increasing the treshold will result in detecting less objects
  #Use SSD-MobileNet-V2 for object detection, it has 91 different objects
  net = jetson.inference.detectNet("pednet", threshold=0.5)

  #Importing the camera stream
  #camera = jetson.utils.gstCamera(2560, 720, "/dev/video0") #ZED camera
  camera = jetson.utils.gstCamera(1920, 1080, "0") #RPI Cam
  display = jetson.utils.glDisplay()
  detects = 0
  talker(4) #human detection starts
  print("human detection starts now.")
  global AIDone

  while detects < 251 and not(AIDone):
    img, width, height = camera.CaptureRGBA()
    detections = net.Detect(img, width, height)
    #display.RenderOnce(img, width, height)
    #display.SetSize(1920, 1080)
    #display.SetTitle("Search and Rescue Drone Advanced Detection System | Framerate {:.0f} FPS".format(net.GetNetworkFPS()))
    for detect in detections:
      #time.sleep(1)
		  #print(detect)
      ID = detect.ClassID
      item = net.GetClassDesc(ID)
      center = detect.Center
      if (0 < center[0] < 641) and (720 < center[1] < 1081):
        if detects % 5 == 0:
          talker(5)
        #print("1") #bottom left
        detects = detects + 1
        #print(detects)
      elif (640 < center[0] < 1281) and (720 < center[1] < 1081):
        if detects % 5 == 0:
          talker(6)
        #print("2") #bottom mid
        detects = detects + 1
        #print(detects)
      elif (1280 < center[0] < 1921) and (720 < center[1] < 1081):
        if detects % 5 == 0:
          talker(7)
        #print("3") #bottom right
        detects = detects + 1
        #print(detects)
      elif (0 < center[0] < 641) and (360 < center[1] < 721):
        if detects % 5 == 0:
          talker(8)
        #print("4") #mid left
        detects = detects + 1
        #print(detects)
      elif (640 < center[0] < 1281) and (360 < center[1] < 721):
        if detects % 5 == 0:
          talker(9)
        #print("5") #mid
        detects = detects + 1
        #print(detects)
      elif (1280 < center[0] < 1921) and (360 < center[1] < 721):
        if detects % 5 == 0:
          talker(10)
        #print("6") #mid right
        detects = detects + 1
        #print(detects)
      elif (0 < center[0] < 641) and (0 < center[1] < 361):
        if detects % 5 == 0:
          talker(11)
        #print("7") #top left
        detects = detects + 1
        #print(detects)
      elif (640 < center[0] < 1281) and (0 < center[1] < 361):
        if detects % 5 == 0:
          talker(12)
        #print("8") #top mid
        detects = detects + 1
        #print(detects)
      elif (1280 < center[0] < 1921) and (0 < center[1] < 361):
        if detects % 5 == 0:
          talker(13)
        #print("9") #top right
        detects = detects + 1
        #print(detects)
      else:
        talker(14) #no human, detection resets
        detects = 0

  if not AIDone:
    talker(16) #human recognition succeeded
    print("Human recognition succeeded")
    time.sleep(3)
    GPIO.cleanup()
    talker(17) #send GPS coordinates
  else:
    talker(15) #human recognition not succeeded, back to bodytemp detection
    print("Human recognition not succeeded, back to bodytemp detection")

if _name_ == '_main_':
  listener()
  AIDone
  while True:
    if callbackData == 5:
      servo(4)
      time.sleep(3)
      ircam()
      timer = threading.Timer(30.0, TimerCallback)
      timer.start()
      AI()
      if AIDone != False:
        servo(8)
        callbackData = 5
        print("false")
      else:
        callbackData = 0
        print("true")