#!/usr/bin/env python
# -- coding: utf-8 --

import Jetson.GPIO as GPIO
import time
from multiprocessing.connection import wait
from uvctypes import *
import time
import cv2
import numpy as np
import jetson.inference
import jetson.utils
try:
  from queue import Queue
except ImportError:
  from Queue import Queue
import platform

BUF_SIZE = 2
q = Queue(BUF_SIZE)

def py_frame_callback(frame, userptr):

  array_pointer = cast(frame.contents.data, POINTER(c_uint16 * (frame.contents.width * frame.contents.height)))
  data = np.frombuffer(
    array_pointer.contents, dtype=np.dtype(np.uint16)
  ).reshape(
    frame.contents.height, frame.contents.width
  ) # no copy

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

def main():
  #8 is stop
  #4 is 45 degr
  #2.5 is 90 degr
  control = [2.5,4.5,8]
  servo = 32 #GPIO pin

  GPIO.setmode(GPIO.BOARD)
  GPIO.setup(servo, GPIO.OUT)
  p=GPIO.PWM(servo,50) # 50hz frequency
  p.start(2.5) # starting duty cycle ( it set the servo to 0 degree )
  p.ChangeDutyCycle(8)
  
  #loading the detection model with a default treshold of 50 percent 
  #Decreasing the treshold will result in detecting more objects
  #Increasing the treshold will result in detecting less objects
  #Use SSD-MobileNet-V2 for object detection, it has 91 different objects
  net = jetson.inference.detectNet("pednet", threshold=0.5)

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
      p.ChangeDutyCycle(2.5)

      try:
        while True:
          data = q.get(True, 500)
          if data is None:
            break
          
          x = 0
          y = 0
          counter = 0
          while x < 120:
            while y < 160:
              if data[x,y] > 28815: #Pixel more than 15 degrees Celcius
                if data[x,y] < 30815: #Pixel less than 35 degrees Celcius
                  counter = counter + 1
                  y = y + 1
            y = 0
            x = x + 1
          if counter > 150: #If there is more than 200 out of 19200)pixels that has a value between 20 and 35 degrees Celcius, print bodytemp
            counterC = counterC + 1
            print( counterC, "bodytemp")
          elif counter < 199:
            ounterC = counterC + 1 #Reset main counter if next frame does not have enough warm pixels
          counter = 0 #Reset counter for next frame
          x = 0
          data = cv2.resize(data[:,:], (640, 480))
          img = raw_to_8bit(data)
          #cv2.imshow('Body temperature detection', img)
          if (counterC >= 50):
            break

        cv2.destroyAllWindows()
      finally:
        libuvc.uvc_stop_streaming(devh)

      print("done")
    finally:
      libuvc.uvc_unref_device(dev)
  finally:
    libuvc.uvc_exit(ctx)

  #Importing the camera stream
  #camera = jetson.utils.gstCamera(2560, 720, "/dev/video0") #ZED camera
  camera = jetson.utils.gstCamera(1920, 1080, "0") #RPI Cam
  #display = jetson.utils.glDisplay()
  detects = 0
  print("human detection starts now.")

  while detects < 120:
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
      if (0 < center[0] < 640) and (719 < center[1] < 1081):
        print("1") #bottom left
        detects = detects + 1
        #print(detects)
      elif (639 < center[0] < 1280) and (719 < center[1] < 1081):
        print("2") #bottom mid
        detects = detects + 1
        #print(detects)
      elif (1279 < center[0] < 1921) and (719 < center[1] < 1081):
        print("3") #bottom right
        detects = detects + 1
        #print(detects)
      elif (0 < center[0] < 640) and (359 < center[1] < 720):
        print("4") #mid left
        detects = detects + 1
        #print(detects)
      elif (639 < center[0] < 1280) and (359 < center[1] < 720):
        print("5") #mid
        detects = detects + 1
        #print(detects)
      elif (1279 < center[0] < 1921) and (359 < center[1] < 720):
        print("6") #mid right
        detects = detects + 1
        #print(detects)
      elif (0 < center[0] < 640) and (0 < center[1] < 360):
        print("7") #top left
        detects = detects + 1
        #print(detects)
      elif (639 < center[0] < 1280) and (0 < center[1] < 360):
        print("8") #top mid
        detects = detects + 1
        #print(detects)
      elif (1279 < center[0] < 1921) and (0 < center[1] < 360):
        print("9") #top right
        detects = detects + 1
        #print(detects)
      else:
        detects = 0

  print("Human recognition succeeded")
  p.ChangeDutyCycle(8)
  time.sleep(3)
  p.stop()
  GPIO.cleanup()

if _name_ == '_main_':
  main()