#importing modules
import jetson.inference
import jetson.utils
import time

#loading the detection model with a default treshold of 50 percent 
#Decreasing the treshold will result in detecting more objects
#Increasing the treshold will result in detecting less objects
#Use SSD-MobileNet-V2 for object detection
net = jetson.inference.detectNet("pednet", threshold=0.5)

#Importing the camera stream
#camera = jetson.utils.gstCamera(2560, 720, "/dev/video0") #ZED camera
camera = jetson.utils.gstCamera(1920, 1080, "0") #RPI Cam
display = jetson.utils.glDisplay()

#using a while function to run the program while the display is open
while display.IsOpen():
	img, width, height = camera.CaptureRGBA()
	detections = net.Detect(img, width, height)
	display.RenderOnce(img, width, height)
	display.SetTitle("Search and Rescue Drone Advanced Detection System | Framerate {:.0f} FPS".format(net.GetNetworkFPS()))
	coordinates = ("1920", "1080")
	for detect in detections:
		#time.sleep(1)
        	#print(detect)
        	ID = detect.ClassID
        	item = net.GetClassDesc(ID)
		center = detect.Center
		#print(item, center)
		if (0 < center[0] < 640) and (719 < center[1] < 1081):
			print("1")
		elif (639 < center[0] < 1280) and (719 < center[1] < 1081):
			print("2")
		elif (1279 < center[0] < 1921) and (719 < center[1] < 1081):
			print("3")
		elif (0 < center[0] < 640) and (359 < center[1] < 720):
			print("4")
		elif (639 < center[0] < 1280) and (359 < center[1] < 720):
			print("5")
		elif (1279 < center[0] < 1921) and (359 < center[1] < 720):
			print("6")
		elif (0 < center[0] < 640) and (0 < center[1] < 360):
			print("7")
		elif (639 < center[0] < 1280) and (0 < center[1] < 360):
			print("8")
		elif (1279 < center[0] < 1921) and (0 < center[1] < 360):
			print("9")
