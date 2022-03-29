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
        	#top = detect.Top
        	#left = detect.Left
        	#bottom = detect.Bottom
        	#right = detect.Right
        	item = net.GetClassDesc(ID)
		center = detect.Center
		#xasC = 540 - center[0]
		#yasC = 960 - center[1] 
		print(item, center)
		if center[0] < 960 and center[1] < 540:
			print("Human is top left")
		elif center[0] < 960 and center[1] > 540:
			print("Human is bottom left")
		elif center[0] > 960 and center[1] > 540:
			print("Human is bottom right")
		elif center[0] > 960 and center[1] < 540:
			print("Human is top right")