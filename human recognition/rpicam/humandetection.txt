#importing modules
import jetson.inference
import jetson.utils

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
	display.SetTitle("Search and Rescue Drone Object Detection System | Network {:.0f} FPS".format(net.GetNetworkFPS()))
