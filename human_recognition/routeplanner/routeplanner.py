#!/usr/bin/env python
import time

EdgeBL = [3,0] #Corner Bottom Left
EdgeBR = [15,0] #Corner Bottom Right
EdgeTL = [4,25] #Corner Top Left
EdgeTR = [17,17] #Corner Top Right

DistanceB = 0 #Distance bottom
DistanceR = 0 #Distance right
DistanceT = 0 #Distance top
DistanceL = 0 #Distance left
YDistanceT = 0
YDistanceB = 0
XDistanceR = 0
XDistanceL = 0
PointsPL = 2 #Stops per line
YpointsR = []
YpointsL = []
XpointsR = []
XpointsL = []

def distancecalc():
    global EdgeBL
    global EdgeBR
    global EdgeTL
    global EdgeTR

    global DistanceB
    global DistanceR
    global DistanceT
    global DistanceL

    global YDistanceT #Difference between the Y values between EDGETL and EDGETR
    global YDistanceB #Difference between the Y values between EDGEBL and EDGEBR
    global XDistanceR #Difference between the X values between EDGEBR and EDGETR
    global XDistanceL #Difference between the X values between EDGETL and EDGEBL

    DistanceB = EdgeBR[0] - EdgeBL[0]
    DistanceR = EdgeTR[1] - EdgeBR[1]
    DistanceT = EdgeTR[0] - EdgeTL[0] 
    DistanceL = EdgeTL[1] - EdgeBL[1]
    
    print("Distance bottom line is: ", DistanceB)
    print("Distance right line is: ", DistanceR)
    print("Distance top line is: ", DistanceT)
    print("Distance left line is: ", DistanceL)

    print("")

    if DistanceB != DistanceL != DistanceR != DistanceL:
        if EdgeBL[0] >= EdgeTL[0]:
            XDistanceL = EdgeBL[0] - EdgeTL[0]
        else:
            XDistanceL = EdgeTL[0] - EdgeBL[0]
        print(XDistanceL)
        if EdgeTL[1] >= EdgeTR[1]:
            YDistanceT = EdgeTL[1] - EdgeTR[1]
        else:
            YDistanceT = EdgeTR[1] - EdgeTL[1]
        print(YDistanceT)
        if EdgeTR[0] >= EdgeBR[0]:
            XDistanceR = EdgeTR[0] - EdgeBR[0]
        else:
            XDistanceR = EdgeBR[0] - EdgeTR[0]
        print(XDistanceR)
        if EdgeBL[1] >= EdgeBR[1]:
            YDistanceB = EdgeBL[1] - EdgeBR[1]
        else:
            YDistanceB = EdgeBR[1] - EdgeBL[1]
        print(YDistanceB)
        print("")


def routeplanner():
    global PointsPL
    global YpointsR
    global YpointsL
    global XpointsR
    global XpointsL

    global DistanceB
    global DistanceR
    global DistanceT
    global DistanceL

    global YDistanceT
    global YDistanceB
    global XDistanceR
    global XDistanceL

    counter = 0
    while counter != DistanceR:
        if counter % PointsPL == 0:
            YpointsR.append(counter)
        counter += 1
    
    if XDistanceL > 0:
        AXL = XDistanceL / PointsPL
        print(AXL)

    counter = 0
    while counter != DistanceL:
        if counter % PointsPL == 0:
            YpointsL.append(counter)
        counter += 1

    if XDistanceR > 0:
        AXR = XDistanceR / PointsPL
        print(AXR)

    print("The calculated Y values on line R are: ", YpointsR) 
    print("The calculated X values on line R are: ", XpointsR) 
    print("The calculated Y values on line L are: ", YpointsL)
    print("The calculated X values on line L are: ", XpointsL)
    print("")


if _name=='main_':
    distancecalc()
    routeplanner()