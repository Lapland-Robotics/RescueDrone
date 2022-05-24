#!/usr/bin/env python
import time

EdgeBL = [25,37] #Corner Bottom Left
EdgeBR = [47,28] #Corner Bottom Right
EdgeTL = [28,48] #Corner Top Left
EdgeTR = [66,61] #Corner Top Right

DistanceB = 0 #Distance bottom
DistanceR = 0 #Distance right
DistanceT = 0 #Distance top
DistanceL = 0 #Distance left
YDistanceT = 0
YDistanceB = 0
XDistanceR = 0
XDistanceL = 0
PointsPL = 2 #Stops per line
coordinates = []
done = False

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

    if EdgeBL[0] > EdgeTL[0]:
        EdgeTL[0] = EdgeBL[0]
    else:
        EdgeBL[0] = EdgeTL[0]
    if EdgeTL[1] > EdgeTR[1]:
        EdgeTR[1] = EdgeTL[1]
    else:
        EdgeTL[1] = EdgeTR[1]
    if EdgeTR[0] > EdgeBR[0]:
       EdgeBR[0] = EdgeTR[0]
    else:
        EdgeTR[0] = EdgeBR[0]
    if EdgeBL[1] > EdgeBR[1]:
        EdgeBR[1] = EdgeBL[1]
    else:
        EdgeBL[1] = EdgeBR[1]

    print("Coordinates bottom left: ", EdgeBL)
    print("Coordinates top left: ", EdgeTL)
    print("Coordinates top right: ", EdgeTR)
    print("Coordinates bottom right: ", EdgeBR)
    print("")

    DistanceB = EdgeBR[0] - EdgeBL[0]
    DistanceR = EdgeTR[1] - EdgeBR[1]
    DistanceT = EdgeTR[0] - EdgeTL[0] 
    DistanceL = EdgeTL[1] - EdgeBL[1]
    
    print("Distance bottom line is: ", DistanceB)
    print("Distance right line is: ", DistanceR)
    print("Distance top line is: ", DistanceT)
    print("Distance left line is: ", DistanceL)

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

    global coordinates
    global done
    distance = 0

    #DistanceR = 25

    counter = 0
    while counter != DistanceR:
        if counter % PointsPL == 0:
            if done == False:
                coordinates.append(DistanceB)
                coordinates.append(PointsPL)
                distance += PointsPL
                done = True
            else:
                coordinates.append(DistanceB - DistanceB - DistanceB)
                coordinates.append(PointsPL)
                distance += PointsPL
                done = False
        counter += 1

    if distance != DistanceR:
        if coordinates[-2] < 0:
            coordinates.append(DistanceB)
            coordinates.append(PointsPL)
        else:
            coordinates.append(DistanceB - DistanceB - DistanceB)
            coordinates.append(PointsPL)
            distance += PointsPL

    print(distance)
    print(coordinates)


if _name=='main_':
    distancecalc()
    routeplanner()