#!/usr/bin/env python
import time

EdgeBL = [15,0]
EdgeBR = [45,0]
EdgeTL = [33,37]
EdgeTR = [74,45]

DistanceB = 0
DistanceR = 0
DistanceT = 0
DistanceL = 0
YDistanceT = 0
YDistanceB = 0
XDistanceR = 0
XDistanceL = 0
PointsPL = 2
pointsR = []
pointsL = []

def distancecalc():
    global EdgeBL
    global EdgeBR
    global EdgeTL
    global EdgeTR

    global DistanceB
    global DistanceR
    global DistanceT
    global DistanceL

    global YDistanceT
    global YDistanceB
    global XDistanceR
    global XDistanceL


    # EdgeTL     EdgeTR
    #  .____.
    #  |          |
    #  |          |
    #  |          |
    #  .____.
    # EdgeBL     EdgeBR

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
        
        if EdgeTL[1] >= EdgeTR[1]:
            YDistanceT = EdgeTL[1] - EdgeTR[1]
        else:
            YDistanceT = EdgeTR[1] - EdgeTL[1]

        if EdgeTR[0] >= EdgeBR[0]:
            XDistanceR = EdgeTR[0] - EdgeBR[0]
        else:
            XDistanceR = EdgeBR[0] - EdgeTR[0]

        if EdgeBL[1] >= EdgeBR[1]:
            YDistanceB = EdgeBL[1] - EdgeBR[1]
        else:
            YDistanceB = EdgeBR[1] - EdgeBL[1]

def routeplanner():
    global PointsPL
    global pointsR
    global pointsL

    global DistanceB
    global DistanceR
    global DistanceT
    global DistanceL

    counter = 0
    amount = 0
    while counter != DistanceR:
        if counter % PointsPL == 0:
            pointsR.append(counter)
            pointsL.append(counter)
            amount += 1
        counter += 1

    print("The calculated Y values on line R are: ", pointsR) 
    print("The calculated Y values on line L are: ", pointsL)
    print("")


if _name=='main_':
    distancecalc()
    #routeplanner()