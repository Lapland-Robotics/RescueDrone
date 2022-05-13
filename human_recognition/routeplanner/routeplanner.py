#!/usr/bin/env python
import time

EdgeBL = [37,0]
EdgeBR = [74,0]
EdgeTL = [37,37]
EdgeTR = [74,37]

DistanceB = 0
DistanceR = 0
DistanceT = 0
DistanceL = 0
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
        print("It is not a square.")

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
    routeplanner()