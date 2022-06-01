#!/usr/bin/env python
import time

import rospy
from sar_drone.srv import routeplanner, routeplannerResponse, routeplannerRequest
from sar_drone.msg import rel_coordinates

PointsPL = 2 #line per Xm

GloReq = routeplannerRequest()

DistanceB = 0 #Distance bottom
DistanceR = 0 #Distance right
DistanceT = 0 #Distance top
DistanceL = 0 #Distance left

YDistanceT = 0
YDistanceB = 0
XDistanceR = 0
XDistanceL = 0

Route = []
done = False

def handle_route_planner(req): 
    rospy.logwarn("got task")   
    global DistanceB
    global DistanceR
    global DistanceT
    global DistanceL

    global YDistanceT #Difference between the Y values between EDGETL and EDGETR
    global YDistanceB #Difference between the Y values between EDGEBL and EDGEBR
    global XDistanceR #Difference between the X values between EDGEBR and EDGETR
    global XDistanceL #Difference between the X values between EDGETL and EDGEBL

    global Route
    global GloReq

    DistanceB = 0
    DistanceR = 0
    DistanceT = 0
    DistanceL = 0

    YDistanceT = 0 
    YDistanceB = 0
    XDistanceR = 0
    XDistanceL = 0
    
    Route = []

    GloReq = req

    distancecalc()
    route_planner()
    return_value = routeplannerResponse()
    return_value.route = Route
    rospy.logwarn("i'm done with the task")
    return return_value

def distancecalc():
    global GloReq
    global Route

    global DistanceB
    global DistanceR
    global DistanceT
    global DistanceL

    global YDistanceT #Difference between the Y values between EDGETL and EDGETR
    global YDistanceB #Difference between the Y values between EDGEBL and EDGEBR
    global XDistanceR #Difference between the X values between EDGEBR and EDGETR
    global XDistanceL #Difference between the X values between EDGETL and EDGEBL

    Start_corner_tmp = rel_coordinates()
    if(GloReq.Closest == 1): 
        Start_corner_tmp.x = GloReq.BL.x
        Start_corner_tmp.y = GloReq.BL.y
    elif(GloReq.Closest == 2):
        Start_corner_tmp.x = GloReq.BR.x 
        Start_corner_tmp.y = GloReq.BR.y
    elif(GloReq.Closest == 3): 
        Start_corner_tmp.x = GloReq.TL.x
        Start_corner_tmp.y = GloReq.TL.y
    elif(GloReq.Closest == 4): 
        Start_corner_tmp.x = GloReq.TR.x
        Start_corner_tmp.y = GloReq.TR.y

    GloReq.TL.y = GloReq.BL.y if GloReq.BL.y < GloReq.TL.y else GloReq.TL.y
    GloReq.BL.y = GloReq.TL.y
    
    GloReq.TL.x = GloReq.TL.x if GloReq.TL.x > GloReq.TR.x else GloReq.TR.x
    GloReq.TR.x = GloReq.TL.x

    GloReq.TR.y = GloReq.TR.y if GloReq.TR.y > GloReq.BR.y else GloReq.BR.y
    GloReq.BR.y = GloReq.TR.y

    GloReq.BL.x = GloReq.BL.x if GloReq.BL.x < GloReq.BR.x else GloReq.BR.x
    GloReq.BR.x = GloReq.BL.x

    if(GloReq.Closest == 1): 
        if(Start_corner_tmp.x != GloReq.BL.x or Start_corner_tmp.y != GloReq.BL.y):
            tmp = rel_coordinates()
            tmp.x = GloReq.BL.x - Start_corner_tmp.x
            tmp.y = GloReq.BL.y - Start_corner_tmp.y
            Route.append(tmp)

    elif(GloReq.Closest == 2): 
        if(Start_corner_tmp.x != GloReq.BR.x or Start_corner_tmp.y != GloReq.BR.y):
            tmp = rel_coordinates()
            tmp.x = GloReq.BR.x - Start_corner_tmp.x
            tmp.y = GloReq.BR.y - Start_corner_tmp.y
            Route.append(tmp)

    elif(GloReq.Closest == 3): 
        if(Start_corner_tmp.x != GloReq.TL.x or Start_corner_tmp.y != GloReq.TL.y):
            tmp = rel_coordinates()
            tmp.x = GloReq.TL.x - Start_corner_tmp.x
            tmp.y = GloReq.TL.y - Start_corner_tmp.y
            Route.append(tmp)

    elif(GloReq.Closest == 4): 
        if(Start_corner_tmp.x != GloReq.TR.x or Start_corner_tmp.y != GloReq.TR.y):
            tmp = rel_coordinates()
            tmp.x = GloReq.TR.x - Start_corner_tmp.x
            tmp.y = GloReq.TR.y - Start_corner_tmp.y
            Route.append(tmp)
    
    DistanceB = abs(GloReq.BR.y - GloReq.BL.y)
    DistanceR = abs(GloReq.TR.x - GloReq.BR.x)
    DistanceT = abs(GloReq.TR.y - GloReq.TL.y)
    DistanceL = abs(GloReq.TL.x - GloReq.BL.x)
    
    rospy.loginfo("Coordinates bottom left: (%s,%s)", GloReq.BL.x, GloReq.BL.y)
    rospy.loginfo("Coordinates top left: (%s,%s)", GloReq.TL.x, GloReq.TL.y)
    rospy.loginfo("Coordinates top right: (%s,%s)", GloReq.TR.x, GloReq.TR.y)
    rospy.loginfo("Coordinates bottom right: (%s,%s)", GloReq.BR.x, GloReq.BR.y)
    rospy.loginfo("")
    rospy.loginfo("length bottom line is: %s", DistanceB)
    rospy.loginfo("length right line is: %s", DistanceR)
    rospy.loginfo("length top line is: %s", DistanceT)
    rospy.loginfo("length left line is: %s", DistanceL)
    rospy.loginfo("")


def route_planner():
    global Route
    global GloReq
    global done

    distDone = 0

    YDir = (GloReq.Closest % 2 == 1)
    XDir = (int(GloReq.Closest / 3) == 0)
    #DistanceR = 25
    NumberOfHorizonLines = int(DistanceR / PointsPL)
    print(NumberOfHorizonLines)
    for x in range(NumberOfHorizonLines):
        tmp = rel_coordinates()
        tmp.x = 0
        tmp.y = DistanceB if YDir else -DistanceB 
        YDir = not YDir
        Route.append(tmp)
        
        tmp = rel_coordinates()
        tmp.x = PointsPL if XDir else -PointsPL
        tmp.y = 0
        distDone += PointsPL
        Route.append(tmp)
    
    tmp = rel_coordinates()
    tmp.x = 0
    tmp.y = DistanceB if YDir else -DistanceB 
    YDir = not YDir
    Route.append(tmp)

    if(distDone < DistanceR):
        tmp = rel_coordinates()
        tmp.x = DistanceR - distDone if XDir else -1 * (DistanceR - distDone)
        tmp.y = 0
        distDone += abs(tmp.x)
        Route.append(tmp)

        tmp = rel_coordinates()
        tmp.x = 0
        tmp.y = DistanceB if YDir else -DistanceB 
        YDir = not YDir
        Route.append(tmp)

    rospy.loginfo("-----------")
    rospy.loginfo("%s", distDone)

    # counter = 0
    for x in Route:
        # print (counter)
        # counter += 1
        rospy.logdebug("(%s,%s)", x.x, x.y)


if __name__ == '__main__':
    rospy.init_node('route_planner')
    s = rospy.Service('route_planner', routeplanner, handle_route_planner)
    rospy.logwarn("Ready to plan the routes.")
    rospy.spin()

    # GloReq.BL.x = 37
    # GloReq.BL.y = 25
    # GloReq.BR.x = 28
    # GloReq.BR.y = 47
    # GloReq.TL.x = 48
    # GloReq.TL.y = 28
    # GloReq.TR.x = 61
    # GloReq.TR.y = 66

    # GloReq.TR.x = -37
    # GloReq.TR.y = -25
    # GloReq.TL.x = -28
    # GloReq.TL.y = -47
    # GloReq.BR.x = -48
    # GloReq.BR.y = -28
    # GloReq.BL.x = -61
    # GloReq.BL.y = -66

    # GloReq.Closest = 1

    # distancecalc()
    # route_planner()

    # Route = []
    # GloReq.Closest = 2

    # distancecalc()
    # route_planner()
    
    # Route = []
    # GloReq.Closest = 3

    # distancecalc()
    # route_planner()
    
    # Route = []
    # GloReq.Closest = 4

    # distancecalc()
    # route_planner()