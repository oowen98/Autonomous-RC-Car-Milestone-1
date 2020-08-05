#Module to detect Lines 

import cv2
import numpy as np
import Perception_functions


curveListSize = 10
curveList = []

def get_Path(frame, display=1): #Get the curve of the path.
    i = 0
   
    while True:

        frameCopy = frame.copy() #Copy of frame
        
        Threshold_frame = Perception_functions.Thresholding(frame) #Get the threshold

        Height, Width, Channels = frame.shape
        #Use if tuning trackbar values
        #points = Perception_functions.ROI_TrackbarVals(Width, Height) 
        #From initial trackbar values, removed trackbars to speed up computation
        points = np.float32([(0, 200), (640, 200),
                            (0,550), (640,550)])
        #print(points)
        WarpedFrame = Perception_functions.Warp_Frame(Threshold_frame, points, Width, Height) #Getting Birds Eye view of path
        #WarpedFramePoints = Perception_functions.draw_Points(frameCopy, points)

        MidPoint, sum_hist = Perception_functions.Curve_Histogram(WarpedFrame, display=False, min_percentage=0.35, region=3) #Creating histogram of path

        Feedback = Width // 2 #Centre of the car is the feedback position
        
        if display != 0:
            imgResult = frame.copy()
            imgInvWarp = Perception_functions.Warp_Frame(WarpedFrame, points, Width, Height, inverse=True)
            imgInvWarp = cv2.cvtColor(imgInvWarp, cv2.COLOR_GRAY2BGR)
            imgInvWarp[0:Height // 4, 0:Width] = 0, 0, 0
            imgLaneColor = np.zeros_like(frame)
            imgLaneColor[:] = 255, 255, 255 #colour of detected lane
            imgLaneColor = cv2.bitwise_and(imgInvWarp, imgLaneColor)
            imgResult = cv2.addWeighted(imgResult, 1, imgLaneColor, 1, 0) #Overlaying the Lane detection with the original video frame
            
            midY = 450 #Height Value of centre markings
            #cv2.putText(imgResult, str((Feedback - MidPoint)/(-2.5)), (Width // 2 - 80, 85), cv2.FONT_HERSHEY_COMPLEX, 2, (255, 0, 255), 3) #Display AvgCurve Value on Screen
            
            #Feedback is the centre of the car / frame
            FrameCentrePt1 = (Feedback, midY - 50) #Line for the centre of the car
            FrameCentrePt2 = (Feedback, midY +50) 
            cv2.line(imgResult, FrameCentrePt1, FrameCentrePt2, (255,0,255), 5)
            
            #Midpoint is the middle of the green lines
            CommandCentrePt1 = (MidPoint, midY -50)
            CommandCentrePt2 = (MidPoint, midY +50)
            cv2.line(imgResult, CommandCentrePt1, CommandCentrePt2, (0, 255,255), 5)

            #Visualizing Difference between Command and Feedback 
            cv2.line(imgResult, (MidPoint, midY), (Width//2 , midY), (255,255,255), 5)

            #Feedback - Command 
            #Positive = Left Turn
            #Negative = Right Turn
            #print("Feedback - Command: ", Feedback - MidPoint)
            cv2.imshow('Result', imgResult)

        return  Feedback, MidPoint, sum_hist
