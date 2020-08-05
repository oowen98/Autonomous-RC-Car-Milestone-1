import cv2
import numpy as np

def Thresholding(frame): #Convert image to HSV Space and create mask of desired color (Green)
    frame_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    Color_min = np.array([71,54,89]) #Lower HSV Values of Color 44,47,69 / 50,24,65
    Color_max = np.array([98,255,255]) #Upper HSV Values of Color 97,255,235 / 90,119,255
    mask = cv2.inRange(frame_HSV, Color_min, Color_max)

    return mask

def Warp_Frame(frame, points, width, height, inverse = False): #Get a bird's eye view of the frame
    points1 = np.float32(points) #Points of the Region of Interest
    points2 = np.float32([[0,0], [width, 0], [0, height], [width, height]]) #Points of the entire frame

    if inverse == True:
        matrix = cv2.getPerspectiveTransform(points2, points1) 
    else:
        matrix = cv2.getPerspectiveTransform(points1, points2)

    WarpedFrame = cv2.warpPerspective(frame, matrix, (width, height))

    return WarpedFrame

def nothing(a):
    pass

def ROI_InitTrackbars(init_TrackbarVals, Width = 640, Height = 480): #Creating trackbars to tune Region of Interest Points
    cv2.namedWindow("Trackbars")
    cv2.resizeWindow("Trackbars", 640, 240)
    cv2.createTrackbar("Top Width", "Trackbars", init_TrackbarVals[0], Width//2, nothing)
    cv2.createTrackbar("Top Height", "Trackbars", init_TrackbarVals[1], Height, nothing)
    cv2.createTrackbar("Bottom Width", "Trackbars", init_TrackbarVals[2], Width//2, nothing)
    cv2.createTrackbar("Bottom Height", "Trackbars", init_TrackbarVals[3], Height, nothing)

def ROI_TrackbarVals(Width = 640, Height = 480):
    TopWidth = cv2.getTrackbarPos("Top Width", "Trackbars")
    TopHeight = cv2.getTrackbarPos("Top Height", "Trackbars")
    BottomWidth = cv2.getTrackbarPos("Bottom Width", "Trackbars")
    BottomHeight = cv2.getTrackbarPos("Bottom Height", "Trackbars")

    points = np.float32([(TopWidth, TopHeight), (Width - TopWidth, TopHeight),
                         (BottomWidth, BottomHeight), (Width - BottomWidth, BottomHeight)])
    print(points)
    return points

def draw_Points(frame, points): #function to draw the 4 points for the Region of Interest
    for x in range(4):
        cv2.circle(frame, (int(points[x][0]), int(points[x][1])),10, (0,255,0), cv2.FILLED)
    
    return frame

def Curve_Histogram(frame, min_percentage = 0.1, display = False, region = 1): #Get a histogram of the path
    if region == 1:
        histogram_vals = np.sum(frame, axis = 0) #Pixel Summation accross the entire Height of frame (640 values for entire width of frame)
    else:
        #larger the region value, the smaller the region starting from the bottom -> up
        histogram_vals = np.sum(frame[frame.shape[0]//region:,:], axis = 0) #Pixel Summation accross portion of frame

    sum_hist = np.sum(histogram_vals) #For controls logic 

    max_histogram_vals = np.max(histogram_vals) #max pixel value
    min_histogram_vals = min_percentage*max_histogram_vals #Minimum histogram value to be considered as part of the path. (Reduce Noise)
    #print("min Histogram Values: ",min_histogram_vals)
    #print("Histogram Values: ", sum_hist)
    Path_indices = np.where(histogram_vals >= min_histogram_vals) #Indices of the path. Path will have larger histogram values 
    Midpoint = int(np.average(Path_indices)) #Mid point of the path in pixels. Width of frame = 640 pixels
    #print("Midpoint: ", Midpoint)

    if display == True:
        Histogram_frame = np.zeros((frame.shape), np.uint8) #.shape[0] = Height, .shape[1] = Width, .shape[2] = # of Channels
        for x,intensity in enumerate(histogram_vals):
            cv2.line(Histogram_frame, (x, frame.shape[0]), (x, int(frame.shape[0] - intensity // 255// region)), (255, 0, 255)) #Draw the Path
            cv2.circle(Histogram_frame, (Midpoint, frame.shape[0]), 20, (0,255,255), cv2.FILLED) #Draw the Midpoint of the path
        
        return Midpoint, sum_hist, Histogram_frame
    
    else:
        return Midpoint, sum_hist



