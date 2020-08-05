import Perception
import Perception_functions
import Controls
import cv2
import numpy as np
import time
import threading
from Camera import Camera_Thread
import i2c
from FPS_Calculate import FPS
import Camera
import multiprocessing as mp

from datetime import datetime

if __name__ == '__main__':

    #For tuning Region of Interest of camera
    #init_TrackBarVals = [0, 263, 0, 512]
    #Perception_functions.ROI_InitTrackbars(init_TrackBarVals, 640, 480) 

    i2c_break_q = mp.Queue()
    data_q=mp.Queue()
    #display = 1
    
    camera = Camera_Thread().start() #Seperate Thread for streaming video from the camera 
    
    i2c_process = mp.Process(target=i2c.i2c_process, args=(data_q, i2c_break_q))
    i2c_process.start()

    time.sleep(3.0)
    fps = FPS().start() #To determine approximate throughput of camera and program
    frame_cnt = 60
    i = 0
    start_time = (time.time()-1)

    #now = datetime.now()
    #prev_time = now.strftime("%H%M%S%f")
    while True:      
        #Reading from the camera Thread
        frame = camera.read()

        #Perception Module
        Feedback, Path_Command, sum_hist = Perception.get_Path(frame, display=1)
        
        #Controls Module
        SteeringCommand = Controls.PID_SteeringControl(start_time, Path_Command, Feedback, sum_hist)
        ThrottleCommand = Controls.Throttle_Control(SteeringCommand)

        #If not using PID Control
        #SteeringCommand = int((Feedback - Path_Command)/(-1.17))

        #print("PID Steering: ", SteeringCommand, "Raw Steering: ", (Feedback - Path_Command))
        #print("Throttle Command: ", ThrottleCommand)

        #Sending Throttle and Steering Values over i2c to Arduino
        data_q.put([ThrottleCommand, SteeringCommand])

        fps.update()
        i += 1
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Main Thread Loops: ", i)
            fps.stop()
            i2c_break_q.put(1)
            time.sleep(2)
            i2c_process.terminate()
            i2c_process.join()
            camera.stop()
            break 


    print("Elapsed Time: ", fps.elapsed())
    print("Approx FPS: ", fps.fps())




