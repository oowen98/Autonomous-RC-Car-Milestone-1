import cv2
import time
from gstreamer import gstreamer_pipeline
import threading
import multiprocessing as mp

#CameraLock = threading.Lock()

class Camera_Thread:
    def __init__(self):
        self.cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=2), cv2.CAP_GSTREAMER)

        self.frame = None
        self.stopped = False
        self.success = False
        print("Initializing Camera Thread")
        
    def start(self):
        threading.Thread(target=self.update, args=()).start()
        return self
    
    def update(self):
        i = 0
        while True:
            #CameraLock.acquire()
            self.success, self.frame = self.cap.read()
            #CameraLock.release()
            i += 1

            if self.stopped:
                print("Camera Loops: ", i)
                break
                self.cap.release()
                cv2.destroyAllWindows()

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True
    

class VideoShow_Thread:

    def __init__(self, frame=None):
        self.frame = frame
        self.stopped = False
    
    def start(self):
        threading.Thread(target=self.show, args=()).start()
        return self

    def show(self):
        while not self.stopped:
            cv2.imshow('Result', self.frame)
            if cv2.waitKey(1) == ord("q"):
                self.stopped = True

    def stop(self):
        self.stopped = True


def VideoShow_Process(frame_q, break_q):
    print("Initializing videohow process")
    while True:
        if(frame_q.empty() == 0):
            frame = frame_q.get()

            cv2.imshow('Result', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                frame_q.close()
                break_q.put(2)
                break