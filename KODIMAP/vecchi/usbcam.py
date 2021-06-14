import cv2
from multiprocessing import  queues

class VideoCapture:
    def __init__(self, name, dx, dy):
        self.cap = cv2.VideoCapture(name)
        self.set(3,dx)
        self.set(4,dy)
        
        self.set(cv2.CAP_PROP_BRIGHTNESS,0.6)
        self.set(cv2.CAP_PROP_CONTRAST,0.6)
        self.set(cv2.CAP_PROP_SATURATION,0.7)
        self.set(cv2.CAP_PROP_HUE,0.5)
        self.set(cv2.CAP_PROP_GAIN,0)
        
        print("BRI",self.get(cv2.CAP_PROP_BRIGHTNESS))
        print("CONT",self.get(cv2.CAP_PROP_CONTRAST))
        print("SAT",self.get(cv2.CAP_PROP_SATURATION))
        print("HUE",self.get(cv2.CAP_PROP_HUE))
        print("GAIN",self.get(cv2.CAP_PROP_GAIN))
        print("EXP",self.get(cv2.CAP_PROP_EXPOSURE))
        print("--------------")
    def set(self, prop, val):
        self.cap.set(prop, val)
      
    def get(self, prop):
        return self.cap.get(prop)
    def release(self):
        return self.cap.release()
    
    def _reader(self, q):
        while True:
            
            ret, frame = self.cap.read()
            if not ret:
                 break
            if not q.empty():
                try:
                    q.get(False)
                except queues.Empty:
                    None
            q.put(frame)
