import time
import sys
import threading
import socket

from TrackerModule.tracker import LocationTracker
from CameraModule.camera import CameraController
from gui import GUI
from VideoGet import VideoGet
from detector import Yolov3_Detector

# Server Static IP (ethernet)
SERVER_IP = '192.168.70.4'
SERVER_PORT = 8802

# Camera Position
CAMERA_POSITION = [0.03,2.81,1.33]
REFPOINT_POSITION = [0.85, 0.0, 1.35]

def main():
    s = socket.socket()
    s.connect((SERVER_IP, SERVER_PORT))
    print(s.recv(1024))

    video_getter = VideoGet(remote=True).start()
    tracker = LocationTracker(address=('', 8801), remote=True, use_mockdata=False)
    camera = CameraController(CAMERA_POSITION, socket=s, remote=True)
    camera.init(REFPOINT_POSITION)
    detector = Yolov3_Detector(video_getter)        
    
    detector_thread = threading.Thread(target=detector.Inference_Thread)
    servo_thread = threading.Thread(target=camera.CameraRotation_Thread)
    detector_thread.start()
    servo_thread.start()

    gui = GUI(camera, tracker, video_getter, detector, socket=s, remote=True)
    gui.root.mainloop()
    detector_thread.join()
    servo_thread.join()
    s.close()
      

if __name__== "__main__":
    main()
