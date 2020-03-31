import signal
from functools import partial
import time
import sys
import cv2

import argparse
import numpy as np
from multiprocessing import Manager, Process
import threading

from TrackerModule.tracker import LocationTracker
from CameraModule.camera import CameraController
from pid import PID
from gui import GUI
from VideoGet import VideoGet

#cap = cv2.VideoCapture(0)

def core_signal_handler(sig, frame):
    sys.exit()

def pid_signal_handler(sig, frame):
    sys.exit()
    
def core_process(output_path, pan, tilt, pan_error, tilt_error, pid_active):
        signal.signal(signal.SIGINT, core_signal_handler)
        video_getter = VideoGet(0).start()
        tracker = LocationTracker()
        camera = CameraController(camera_position=[0.03,2.82,1.33])

        camera.init(refpoint_world=[0.85, 0.0, 1.35],
                    use_pid = True, pid_active=pid_active, pan_error=pan_error, tilt_error=tilt_error)
                
        servo_thread = threading.Thread(target=camera.CameraRotation_Thread, args=(pan, tilt, video_getter))
        servo_thread.start()
        gui = GUI(output_path, camera, tracker, video_getter)
        gui.root.mainloop()
        servo_thread.join()

def pid_process(output, p, i, d, error, pid_active):
    signal.signal(signal.SIGINT, pid_signal_handler)
    
    pid_control = PID(p.value, i.value, d.value)
    pid_control.initialize()
    
    while True:
        if (pid_active.value == True):
            output.value = pid_control.update(error.value)
    
if __name__== "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-o", "--output", default="./",
        help="path to output directory to store snapshots (default: current folder")
    args = vars(ap.parse_args())

    # start the app
    with Manager() as manager:
        pan_error = manager.Value("i", 0)
        tilt_error = manager.Value("i", 0)
        pan = manager.Value("i", 0)
        tilt = manager.Value("i", 0)
        
        panP = manager.Value("f", 0.188) #188
        panI = manager.Value("f", 0.0)
        panD = manager.Value("f", 0.0)
        
        tiltP = manager.Value("f", 0.1558) # 1558
        tiltI = manager.Value("f", 0.0)
        tiltD = manager.Value("f", 0.0)
        
        pid_active = manager.Value("b", False)
        
        processCore = Process(target=core_process,
                              args=(args["output"], pan, tilt, pan_error, tilt_error, pid_active))
        processPanPID = Process(target=pid_process,
                              args=(pan, panP, panI, panD, pan_error, pid_active))
        processTiltPID = Process(target=pid_process,
                              args=(tilt, tiltP, tiltI, tiltD, tilt_error, pid_active))
        
        processCore.start()
        processPanPID.start()
        processTiltPID.start()
        
        processCore.join()
        processPanPID.join()
        processTiltPID.join()
        
    