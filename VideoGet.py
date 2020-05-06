# Modified version from: https://github.com/nrsyed/computer-vision/tree/master/multithread
from threading import Thread
import cv2

class VideoGet:
    """
    Class that continuously gets frames from a VideoCapture object
    with a dedicated thread.
    """

    def __init__(self, src=0, dim=(640, 480), remote=False):
        if remote:
            self.stream = cv2.VideoCapture('tcpclientsrc host=192.168.70.4 port=5000  ! gdpdepay !  rtph264depay ! avdec_h264 ! videoconvert ! appsink sync=false', cv2.CAP_GSTREAMER)
        else:
            self.stream = cv2.VideoCapture(src)
            self.stream.set(3, dim[0])
            self.stream.set(4, dim[1])
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False

    def start(self):
        Thread(target=self.get, args=()).start()
        return self

    def get(self):
        while not self.stopped:
            if not self.grabbed:
                self.stop()
            else:
                (self.grabbed, self.frame) = self.stream.read()

    def stop(self):
        self.stopped = True
