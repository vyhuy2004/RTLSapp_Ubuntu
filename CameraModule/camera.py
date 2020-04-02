import cv2
import numpy as np
import time
import pickle

class CameraController():
    # Rotation Servo Parameters
    TILT_SERVO      = 0
    PAN_SERVO       = 1
    MAX_TILT_ANGLE  = 179
    MIN_TILT_ANGLE  = 100
    MAX_PAN_ANGLE   = 179
    MIN_PAN_ANGLE   = 0
    # Rotation Commands
    CMD_UP          = 'up'
    CMD_DOWN        = 'down'
    CMD_LEFT        = 'left'
    CMD_RIGHT       = 'right'
    # PID and Outlier Correction Parameters
    MAX_TILT_ERROR = 15 # unit: pixel
    MAX_PAN_ERROR  = 15
    IDLE_PATIENCE = 8 # unit: times
    OUTLIER_LIMIT = 20
    WEIGHTED_AVERAGE_COEF = 0.9

    def __init__(self, camera_position = None, output_dimension = None, default_pan=90, default_tilt=179, remote=False, socket=None):
        # Setup internal camera parameters
        self.ready = False
        self.remote = remote
        cam_params = pickle.load(open("CameraModule/camera.p", "rb"))
        self.intrinsic = cam_params['intrinsic']
        self.distortion_coefs = cam_params['distortion_coefs']
        self.cam_pos = np.array(camera_position, np.float)
        if output_dimension == None:
            self.height, self.width = 480, 640             
        else:
            self.height, self.width = output_dimension[0], output_dimension[1]
            
        # Initialize servo motors and Rotate to default position
        self.DEFAULT_PAN = default_pan
        self.DEFAULT_TILT = default_tilt
        self.tilt = default_tilt
        self.pan = default_pan
        self.step = 1
        self.active_cmd = None
        self.stop_thread = False   
        
        if remote == False:
            from servo.PCA9685 import PCA9685
            self.pwm = PCA9685()
            try:
                self.pwm.setPWMFreq(50)
                self.pwm.setRotationAngle(self.TILT_SERVO, self.tilt)
                self.pwm.setRotationAngle(self.PAN_SERVO, self.pan)
                #self.pwm_lastactive = time.time()
            except:
                self.pwm.exit_PCA9685()
                print ("[CAMERA] Servo Initialization Failed!")
        else:
            self.socket = socket
        
    def init(self, refpoint_world=[0., 0., 0.], use_pid = True, pid_active=None, pan_error=None, tilt_error=None):
        # todo: look for circle -> get pixel coordinate of the center of circle
        
        # todo: move camera so that camera center is right at the center of circle
        # set this as 0 degree, we also need to know the location of this circle (manually or through dwm)
        
        # todo: calculate xyz axes
        self.refpoint_world = np.array(refpoint_world, np.float)
        self.refpoint_pixel = None
        self.center = (self.width//2, self.height//2)
        if use_pid:
            self.hit_counter = 0
            self.idle_counter = 0
            self.pid_active = pid_active
            self.pid_active.value = False        
            self.pan_error = pan_error
            self.tilt_error = tilt_error
            
        self.z_vec = (refpoint_world - self.cam_pos) / np.linalg.norm(refpoint_world - self.cam_pos)
        
        self.x_vec = np.array([self.z_vec[1], -self.z_vec[0], 0.], np.float)
        self.x_vec /= np.linalg.norm(self.x_vec)
        
        self.y_vec = np.cross(self.z_vec, self.x_vec)
        self.y_vec /= np.linalg.norm(self.y_vec)
        
        # construct extrinsic matrix
        self.extrinsic = np.zeros((4, 4), np.float)
        self.extrinsic[:3, :3] = np.vstack((self.x_vec, self.y_vec, self.z_vec)).T
        self.extrinsic[:, 3] = [*self.cam_pos, 1]
        self.extrinsic = np.linalg.inv(self.extrinsic)
        self.ready = True
        
    def world_to_screen(self, points, return_valid_only=True):
        pixel_pos = []
        for pnt in points:
            world = np.array([*pnt, 1], np.float)
            cam = self.extrinsic.dot(world)[:-1]
            cam = cam / cam[2]                # Divide x,y by z
            pixpos = self.intrinsic.dot(cam)[:2]
            on_screen = (pixpos[0]>=0) and (pixpos[0]<self.height) and \
                        (pixpos[1]>=0) and (pixpos[1]<self.width)
            if return_valid_only and (on_screen == False):
                continue
            pixel_pos.append(pixpos)     # Pixel location, only have x & y
        return np.array(pixel_pos, np.int32)
    
    def draw_points(self, image, points, undistort=True, radius=25, color=(0, 255, 0)):
        if undistort:
            new_img = cv2.undistort(image, self.intrinsic, self.distortion_coefs, None, self.intrinsic)
        mask = np.zeros_like(image)
        for pos in points:
            x, y = pos[0], pos[1]
            if (x>0) and (x<self.width) and (y>0) and (y<self.height):
                mask = cv2.circle(mask, tuple(pos), radius, color, -1)
        new_img = cv2.addWeighted(new_img, 1, mask, 0.3, 0)
        return new_img
    
    def draw_refpoint_estimation(self, image):
        if self.refpoint_pixel == None:
            return image
        else:
            new_image = cv2.circle(image, self.refpoint_pixel, 2, (255, 0, 0), -1)
            new_image = cv2.circle(image, self.center, 2, (255, 0, 0), -1)
            new_image = cv2.line(image, self.center, self.refpoint_pixel, (255, 0, 0), 1)
            return new_image
                
    def setPan(self, angle, find_best=False, idle_counter=None):
        prev_pan = self.pan
        if (angle == self.pan):
            pass
        elif (angle>=self.MIN_PAN_ANGLE) and (angle<=self.MAX_PAN_ANGLE):
            self.pan = angle
        elif (angle<self.MIN_PAN_ANGLE) and find_best:
            self.pan = self.MIN_PAN_ANGLE
        elif (angle>self.MAX_PAN_ANGLE) and find_best:
            self.pan = self.MAX_PAN_ANGLE
        else:
            print(f"[CAMERA] set-angle is out of reach | {angle}")
            return False
        if self.remote:
            self.socket.send(f'pan {self.pan}'.encode())
        else:
            self.pwm.setRotationAngle(self.PAN_SERVO, self.pan)
        if not (idle_counter == None):
            if (prev_pan == self.pan):
                idle_counter = idle_counter + 1
            else:
                idle_counter = max(idle_counter - 1, 0)
        return True
    
    def setTilt(self, angle, find_best=False, idle_counter=None):
        prev_tilt = self.tilt            
        if (angle == self.tilt):
            pass
        elif (angle>=self.MIN_TILT_ANGLE) and (angle<=self.MAX_TILT_ANGLE):
            self.tilt = angle
        elif (angle<self.MIN_TILT_ANGLE) and find_best:
            self.tilt = self.MIN_TILT_ANGLE
        elif (angle>self.MAX_TILT_ANGLE) and find_best:
            self.tilt = self.MAX_TILT_ANGLE
        else:
            print(f"[CAMERA] set-angle is out of reach | {angle}")
            return False
        if self.remote:
            self.socket.send(f'tilt {self.tilt}'.encode())
        else:
            self.pwm.setRotationAngle(self.TILT_SERVO, self.tilt)
        if not (idle_counter == None):
            if (prev_tilt == self.tilt):
                idle_counter = idle_counter + 1
            else:
                idle_counter = max(idle_counter - 1, 0)          
        return True
        
    def setCMD(self, command):
        self.active_cmd = command

    def resetCMD(self):
        self.active_cmd = None
        
    def enablePID(self):
        self.pid_active.value = True
    
    def disablePID(self):
        self.pid_patience = 0
        self.idle_counter = 0
        self.pid_active.value = False        
       
    def CameraRotation_Thread(self, target_pan=None, target_tilt=None, video_getter=None):
        print("[CAMERA] Camera Rotation Thread started")
        while True:
            if self.stop_thread:
                if self.remote == False:
                    self.pwm.exit_PCA9685()
                break
            # serve active command (if any)
            if not (self.active_cmd == None):
                if (self.active_cmd == self.CMD_DOWN) and (self.tilt < self.MAX_TILT_ANGLE):
                    self.setTilt(self.tilt + self.step)
                elif (self.active_cmd == self.CMD_UP) and (self.tilt > self.MIN_TILT_ANGLE):
                    self.setTilt(self.tilt - self.step)
                elif (self.active_cmd == self.CMD_LEFT) and (self.pan < self.MAX_PAN_ANGLE):
                    self.setPan(self.pan + self.step)
                elif (self.active_cmd == self.CMD_RIGHT) and (self.pan > self.MIN_PAN_ANGLE):
                    self.setPan(self.pan - self.step)      
                time.sleep(0.1)

            # Handle angle adjustment made by PID processes
            if self.pid_active.value == True:  # PID is running -> find reference point
                self.setTilt(-target_tilt.value + self.DEFAULT_TILT, find_best=True, idle_counter=self.idle_counter)
                self.setPan(target_pan.value + self.DEFAULT_PAN, find_best=True, idle_counter=self.idle_counter)
                time.sleep(0.3)
                self.refpoint_pixel = self.find_refpoint(video_getter.frame, use_canny=True)
                if self.pid_conditional_exit():
                    continue
                # If reference point found, calculate errors
                new_pan_error = self.center[0] - self.refpoint_pixel[0]
                new_tilt_error = self.center[1] - self.refpoint_pixel[1]
                #print(f"new pan error: {new_pan_error} | new tilt error: {new_tilt_error} | hit counter: {self.hit_counter}")

                # Check if this value is valid (not outlier)
                if (abs(new_pan_error - self.pan_error.value) > self.OUTLIER_LIMIT) or (abs(new_tilt_error - self.tilt_error.value) > self.OUTLIER_LIMIT):
                    # Need this to handle cold start
                    if (self.pan_error.value == 0):
                        self.pan_error.value = new_pan_error
                    if (self.tilt_error.value == 0):
                        self.tilt_error.value = new_tilt_error
                    continue
                self.pan_error.value = int(self.WEIGHTED_AVERAGE_COEF * new_pan_error      \
                                           +(1-self.WEIGHTED_AVERAGE_COEF) * self.pan_error.value)
                self.tilt_error.value = int(self.WEIGHTED_AVERAGE_COEF * new_tilt_error      \
                                           +(1-self.WEIGHTED_AVERAGE_COEF) * self.tilt_error.value)
                print(f"pan error: {self.pan_error.value} | tilt error: {self.tilt_error.value} | hit counter: {self.hit_counter}")
                if (abs(self.pan_error.value) < self.MAX_PAN_ERROR) and (abs(self.tilt_error.value) < self.MAX_TILT_ERROR):
                    self.hit_counter = self.hit_counter + 1
                else:
                    self.hit_counter = max(self.hit_counter-1, 0)
                    
                        
    def pid_conditional_exit(self):
        if (self.idle_counter == self.IDLE_PATIENCE) or (self.hit_counter == self.IDLE_PATIENCE):
            self.disablePID()                    
            print("[CAMERA] Camera has been idle for a while. PID stopped.")
            return True
        elif self.refpoint_pixel == None: # No reference point found -> disable PID, skip
            self.disablePID()
            print("[CAMERA] Cannot locate reference point! PID disabled.")
            return True
        return False
               
    
    def find_refpoint(self, img, use_canny=True):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (3,3), 0)
        if use_canny: # more accurate but slow
            edged = cv2.Canny(blur, 200, 300)
            contours, hierarchy = cv2.findContours(edged, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) == 0:
                return None
            refpoint = tuple(np.mean(contours[0], axis=0, dtype=np.int32)[0])
        else: # only use color threshold -> less accurate
            ret, thresh = cv2.threshold(blur, 40, 255, cv2.THRESH_BINARY_INV)
            ys, xs = np.nonzero(thresh)
            if len(ys) == 0:
                return None
            refpoint = (int(np.median(xs)), int(np.median(ys)))
        return refpoint