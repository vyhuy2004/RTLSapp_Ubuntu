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
        
    def init(self, refpoint_world=[0., 0., 0.]):               
        # calculate xyz axes
        self.refpoint_world = np.array(refpoint_world, np.float)
        self.refpoint_pixel = None
        self.center = (self.width//2, self.height//2)
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
        # Perform world-to-screen matrix transformations
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
        # draw circles on the provided image
        if undistort:
            new_img = cv2.undistort(image, self.intrinsic, self.distortion_coefs, None, self.intrinsic)
        mask = np.zeros_like(image)
        for pos in points:
            x, y = pos[0], pos[1]
            if (x>0) and (x<self.width) and (y>0) and (y<self.height):
                mask = cv2.circle(mask, tuple(pos), radius, color, -1)
        new_img = cv2.addWeighted(new_img, 1, mask, 0.3, 0)
        return new_img    
                
    def setPan(self, angle, find_best=False, idle_counter=None):
        # set the pan angle of camera
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
        return True
    
    def setTilt(self, angle, find_best=False, idle_counter=None):
        # set the tilt angle of camera
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
        return True
        
    def setCMD(self, command):
        self.active_cmd = command

    def resetCMD(self):
        self.active_cmd = None
        
    def CameraRotation_Thread(self):
        # camera thread controls camera rotation
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
            time.sleep(0.2)
