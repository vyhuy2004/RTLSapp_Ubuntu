import os
import tkinter as tk
import datetime
import cv2
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
from PIL import Image, ImageTk

MARGIN = 10

class GUI:
    def __init__(self, camera, tracker, video_getter, detector, remote=False, socket=None):
        self.output_path = "./"    
        self.current_image = None
        
        # Keep references to other class objects
        self.camera = camera
        self.tracker = tracker
        self.video_getter = video_getter
        self.detector = detector
        if remote:
            self.remote = True
            self.socket = socket
        else:
            self.remote = False
        # Create Main Window
        self.root = tk.Tk()  
        self.root.title("RTLS & Camera Integrated System")  
        self.root.protocol('WM_DELETE_WINDOW', self.destructor)
        
        # Create Main Blocks
        self.control_frame = tk.Label(self.root, padx=5, pady=5)
        self.plot_frame = tk.LabelFrame(self.root, text="Grid Map", padx=5, pady=5, labelanchor='n')
        self.video_frame = tk.LabelFrame(self.root, text="Camera", padx=5, pady=5, labelanchor='n')
        
        self.control_frame.grid(row = 0, column = 0, columnspan=2, sticky = 'WE')
        self.plot_frame.grid(row = 1, column = 0, sticky = 'NSEW')
        self.video_frame.grid(row = 1, column = 1, sticky = 'NSEW')

        self.add_control_content(self.control_frame)
        self.add_plot_content(self.plot_frame)
        self.add_video_content(self.video_frame)


    def add_control_content(self, master):
        tk.Grid.rowconfigure(master, 0, weight=1)
        tk.Grid.columnconfigure(master, 0, weight=1)
        tk.Grid.columnconfigure(master, 1, weight=1)

        b1 = tk.Button(master, text = " Run ", command=self.run_tracker) 
        b2 = tk.Button(master, text = "Stop", command=self.tracker.stop) 

        b1.grid(row = 0, column = 0, sticky = 'NSEW') 
        b2.grid(row = 0, column = 1, sticky = 'NSEW') 


    def add_plot_content(self, master):
        self.plot_fig, self.plot_ax = plt.subplots()
        self.plot_bar = FigureCanvasTkAgg(self.plot_fig, master)
        self.plot_bar.draw()
        self.plot_bar.get_tk_widget().grid(row = 0, column = 0, sticky = 'WE')
        self.plot_ani = animation.FuncAnimation(self.plot_fig, self.tracker._update_plot, interval=200, 
                                     init_func= lambda: self.tracker._setup_plot(self.plot_ax), blit=True)

    def add_video_content(self, master):
        self.img_up = tk.PhotoImage(file="GUI_resources/up.gif", master=master)
        self.img_down = tk.PhotoImage(file="GUI_resources/down.gif", master=master)
        self.img_left = tk.PhotoImage(file="GUI_resources/left.gif", master=master)
        self.img_right = tk.PhotoImage(file="GUI_resources/right.gif", master=master)

        self.vid_panel = tk.Label(master)  # initialize image panel

        self.vid_panel.grid(row = 0, column = 0, columnspan=3, sticky = 'nsew')
        self.vid_btn_up = tk.Button(master, image=self.img_up)
        self.vid_btn_up.bind("<Button-1>", lambda event: self.camera.setCMD('up'))
        self.vid_btn_up.bind("<ButtonRelease-1>", lambda event: self.camera.resetCMD())

        self.vid_btn_down = tk.Button(master, image=self.img_down)
        self.vid_btn_down.bind("<Button-1>", lambda event: self.camera.setCMD('down'))
        self.vid_btn_down.bind("<ButtonRelease-1>", lambda event: self.camera.resetCMD())

        self.vid_btn_left = tk.Button(master, image=self.img_left)
        self.vid_btn_left.bind("<Button-1>", lambda event: self.camera.setCMD('left'))
        self.vid_btn_left.bind("<ButtonRelease-1>", lambda event: self.camera.resetCMD())
        
        self.vid_btn_right = tk.Button(master, image=self.img_right)
        self.vid_btn_right.bind("<Button-1>", lambda event: self.camera.setCMD('right'))
        self.vid_btn_right.bind("<ButtonRelease-1>", lambda event: self.camera.resetCMD())
        
        self.yolo_str, self.tags_str = tk.StringVar(), tk.StringVar()
        self.yolo_str.set("DETECTOR: OFF")
        self.tags_str.set("SHOW TAG: OFF")
        self.yolo_bol = False 
        self.mode = 'combined'
        self.draw_tags = False
        self.vid_btn_yolo = tk.Button(master, textvariable=self.yolo_str, bg="#b8dcff", command=self.toggle_yolo)
        self.vid_btn_capture = tk.Button(master, text = " CAPTURE ", bg="#b8dcff", command=self.video_capture) 
        self.vid_btn_drawtags = tk.Button(master, textvariable=self.tags_str, bg="#b8dcff", command=self.toggle_drawtags) 

        # arranging button widgets 
        self.vid_btn_up.grid(row = 1, column = 1, sticky = 'nsew') 
        self.vid_btn_down.grid(row = 2, column = 1, sticky = 'nsew')  
        self.vid_btn_left.grid(row = 1, column = 0, rowspan=2, sticky = 'nsew')
        self.vid_btn_right.grid(row = 1, column = 2, rowspan=2, sticky = 'nsew') 
        self.vid_btn_yolo.grid(row = 3, column = 0, sticky = 'nsew')
        self.vid_btn_capture.grid(row = 3, column = 2, sticky = 'nsew')
        self.vid_btn_drawtags.grid(row = 3, column = 1, sticky = 'nsew')       
        self.video_update()
        
    def video_update(self):
        # Update tags' pixel locations
        self.pixpts = []
        frame = self.video_getter.frame

        if not self.tracker.tags_latest.empty:
            pts = self.tracker.tags_latest[['x', 'y', 'z']].to_numpy().astype(float)
            pts = np.where(pts<0, 0, pts) 
            self.pixpts = self.camera.world_to_screen(pts)
            if self.draw_tags:
                frame = self.camera.draw_points(frame, self.pixpts)

        # If detector is active, we also have the bounding boxes of human
        if (self.yolo_bol==True):
            if self.mode == 'raw':
                frame = self.detector.frame
            else:
                bboxes = self.detector.bboxes
                fontScale = 0.5
                bbox_thick = int(0.6 * (frame.shape[0] + frame.shape[1]) / 600)
                matched_boxes = []
                matched_names = []
                for j, pt in enumerate(self.pixpts):
                    min_area = 0
                    pt_x, pt_y = pt
                    for i, bbox in enumerate(bboxes):
                        (c1, c2) = bbox
                        x_min, y_min, x_max, y_max = c1[0], c1[1], c2[0], c2[1]
                        area = (x_max - x_min) * (y_max - y_min)
                        if (pt_x - MARGIN > x_min) and (pt_x + MARGIN < x_max) and \
                           (pt_y - MARGIN > y_min) and (pt_y + MARGIN < y_max): 
                            if (min_area == 0) or (area < min_area):
                                min_area = area
                                min_idx = i
                                min_name = f'{self.tracker.names[self.tracker.tags_latest.iloc[j, 0]]}' + \
                                           f'({self.tracker.tags_latest.iloc[j, 1]},{self.tracker.tags_latest.iloc[j, 2]})'   
                    if (min_area > 0) and (min_idx not in matched_boxes):
                        matched_boxes.append(min_idx)
                        matched_names.append(min_name)

                for i, bbox in enumerate(bboxes):
                    if i not in matched_boxes:
                        color = (255, 0, 0)
                        name = 'customer'
                        (c1, c2) = bbox 
                        cv2.rectangle(frame, c1, c2, color, bbox_thick)
                        t_size = cv2.getTextSize(name, 0, fontScale, thickness=bbox_thick//2)[0]
                        cv2.rectangle(frame, c1, (c1[0] + t_size[0], c1[1] - t_size[1] - 3), color, -1)  # filled
                        cv2.putText(frame, name, (c1[0], c1[1]-2), cv2.FONT_HERSHEY_SIMPLEX, fontScale, (0, 0, 0), bbox_thick//2, lineType=cv2.LINE_AA)

                for i, bbox_idx in enumerate(matched_boxes):
                    color = (0, 255, 0)
                    name = matched_names[i]
                    (c1, c2) = bboxes[bbox_idx] 
                    cv2.rectangle(frame, c1, c2, color, bbox_thick)
                    t_size = cv2.getTextSize(name, 0, fontScale, thickness=bbox_thick//2)[0]
                    cv2.rectangle(frame, c1, (c1[0] + t_size[0], c1[1] - t_size[1] - 3), color, -1)  # filled
                    cv2.putText(frame, name, (c1[0], c1[1]-2), cv2.FONT_HERSHEY_SIMPLEX, fontScale, (0, 0, 0), bbox_thick//2, lineType=cv2.LINE_AA)
                

        frame = cv2.circle(frame, self.camera.center, 2, (0, 255, 0), -1)
        cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # convert colors from BGR to RGBA
        self.current_image = Image.fromarray(cv2image)  # convert image for PIL
        imgtk = ImageTk.PhotoImage(image=self.current_image)  # convert image for tkinter
        self.vid_panel.imgtk = imgtk  # anchor imgtk so it does not be deleted by garbage-collector
        self.vid_panel.config(image=imgtk)  # show the image
        self.after_id = self.root.after(50, self.video_update)  # call the same function after 30 milliseconds

    def video_capture(self):
        """ Take snapshot and save it to the file """
        ts = datetime.datetime.now() # grab the current timestamp
        filename = "{}.jpg".format(ts.strftime("%Y-%m-%d_%H-%M-%S"))  # construct filename
        p = os.path.join(self.output_path, filename)  # construct output path
        self.current_image.save(p, "JPEG")  # save image as jpeg file
        print("[APP] saved {}".format(filename))
        
    def toggle_yolo(self):
        if self.yolo_bol == False:
            self.yolo_str.set("DETECTOR: ON")
            self.yolo_bol = True
            self.detector.active = True
        else:
            self.yolo_str.set("DETECTOR: OFF")
            self.yolo_bol = False
            self.detector.active = False

    def toggle_drawtags(self):
        if self.draw_tags == False:
            self.tags_str.set("SHOW TAG: ON")
            self.draw_tags = True            
        else:
            self.tags_str.set("SHOW TAG: OFF")
            self.draw_tags = False
                        
    def run_tracker(self):
        self.tracker.run()
        if (self.remote==True) and (self.tracker.mock==False):
            self.socket.send(b'run')

    def destructor(self):
        """ Destroy the root object and release all resources """
        print("[APP] closing...")
        if self.remote:
            self.socket.close()
        self.camera.stop_thread = True
        self.detector.stop_thread = True
        self.video_getter.stop()
        self.video_getter.stream.release()
        self.root.after_cancel(self.after_id)
        self.root.destroy()
        cv2.destroyAllWindows()  # it is not mandatory in this application

