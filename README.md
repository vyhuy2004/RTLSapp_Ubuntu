### Real-time Location Tracking System with Camera Tracking Capability
<b>Objective:</b> The goal of this project is to build a security camera system that leverages the power of both Real-time Location System (RTLS) and Computer Vision (CV). Locations of target objects will be mapped from world coordinates to imagery coordinates, which allows us to see exactly where these targets are on the screen. By combining these results with outputs from our human detection algorithm (based on Yolov3), our system can recognize individual person based on the tracker that he/she is carrying.

---
#### Features:

- Multiple-object location tracking with visualization
- World-to-Screen mapping of target object's location
- Human Detection using Yolov3
- Individual human recognition by combining RTLS & Yolov3

---
#### Setup instructions:
**Basic requirements**

    - At least 4 anchors
    - At least 1 tag (tracker)
    - One listener node connected to Raspberry Pi via Uart port
    - Camera location & reference point are measured and entered in main program
    - Download YOLOv3 weights and place it in yolov3 folder 
    (https://drive.google.com/file/d/1YICHUHOtbpF9HJ_UqgEGrNcM1oHPXXxs/view?usp=sharing)

**To run locally on Raspberry Pi**

	- 'workon py3cv4'
	- 'python main.py'

**To run remotely on any computer sharing the same network with Raspberry Pi**
<br>Using remote_client_nopid.py is recommended since it is faster and more accurate. A new version of remote_client.py (with pid) will be released soon.

	- modify IP adrress & port number in remote_client.py & remote_server.py
	- remote_server.py should be run on the raspberry pi (on "py3cv4" virtual environment -> "workon py3cv4")
	- remote_client.py or remote_client_nopid.py should be run on remote computer (opencv & gstreamer are required for video streaming)

---
#### Credits:

Our Yolov3 model in Tensorflow is a modified version of YunYang1994's Yolov3 implementation (https://github.com/YunYang1994/tensorflow-yolov3)