import cv2
import time
import numpy as np
import yolov3.utils as utils
import tensorflow.compat.v1 as tf
from VideoGet import VideoGet

return_elements = ["input/input_data:0", "pred_sbbox/concat_2:0", "pred_mbbox/concat_2:0", "pred_lbbox/concat_2:0"]
pb_file         = "./yolov3/yolov3_coco.pb"
#video_path      = "./docs/images/road.mp4"
video_path      = 0
num_classes     = 80
input_size      = 416
graph           = tf.Graph()
return_tensors  = utils.read_pb_return_tensors(graph, pb_file, return_elements)

class Yolov3_Detector():
	def __init__(self, video_getter):
		self.video_getter = video_getter
		self.frame = self.video_getter.frame
		self.active = False
		self.stop_thread = False
		self.bboxes = []

	def Inference_Thread(self):
		tf.disable_v2_behavior()
		with tf.Session(graph=graph) as sess:
			while True:
				if self.stop_thread:
					break
				if self.active:
					frame = self.video_getter.frame
					frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
					frame_size = frame.shape[:2]
					image_data = utils.image_preporcess(np.copy(frame), [input_size, input_size])
					image_data = image_data[np.newaxis, ...]

					pred_sbbox, pred_mbbox, pred_lbbox = sess.run([return_tensors[1], return_tensors[2], return_tensors[3]],
					feed_dict={ return_tensors[0]: image_data})

					pred_bbox = np.concatenate([np.reshape(pred_sbbox, (-1, 5 + num_classes)),
					np.reshape(pred_mbbox, (-1, 5 + num_classes)),
					np.reshape(pred_lbbox, (-1, 5 + num_classes))], axis=0)

					bboxes = utils.postprocess_boxes(pred_bbox, frame_size, input_size, 0.3)
					bboxes = utils.nms(bboxes, 0.45, method='nms')
					self.bboxes = utils.get_human_bboxes(bboxes)
					self.frame = utils.draw_bbox(frame, bboxes)

					# cv2.namedWindow("result", cv2.WINDOW_AUTOSIZE)
					# result = cv2.cvtColor(self.frame, cv2.COLOR_RGB2BGR)
					# cv2.imshow("result", result)
					# if cv2.waitKey(1) & 0xFF == ord('q'): 
					# 	break
				else:
					time.sleep(0.1)

# if __name__ == '__main__':
# 	video_getter = VideoGet().start()
# 	detector = Yolov3_Detector(video_getter)
# 	detector.inference_thread()
# 	video_getter.stop()
# 	video_getter.stream.release()