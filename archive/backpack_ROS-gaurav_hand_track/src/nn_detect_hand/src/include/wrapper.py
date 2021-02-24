import include.utils.detector_utils as detector_utils
# import utils.detector_utils as detector_utils
import cv2
import tensorflow as tf
import datetime
import argparse

detection_graph, sess = detector_utils.load_inference_graph()

class Hands:
	def __init__(self):
		print("HELLO")
		self.init_parser()
		self.num_hands_detect = 2


	def init_parser(self):
		self.parser = argparse.ArgumentParser()
		self.parser.add_argument(
			'-sth',
			'--scorethreshold',
			dest='score_thresh',
			type=float,
			default=0.8,
			help='Score threshold for displaying bounding boxes')
		self.parser.add_argument(
			'-fps',
			'--fps',
			dest='fps',
			type=int,
			default=1,
			help='Show FPS on detection/display visualization')
		self.parser.add_argument(
			'-src',
			'--source',
			dest='video_source',
			default=0,
			help='Device index of the camera.')
		self.parser.add_argument(
			'-wd',
			'--width',
			dest='width',
			type=int,
			default=320,
			help='Width of the frames in the video stream.')
		self.parser.add_argument(
			'-ht',
			'--height',
			dest='height',
			type=int,
			default=180,
			help='Height of the frames in the video stream.')
		self.parser.add_argument(
			'-ds',
			'--display',
			dest='display',
			type=int,
			default=1,
			help='Display the detected images using OpenCV. This reduces FPS')
		self.parser.add_argument(
			'-num-w',
			'--num-workers',
			dest='num_workers',
			type=int,
			default=4,
			help='Number of workers.')
		self.parser.add_argument(
			'-q-size',
			'--queue-size',
			dest='queue_size',
			type=int,
			default=5,
			help='Size of the queue.')
		cv2.namedWindow('Single-Threaded Detection', cv2.WINDOW_NORMAL)
		self.args = self.parser.parse_args()

	def initialize(self):
		self.cap = cv2.VideoCapture(self.args.video_source)
		self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.args.width)
		self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.args.height)
		self.im_width, self.im_height = (self.cap.get(3), self.cap.get(4))
		# max number of hands we want to detect/track


	def getxy(self,image,REALSENSE):
		# Expand dimensions since the model expects images to have shape: [1, None, None, 3]

		if not (REALSENSE):
			ret, image_np = self.cap.read()
		else:
			self.im_width =image.shape[1]
			self.im_height  = image.shape[0]
			image_np = image
		# print(image_np.shape)
		# image_np = cv2.flip(image_np, 1)

		try:
			image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
		except:
			print("Error converting to RGB")

		# Actual detection. Variable boxes contains the bounding box cordinates for hands detected,
		# while scores contains the confidence for each of these boxes.
		# Hint: If len(boxes) > 1 , you may assume you have found atleast one hand (within your score threshold)

		boxes, scores = detector_utils.detect_objects(image_np,
													  detection_graph, sess)

		# draw bounding boxes on frame
		centre = detector_utils.draw_box_on_image(self.num_hands_detect, self.args.score_thresh,
										 scores, boxes, self.im_width, self.im_height,
										 image_np)

		cv2.imshow('Single-Threaded Detection',
				   cv2.cvtColor(image_np, cv2.COLOR_RGB2BGR))

		cv2.waitKey(1)
		return centre

def main():
	K=Hands()
	K.initialize()
	k=1
	while True:
		K.getxy(k, False)
		if cv2.waitKey(25) & 0xFF == ord('q'):
			cv2.destroyAllWindows()
			break


