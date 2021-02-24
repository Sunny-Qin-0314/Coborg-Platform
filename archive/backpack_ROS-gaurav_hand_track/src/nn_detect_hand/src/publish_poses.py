#!/usr/bin/env python3
import roslib
roslib.load_manifest('nn_detect_hand')
import pyrealsense2 as rs
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from include.hand_depth import img_correction,hand_distance
from include.wrapper import Hands,main
from geometry_msgs.msg import PoseStamped

class interface(object):
    def __init__(self):
        super(interface, self).__init__()
        # self.bridge = CvBridge()
        # self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.__callback__)
        self.xyd_pub = rospy.Publisher("/nn_hand/pose",PoseStamped,queue_size=1)
        # print("OK")
        #Parameters
        self.DTreshold=1.0 # set this according to average lengh of a human hand


    def publish(self, xyd):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = xyd[0][0]
        pose.pose.position.y = xyd[0][1]
        pose.pose.position.z = xyd[0][2]

        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1

        self.xyd_pub.publish(pose)


    def start_realsense(self):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        #rs.config.enable_device_from_file(self.config,bag_file_path)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        pipe_profile = self.pipeline.start(self.config)
        depth_sensor = pipe_profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()

        align_to = rs.stream.color
        return rs.align(align_to)


    def get_frames(self):
        return self.pipeline.wait_for_frames()

    def __callback__(self, data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        except CvBridgeError as e:
            print(e)

        (rows, cols, channels) = cv_image.shape
        if cols > 60 and rows > 60:
            cv2.circle(cv_image, (50, 50), 10, 255)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

        try:
            print("OK")
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "rgb8"))
        except CvBridgeError as e:
            print(e)



if __name__ == '__main__':
    rospy.init_node('get_hand_position', anonymous=True)
    REALSENSE = True
    if not REALSENSE:
        main()
    else:

        haath = Hands()  # haath in english means hands
        # haath.init_parser()
        # haath.initialize()


        itf=interface()
        align = itf.start_realsense()

        rate = rospy.Rate(5.0)

        while not rospy.is_shutdown():
            try:
                img, depth, depth_intr = img_correction(itf.get_frames(),align,itf.DTreshold)
                centre = haath.getxy(img,REALSENSE)
                xyd = hand_distance(depth,depth_intr,centre, itf.DTreshold)
                print(xyd)
                # print(type(xyd))
                if len(xyd)>0:
                    itf.publish(xyd)
                    print(xyd[0][0], xyd[0][1])
            except KeyboardInterrupt:
                print("Shutting down")
        im.pipeline.stop()
        cv2.destroyAllWindows()
