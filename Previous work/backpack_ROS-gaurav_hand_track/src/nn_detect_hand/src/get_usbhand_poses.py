#!/usr/bin/env python3
# import sys
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import roslib
roslib.load_manifest('nn_detect_hand')
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# from include.wrapper import Hands



class rosinterface(object):
    def __init__(self):
        super(rosinterface, self).__init__()
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.__callback__)
        self.image_pub = rospy.Publisher("image_topic_2", Image)
        # print("OK")

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
    # K=Hands()
    # K.init_parser()
    # K.initialize()
    rospy.init_node('get_hand_position', anonymous=True)
    im=rosinterface()

    try:
        # K.run()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()

    # while True:
    #
    #     if cv2.waitKey(25) & 0xFF == ord('q'):
    #         cv2.destroyAllWindows()
    #         break
