#!/usr/bin/env python3

import rospy
from my_robot_tutorial.srv import DronePackage, DronePackageResponse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

def capture_image(req):
    """
    Capture an image from the drone's camera and return it as a response.
    """
    # Assuming you have a function to capture an image from the drone
    # Here, we'll just read a sample image from the file system for demonstration
    image_path = "/home/harry/catkin_ws/src/my_robot_tutorial/scripts/imgs/", f"{angle}.png"
    if os.path.exists(image_path):
        cv_image = cv2.imread(image_path)

        if cv_image is not None:
            cv_bridge = CvBridge()
            image_msg = cv_bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            return ImageRequestResponse(success=True, image=image_msg)
        else:
            rospy.logerr("Failed to read the image: %s", image_path)
            return ImageRequestResponse(success=False, image=None)
    else:
        rospy.logerr("Image not found: %s", image_path)
        return ImageRequestResponse(success=False, image=None)

def image_request_server():
    """
    Initialize the ROS node and advertise the image request service.
    """
    rospy.init_node("image_request_server")
    rospy.Service("image_request_service", ImageRequest, capture_image)
    rospy.loginfo("Image request service ready.")
    rospy.spin()

if __name__ == "__main__":
    try:
        image_request_server()
    except rospy.ROSInterruptException:
        pass
