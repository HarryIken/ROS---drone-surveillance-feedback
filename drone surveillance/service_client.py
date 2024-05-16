#!/usr/bin/env python3

import rospy
from drone_package.srv import ImageRequest, ImageRequestResponse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def show_image(image):
    """
    Display the received image.
    """
    cv_bridge = CvBridge()
    cv_image = cv_bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
    cv2.imshow("Drone Image", cv_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def request_image(image_request_proxy):
    """
    Request an image from the drone and display it.
    """
    try:
        response = image_request_proxy()  # Send request to the drone
        if response.success:
            rospy.loginfo("Image received successfully.")
            show_image(response.image)  # Display the received image
        else:
            rospy.logwarn("Failed to receive image from the drone.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == "__main__":
    rospy.init_node("drone_image_client_node")

    # Wait for the service to be available
    rospy.wait_for_service("image_request_service")

    # Create a service proxy to communicate with the drone
    try:
        image_request_proxy = rospy.ServiceProxy("image_request_service", ImageRequest)

        # Request an image from the drone
        request_image(image_request_proxy)
    except rospy.ROSException as e:
        rospy.logerr("Failed to create service proxy: %s", e)
