#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
#check world debug
import sys
from sensor_msgs.msg import CompressedImage
import cv2
import imutils

video_size = 700, 500
image_pub = rospy.Publisher('world_observation/features', CompressedImage, queue_size=10)
firstFrame = None

def callback(ros_data):
    global firstFrame
    np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    gray = cv2.cvtColor(image_np, cv2.COLOR_RGB2GRAY)
    gray = cv2.GaussianBlur(gray, (21, 21), 0)
    if firstFrame is None:
        firstFrame = gray
    frameDelta = cv2.absdiff(firstFrame, gray)
    thresh = cv2.threshold(frameDelta, 25, 255, cv2.THRESH_BINARY)[1]
    thresh = cv2.dilate(thresh, None, iterations=2)
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[1]
    rospy.loginfo(len(cnts))
    for c in cnts:
        if not (cv2.contourArea(c) < 100 or cv2.contourArea(c) > 2000):
            (x, y, w, h) = cv2.boundingRect(c)
            cv2.rectangle(image_np, (x, y), (x + w, y + h), (255, 255, 255), 2)
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    compressed_images = cv2.imencode('.jpg', image_np)
    msg.data = np.array(compressed_images[1]).tostring()
    image_pub.publish(msg)
    firstFrame = gray

def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('world_observation_features', anonymous=True)
    subscriber = rospy.Subscriber('world_observation/image_raw', CompressedImage, callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Gym Image Viewer module")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
