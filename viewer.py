#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
#check world debug
import sys
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
import pygame
import cv2

video_size = 700, 500
screen = pygame.display.set_mode(video_size)

def callback(ros_data):
    np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    surf = pygame.surfarray.make_surface(image_np)
    screen.blit(surf, (0, 0))
    pygame.display.update()

def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('gym_viewer', anonymous=True)
    subscriber = rospy.Subscriber('world_observation/image_raw', CompressedImage, callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Gym Image Viewer module")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
