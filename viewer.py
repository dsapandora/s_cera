#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
#check world debug
import sys
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
import pygame
from pygame.locals import *
import cv2

video_size = 700, 500
velocity_publisher = rospy.Publisher('world_observation/cmd_vel', Twist, queue_size=10)

def key_action():
    vel_msg = Twist()
    keys=pygame.key.get_pressed()
    if keys[K_LEFT]:
        vel_msg.linear.x = -1
    if keys[K_UP]:
        vel_msg.linear.y = 1
    if keys[K_RIGHT]:
        vel_msg.linear.x = 1
    if keys[K_DOWN]:
        vel_msg.linear.y = -1
    return vel_msg


def callback(ros_data):
    np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    screen = pygame.display.set_mode(video_size)
    surf = pygame.surfarray.make_surface(image_np)
    screen.blit(surf, (0, 0))
    pygame.display.update()
    vel_msg = key_action()
    velocity_publisher.publish(vel_msg)

def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('world_observation_client', anonymous=True)
    subscriber = rospy.Subscriber('world_observation/image_raw', CompressedImage, callback)
    try:
        screen = pygame.display.set_mode(video_size)
        vel_msg = key_action()
        velocity_publisher.publish(vel_msg)
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Gym Image Viewer module")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
