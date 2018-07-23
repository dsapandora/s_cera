#!/usr/bin/env python
# license removed for brevity
import rospy
import retro
import cv2
import numpy as np
import imutils
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
#check world debug

env = retro.make(game='SonicTheHedgehog-Genesis', state='GreenHillZone.Act1')
image_pub = rospy.Publisher('world_observation/image_raw', CompressedImage, queue_size=10)

def key_action(vel_msg):
    #["B", "A", "MODE", "START", "UP", "DOWN", "LEFT", "RIGHT", "C", "Y", "X", "Z"]
    keys = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    if vel_msg.linear.x == -1:
        keys[6] = 1
    if vel_msg.linear.y == 1:
        keys[0] = 1
    if vel_msg.linear.x == 1:
        keys[7] = 1
    if vel_msg.linear.y == -1:
        keys[5] = 1
    return keys

def pub_image(env):
    #GYM RENDER AS IMAGE
    img = env.render(mode='rgb_array')
    # ROTATE THE IMAGE THE MATRIX IS 90 grates and mirror
    img = np.flipud(np.rot90(img))
    image_np = imutils.resize(img, width=500)
    # Publish new image
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    compressed_images = cv2.imencode('.jpg', image_np)
    msg.data = np.array(compressed_images[1]).tostring()
    image_pub.publish(msg)

def open_world(vel_msg):
    pub_image(env)
    action = key_action(vel_msg)
    obs, rew, done, info = env.step(action)

if __name__ == '__main__':
    rospy.init_node('world_observation_server', anonymous=True)
    try:
        rospy.Subscriber("world_observation/cmd_vel", Twist, open_world)
        env.reset()
        pub_image(env)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
