#!/usr/bin/env python
# license removed for brevity
import rospy
import retro
import cv2
import numpy as np
import imutils
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
#check world debug


env = retro.make(game='SonicTheHedgehog-Genesis', state='GreenHillZone.Act1')

def open_world():
    rospy.init_node('domain_server', anonymous=True)
    image_pub = rospy.Publisher('world_observation/image_raw', CompressedImage, queue_size=10)
    env.reset()
    action = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
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
        obs, rew, done, info = env.step(action)

if __name__ == '__main__':
    try:
        open_world()
    except rospy.ROSInterruptException:
        pass
