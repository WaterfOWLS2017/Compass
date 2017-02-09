import rospy
from compass.msg import heading
from os import environ
from HMC5883L import HMC5883L
from math import atan2, degrees

# attempt to get publishing rate from environment
rate = environ.get('COMPASS_PUBRATE')
if rate == None:
    rate = environ.get('ROS_PUBRATE')
if rate == None:
    rate = 10

# Initialize and configure ROS node and publisher
rospy.init_node('compass_node')
compass_ch = rospy.Publisher('/compass', heading, queue_size=2)
rate = rospy.Rate(rate)

# Initialize and configure compass
compass = HMC5883L(1)

while not rospy.is_shutdown():
    (X, Y, Z) = compass.read()
    # perform calculation to obtain heading in degrees off north
    d = degrees(atan2(Y, X))
    if d < 0:
        d = 360 + d
    # publish degrees
    compass_ch.publish(heading=d)
    # sleep and repeat
    rate.sleep()
