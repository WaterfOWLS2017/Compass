import rospy
from compass.msg import compass_heading
from os import environ
from HMC25583L import HMC5883L

# attempt to get publishing rate from environment
rate = os.environ.get('COMPASS_PUBRATE')
if rate == None:
    rate = os.environ.get('ROS_PUBRATE')
if rate == None:
    rate = 10

# Initialize and configure ROS node and publisher
rospy.init_node('compass_node')
compass_ch = rospy.Publisher('/compass', compass_heading, queue_size=2)
rate = rospy.Rate(rate)

# Initialize and configure compass
compass = HMC5883L(1)

while not rospy.is_shutdown():
    (X, Y, Z) = compass.read()
    # perform calculation to obtain heading in degrees off north
    d = math.degrees(math.atan2(Y, X))    
    # publish degrees
    compass_ch.publish(degrees=d)
    # sleep and repeat
    rate.sleep()
