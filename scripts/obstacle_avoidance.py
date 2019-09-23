#!/usr/bin/env python
import rospy
from ros_tutorial.msg import Distance
from ros_tutorial.msg import Direction

straight = 1
right = 2
left = 3

pub = rospy.Publisher('direction', Direction)

def decide_direction(distance):
    distance1 = distance.distance1
    distance2 = distance.distance2
    distance3 = distance.distance3
    direction = straight

    if (distance1 > distance2) and (distance1 > distance3):
        direction = straight
    elif (distance2 > distance1) and (distance2 > distance3):
        direction = right
    elif (distance3 > distance1) and (distance3 > distance2):
        direction = left

    return direction

def callback(data):
    direction = decide_direction(data)
    #rospy.loginfo("Front: {}, Right: {}, Left: {}".format(data.distance1, data.distance2, data.distance3))
    rospy.loginfo(direction)
    msg = Direction()
    msg.direction = direction
    pub.publish(msg)

def obstacle_avoidance():
    rospy.init_node('obstacle_avoidance', anonymous=True)
    rospy.Subscriber("distance", Distance, callback)

    rospy.spin()

if __name__ == '__main__':
    obstacle_avoidance()
