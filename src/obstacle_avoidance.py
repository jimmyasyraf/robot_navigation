#!/usr/bin/env python
import rospy
from robot_navigation.msg import Distance
from robot_navigation.msg import Direction

straight = 1
right = 2
left = 3
stop = 4

pub = rospy.Publisher('direction', Direction, queue_size=10)

def decide_direction(distance):
    distance1 = distance.distance1
    distance2 = distance.distance2
    distance3 = distance.distance3
    direction = straight

    if (distance1 < 30):
        if (distance2 > distance3):
            direction = right
        else:
            direction = left
    else:
        if (distance2 < distance3) and (distance2 < 30):
            direction = left
        elif (distance3 < distance2) and (distance3 < 30):
            direction = right
        else:
            direction = straight

    return direction

def callback(data):
    direction = decide_direction(data)
    rospy.loginfo(direction)
    msg = Direction()
    msg.direction = direction
    pub.publish(msg)

def exit_gracefully():
    msg = Direction()
    msg.direction = stop
    pub.publish(msg)

def obstacle_avoidance():
    rospy.init_node('obstacle_avoidance', anonymous=True)
    rospy.Subscriber("distance", Distance, callback)
    rospy.on_shutdown(exit_gracefully)
    rospy.spin()

if __name__ == '__main__':
    obstacle_avoidance()
