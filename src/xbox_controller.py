#!/usr/bin/env python
import rospy
import curses
#---------
from sensor_msgs.msg import Joy
from assignment2_team3.msg import Int16Stamped
#---------

#---------
drive_throttle = rospy.Publisher("drive_throttle", Int16Stamped, queue_size = 1)
drive_steering = rospy.Publisher("drive_steering", Int16Stamped, queue_size = 1)

#---------

stdscr = curses.initscr()

def joy_callback(data):
    #---------
    # YOUR CODE (as specified in the lecture note)
    throttle = data.axes[1] * 20
    steering = data.axes[2] * 100
    emergency_stop = data.buttons[3]
    if emergency_stop:
        throttle = 0
        steering = 0
    # Use the following code to print out messages on the screen
    stdscr.refresh()
    stdscr.addstr(1, 25, 'Xbox Controller       ')
    stdscr.addstr(2, 25, 'Throttle: %.2f  ' % throttle)
    stdscr.addstr(3, 25, 'Steering: %.2f  ' % steering)
    if emergency_stop:    
        stdscr.addstr(4, 25, 'Emergency Stop')
    else:
        stdscr.addstr(4, 25, 'Not Emergency Stop')
    drive_throttle.publish(Int16Stamped(throttle, rospy.Time().now()))
    drive_steering.publish(Int16Stamped(steering, rospy.Time().now()))    
#---------

def on_shutdown():
    curses.endwin()

if __name__ == '__main__':
    
    #---------
    rospy.init_node("xbox_controller", anonymous = False)
    rospy.Subscriber("joy", Joy, joy_callback)
    #YOUR CODE (initialize node & declare subscriber)
    #---------
    
    rospy.on_shutdown(on_shutdown)
    stdscr.refresh()
    rospy.spin()
