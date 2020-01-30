#!/usr/bin/env python

##############################
# Robotics                   #
# Homework 2                 #
# Student Name: Sabrina Cara #
# Student Number: 150160914  #
##############################


import rospy
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Twist
import tf
from tf import transformations
import math

waypoint=None
previousRotDifference = 0

#waypoint callback
def waypoint_callback(msg): #  callback

    #***************************************
    #***          Obtain current destination
    #***************************************

    #save waypoint data for printing out in main loop
    global waypoint
    waypoint=msg;


if __name__ == '__main__':

    #setup ROS node, subscribe waypoint_cb to the topic /waypoint_cmd & publish motor commands
    rospy.init_node("crazy_driver_456")
    waypoint_subscriber = rospy.Subscriber("/waypoint_cmd", Transform, waypoint_callback) # <--- set up callback
    motor_command_publisher = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=100)
    #you could in principle also subscribe to the laser scan as is done in assignment 1.

    #setup transform cache manager
    listener = tf.TransformListener()

    #start a loop; one loop per second
    delay = rospy.Rate(1.0); # perhaps this could be faster for a controller?
    while not rospy.is_shutdown():


        #***************************************
        #***          Obtain current robot pose
        #***************************************
        
        try:
            #grab the latest available transform from the odometry frame (robot's original location - usually the same as the map unless the odometry becomes inaccurate) to the robot's base.
            
            (translation,orientation) = listener.lookupTransform("/odom", "/base_footprint", rospy.Time(0));
        except  (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("EXCEPTION:",e)
            #if something goes wrong with this just go to bed for a second or so and wake up hopefully refreshed.
            delay.sleep()
            continue
        

        #***************************************
        #***          Print current robot pose
        #***************************************

        #Print out the x,y coordinates of the transform
        print("Robot is believed to be at (x,y): (",translation[0],",",translation[1],")")

        #Convert the quaternion-based orientation of the latest message to Euler representation in order to get z axis rotation
        r_xorient, r_yorient, r_zorient = transformations.euler_from_matrix(transformations.quaternion_matrix(orientation))
        robot_theta = r_zorient  # only need the z axis
        print("Robot is believed to have orientation (theta): (",robot_theta,")\n")

        #***************************************
        #***          Print current destination
        #***************************************

        # the waypoint variable is filled in in the waypoint_callback function above, which comes from incoming messages
        # subscribed to in the .Subscriber call above.

        #Print out the x,y coordinates of the latest message
        print("Current waypoint (x,y): (",waypoint.translation.x,",",waypoint.translation.y,")")

        #Convert the quaternion-based orientation of the latest message to angle-axis in order to get the z rotation & print it.
        waypointrotq = [waypoint.rotation.x,waypoint.rotation.y,waypoint.rotation.z,waypoint.rotation.w]
        w_xorient, w_yorient, w_zorient = transformations.euler_from_matrix(transformations.quaternion_matrix(waypointrotq))
        waypoint_theta=w_zorient # only need the z axis
        print("Current waypoint (theta): (",waypoint_theta,")\n")

        #************************************************************************************************
        #***          DRIVE THE ROBOT HERE (same as with assignment 1 
        #**           - except you are driving towards a goal not away from an obstacle)
        #************************************************************************************************
        #
        # This homework consists in finding the waypoint's coordinates according to the robot
        # How we compute this is by using the efficiency of the rotational matrix and 
        # real coordinates of both robot and waypoint. 
        # More specifically robot can obtain the waypoints coordinates if it divides the
        # difference between it's real coordinates and waypoints coordinates by the rotational matrix
        #
        #************************************************************************************************

        #for containing the motor commands to send to the robot
        motor_command=Twist()

    	#get sin and cos values of theta angle using math library
        
        sine = math.sin(robot_theta)
	cosine = math.cos(robot_theta)
	
	#In python x and y coordinates are taken using translation
	
	XCoordinate = waypoint.translation.x - translation[0]
	YCoordinate = waypoint.translation.y - translation[1]

	#calculate new x and y values using waypoint and current coordinates
	x = cosine * (XCoordinate) + sine * (YCoordinate)
	y = cosine * (YCoordinate) - sine * (XCoordinate)
		
	#calculate distance between robot and waypoint
	Distance = math.hypot(x, y)
	#calculate rotational difference between robot's and waypoint's direction
	RotDiff = math.atan2(y, x)
	
        # find cosine of Rotational Difference and difference between new and previous value
        # this is done to achieve smoothness when robot takes a turn
        absolute_value_cos = math.cos(abs(RotDiff))
	delta = RotDiff - previousRotDiff
	
	#constants we use when assigning new speed values to achieve best actions
	a = 0.8
	b = 0.8
	c = 0.5
	
	#assign new values of linear and angular speed
	motor_command.linear.x = a * Distance * absolute_value_cos
	motor_command.angular.z = b * RotDiff + c * delta

	#save new value of Rotational difference 
	previousRotDiff = RotDiff

	#publish all motor commands
	motor_command_publisher.publish(motor_command)


        #######################################################################

        delay.sleep()
        # we don't need to call spinOnce like in roscpp as callbacks happen in different threads
    
    
    print("ROS shutdown now I will also go to sleep. I hope I didn't crash. Night night.")
