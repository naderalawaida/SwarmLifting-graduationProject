#!/usr/bin/env python
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge 
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import String
#from sensor_msgs.msg import CompressedImage
import time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from sensor_msgs.msg import JointState
#from apriltag_ros.msg import AprilTagDetectionArray
from moveit_commander.conversions import pose_to_list
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
#from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
import rospy
#from nav_msgs.msg import Odometry
from math import radians
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
#from moveit_msgs.msg import Grasp, PlaceLocation


PI = 3.1415926535897


 # MOVEIT INIT

group_arm = "arm"
group_name = "gripper"

    
#moveit_commander.roscpp_initialize(sys.argv)
robot0 = moveit_commander.RobotCommander("/tb3_0/robot_description","tb3_0")
#scene = moveit_commander.PlanningSceneInterface("/tb3_0/robot_description","tb3_0")
group0 = moveit_commander.MoveGroupCommander(group_name,"/tb3_0/robot_description","tb3_0")
grouparm0 = moveit_commander.MoveGroupCommander(group_arm,"/tb3_0/robot_description","tb3_0")



robot1 = moveit_commander.RobotCommander("/tb3_1/robot_description","tb3_1")
group1 = moveit_commander.MoveGroupCommander(group_name,"/tb3_1/robot_description","tb3_1")
grouparm1 = moveit_commander.MoveGroupCommander(group_arm,"/tb3_1/robot_description","tb3_1")

'''
robot2 = moveit_commander.RobotCommander("/tb3_2/robot_description","tb3_2")
group2 = moveit_commander.MoveGroupCommander(group_name,"/tb3_2/robot_description","tb3_2")
grouparm2 = moveit_commander.MoveGroupCommander(group_arm,"/tb3_2/robot_description","tb3_2")
'''



'''
group1.set_planning_time(50) 
grouparm1.set_planning_time(50)

group2.set_planning_time(50) 
grouparm2.set_planning_time(50)
'''



def pick0(j = pi/2):
    joint_goal = grouparm0.get_current_joint_values()
    joint_goal[2] = -j/2
    joint_goal[1] = 0/2
    joint_goal[3] = 0/2
    grouparm0.go(joints=joint_goal, wait=True)
    grouparm0.stop()
    grouparm0.clear_pose_targets()    


def pick_up0(j = pi/2):
    joint_goal = grouparm0.get_current_joint_values()

    joint_goal[1] = -j/2 - 0.30
    joint_goal[2] = j/2 + 0.30
    joint_goal[3] = j/2 - 0.30
    grouparm0.go(joints=joint_goal, wait=True)
    grouparm0.stop()
    grouparm0.clear_pose_targets()

def grab_up0(j = pi/2):
    joint_goal = grouparm0.get_current_joint_values()
        
    joint_goal[1] = j/2 + 0.60
    joint_goal[2] = -j/2
    joint_goal[3] = -j/2 + 0.80
    grouparm0.go(joints=joint_goal, wait=True)
    grouparm0.stop()
    grouparm0.clear_pose_targets()
    print "Grab"   


def open_grip0():
    print "Open Grip"
    jv = group0.get_current_joint_values()  
    jv = [0.019,0.019]
    print(jv)
    group0.go(joints=jv, wait=True)
    group0.stop()


def close_grip0():
    print "Close Grip"
    jv = group0.get_current_joint_values() 
    jv = [-0.010,-0.010]
    print(jv)
    group0.go(joints=jv, wait=True)
    group0.stop()



def pick1(j = pi/2):
    joint_goal = grouparm1.get_current_joint_values()
    joint_goal[2] = -j/2
    joint_goal[1] = 0/2
    joint_goal[3] = 0/2
    grouparm1.go(joints=joint_goal, wait=True)
    grouparm1.stop()
    grouparm1.clear_pose_targets()    


def pick_up1(j = pi/2):
    joint_goal = grouparm1.get_current_joint_values()

    joint_goal[1] = -j/2 - 0.30
    joint_goal[2] = j/2 + 0.30
    joint_goal[3] = j/2 - 0.30
    grouparm1.go(joints=joint_goal, wait=True)
    grouparm1.stop()
    grouparm1.clear_pose_targets()

def grab_up1(j = pi/2):
    joint_goal = grouparm1.get_current_joint_values()
        
    joint_goal[1] = j/2 + 0.60
    joint_goal[2] = -j/2
    joint_goal[3] = -j/2 + 0.80
    grouparm1.go(joints=joint_goal, wait=True)
    grouparm1.stop()
    grouparm1.clear_pose_targets()
    print "Grab"   


def open_grip1():
    print "Open Grip"
    jv = group1.get_current_joint_values()  
    jv = [0.019,0.019]
    print(jv)
    group1.go(joints=jv, wait=True)
    group1.stop()


def close_grip1():
    print "Close Grip"
    jv = group1.get_current_joint_values() 
    jv = [-0.010,-0.010]
    print(jv)
    group1.go(joints=jv, wait=True)
    group1.stop()


def pick2(j = pi/2):
    joint_goal = grouparm2.get_current_joint_values()
    joint_goal[2] = -j/2
    joint_goal[1] = 0/2
    joint_goal[3] = 0/2
    grouparm2.go(joints=joint_goal, wait=True)
    grouparm2.stop()
    grouparm2.clear_pose_targets()    


def pick_up2(j = pi/2):
    joint_goal = grouparm2.get_current_joint_values()

    joint_goal[1] = -j/2 - 0.30
    joint_goal[2] = j/2 + 0.30
    joint_goal[3] = j/2 - 0.30
    grouparm2.go(joints=joint_goal, wait=True)
    grouparm2.stop()
    grouparm2.clear_pose_targets()

def grab_up2(j = pi/2):
    joint_goal = grouparm2.get_current_joint_values()
        
    joint_goal[1] = j/2 + 0.60
    joint_goal[2] = -j/2
    joint_goal[3] = -j/2 + 0.80
    grouparm2.go(joints=joint_goal, wait=True)
    grouparm2.stop()
    grouparm2.clear_pose_targets()
    print "Grab"   


def open_grip2():
    print "Open Grip"
    jv = group2.get_current_joint_values()  
    jv = [0.019,0.019]
    print(jv)
    group2.go(joints=jv, wait=True)
    group2.stop()


def close_grip2():
    print "Close Grip"
    jv = group2.get_current_joint_values() 
    jv = [-0.010,-0.010]
    print(jv)
    group2.go(joints=jv, wait=True)
    group2.stop()



def goto(pos, quat):

            # Send a goal
            # DEFINE PARAMETERS
    '''        
    group.set_goal_position_tolerance(0.1)          # GOAL TOLERANCE
    group.set_planning_time(5)                      # TIME TO PLANNING
    grouparm.set_goal_position_tolerance(0.1)       # GOAL TOLERANCE
    grouparm.set_planning_time(5)
    '''

    goal_sent = False

        # What to do if shut down (e.g. Ctrl-C or failure)
    #rospy.on_shutdown(self.shutdown)
        
        # Tell the action client that we want to spin a thread by default
    move_base = actionlib.SimpleActionClient("tb3_0/move_base", MoveBaseAction)
    rospy.loginfo("Wait for the action server to come up")

        # Allow up to 5 seconds for the action server to come up
    move_base.wait_for_server(rospy.Duration(5))


    #function starts here
    goal_sent = True
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

    # Start moving
    move_base.send_goal(goal)

    # Allow TurtleBot up to 60 seconds to complete task
    success = move_base.wait_for_result(rospy.Duration(60)) 

    state = move_base.get_state()
    result = False

    if success and state == GoalStatus.SUCCEEDED:
            # We made it!
        result = True
    else:
        move_base.cancel_goal()
        print('Goal is cancelled')

    goal_sent = False
    return result


def go_to_station():
    position = {'x': 3.4, 'y' : 2.7, 'z' : 0.}
    quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
    goto(position, quaternion)





def drive_callback0(data, args):
    #status = True


    x = args[0]
    y = args[1]


    global vel, pub_vel
    ball_x  = data.x
    ball_y  = data.y
    width   = data.z
              
             # publish to /cmd_vel topic the angular-z velocity change
    pub_vel = rospy.Publisher('tb3_0/cmd_vel', Twist, queue_size=5)
              # Create Twist() instance
    vel = Twist()
                # Determine center-x, normalized deviation from center
    mid_x   = int(width/2)
    delta_x = ball_x - mid_x
    norm_x  = delta_x/width
    mid_y   = int(width/2)
    delta_y = ball_y - mid_y
    norm_y  = delta_y/width


    angle = 90
    clockwise = 1

              #Converting from angles to radians
              #angular_speed = speed*2*PI/360
    angular_speed = 30*2*PI/360
    relative_angle = angle*2*PI/360


              #We wont use linear components
    vel.linear.x=0
    vel.linear.y=0
    vel.linear.z=0
    vel.angular.x = 0
    vel.angular.y = 0


            # Checking if our movement is CW or CCW
            
    vel.angular.z = 0.2
                    

    r = rospy.Rate(80);

    iterate = -100

    

    r1 = rospy.Rate(80);

    temp_count = 0
    final_temp = 0

    print('ball_x first value: ', ball_x)
    
    if iterate < 0:

        print('ballx initial value ', ball_x)
        while (not rospy.is_shutdown() and final_temp < 1):
            #print('Final Search')
            #print('ball_x first value: ', ball_x)
            
                    # as long as you haven't ctrl + c keeping doing...
            
                
                    #go back at 0.1 m/s for 2 seconds
            if ball_x < 0 and ball_y < 0:
                #print('Searching...')
                while (not rospy.is_shutdown() and temp_count < 1):
                         # publish the velocity
                    pub_vel.publish(vel)
                                # wait for 0.1 seconds (10 HZ) and publish again
                    temp_count = temp_count + 1
                    
                    r.sleep()
                            
                        #make sure TurtleBot stops by sending a default Twist()
                pub_vel.publish(Twist())

                        #print('Ball_x', ball_x)
                        
            else:
                a = func0(add=-1)
                print('a :', a)
                #print('Box Found!')
                if norm_x > 0.02:
                    print ("delX: {:.3f}. Turn right".format(norm_x))
                    vel.angular.z = -0.2
                    vel.linear.x=0.1
                    print('Y: ', norm_y)

                elif norm_x < -0.02:
                    print ("delX: {:.3f}. Turn left".format(norm_x))
                    vel.angular.z = 0.2
                    vel.linear.x=0.1
                    print('Y: ', norm_y)
                    
                            
                else:
                    if norm_y < 0.105:
                        print ("delX: {:.3f}. Stay in center".format(norm_x))
                        vel.angular.z = 0
                        vel.linear.x=0.1
                        print('Y: ', norm_y)
                    else:
                        a = func0(add=-1)
                        print('Stopped')
                        vel.angular.z=0
                        vel.linear.x=0
                        pub_vel.publish(Twist())
                        open_grip0()
                        pick_up0()
                        grab_up0()
                        close_grip0() 
                        pick0()
                        print('End main loop')
                        
                        
                #rospy.signal_shutdown("Shutdown")        
                #continue        
                       
                # publish vel on the publisher
            print('End of last main loop')
            pub_vel.publish(vel)
            final_temp = final_temp + 1
            a = func0(add = 1)
            print('a :', a)
            r1.sleep()

            if a > 1000:
                print('Search Mission Aborted')
                pub_vel.publish(Twist())
                rospy.spin()

                                

        #make sure TurtleBot stops by sending a default Twist()
        pub_vel.publish(Twist())
        print('Final Ball_x', ball_x)




def drive_callback1(data, args):
    #status = True


    x = args[0]
    y = args[1]


    global vel, pub_vel
    ball_x  = data.x
    ball_y  = data.y
    width   = data.z
              
             # publish to /cmd_vel topic the angular-z velocity change
    pub_vel = rospy.Publisher('tb3_1/cmd_vel', Twist, queue_size=5)
              # Create Twist() instance
    vel = Twist()
                # Determine center-x, normalized deviation from center
    mid_x   = int(width/2)
    delta_x = ball_x - mid_x
    norm_x  = delta_x/width
    mid_y   = int(width/2)
    delta_y = ball_y - mid_y
    norm_y  = delta_y/width


    angle = 90
    clockwise = 1

              #Converting from angles to radians
              #angular_speed = speed*2*PI/360
    angular_speed = 30*2*PI/360
    relative_angle = angle*2*PI/360


              #We wont use linear components
    vel.linear.x=0
    vel.linear.y=0
    vel.linear.z=0
    vel.angular.x = 0
    vel.angular.y = 0


            # Checking if our movement is CW or CCW
            
    vel.angular.z = 0.2
                    

    r = rospy.Rate(80);

    iterate = -100

    

    r1 = rospy.Rate(80);

    temp_count = 0
    final_temp = 0

    print('ball_x first value: ', ball_x)
    
    if iterate < 0:

        print('ballx initial value ', ball_x)
        while (not rospy.is_shutdown() and final_temp < 1):
            #print('Final Search')
            #print('ball_x first value: ', ball_x)
            
                    # as long as you haven't ctrl + c keeping doing...
            
                
                    #go back at 0.1 m/s for 2 seconds
            if ball_x < 0 and ball_y < 0:
                #print('Searching...')
                while (not rospy.is_shutdown() and temp_count < 1):
                         # publish the velocity
                    pub_vel.publish(vel)
                                # wait for 0.1 seconds (10 HZ) and publish again
                    temp_count = temp_count + 1
                    
                    r.sleep()
                            
                        #make sure TurtleBot stops by sending a default Twist()
                pub_vel.publish(Twist())

                        #print('Ball_x', ball_x)
                        
            else:
                a = func1(add=-1)
                print('a :', a)
                #print('Box Found!')
                if norm_x > 0.02:
                    print ("delX: {:.3f}. Turn right".format(norm_x))
                    vel.angular.z = -0.2
                    vel.linear.x=0.1
                    print('Y= ', norm_y)  

                elif norm_x < -0.02:
                    print ("delX: {:.3f}. Turn left".format(norm_x))
                    vel.angular.z = 0.2
                    vel.linear.x=0.1
                    print('Y= ', norm_y)
                            
                else:
                    if norm_y < 0.105:
                        print ("delX: {:.3f}. Stay in center".format(norm_x))
                        vel.angular.z = 0
                        vel.linear.x=0.1
                        print('Y= ', norm_y)
                    else:
                        a = func1(add=-1)
                        print('Stopped')
                        vel.angular.z=0
                        vel.linear.x=0
                        pub_vel.publish(Twist())
                        open_grip1()
                        pick_up1()
                        grab_up1()
                        close_grip1() 
                        pick1()
                        print('End main loop')
                        
                        
                #rospy.signal_shutdown("Shutdown")        
                #continue        
                       
                # publish vel on the publisher
            print('End of last main loop')
            pub_vel.publish(vel)
            final_temp = final_temp + 1
            a = func1(add = 1)
            r1.sleep()

            if a > 1000:
                print('Search Mission Aborted')

                                

        #make sure TurtleBot stops by sending a default Twist()
        pub_vel.publish(Twist())
        print('Final Ball_x', ball_x)



def drive_callback2(data, args):
    #status = True


    x = args[0]
    y = args[1]


    global vel, pub_vel
    ball_x  = data.x
    ball_y  = data.y
    width   = data.z
              
             # publish to /cmd_vel topic the angular-z velocity change
    pub_vel = rospy.Publisher('tb3_2/cmd_vel', Twist, queue_size=5)
              # Create Twist() instance
    vel = Twist()
                # Determine center-x, normalized deviation from center
    mid_x   = int(width/2)
    delta_x = ball_x - mid_x
    norm_x  = delta_x/width
    mid_y   = int(width/2)
    delta_y = ball_y - mid_y
    norm_y  = delta_y/width


    angle = 90
    clockwise = 1

              #Converting from angles to radians
              #angular_speed = speed*2*PI/360
    angular_speed = 30*2*PI/360
    relative_angle = angle*2*PI/360


              #We wont use linear components
    vel.linear.x=0
    vel.linear.y=0
    vel.linear.z=0
    vel.angular.x = 0
    vel.angular.y = 0


            # Checking if our movement is CW or CCW
            
    vel.angular.z = 0.2
                    

    r = rospy.Rate(80);

    iterate = -100

    

    r1 = rospy.Rate(80);

    temp_count = 0
    final_temp = 0

    print('ball_x first value: ', ball_x)
    
    if iterate < 0:

        print('ballx initial value ', ball_x)
        while (not rospy.is_shutdown() and final_temp < 1):
            #print('Final Search')
            #print('ball_x first value: ', ball_x)
            
                    # as long as you haven't ctrl + c keeping doing...
            
                
                    #go back at 0.1 m/s for 2 seconds
            if ball_x < 0 and ball_y < 0:
                #print('Searching...')
                while (not rospy.is_shutdown() and temp_count < 1):
                         # publish the velocity
                    pub_vel.publish(vel)
                                # wait for 0.1 seconds (10 HZ) and publish again
                    temp_count = temp_count + 1
                    
                    r.sleep()
                            
                        #make sure TurtleBot stops by sending a default Twist()
                pub_vel.publish(Twist())

                        #print('Ball_x', ball_x)
                        
            else:
                a = func2(add=-1)
                print('a :', a)
                #print('Box Found!')
                if norm_x > 0.02:
                    print ("delX: {:.3f}. Turn right".format(norm_x))
                    vel.angular.z = -0.2
                    vel.linear.x=0.1
                    print('Y= ', norm_y)  

                elif norm_x < -0.02:
                    print ("delX: {:.3f}. Turn left".format(norm_x))
                    vel.angular.z = 0.2
                    vel.linear.x=0.1
                    print('Y= ', norm_y)
                            
                else:
                    if norm_y < 0.105:
                        print ("delX: {:.3f}. Stay in center".format(norm_x))
                        vel.angular.z = 0
                        vel.linear.x=0.1
                        print('Y= ', norm_y)
                    else:
                        a = func2(add=-1)
                        print('Stopped')
                        vel.angular.z=0
                        vel.linear.x=0
                        pub_vel.publish(Twist())
                        open_grip2()
                        pick_up2()
                        grab_up2()
                        close_grip2() 
                        pick2()
                        print('End main loop')
                        
                        
                #rospy.signal_shutdown("Shutdown")        
                #continue        
                       
                # publish vel on the publisher
            print('End of last main loop')
            pub_vel.publish(vel)
            final_temp = final_temp + 1
            a = func2(add = 1)
            r1.sleep()

            if a > 1000:
                print('Search Mission Aborted')

                                

        #make sure TurtleBot stops by sending a default Twist()
        pub_vel.publish(Twist())
        print('Final Ball_x', ball_x)




def func0(_static={'counter': 0},add=1):
        _static['counter'] += add
            
        return _static['counter']



def func1(_static={'counter': 0},add=1):
        _static['counter'] += add
            
        return _static['counter']



def func2(_static={'counter': 0},add=1):
        _static['counter'] += add
            
        return _static['counter']





if __name__ == "__main__":

  rospy.init_node("testing")


  img_sub0 = rospy.Subscriber("/coordinates00",Point, drive_callback0, (1.0,0.0))

  #img_sub1 = rospy.Subscriber("/coordinates01",Point, drive_callback1, (-2.0,-2.0))

  #img_sub2 = rospy.Subscriber("/coordinates02",Point, drive_callback2, (-2.0,2.0))


  #go_to_station()

  rospy.spin()