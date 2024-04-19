#!/usr/bin/env python3
import rospy
import math
import tf2_ros
from tf.transformations import *
from geometry_msgs.msg import Quaternion
import tf2_geometry_msgs
from ur5e_control.msg import Plan
from geometry_msgs.msg import Twist
from robot_vision_lectures.msg import SphereParams
from geometry_msgs.msg import PointStamped
from std_msgs.msg import UInt8

points_in_camera = []
first_point = []
counter = 0

def xyzCallback(data):
    global points_in_camera
    x = data.xc
    y = data.yc
    z = data.zc
    points_in_camera = [x, y, z]

def tpCallback(data):
    global first_point
    global counter
    while counter == 0:
        x = data.linear.x
        y = data.linear.y
        z = data.linear.z
        xa = data.angular.x
        ya = data.angular.y
        za = data.angular.z
        first_point = [x, y, z, xa, ya, za]
        counter += 1

def main():
    rospy.init_node('simple_planner', anonymous=True)
    plan_pub = rospy.Publisher('/plan', Plan, queue_size=10)
   
    rospy.Subscriber('sphere_params', SphereParams, xyzCallback)
    rospy.Subscriber('/ur5e/toolpose', Twist, tpCallback)
   
    loop_rate = rospy.Rate(10)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    while not rospy.is_shutdown():
        if len(points_in_camera) != 0:
            
            plan = Plan()
            
            plan_point1 = Twist()
            point_mode1 = UInt8()

            plan_point1.linear.x = first_point[0]
            plan_point1.linear.y = first_point[1] 
            plan_point1.linear.z = first_point[2]
            plan_point1.angular.x = 3.09069
            plan_point1.angular.y = 0.06071
            plan_point1.angular.z = 1.56788
            point_mode1.data = 0
            
            plan.points.append(plan_point1)
            plan.modes.append(point_mode1)
            
			#--------------------------------------------------------------------
           
            pt_in_cam = PointStamped()
            pt_in_cam.header.frame_id = 'camera_color_optical_frame'
            pt_in_cam.header.stamp = rospy.get_rostime()
            pt_in_cam.point.x = points_in_camera[0]
            pt_in_cam.point.y = points_in_camera[1]
            pt_in_cam.point.z = points_in_camera[2]
            points_in_base = tfBuffer.transform(pt_in_cam, 'base', rospy.Duration(1.0))
   			
   			#--------------------------------------------------------------------
           
            plan_point2 = Twist()
            point_mode2 = UInt8()
            
            plan_point2.linear.x = points_in_base.point.x + 0.1
            plan_point2.linear.y = points_in_base.point.y
            plan_point2.linear.z = points_in_base.point.z
            plan_point2.angular.x = 3.09069
            plan_point2.angular.y = 0.06071
            plan_point2.angular.z = 1.56788
            point_mode2.data = 1
            
            plan.points.append(plan_point2)
            plan.modes.append(point_mode2)
            
            #---------------------------------------------------------------------
            
            plan_point3 = Twist()
            point_mode3 = UInt8()
            
            plan_point3.linear.x = points_in_base.point.x + 0.06
            plan_point3.linear.y = points_in_base.point.y
            plan_point3.linear.z = points_in_base.point.z
            plan_point3.angular.x = 3.09069
            plan_point3.angular.y = 0.06071
            plan_point3.angular.z = 1.56788
            point_mode3.data = 2
            
            plan.points.append(plan_point2)
            plan.modes.append(point_mode3)
            
            #--------------------------------------------------------------------
           
            plan_point4 = Twist()
            point_mode4 = UInt8()
            
            plan_point4.linear.x = -0.5
            plan_point4.linear.y = .3
            plan_point4.linear.z = 0.4
            plan_point4.angular.x = 3.09069
            plan_point4.angular.y = 0.06071
            plan_point4.angular.z = 1.56788
            point_mode4.data = 2
            
            plan.points.append(plan_point4)
            plan.modes.append(point_mode4)
            
            #---------------------------------------------------------------------
   
            plan_point5 = Twist()
            point_mode5 = UInt8()
            
            plan_point5.linear.x = -0.5
            plan_point5.linear.y = .3
            plan_point5.linear.z = 0.2
            plan_point5.angular.x = 3.09069
            plan_point5.angular.y = 0.06071
            plan_point5.angular.z = 1.56788
            point_mode5.data = 1
            
            plan.points.append(plan_point5)
            plan.modes.append(point_mode5)
            
            #---------------------------------------------------------------------

            plan_pub.publish(plan)
            loop_rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
