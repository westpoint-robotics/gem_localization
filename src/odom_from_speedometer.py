#!/usr/bin/env python

""" Code written by: Dominic Larkin 15OCT2019
    
    Subscribe to sppedometer in m/s and compute odometry.    
    
"""
import rospy 
from std_msgs.msg import Float64
from pacmod_msgs.msg import MotorRpt1

from nav_msgs.msg import Odometry

import math
import tf

class VirtualOdom():

    def __init__(self):     
        rospy.init_node('virtual_odom', anonymous=True)        
        self.br = tf.TransformBroadcaster()
        self.odom_pub = rospy.Publisher("speed_odom", Odometry, queue_size=1 ) 
        rospy.Subscriber('/pacmod/as_tx/vehicle_speed', Float64, self.update_veh_speed)
        rospy.Subscriber('/pacmod/parsed_tx/steer_rpt_detail_1', MotorRpt1, self.update_steering)
        self.last_time = rospy.Time.now() 
        self.veh_speed = 0.0
        self.veh_steer = 0.0 
        self.wheelbase = 175.3    
        self.yaw = 0.0
        self.x = 0.0
        self.y = 0.0
    
    def update_veh_speed(self, data):
        self.veh_speed = data.data
        
    def update_steering(self, data):
        self.veh_steer = data.position - 1.5708
        
    def calculate_odom(self): 
        pub_tf=rospy.get_param('~pub_tf',False)
        odom_msg = Odometry()
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        angular_velocity = self.veh_speed * math.tan(self.veh_steer) / self.wheelbase
        x_dot = self.veh_speed * math.cos(self.yaw)
        y_dot = self.veh_speed * math.sin(self.yaw)
        self.x += x_dot * dt
        self.y += y_dot * dt
        self.yaw += angular_velocity * dt
        
       # Build the odometry message to publish
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.yaw/2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.yaw/2.0)
        odom_msg.pose.covariance[0] = 0.2  # x
        odom_msg.pose.covariance[7] = 0.2  # y
        odom_msg.pose.covariance[35] = 0.4 # yaw
        
        odom_msg.twist.twist.linear.x = self.veh_speed
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = angular_velocity
        hello_str = "Speed: %f, steer: %f" % ( v_odom.veh_speed, v_odom.veh_steer)
        rospy.loginfo(hello_str)
        self.odom_pub.publish(odom_msg)
        
              
        
        
        '''
        
        delta_x = (vx * math.cos(self.th) - vy * math.sin(self.th)) * dt
        delta_y = (vx * math.sin(self.th) + vy * math.cos(self.th)) * dt
        delta_th = vth * dt
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        odom_quat = tf.transformations.quaternion_from_euler(0.0, 0.0, self.th)
        odom_msg.pose.pose.orientation.x = odom_quat[0]
        odom_msg.pose.pose.orientation.y = odom_quat[1]
        odom_msg.pose.pose.orientation.z = odom_quat[2]
        odom_msg.pose.pose.orientation.w = odom_quat[3]
        odom_msg.pose.covariance = [   5.0, 0.0,       0.0,        0.0,        0.0,        0.0, 
                                       0.0, 99999.0,   0.0,        0.0,        0.0,        0.0, 
                                       0.0, 0.0,       99999.0,    0.0,        0.0,        0.0,   
                                       0.0, 0.0,       0.0,        99999.0,    0.0,        0.0, 
                                       0.0, 0.0,       0.0,        0.0,        99999.0,    0.0, 
                                       0.0, 0.0,       0.0,        0.0,        0.0,        99999.0] 
        
 
        odom_msg.child_frame_id = "base_link"
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = vth
        odom_msg.twist.covariance = [  5.0, 0.0,       0.0,        0.0,        0.0,        0.0, 
                                       0.0, 99999.0,   0.0,        0.0,        0.0,        0.0, 
                                       0.0, 0.0,       99999.0,    0.0,        0.0,        0.0,   
                                       0.0, 0.0,       0.0,        99999.0,    0.0,        0.0, 
                                       0.0, 0.0,       0.0,        0.0,        99999.0,    0.0, 
                                       0.0, 0.0,       0.0,        0.0,        0.0,        99999.0] 
        if((vx+vth)<0.10): #if stopped or near stopped believe this more.
            odom_msg.twist.covariance[0]=0.0001
            odom_msg.twist.covariance[7]=0.0001
        print "PUBTF",pub_tf
        if (pub_tf):
           self.br.sendTransform((self.x, self.y, 0),
                     tf.transformations.quaternion_from_euler(0.0, 0.0, self.th),
                     rospy.Time.now(),
                     odom_msg.child_frame_id,
                     odom_msg.header.frame_id)
        #Publish the odometry message
        self.last_time = current_time
        self.odom_pub.publish(odom_msg)
          '''

if __name__ == '__main__':
    try:
        v_odom = VirtualOdom()
        rate = rospy.Rate(30) # 10hz
        while not rospy.is_shutdown():
            v_odom.calculate_odom()
            #pub.publish(hello_str)
            rate.sleep()        
        
        
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Command Velocity to Odometry finished.")


'''
/pacmod/parsed_tx/steer_rpt_detail_1

header: 
  seq: 4215
  stamp: 
    secs: 1561130944
    nsecs: 981564176
  frame_id: "pacmod"
current: -0.231
position: 1.359

'''
