#!/usr/bin/env python3

# given a target position in the world frame and local target velocity, this controller calculates the velocity commands for the robot to reach the target position

import rospy
from geometry_msgs.msg import Pose, Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf import transformations, broadcaster
import math

class DecentralizedLeaderFollowerController:
    
    def __init__(self) -> None:
        self.config()
        rospy.Subscriber(self.leader_pose_topic, PoseStamped, self.target_pose_callback)
        rospy.Subscriber(self.actual_pose_topic, PoseStamped, self.actual_pose_callback)
        rospy.Subscriber(self.leader_velocity_topic, Twist, self.target_velocity_callback)
        self.target_pose_broadcaster = broadcaster.TransformBroadcaster()
        self.cmd_vel_publisher = rospy.Publisher(self.follower_cmd_vel_topic, Twist, queue_size=10)
        self.cmd_vel_output = Twist()


    def config(self):
        self.leader_pose_topic = rospy.get_param("~leader_pose_topic", "/target_pose")
        self.actual_pose_topic = rospy.get_param("~actual_pose_topic", "/actual_pose")
        self.leader_velocity_topic = rospy.get_param("~leader_velocity_topic", "/target_velocity")
        self.follower_cmd_vel_topic = rospy.get_param("~follower_cmd_vel_topic", "/cmd_vel")
        self.control_rate = rospy.get_param("~control_rate", 100)
        self.Kp_x = rospy.get_param("~Kp_x", 1.0)
        self.Kp_y = rospy.get_param("~Kp_y", 1.0)
        self.Kp_phi = rospy.get_param("~Kp_phi", 1.0)
        self.relative_position = rospy.get_param("~relative_position", [0.0,0.0,0.0])
        self.lin_vel_max = rospy.get_param("~lin_vel_max", 0.2)
        self.ang_vel_max = rospy.get_param("~ang_vel_max", 0.3)

    def run(self):
        
        # wait for target pose, actual pose and target velocity
        rospy.wait_for_message(self.leader_pose_topic, PoseStamped)
        rospy.wait_for_message(self.actual_pose_topic, PoseStamped)
        rospy.wait_for_message(self.leader_velocity_topic, Twist)
        
        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown():
            if self.target_pose is not None and self.actual_pose is not None and self.target_velocity is not None:
                u_v, u_w = self.update()
            else:
                u_v = 0.0
                u_w = 0.0
                rospy.loginfo("Waiting for target pose, actual pose and target velocity")
            
            # limit the velocities
            if abs(u_v) > self.lin_vel_max:
                u_v = self.lin_vel_max * u_v/abs(u_v)
            if abs(u_w) > self.ang_vel_max:
                u_w = self.ang_vel_max * u_w/abs(u_w)
            
            self.cmd_vel_output.linear.x = u_v
            self.cmd_vel_output.angular.z = u_w
            self.cmd_vel_publisher.publish(self.cmd_vel_output)
            rate.sleep()
            
            
    def update(self):
        
        # compute target pose in the world frame based on leader pose and relative position
        target_pose = self.target_pose
        phi = transformations.euler_from_quaternion([self.target_pose.orientation.x,self.target_pose.orientation.y,self.target_pose.orientation.z,self.target_pose.orientation.w])[2]
        
        target_pose.position.x = self.target_pose.position.x + self.relative_position[0]*math.cos(phi) - self.relative_position[1]*math.sin(phi)
        target_pose.position.y = self.target_pose.position.y + self.relative_position[0]*math.sin(phi) + self.relative_position[1]*math.cos(phi)
                
        # compute the target velocity in the world frame based on leader velocity and relative position
        target_velocity = self.target_velocity
        r = math.sqrt(self.relative_position[0]**2 + self.relative_position[1]**2)
        target_velocity.linear.x = self.target_velocity.linear.x + r*self.target_velocity.angular.z  # todo: check if this is correct
        
        u_v, u_w = self.cartesian_controller(self.actual_pose, target_pose, target_velocity)
        return u_v, u_w        
    
            
    def cartesian_controller(self,actual_pose = Pose(),target_pose = Pose(),target_velocity = Twist(),i = 0):
            phi_act = transformations.euler_from_quaternion([actual_pose.orientation.x,actual_pose.orientation.y,actual_pose.orientation.z,actual_pose.orientation.w])
            phi_target = transformations.euler_from_quaternion([target_pose.orientation.x,target_pose.orientation.y,target_pose.orientation.z,target_pose.orientation.w])
            # R = transformations.quaternion_matrix([actual_pose.orientation.x,actual_pose.orientation.y,actual_pose.orientation.z,actual_pose.orientation.w])

            e_x = (target_pose.position.x- actual_pose.position.x)
            e_y = (target_pose.position.y - actual_pose.position.y)
            e_phi = phi_target[2]-phi_act[2]
            
            # wrap to [-pi,pi] interval
            if e_phi > math.pi:
                e_phi = e_phi - 2*math.pi
            elif e_phi < -math.pi:
                e_phi = e_phi + 2*math.pi
            
            # print("e_x: " + str(e_x))
            # print("e_y: " + str(e_y))
            # print("e_phi: " + str(e_phi))
            
            e_local_x = e_x * math.cos(phi_target[2]) + e_y * math.sin(phi_target[2])
            e_local_y = -e_x * math.sin(phi_target[2]) + e_y * math.cos(phi_target[2])
            
            # print("e_local_x: " + str(e_local_x))
            # print("e_local_y: " + str(e_local_y))

            # broadcast actual pose
            self.target_pose_broadcaster.sendTransform((actual_pose.position.x, actual_pose.position.y, 0.0),
                                                (actual_pose.orientation.x, actual_pose.orientation.y, actual_pose.orientation.z, actual_pose.orientation.w),
                                                rospy.Time.now(),
                                                "actual_pose",
                                                "map")

            # broadcast target pose
            self.target_pose_broadcaster.sendTransform((target_pose.position.x, target_pose.position.y, 0.0),
                                                (target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w),
                                                rospy.Time.now(),
                                                "target_pose_control",
                                                "map")

            u_v = target_velocity.linear.x * math.cos(e_phi) + self.Kp_x*e_local_x
            u_w = target_velocity.angular.z + u_v * ( self.Kp_y * e_local_y + self.Kp_phi * math.sin(e_phi))
            
            # publish metadata
            # self.metadata_publisher.publish_controller_metadata(target_pose = target_pose, actual_pose = actual_pose, target_velocity = target_velocity, publish = True,
            # error = [e_local_x,e_local_y,phi_target[2]-phi_act[2]], robot_id = i) 

            return u_v, u_w
        
    def target_pose_callback(self, msg):
        self.target_pose = msg.pose    
    
    def actual_pose_callback(self, msg):
        self.actual_pose = msg.pose  
    
    def target_velocity_callback(self, msg):
        self.target_velocity = msg
        
        
if __name__ == '__main__':
    try:
        rospy.init_node('decentralized_leader_follower_controller', anonymous=True)
        controller = DecentralizedLeaderFollowerController().run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    