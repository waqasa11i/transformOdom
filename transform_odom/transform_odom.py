#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import numpy as np
from scipy.spatial.transform import Rotation as R

class OdomTransformer(Node):
    def __init__(self):
        super().__init__('odom_transformer')
        
        self.initial_odom_received = False
        self.initial_pose = None
        self.last_kiss_odom = None
        
        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Publishers and Subscribers
        self.transformed_odom_pub = self.create_publisher(
            Odometry, 
            '/transformed_odom', 
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/a201_0000/platform/odom/filtered',
            self.odom_callback,
            10
        )
        
        self.kiss_odom_sub = self.create_subscription(
            Odometry,
            '/kiss/odometry',
            self.kiss_odom_callback,
            10
        )
        
        self.get_logger().info('Odom Transformer node started')
        self.get_logger().info('Waiting for initial pose...')
        
    def odom_callback(self, msg):
        """
        Callback for Gazebo pose. Stores the initial pose.
        """
        if not self.initial_odom_received:
            self.initial_pose = msg.pose.pose
            self.initial_odom_received = True
            self.get_logger().info('Initial pose received')
            
            #self.publish_initial_transform(msg)
            
    def kiss_odom_callback(self, msg):
        """
        Callback for odometry.
        """
        if not self.initial_odom_received:
            self.get_logger().warn('Initial pose not received yet ... ')
            return
            
        self.last_kiss_odom = msg
        
        transformed_odom = self.transform_kiss_odometry(msg)
        
        self.transformed_odom_pub.publish(transformed_odom)
        
        #self.publish_odom_tf(transformed_odom)
        
    def transform_kiss_odometry(self, kiss_odom):

        transformed_odom = Odometry()
        transformed_odom.header.stamp = kiss_odom.header.stamp
        transformed_odom.header.frame_id = kiss_odom.header.frame_id
        transformed_odom.child_frame_id = kiss_odom.child_frame_id
        
        kiss_pos = kiss_odom.pose.pose.position
        kiss_orient = kiss_odom.pose.pose.orientation
        
        initial_pos = self.initial_pose.position
        initial_orient = self.initial_pose.orientation
        
        kiss_position = np.array([kiss_pos.x, kiss_pos.y, kiss_pos.z])
        kiss_orientation = np.array([kiss_orient.x, kiss_orient.y, kiss_orient.z, kiss_orient.w])
        
        initial_position = np.array([initial_pos.x, initial_pos.y, initial_pos.z])
        initial_orientation = np.array([initial_orient.x, initial_orient.y, initial_orient.z, initial_orient.w])
        
        rot_initial = R.from_quat(initial_orientation)
        rotated_kiss_pos = rot_initial.apply(kiss_position)
        
        transformed_position = initial_position + rotated_kiss_pos
        
        rot_kiss = R.from_quat(kiss_orientation)
        transformed_rotation = rot_initial * rot_kiss
        transformed_quat = transformed_rotation.as_quat()
        
        transformed_odom.pose.pose.position.x = transformed_position[0]
        transformed_odom.pose.pose.position.y = transformed_position[1]
        transformed_odom.pose.pose.position.z = transformed_position[2]
        
        transformed_odom.pose.pose.orientation.x = transformed_quat[0]
        transformed_odom.pose.pose.orientation.y = transformed_quat[1]
        transformed_odom.pose.pose.orientation.z = transformed_quat[2]
        transformed_odom.pose.pose.orientation.w = transformed_quat[3]
        
        kiss_twist_linear = np.array([
            kiss_odom.twist.twist.linear.x,
            kiss_odom.twist.twist.linear.y,
            kiss_odom.twist.twist.linear.z
        ])
        
        transformed_twist_linear = rot_initial.apply(kiss_twist_linear)
        
        transformed_odom.twist.twist.linear.x = transformed_twist_linear[0]
        transformed_odom.twist.twist.linear.y = transformed_twist_linear[1]
        transformed_odom.twist.twist.linear.z = transformed_twist_linear[2]
        
        transformed_odom.twist.twist.angular = kiss_odom.twist.twist.angular
        
        transformed_odom.pose.covariance = kiss_odom.pose.covariance
        transformed_odom.twist.covariance = kiss_odom.twist.covariance
        
        return transformed_odom
        
    def publish_initial_transform(self, odom_msg):

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        
        transform.transform.translation.x = odom_msg.pose.pose.position.x
        transform.transform.translation.y = odom_msg.pose.pose.position.y
        transform.transform.translation.z = odom_msg.pose.pose.position.z
        
        transform.transform.rotation = odom_msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().info('Published initial pose')
        
    def publish_odom_tf(self, odom_msg):

        transform = TransformStamped()
        transform.header.stamp = odom_msg.header.stamp
        transform.header.frame_id = odom_msg.header.frame_id
        transform.child_frame_id = odom_msg.child_frame_id
        
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(transform)
        
    def destroy_node(self):
        self.get_logger().info('Shutting down Odom Transformer node')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = OdomTransformer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
