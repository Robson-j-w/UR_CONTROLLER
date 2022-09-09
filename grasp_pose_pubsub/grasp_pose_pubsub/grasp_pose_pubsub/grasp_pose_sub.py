import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, Pose
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_ros

# import tf_transformations

import math
import numpy as np

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('grasp_pose_subscriber')
        self.subscription = self.create_subscription(PoseArray, 'grasp_poses', self.listener_callback, 3)
        self.subscription  # prevent unused variable warning
        
        # Initialize the transform broadcaster
        self.br = tf2_ros.TransformBroadcaster(self)
        
        # For the TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.publisher_ = self.create_publisher(Marker, "/visualization_marker", 2)
        
    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
    
    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q
        
    def timer_callback(self):
            print("sending transform...")
            self.t.header.stamp = self.get_clock().now().to_msg()
            self.br.sendTransform(self.t)
            # trans = self.tf_buffer.lookup_transform('base_link', 'target_pose', rclpy.time.Time())
            
            # marker = Marker()

            # marker.header.frame_id = "/base_link"
            # marker.header.stamp = self.get_clock().now().to_msg()

            # # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
            # marker.type = 2
            # marker.id = 0

            # # Set the scale of the marker
            # marker.scale.x = 1.0
            # marker.scale.y = 1.0
            # marker.scale.z = 1.0

            # # Set the color
            # marker.color.r = 0.0
            # marker.color.g = 1.0
            # marker.color.b = 0.0
            # marker.color.a = 1.0

            # # Set the pose of the marker
            # marker.pose.position.x = trans.position.x
            # marker.pose.position.y = trans.position.y
            # marker.pose.position.z = trans.position.z
            # marker.pose.orientation.x = trans.orientation.x
            # marker.pose.orientation.y = trans.orientation.y
            # marker.pose.orientation.z = trans.orientation.z
            # marker.pose.orientation.w = trans.orientation.w
            
            # self.publisher_.publish(marker)

    def listener_callback(self, pose_array):
        # self.get_logger().info('I heard: "%s"' % pose_array.poses[0])
        _pose = pose_array.poses[0].position
        _quaternion = pose_array.poses[0].orientation
        
        # converting to base link frame of reference
        # _pose.x = _pose.x + 1.000
        # _pose.y = _pose.y + 0.000
        # _pose.z = _pose.z + 0.500
        
        # _quaternion.x = _quaternion.x + 1.000
        # _quaternion.y = _quaternion.y + 0.000
        # _quaternion.z = _quaternion.z + 0.000
        # _quaternion.w = _quaternion.w + -0.001
                
        self.t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        self.t.header.stamp = self.get_clock().now().to_msg()
        self.t.header.frame_id = 'camera_optical_frame'
        self.t.child_frame_id = 'target_pose'

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        self.t.transform.translation.x = _pose.x
        self.t.transform.translation.y = _pose.y
        self.t.transform.translation.z = _pose.z

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        self.t.transform.rotation.x = _quaternion.x
        self.t.transform.rotation.y = _quaternion.y
        self.t.transform.rotation.z = _quaternion.z
        self.t.transform.rotation.w = _quaternion.w
        
        # print(t)
        self.timer = self.create_timer(0.05, self.timer_callback)    
        
        # while True:
        #     try:
        #         trans = self.tf_buffer.lookup_transform('base_link', 'target_pose', rclpy.time.Time())
        #         print (trans)
        #         self.br.sendTransform(trans)
        #     except TransformException as ex:
        #         self.get_logger().error( f'Could not transform: {ex}')
        
        
            

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()