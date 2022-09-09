import rclpy
from rclpy.node import Node
# import rospy

from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, Pose
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_ros

class FrameListener(Node):

    def __init__(self):
        super().__init__('frame_transformer')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(0.02, self.on_timer)
        
        self.MarkerPublisher_ = self.create_publisher(Marker, "target_visualization_marker", 2)
        
    def on_timer(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'base_link',
                'target_pose',
                now)
            
            print(trans)
            
            marker = Marker()

            marker.header.frame_id = "/base_link"
            marker.header.stamp = self.get_clock().now().to_msg()

            # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
            marker.type = 2
            marker.id = 0

            # Set the scale of the marker
            marker.scale.x = .07
            marker.scale.y = .07
            marker.scale.z = .07

            # Set the color
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            # Set the pose of the marker
            marker.pose.position.x = trans.transform.translation.x
            marker.pose.position.y = trans.transform.translation.y
            marker.pose.position.z = trans.transform.translation.z
            marker.pose.orientation.x = trans.transform.rotation.x
            marker.pose.orientation.y = trans.transform.rotation.y
            marker.pose.orientation.z = trans.transform.rotation.z
            marker.pose.orientation.w = trans.transform.rotation.w
            
            self.MarkerPublisher_.publish(marker)            
            
        except TransformException as ex:
            self.get_logger().info(f'Could not transform: {ex}')
            return
        
def main(args=None):
    rclpy.init(args=args)

    tf_transformer = FrameListener()

    rclpy.spin(tf_transformer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tf_transformer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()