import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
import tf_transformations as tf
import numpy as np

class FrameListener(Node):

    def __init__(self):
        super().__init__('pose_transformer')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create lookup transform pose publisher
        self.pose_publisher = self.create_publisher(PoseStamped, 'tag38/pose', 1)

        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)
        
        self.got_static_transform = False
        self.tag38_to_map = TransformStamped()
        self.measured_tag38 = TransformStamped()
        
    def on_timer(self):
        # First get the static transform, then continue with measured transform
        # Could not get the static transform from inside the constructor, don't know why????
        if self.got_static_transform:
            try:
                self.measured_tag38 = self.tf_buffer.lookup_transform(
                    'tag36h11:38',
                    'base_footprint',
                    rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().info('Could not transform tag36h11:38 to base_footprint')
                return
        else:
            # Look up for the static transform
            try:
                self.tag38_to_map = self.tf_buffer.lookup_transform(
                    'map',
                    'tag_frame_38',
                    rclpy.time.Time())
                self.got_static_transform = True
            except TransformException as ex:
                self.get_logger().info('Could not transform tag_frame_38 to map')
                return
       
        v = self.tag38_to_map.transform.translation
        q = self.tag38_to_map.transform.rotation
        mat1 = np.dot(tf.translation_matrix([v.x, v.y, v.z]), tf.quaternion_matrix([q.x, q.y, q.z, q.w]))

        v = self.measured_tag38.transform.translation
        q = self.measured_tag38.transform.rotation
        mat2 = np.dot(tf.translation_matrix([v.x, v.y, v.z]), tf.quaternion_matrix([q.x, q.y, q.z, q.w]))

        mat3 = np.dot(mat1, mat2)
        v = tf.translation_from_matrix(mat3)
        q = tf.quaternion_from_matrix(mat3)

        #self.get_logger().info('translation %s' %len(v))
        #self.get_logger().info('rotation %s' %len(q))
        # find the unknown map to base_footprint transform
        #robot_tag38 = measured_tag38 * self.tag38_to_map

        robot_pose_msg = PoseStamped()
        robot_pose_msg.header.stamp = self.get_clock().now().to_msg()
        robot_pose_msg.header.frame_id = "map"
        robot_pose_msg.pose.position.x = v[0]
        robot_pose_msg.pose.position.y = v[1]
        robot_pose_msg.pose.position.z = v[2]
        robot_pose_msg.pose.orientation.w = q[3]
        robot_pose_msg.pose.orientation.x = q[0]
        robot_pose_msg.pose.orientation.y = q[1]
        robot_pose_msg.pose.orientation.z = q[2]
        self.pose_publisher.publish(robot_pose_msg)

def main():
    rclpy.init()
    pose_transform_node = FrameListener()
    
    try:
        rclpy.spin(pose_transform_node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
