import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class StaticFramePublisher(Node):

    def __init__(self):
        super().__init__('static_tag_broadcaster')

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

    def make_transform_frames(self, tags):
        trans = []
        for tag in tags:
            t = TransformStamped()
            
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = 'tag_frame_' + str(tag[0])
            
            t.transform.translation.x = float(tag[1])
            t.transform.translation.y = float(tag[2])
            t.transform.translation.z = float(tag[3])
            t.transform.rotation.w = tag[4]
            t.transform.rotation.x = tag[5]
            t.transform.rotation.y = tag[6]
            t.transform.rotation.z = tag[7]
            
            trans.append(t)
            
        self.tf_static_broadcaster.sendTransform(trans)

def main():

    rclpy.init()
    
    tag_frame_node = StaticFramePublisher()
    
    # hard code a list of tags, should eventually be a parameter
    tag_list = []
    tag_list.append([38, 1.0, 0.0, 0.175, 0.5, 0.5, -0.5, -0.5])
    tag_list.append([37, -3.0, 0.0, 1.0, 0.5, 0.0, 0.0, 0.866025404])
    
    tag_frame_node.make_transform_frames(tag_list)

    try:
        rclpy.spin(tag_frame_node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
