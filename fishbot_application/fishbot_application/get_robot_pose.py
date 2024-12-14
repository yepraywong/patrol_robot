import rclpy
from rclpy.node import Node
import rclpy.time
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion


class TFListener(Node):
    def __init__(self):
        super().__init__('tf2_listener')
        self.buffer = Buffer()
        self.Listener = TransformListener(self.buffer, self)
        self.timer = self.create_timer(1.0, self.get_transform)

    def get_transform(self):
        try:
            tf = self.buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time(seconds=0), rclpy.time.Duration(seconds=1))
            transform = tf.transform
            rotation_euler = euler_from_quaternion([transform.rotation.x,
                                                    transform.rotation.y,
                                                    transform.rotation.z,
                                                    transform.rotation.w])
            self.get_logger().info(f'translation:{transform.translation}, rotation quaternion:{transform.rotation}:rotation euler:{rotation_euler}')
        except Exception as e:
            self.get_logger().warn(f'can not obtain transform info: {str(e)}')

def main():
    rclpy.init()
    node = TFListener()
    rclpy.spin(node)
    rclpy.shutdown()

