from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy


def main():
    rclpy.init()
    nav = BasicNavigator()  # BasicNavigator is a node
    init_pose = PoseStamped()
    init_pose.header.frame_id = 'map'
    init_pose.header.stamp = nav.get_clock().now().to_msg()  # obtain current time
    init_pose.pose.position.x = 0.0
    init_pose.pose.position.y = 0.0
    init_pose.pose.orientation.w = 1.0  # w = 1 denotes oriention is 0
    nav.setInitialPose(init_pose)  # initialize the init_pose
    nav.waitUntilNav2Active()  # wait the transform and navigation to be activated
    rclpy.spin(nav)
    rclpy.shutdown()
    