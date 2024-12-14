"""
This is the python script version of the command below:

ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose 
"{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2, y: 2, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}}}" --feedback
"""

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration


def main():
    rclpy.init()
    nav = BasicNavigator()  # BasicNavigator is a node
    nav.waitUntilNav2Active()  # wait the transform and navigation to be activated

    # This reason why use PoseStamped is the interface definition of 'nav2_msgs/action/NavigateToPose' is:
    # geometry_msgs/PoseStamped pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav.get_clock().now().to_msg()  # obtain current time
    goal_pose.pose.position.x = 2.0
    goal_pose.pose.position.y = 1.0
    goal_pose.pose.orientation.w = 1.0  # w = 1 denotes oriention is 0
    nav.goToPose(goal_pose)  # send this service
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        nav.get_logger().info(f"Estimated time remaining:{Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9} s")
        # Timeout Auto-Cancellation
        if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
            nav.cancelTask()
    # Final Outcome Determination
    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        nav.get_logger().info('Navigation Goal Succeed')
    elif result == TaskResult.CANCELED:
        nav.get_logger().warn('Navigation Goal Cancled')
    elif result == TaskResult.FAILED:
        nav.get_logger().error('Navigation Goal Failed')
    else:
        nav.get_logger().error('Navigation Result: Returned Status Invalid')

    