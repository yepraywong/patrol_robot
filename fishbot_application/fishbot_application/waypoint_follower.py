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

    # This reason why use PoseStamped is the interface definition of 'nav2_msgs/action/FollowWaypoints' is:
    # geometry_msgs/PoseStamped[] poses
    # the PoseStamped here is an array

    goal_poses = []

    # 1st point
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav.get_clock().now().to_msg()  # obtain current time
    goal_pose.pose.position.x = 2.0
    goal_pose.pose.position.y = 1.0
    goal_pose.pose.orientation.w = 1.0  # w = 1 denotes oriention is 0
    goal_poses.append(goal_pose)
    # 2nd point
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = nav.get_clock().now().to_msg()  # obtain current time
    goal_pose1.pose.position.x = 0.0
    goal_pose1.pose.position.y = 1.0
    goal_pose1.pose.orientation.w = 1.0  # w = 1 denotes oriention is 0
    goal_poses.append(goal_pose1)
    # 3rd point
    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = nav.get_clock().now().to_msg()  # obtain current time
    goal_pose2.pose.position.x = 0.0
    goal_pose2.pose.position.y = 1.0
    goal_pose2.pose.orientation.w = 1.0  # w = 1 denotes oriention is 0
    goal_poses.append(goal_pose2)

    nav.followWaypoints(goal_poses)  # send this service

    # 判断结束及获取反馈
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        nav.get_logger().info(f' 当前目标编号：{feedback.current_waypoint}')
        
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

    