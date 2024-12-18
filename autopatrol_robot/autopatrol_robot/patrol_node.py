import rclpy
from geometry_msgs.msg import PoseStamped, Pose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
import rclpy.time
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from autopatrol_interfaces.srv import SpeechText
from sensor_msgs.msg import Image  # msg interface
from cv_bridge import CvBridge  # Convert image format
import cv2  # save image


class PatrolNode(BasicNavigator):
    def __init__(self, node_name='patrol_node'):
        super().__init__(node_name)
        # decline relevant params
        self.declare_parameter('initial_point', [0.0, 0.0, 0.0])  # robot's initial point
        self.declare_parameter('target_point', [0.0, 0.0, 0.0, 1.0, 1.0, 1.57])  # 1.57 means rotate 90 degree 数组的每三位表示一个目标点
        self.initial_point_ = self.get_parameter('initial_point').value
        self.target_point_ = self.get_parameter('target_point').value
        # obtain current position using TF Listener
        self.buffer = Buffer()
        self.Listener = TransformListener(self.buffer, self)

        # Log loaded parameters
        self.get_logger().info(f"Loaded initial_point: {self.initial_point_}")
        self.get_logger().info(f"Loaded target_point: {self.target_point_}")

        # create a client to sythesize speech
        self.speech_client_ = self.create_client(SpeechText, 'speech_text')

        # Customize the save path for images
        # the defaout path is the workspace
        self.declare_parameter('img_save_path', '')
        self.img_save_path_ = self.get_parameter('img_save_path').value
        # convert image format
        self.cv_bridge_ = CvBridge()
        # Save the latest image in real-time
        self.lastest_img_ = None
        self.img_sub_ = self.create_subscription(Image, '/camera_sensor/image_raw', self.img_callback, 1)

    def get_pose_by_xyyaw(self, x, y, yaw):
        """
        input: x, y, yaw
        return: PoseStamped Object
        """
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y

        quat = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        return pose


    def init_robot_pose(self):
        """
        initial robot's position
        """
        self.initial_point_ = self.get_parameter('initial_point').value
        init_pose = self.get_pose_by_xyyaw(self.initial_point_[0], self.initial_point_[1], self.initial_point_[2])
        self.setInitialPose(init_pose)  # initialize the init_pose
        self.waitUntilNav2Active()  # wait the transform and navigation to be activated

    def get_target_point(self):
        """
        通过参数值获取目标点集合
        Obtain the set of target points through parameter values
        """
        points = []
        self.target_point_ = self.get_parameter('target_point').value
        for index in range(int(len(self.target_point_)/3)):
            x = self.target_point_[index*3]
            y = self.target_point_[index*3+1]
            yaw = self.target_point_[index*3+2]
            points.append([x, y, yaw])
            self.get_logger().info(f"Obtain target points{index}->{x},{y},{yaw}")
        return points

    def nav_to_pose(self, target_point):
        """
        navigate to the target point
        """
        self.goToPose(target_point)
        while not self.isTaskComplete():
            feedback = self.getFeedback()
            self.get_logger().info(f"Estimated time remaining:{Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9} s")
            # Timeout Auto-Cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                self.cancelTask()

        # Final Outcome Determination
        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Navigation Goal Succeed')
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('Navigation Goal Cancled')
        elif result == TaskResult.FAILED:
            self.get_logger().error('Navigation Goal Failed')
        else:
            self.get_logger().error('Navigation Result: Returned Status Invalid')

    def get_current_pose(self):
        """
        obtain the current position of robot
        """
        while rclpy.ok():  # If the acquisition fails this time, let the program try again to obtain it
            try:
                tf = self.buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time(seconds=0), rclpy.time.Duration(seconds=1))
                transform = tf.transform
                rotation_euler = euler_from_quaternion([transform.rotation.x,
                                                        transform.rotation.y,
                                                        transform.rotation.z,
                                                        transform.rotation.w])
                self.get_logger().info(f'translation:{transform.translation}, rotation quaternion:{transform.rotation}:rotation euler:{rotation_euler}')
                return transform
            except Exception as e:
                self.get_logger().warn(f'can not obtain transform info: {str(e)}')

    def speech_text(self, text):
        """
        Call service to synthesize speech
        """
        # If the service server does not come online within one second:
        while not self.speech_client_.wait_for_service(timeout_sec=1):
            self.get_logger().info('Speech synthesis service is not online, waiting...')
        request = SpeechText.Request()
        request.text = text
        future = self.speech_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            if response.result == True:
                self.get_logger().info('Speech synthesis successful: {text}')
            else:
                self.get_logger().warn('Speech synthesis failed: {text}')
        else:  # results == None
            self.get_logger().warn('Speech synthesis service response failed')
    
    def img_callback(self, msg):
        self.lastest_img_ = msg

    def record_img(self):
        if self.lastest_img_ is not None:
            pose = self.get_current_pose()
            cv_image = self.cv_bridge_.imgmsg_to_cv2(self.lastest_img_)
            cv2.imwrite(
                f'{self.img_save_path_}img_{pose.translation.x:3.2f}_{pose.translation.y:3.2f}.png',
                cv_image
            )
        

def main():
    rclpy.init()
    patrol = PatrolNode()
    patrol.speech_text('Preparing to initialize position')
    patrol.init_robot_pose()
    patrol.speech_text('Position initialization complete')

    while rclpy.ok():
        points = patrol.get_target_point()
        for point in points:
            x, y, yaw = point[0], point[1], point[2]
            target_pose = patrol.get_pose_by_xyyaw(x, y, yaw)
            patrol.speech_text(text=f'Preparing to go to the target point {x}, {y}')
            patrol.nav_to_pose(target_pose)
            patrol.speech_text(text=f'Target point {x}, {y} reached, preparing to save images')
            patrol.record_img()
            patrol.speech_text(text=f'Images saved')

    rclpy.shutdown()

    