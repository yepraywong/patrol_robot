import numpy as np
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
from sensor_msgs.msg import LaserScan
import math
from sklearn.cluster import DBSCAN


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
        # Save the image in real-time
        self.lastest_img_ = None
        # Camera image subscription
        self.img_sub_ = self.create_subscription(Image, '/camera_sensor/image_raw', self.img_callback, 1)

        # Save the LiDAR data in real-time
        self.lidar_data_ = None
        # Laser scan subscription
        self.lidar_sub_ = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 1)

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
            # self.get_logger().info(f"Estimated time remaining:{Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9} s")
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

    def lidar_callback(self, msg):
        self.lidar_data_ = msg
    '''
    def detect_flowers(self, image):
        """
        Detect flower (assumed to be a specific color) in the camera image.
        This is a simple color-based detection approach.
        """
        # Convert ROS image message to OpenCV format
        cv_image = self.cv_bridge_.imgmsg_to_cv2(image)
        # Convert to HSV for better color segmentation
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # Define the range for flower color in HSV space (e.g., red flowers)
        lower_color = np.array([110, 100, 100])  # lower range of red
        upper_color = np.array([130, 255, 255])  # upper range of red
        # Create a mask that identifies the flower area based on color
        mask = cv2.inRange(hsv_image, lower_color, upper_color)
        # Apply mask to the original image
        result = cv2.bitwise_and(cv_image, cv_image, mask=mask)
        # Find contours of detected flower
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest contour, assuming it's the flower
            largest_contour = max(contours, key=cv2.contourArea)
            # Get the bounding box of the flower
            x, y, w, h = cv2.boundingRect(largest_contour)
            # Return the center position of the flower
            flower_center = (x + w / 2, y + h / 2)
            return flower_center
        else:
            return None
        
    def move_to_flower(self):
        """
        Move robot to the flower's position based on the detected position
        from the camera.
        """
        if self.lastest_img_:
            flower_position = self.detect_flowers(self.lastest_img_)
            if flower_position:
                self.get_logger().info(f"Flower detected at position: {flower_position}")
                # Calculate the pose to move towards the flower (robot-centric)
                # Here we use the flower's x, y position in the image and combine it with lidar data to estimate the distance
                
                # For simplicity, let's assume that the robot moves directly to the detected flower's position.
                # You can expand this by combining Lidar data to check the real-world distance and direction.
                x, y = flower_position
                target_pose = self.get_pose_by_xyyaw(x, y, 0)  # Assuming flower is directly in front
                self.goToPose(target_pose)

                # wait until reaching the target
                # while not self.isTaskComplete():
                #     feedback = self.getFeedback()
                #     self.get_logger().info(f"Estimated time remaining: {feedback.estimated_time_remaining}")

                result = self.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info("Arrived at flower location")
                else:
                    self.get_logger().warn("Failed to reach flower location")
            else:
                self.get_logger().info("No flower detected.")
        else:
            self.get_logger().warn("No image received from camera.")
    '''
    def record_img(self):
        if self.lastest_img_ is not None:
            pose = self.get_current_pose()
            cv_image = self.cv_bridge_.imgmsg_to_cv2(self.lastest_img_)
            cv2.imwrite(
                f'{self.img_save_path_}img_{pose.translation.x:3.2f}_{pose.translation.y:3.2f}.png',
                cv_image
            ) 

    def detect_hydrant_with_lidar(self):
        """
        Use LiDAR data to detect if a hydrant-like object exists in front of the robot.
        """
        if self.lidar_data_ is None:
            self.get_logger().warn("No LiDAR data received.")
            return None

        # Extract ranges from LiDAR data
        ranges = np.array(self.lidar_data_.ranges)
        angle_min = self.lidar_data_.angle_min
        angle_increment = self.lidar_data_.angle_increment

        # Define detection range (e.g., 0.5m to 2m in front)
        min_distance = 0.5  # Minimum distance (meters)
        max_distance = 2.0  # Maximum distance (meters)
        detection_angle = 0.5  # Detect within 0.5 rad (~30 degrees) of front

        # Iterate over ranges to check for objects within range
        detected_points = []
        for i, distance in enumerate(ranges):
            angle = angle_min + i * angle_increment
            if -detection_angle <= angle <= detection_angle and min_distance <= distance <= max_distance:
                detected_points.append((distance, angle))
        
        # If detected points exist, estimate the closest object's position
        if detected_points:
            # Find the closest point
            closest_distance, closest_angle = min(detected_points, key=lambda x: x[0])

            # Calculate the object's position in robot's coordinate frame
            x = closest_distance * np.cos(closest_angle)
            y = closest_distance * np.sin(closest_angle)
            self.get_logger().info(f"Detected hydrant-like object at (x: {x:.2f}, y: {y:.2f})")
            return x, y

        self.get_logger().info("No hydrant detected in front.")
        return None
        """
        if detected_points:
            # Apply clustering to filter out walls
            cluster = self.cluster_points(detected_points)
            if cluster is not None:
                # Return the centroid of the closest cluster
                x_mean = np.mean(cluster[:, 0])
                y_mean = np.mean(cluster[:, 1])
                self.get_logger().info(f"Detected hydrant-like cluster at (x: {x_mean:.2f}, y: {y_mean:.2f})")
                return x_mean, y_mean

        self.get_logger().info("No hydrant detected in front.")
        return None
        """
    
    def convert_to_global(self, x_robot, y_robot, x_robot_map, y_robot_map, robot_yaw):
        """
        Convert coordinates from robot's local frame to global map frame.
        
        Args:
            x, y: Coordinates in the robot's local frame (meters).
            robot_position: Tuple of (x, y) representing robot's position in the global map (meters).
            robot_yaw: Robot's orientation (yaw) in radians in the global map.

        Returns:
            x_global, y_global: Coordinates in the global map frame (meters).
        """
        theta = robot_yaw  # Robot's orientation in radians

        # Apply the coordinate transformation formula
        x_global = x_robot * math.cos(theta) - y_robot * math.sin(theta) + x_robot_map
        y_global = x_robot * math.sin(theta) + y_robot * math.cos(theta) + y_robot_map

        return x_global, y_global
    
    def calculate_aspect_ratio(self, cluster):
        x_min, y_min = np.min(cluster, axis=0)
        x_max, y_max = np.max(cluster, axis=0)
        width = x_max - x_min
        height = y_max - y_min
        return width / height if height > 0 else float('inf')
    
    def cluster_points(self, points, threshold_distance=2.0):
        """
        Cluster points and filter clusters based on geometry and distance.

        Args:
            points: List of tuples [(distance1, angle1), (distance2, angle2), ...]
            threshold_distance: Maximum average distance for valid clusters.

        Returns:
            smallest_cluster: The smallest valid cluster (in Cartesian coordinates),
                            or None if no valid clusters are found.
        """
        if not points:
            return None

        # Step 1: Convert polar coordinates to Cartesian coordinates
        cartesian_points = np.array([
            [r * np.cos(a), r * np.sin(a)] for r, a in points
        ])

        # Step 2: Apply DBSCAN clustering
        clustering = DBSCAN(eps=0.2, min_samples=3).fit(cartesian_points)
        labels = clustering.labels_

        # Step 3: Extract clusters based on labels
        unique_labels = set(labels)
        clusters = [cartesian_points[labels == l] for l in unique_labels if l != -1]

        # Step 4: Filter clusters based on geometry and distance
        def is_valid_cluster(cluster):
            aspect_ratio = self.calculate_aspect_ratio(cluster)
            mean_distance = np.mean(np.linalg.norm(cluster, axis=1))
            return aspect_ratio < 2 and mean_distance < threshold_distance

        valid_clusters = [cluster for cluster in clusters if is_valid_cluster(cluster)]

        # Step 5: Find the smallest valid cluster
        smallest_cluster = min(valid_clusters, key=len, default=None)

        return smallest_cluster
    
    def move_to_hydrant(self, x_global, y_global):
        """
        Use LiDAR to detect a hydrant and navigate to it.
        """
        # hydrant_position = self.convert_to_global()
        if x_global and y_global:
            # x, y = hydrant_position

            self.get_logger().info(f"Navigating to hydrant at global position: {x_global:.2f}, {y_global:.2f}")
            target_pose = self.get_pose_by_xyyaw(x_global, y_global, 0)
            self.goToPose(target_pose)

            # Wait for navigation to complete
            while not self.isTaskComplete():
                feedback = self.getFeedback()
                self.get_logger().info(f"Estimated time remaining: {feedback.estimated_time_remaining}")

            result = self.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info("Arrived at hydrant location")
            else:
                self.get_logger().warn("Failed to reach hydrant location")
        else:
            self.get_logger().info("No hydrant detected.")



def main(args=None):
    rclpy.init()
    patrol_node = PatrolNode()
    patrol_node.speech_text('Preparing to initialize position')
    patrol_node.init_robot_pose()
    patrol_node.speech_text('Position initialization complete')
    # Get the set of target points from parameters
    target_points = patrol_node.get_target_point()

    # Start patrolling by navigating through the set of target points
    for target_point in target_points:
        x, y, yaw = target_point[0], target_point[1], target_point[2]
        target_pose = patrol_node.get_pose_by_xyyaw(x, y, yaw)
        patrol_node.speech_text(text=f'Preparing to go to the target point {x}, {y}')
        patrol_node.nav_to_pose(target_pose)
        # After reaching the target point, check for flowers
        patrol_node.record_img()
        patrol_node.speech_text(text=f'Images saved, preparing to detect hydrant')
        # patrol_node.get_logger().info("Checking for hydrant...")
        result = patrol_node.detect_hydrant_with_lidar()
        if result is not None:
            x_robot, y_robot = result

            current_pose = patrol_node.get_current_pose()
            x_robot_map = current_pose.translation.x
            y_robot_map = current_pose.translation.y
            robot_yaw = euler_from_quaternion([
                current_pose.rotation.x,
                current_pose.rotation.y,
                current_pose.rotation.z,
                current_pose.rotation.w
            ])[2]
            x_global, y_global = patrol_node.convert_to_global(x_robot, y_robot, x_robot_map, y_robot_map, yaw)
            patrol_node.move_to_hydrant(x_global, y_global)
        else:
            patrol_node.get_logger().info("No hydrant detected, skipping...")


    # After completing the patrol, call the speech synthesis service to announce completion
    patrol_node.speech_text("Patrol completed successfully")

    rclpy.spin(patrol_node)
    rclpy.shutdown()
