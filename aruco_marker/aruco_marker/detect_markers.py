import cv2
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
import rclpy.time
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import tf_transformations
import math

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.bridge = CvBridge()

        self.rgb_subscriber = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw", 
            self.rgb_callback,
            10
        )

        self.pose_publisher = self.create_publisher(
            PoseStamped,
            "/detected_marker/marker_pose",
            10
        )

        self.video_publisher = self.create_publisher(
            Image,
            "/detected_markers/image",
            10
        )

        # Define world-to-camera transformation matrix
        angle = 42.0
        self.world_to_camera_tf = np.array([
            [math.cos(math.radians(angle)), 0, math.sin(math.radians(angle)), 15*10**-3],
            [0, 1, 0.0, 9.6*10**-3],
            [-(math.sin(math.radians(angle))), 0, math.cos(math.radians(angle)), 1795*10**-3],
            [0.0, 0.0, 0.0, 1]
        ])
        self.marker_size = 0.1

    def get_camera_to_world_tf(self):
        
        R = self.world_to_camera_tf[:3, :3]
        t = self.world_to_camera_tf[:3, 3]
        R_inv = R.T
        t_inv = -R_inv @ t
        camera_to_world_tf = np.eye(4)
        camera_to_world_tf[:3, :3] = R_inv
        camera_to_world_tf[:3, 3] = t_inv
        return camera_to_world_tf

    def rgb_callback(self, msg):
        # Camera intrinsic parameters
        self.camera_matrix = np.array([[565.6008952774198, 0, 320],
                                        [0, 565.60089527741968, 240],
                                        [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])  # No lens distortion
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
        parameters = cv2.aruco.DetectorParameters()

        try:
            rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
            detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
            corners, ids, _ = detector.detectMarkers(gray)
            if len(corners) != 0 and len(ids) != 0:
                for corner, id in zip(corners, ids):
                    objpoints = np.array([[-self.marker_size, self.marker_size, 0],
                                          [self.marker_size, self.marker_size, 0],
                                          [self.marker_size, -self.marker_size, 0],
                                          [-self.marker_size, -self.marker_size, 0]])
                    corner = corner[0]
                    rvec = np.zeros((3, 1), dtype=np.float64)
                    tvec = np.zeros((3, 1), dtype=np.float64)
                    success = cv2.solvePnP(objpoints, corner, self.camera_matrix, self.dist_coeffs, rvec, tvec,
                                           useExtrinsicGuess=True, flags=cv2.SOLVEPNP_EPNP)
                    if success:
                        distance = np.linalg.norm(tvec)
                        if distance < 0.0001 or distance > 1000:
                            self.video_publisher.publish(msg)
                        else:
                        # self.get_logger().info(f"Detected marker: {id}\nRotation : {rvec}\nTranslation : {tvec}")
                            cv2.drawFrameAxes(rgb_image,self.camera_matrix,self.dist_coeffs,rvec,tvec,1,3)
                            cv2.aruco.drawDetectedMarkers(rgb_image,corners,ids)

                            camera_to_world_tf = self.get_camera_to_world_tf()
                            tvec_h = np.array([tvec[0, 0], tvec[1, 0], tvec[2, 0], 1.0]).reshape(4, 1)
                            tvec_world = camera_to_world_tf @ tvec_h

                            rotation_matrix, _ = cv2.Rodrigues(rvec)
                            rotation_matrix_world = camera_to_world_tf[:3, :3] @ rotation_matrix
                            quaternion = tf_transformations.quaternion_from_matrix(
                                np.vstack((np.hstack((rotation_matrix_world, [[0], [0], [0]])), [0, 0, 0, 1]))
                                )
                            
                            # Publish Pose
                            pose = PoseStamped()
                            pose.header.stamp = self.get_clock().now().to_msg()
                            pose.header.frame_id = f"Detected ID : {id}"
                            pose.pose.position.x = float(tvec_world[0])
                            pose.pose.position.y = float(tvec_world[1])
                            pose.pose.position.z = float(tvec_world[2])
                            pose.pose.orientation.x = float(quaternion[0])
                            pose.pose.orientation.y = float(quaternion[1])
                            pose.pose.orientation.z = float(quaternion[2])
                            pose.pose.orientation.w = float(quaternion[3])
                            self.pose_publisher.publish(pose)
                    else:
                        self.get_logger().error(f"Unable to estimate pose of markers")
                    ros_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding="bgr8")
                    self.video_publisher.publish(ros_msg)
            else:        
                self.video_publisher.publish(msg)
        except CvBridgeError as e:
            self.get_logger().error(f'Error converting ROS msg to cv2 image: {e}')

def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoDetector()
    try:
        rclpy.spin(aruco_detector)
    except KeyboardInterrupt:
        aruco_detector.get_logger().info('Shutting down detect_marker node')
    finally:
        aruco_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
