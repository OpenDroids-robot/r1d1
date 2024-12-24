import cv2
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
import rclpy.time
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
import tf_transformations
from tf2_ros import TransformBroadcaster

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.bridge = CvBridge()
        self.broacaster = TransformBroadcaster(self)
        # self.info_subscriber = self.create_subscription(
        #     CameraInfo,
        #     '/depth_camera/camera_info', 
        #     self.info_callback,
        #     10
        # )

        self.rgb_subscriber = self.create_subscription(
            Image,
            "<put_your_image_topic>", 
            self.rgb_callback,
            10
        )

        self.transform_publisher = self.create_publisher(
            TransformStamped,
            "/detected_marker/transforms",
            10
        )

        # self.tf_publisher = self.create_publisher(
        #     TFMessage,
        #     "/tf",
        #     10
        # )

        self.video_publisher = self.create_publisher(
            Image,
            "/detected_markers/image",
            10
        )

    def info_callback(self,msg):
        self.camera_matrix = msg.k
        self.dist_coeffs = msg.d
        self.get_logger().info(f"k : {msg.k}")
        self.get_logger().info(f"d : {msg.d}")

    def rgb_callback(self,msg):
        # Camera intrinsic parameters obtained fron camera_info topic in gazebo
        self.camera_matrix = np.array([[565.6008952774198, 0, 320],             #hard-coded camera intrinsic matrix and distortion coefficients for gazebo rgbd camera
                                [0, 565.60089527741968, 240],                   #using values from camera_info message. To be modified by calibrating camera 
                                [0,   0,   1]], dtype=np.float32)               #or using camera info for your use case
        self.dist_coeffs = np.array([0.00000000000005,0.00000000000005,0.00000000000005,0.00000000000005,0.00000000000005])  # For no lens distortion, assume very small numbers to avoid errors
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
        parameters = cv2.aruco.DetectorParameters()
        try:
            # Convert the ROS rgb image message to an OpenCV image
            rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
            detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
            corners, ids, rejected = detector.detectMarkers(gray)
            if len(corners) != 0 and len(ids) != 0:
                for (corner,id) in zip(corners,ids):
                    objpoints = np.array([[-0.5,0.5,0],[0.5,0.5,0],[0.5,-0.5,0],[-0.5,-0.5,0]])
                    corner = corner[0]
                    rvec = np.zeros((3,1),dtype=np.float64)
                    tvec = np.zeros((3,1),dtype=np.float64)
                    success = cv2.solvePnP(objpoints,corner,self.camera_matrix,self.dist_coeffs,rvec=rvec,tvec=tvec,useExtrinsicGuess=True,flags=cv2.SOLVEPNP_EPNP)
                    if success:
                        distance = np.linalg.norm(tvec)
                        if distance < 0.0001 or distance > 1000:
                            self.video_publisher.publish(msg)
                        else:
                        # self.get_logger().info(f"Detected marker: {id}\nRotation : {rvec}\nTranslation : {tvec}")

                            transform_built = np.array([[0,0,1,0],[-1,0,0,0],[0,-1,0,0],[0,0,0,1]]) # to fix frame transform between opencv_camera_frame and camera_link 

                            cv2.drawFrameAxes(rgb_image,self.camera_matrix,self.dist_coeffs,rvec,tvec,1,3)
                            cv2.aruco.drawDetectedMarkers(rgb_image,corners,ids)
                            
                            # To publish transform
                            transform_matrix = np.array([[0, 0, 0, 0],[0, 0, 0, 0],[0, 0, 0, 0],[0, 0, 0, 1]],dtype=float)
                            rotation_matrix, _ = cv2.Rodrigues(rvec)

                            # convert the matrix to a quaternion
                            transform_matrix[:3,:3] = rotation_matrix
                            link_transformation_mat = np.dot(transform_matrix,transform_built)
                            link_quaternion = tf_transformations.quaternion_from_matrix(link_transformation_mat)
                            link_tvec = np.dot(transform_built[:3,:3],tvec)

                            # inv_quaternion = np.array([-1*link_quaternion[0],-1*link_quaternion[1],-1*link_quaternion[2],link_quaternion[3]])
                            # inv_rotation_matrix = tf_transformations.quaternion_matrix(inv_quaternion)
                            # inv_tvec = -np.dot(inv_rotation_matrix[:3,:3],link_tvec)

                            transform = TransformStamped()
                            transform.header.stamp = self.get_clock().now().to_msg()
                            transform.header.frame_id = "<put_camera_frame_id>"
                            transform.child_frame_id = f"marker_{id[0]}_frame"
                            
                            transform.transform.translation.x = float(link_tvec[0])
                            transform.transform.translation.y = float(link_tvec[1])
                            transform.transform.translation.z = float(link_tvec[2])
                            transform.transform.rotation.x = float(link_quaternion[0])
                            transform.transform.rotation.y = float(link_quaternion[1])
                            transform.transform.rotation.z = float(link_quaternion[2])
                            transform.transform.rotation.w = float(link_quaternion[3])

                            # tf_message.transforms.append(transform)
                            # self.tf_publisher.publish(tf_message)

                            self.transform_publisher.publish(transform)
                            self.broacaster.sendTransform(transform)
                    else:
                        self.get_logger().error(f"Unable to estimate pose of markers")  

                ros_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding="bgr8")  # Convert the OpenCV image to a ROS Image message                          
                self.video_publisher.publish(ros_msg)
            else:        
                self.video_publisher.publish(msg)
        except CvBridgeError as e:
            self.get_logger().error(f'Error converting ROS msg to cv2 image: {e}')
            return  

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