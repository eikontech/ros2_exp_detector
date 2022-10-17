from rclpy.node import Node

from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import TrackedObjectsStamped

class Detector(Node):

    def __init__(self):
        super().__init__('detector_py')

        # Publisher of detection image
        self.detectionImagePublisher_ = self.create_publisher(
            Image,
            "detection_image",
            10)
        self.detectionImagePublisher_
        self.get_logger().info(f"Detection Image publisher topic name {self.detectionImagePublisher_.topic_name}")

        # Publisher of tracked object
        self.tracksPublisher_ = self.create_publisher(
            TrackedObjectsStamped,
            "tracks",
            10)
        self.tracksPublisher_
        self.get_logger().info(f"Tracks publisher topic name {self.tracksPublisher_.topic_name}")

        # Subscriber for recieve AdultUpadets msg
        self.imageSubscriber_ = self.create_subscription(
            Image,
            'image',
            self.camera_callback,
            10)
        self.imageSubscriber_       # prevent unused variable warning
        self.get_logger().info(f"Image subscription topic name {self.imageSubscriber_.topic_name}")

    def camera_callback(self, image_):
        
        if image_:
            # TODO detection
            l_detection_image = Image()
            l_tracks = TrackedObjectsStamped()

            if self.detectionImagePublisher_.get_subscription_count() > 0:
                self.detectionImagePublisher_.publish(l_detection_image)

            if self.tracksPublisher_.get_subscription_count() > 0:
                self.tracksPublisher_.publish(l_tracks)


