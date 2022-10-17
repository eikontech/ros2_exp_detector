import rclpy
from detector_py.src.detector import Detector


def main(args=None):
    rclpy.init(args=args)

    detector = Detector()

    rclpy.spin(detector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()