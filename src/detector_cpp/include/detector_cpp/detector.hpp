#include <chrono>
#include <functional>
#include <memory>
#include <string>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"

#include <cv_bridge/cv_bridge.h>

// darknet_ros
#include "darknet_ros_msgs/msg/tracked_objects_stamped.hpp"
#include "darknet_ros_msgs/msg/tracked_objects.hpp"

class Detector : public rclcpp::Node
{
public:
    Detector(const rclcpp::NodeOptions &options);

private:
    /*!
     * Callback of camera.
     * @param[in] msg image pointer.
     */
    void cameraCallback(const sensor_msgs::msg::Image::ConstSharedPtr &image_);

    /*!
     * Publishes the detection image.
     * @return true if successful.
     */
    bool publishDetectionImage(const cv::Mat &detectionImage, const std_msgs::msg::Header &header_);

    /*!
     * Publishes the tracked objects.
     * @return true if successful.
     */
    bool publishTrackedObjects(darknet_ros_msgs::msg::TrackedObjects const &tracks_, const std_msgs::msg::Header &header_);

private:

    //! ROS subscriber and publisher.
    image_transport::Subscriber _imageSubscriber;

    //! Publisher of the bounding box image.
    image_transport::Publisher _detectionImagePublisher;

    //! Publisher of the tracked objects.
    rclcpp::Publisher<darknet_ros_msgs::msg::TrackedObjectsStamped>::SharedPtr _tracksPublisher;
};