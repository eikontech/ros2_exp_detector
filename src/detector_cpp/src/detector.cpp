#include "detector_cpp/detector.hpp"

Detector::Detector(const rclcpp::NodeOptions &options)
    : Node("detector", options)
{
    rclcpp::QoS l_detection_image_publisher_qos(10);
    _detectionImagePublisher = image_transport::create_publisher(this, "detection_image", l_detection_image_publisher_qos.get_rmw_qos_profile());
    RCLCPP_INFO_STREAM(this->get_logger(), "Publish detection image on topic: " << _detectionImagePublisher.getTopic());

    rclcpp::QoS l_tracks_publisher_qos(10);
    _tracksPublisher = this->create_publisher<darknet_ros_msgs::msg::TrackedObjectsStamped>("tracks", l_tracks_publisher_qos);
    RCLCPP_INFO_STREAM(this->get_logger(), "Publish tracked object on topic: " << _tracksPublisher->get_topic_name());

    using std::placeholders::_1;
    using std::placeholders::_2;
    rmw_qos_profile_t l_custom_qos_profile = rmw_qos_profile_sensor_data;
    _imageSubscriber = image_transport::create_subscription(
        this, "image",
        std::bind(&Detector::cameraCallback, this, _1),
        "raw",
        l_custom_qos_profile);

    RCLCPP_INFO_STREAM(this->get_logger(), "Subscribed on topic: " << _imageSubscriber.getTopic());
}

/////////////////////////////////////////////////////////////////////////////////////////
void Detector::cameraCallback(const sensor_msgs::msg::Image::ConstSharedPtr &image_)
{
    RCLCPP_DEBUG_STREAM(get_logger(), __FUNCTION__ << " image received.");

    cv_bridge::CvImageConstPtr cam_image;

    try
    {
        cam_image = cv_bridge::toCvShare(image_, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), __FUNCTION__ << " cv_bridge exception: " << e.what());
        return;
    }

    if (cam_image)
    {
        cv::Size l_frame_size = cam_image->image.size();

        static bool first_time = true;
        if (first_time)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), __FUNCTION__ << " Video size: " << l_frame_size);
            first_time = false;
        }

        //// TODO do things
    }
    else
    {
        RCLCPP_WARN_STREAM(this->get_logger(), __FUNCTION__ << " no image in cap!");
    }

    return;
}

//////////////////////////////////////////////////////////////////////////////
bool Detector::publishDetectionImage(const cv::Mat &detectionImage, const std_msgs::msg::Header &header_)
{
    if (_detectionImagePublisher.getNumSubscribers() < 1)
        return false;

    cv_bridge::CvImage cvImage;

    cvImage.header = header_;

    cvImage.encoding = "bgr8";
    cvImage.image = detectionImage;
    _detectionImagePublisher.publish(*cvImage.toImageMsg());
    RCLCPP_DEBUG(get_logger(), "Detection image has been published.");
    return true;
}

//////////////////////////////////////////////////////////////////////////////
bool Detector::publishTrackedObjects(darknet_ros_msgs::msg::TrackedObjects const &tracks_, const std_msgs::msg::Header &header_)
{
    if (_tracksPublisher->get_subscription_count() < 1)
        return false;

    darknet_ros_msgs::msg::TrackedObjectsStamped l_tracked_object_msg;
    l_tracked_object_msg.header = header_;

    l_tracked_object_msg.tracked_objects = tracks_.tracked_objects;

    _tracksPublisher->publish(l_tracked_object_msg);
    RCLCPP_DEBUG(get_logger(), "Tracked objects have been published.");
    return true;
}