/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */
#include "detector_cpp/detector.hpp"

int main(int argc, char ** argv)
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto core_detector = std::make_shared<Detector>(options);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(core_detector);
  exec.spin();

  rclcpp::shutdown();
}