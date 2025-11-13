#include "LIVMapper.h"

#include <spdlog/spdlog.h>

int main(int argc, char **argv)
{
  spdlog::set_level(spdlog::level::info);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "fast_livo", rclcpp::NodeOptions()
  );
  image_transport::ImageTransport it(node);
  LIVMapper mapper(node); 
  mapper.initializeSubscribersAndPublishers(it);
  mapper.run();
  rclcpp::shutdown();
  return 0;
}