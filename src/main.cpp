#include "LIVMapper.h"

#include <spdlog/spdlog.h>

int main(int argc, char **argv) {
#ifdef ENABLE_PERFORMANCE_TIMING
  spdlog::set_level(spdlog::level::debug);
#else
  spdlog::set_level(spdlog::level::info); // Reduce noise; timers still warn
#endif
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "fast_livo", rclcpp::NodeOptions()
  );
  LIVMapper mapper(node); 
  mapper.initializeSubscribersAndPublishers();
  mapper.run();
  rclcpp::shutdown();
  return 0;
}