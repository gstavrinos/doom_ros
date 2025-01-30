// #include <doomgeneric/doomgeneric.h>
// #include "doomgeneric.h"
#include <rclcpp/rclcpp.hpp>
// TODO: image msg

// TODO: Integrate doomgeneric in CMakeLists
// TODO: (Use the following link?)
// https://github.com/byteduck/duckos-doom/blob/master/CMakeLists.txt
class DOOMROS : public rclcpp::Node
{
public:
  DOOMROS() : Node("doom_ros")
  {
    RCLCPP_INFO(this->get_logger(), "TODO!!");
    RCLCPP_INFO(this->get_logger(), "TODO!!");
    RCLCPP_INFO(this->get_logger(), "TODO!!");
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DOOMROS>());
  rclcpp::shutdown();
  return 0;
}
