#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Joystick : public rclcpp::Node
{
  public:
    Joystick(): Node("Joystick")
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("/joystick", 10);
    }

    void start(){
        std_msgs::msg::String s;
        while (rclcpp::ok())
        {
        std::cout<<"Enter key to send to rexy: "<<std::endl;
        std::cin>>s.data;
        publisher_->publish(s);
        }
        
    }
  private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  Joystick joystick;
  joystick.start();
  rclcpp::shutdown();
  return 0;
}