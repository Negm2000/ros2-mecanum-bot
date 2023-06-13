#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class GUI_Subscriber : public rclcpp::Node
{
  public:
    
    GUI_Subscriber() : Node("gui_subscriber"),
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "GUI_Command", 1, std::bind(&GUI_Subscriber::topic_callback, this, _1));
    }

  private:
    std::string topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
    
    }   
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<GUI_Subscriber>());
//   rclcpp::shutdown();
//   return 0;
// }