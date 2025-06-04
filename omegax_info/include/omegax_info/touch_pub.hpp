#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <omegax_info/msg/touch.hpp>
 double masterX, masterY, masterZ;
 double masterRotation[3][3] = {};
namespace touch{
  class publisher : public rclcpp::Node {
    private:
      rclcpp::Publisher<omegax_info::msg::Touch>::SharedPtr touch_pub;
      
    public:
     
      publisher(const std::string& name);
      void publish(double x, double y, double z, double roll, double pitch, double yaw, bool button1, bool button2);
  
  
  };

}
