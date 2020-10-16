#include "rclcpp/rclcpp.hpp"
#include "interface/srv/gate.hpp"

#include <memory>

void movements(const std::shared_ptr<interface::srv::Gate::Request> request,
          std::shared_ptr<interface::srv::Gate::Response>      response)
{ 
  switch(request->position){
    case 1:
      response->movement = "Move Up then Move Left.";
      break;
    case 2:
      response->movement = "Move Up.";
      break;
    case 3:
      response->movement = "Move Up then Move Right.";
      break;
    case 4:
      response->movement = "Move Left.";
      break;
    case 5:
      response->movement = "You are in the Center.";
      break;
    case 6:
      response->movement = "Move Right.";
      break;
    case 7:
      response->movement = "Move Down then Move Left.";
      break;
    case 8:
      response->movement = "Move Down.";
      break;
    case 9:
      response->movement = "Move Down then Move Right.";
      break;
    default:
      response->movement = "Gate is out of region.";
      break;

  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Receiving Request - Gate is at Position: %d",
                request->position);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending - Movements: %s", response->movement.c_str());
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("motion_planning");

  rclcpp::Service<interface::srv::Gate>::SharedPtr service =
node->create_service<interface::srv::Gate>("gate_location", &movements);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}