#include "rclcpp/rclcpp.hpp"
#include "my_client_pkg/srv/add_two_ints.hpp"  // Replace with your custom service definition
#include <memory>

using AddTwoInts = my_client_pkg::srv::AddTwoInts;

class AddTwoIntsClient : public rclcpp::Node
{
public:
  AddTwoIntsClient() : Node("add_two_ints_client")
  {
    // Create the client for the service
    client_ = this->create_client<AddTwoInts>("add_two_ints");

    // Ensure the service is available before making the request
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
    }
  }

  // Method to send the request
  void send_request(int64_t a, int64_t b)
  {
    // Create a shared pointer for the request object
    auto request = std::make_shared<AddTwoInts::Request>();
    request->a = a;
    request->b = b;

    // Send the request asynchronously and assign the callback for the response
    client_->async_send_request(request, std::bind(&AddTwoIntsClient::response_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Request sent: a=%ld, b=%ld", a, b);
  }

private:
  // Callback method that handles the response
  void response_callback(rclcpp::Client<AddTwoInts>::SharedFuture future_response)
  {
    // Extract the response
    auto response = future_response.get();
    RCLCPP_INFO(this->get_logger(), "Received response: sum = %ld", response->sum);
  }

  // Client object for the service
  rclcpp::Client<AddTwoInts>::SharedPtr client_;
};

int main(int argc, char **argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create an instance of the client node
  auto client_node = std::make_shared<AddTwoIntsClient>();

  // Send a request with example values (you can call this method multiple times if needed)
  client_node->send_request(5, 3);  // Replace with actual values you want to use

  // Spin the node to process the callback
  rclcpp::spin(client_node);

  // Shutdown ROS 2 when done
  rclcpp::shutdown();
  return 0;
}
