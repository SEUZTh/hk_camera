#include <chrono>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "hk_camera_interfaces/srv/take_photo.hpp"  // 替换为自定义服务消息的路径

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // 创建节点
  auto node = rclcpp::Node::make_shared("image_client");

  // 创建服务客户端
  auto client = node->create_client<hk_camera_interfaces::srv::TakePhoto>("save_image");

  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Service not available, waiting...");
  }

  // 创建请求
  auto request = std::make_shared<hk_camera_interfaces::srv::TakePhoto::Request>();
  request->save_path = "/home/zth/Pictures/image.jpg"; // 设置图像保存路径

  // 发送请求并等待响应
  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = future.get();
    if (response->success) {
      RCLCPP_INFO(node->get_logger(), response->message);
    } else {
      RCLCPP_ERROR(node->get_logger(), response->message);
    }
  } else {
    RCLCPP_ERROR(node->get_logger(), "Service call failed");
  }

  rclcpp::shutdown();
  return 0;
}
