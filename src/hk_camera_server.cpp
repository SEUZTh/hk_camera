#include <chrono>
#include <memory>
#include <string>
#include <fstream>

#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "hk_camera_interfaces/srv/take_photo.hpp" // 替换为自定义服务消息的路径

#include "hk_camera.hpp"

class ImageServerNode : public rclcpp::Node
{
public:
  ImageServerNode()
      : Node("image_server"), MVS_cap(*this)
  {
    // 创建服务
    image_service_ = create_service<hk_camera_interfaces::srv::TakePhoto>(
        "save_image",
        std::bind(&ImageServerNode::handle_image_service_request, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void handle_image_service_request(
      const std::shared_ptr<hk_camera_interfaces::srv::TakePhoto::Request> request,
      std::shared_ptr<hk_camera_interfaces::srv::TakePhoto::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Received image request");

    // 将收到的图像数据写入文件
    cv::Mat img;
    std::string file_path = request->save_path;

    MVS_cap.ReadImg(img);
    std::cout << "call service" << std::endl;
    if (!img.empty())
    {
      cv::imwrite(file_path, img);

      response->success = true;
      response->message = "Image saved to " + file_path;
      RCLCPP_INFO(this->get_logger(), "Image saved to %s", file_path.c_str());
    }
    else
    {
      response->success = false;
      response->message = "Failed to save image.";
      RCLCPP_ERROR(this->get_logger(), "Failed to save image.");
    }
  }

  rclcpp::Service<hk_camera_interfaces::srv::TakePhoto>::SharedPtr image_service_;
  camera::Camera MVS_cap;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageServerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
