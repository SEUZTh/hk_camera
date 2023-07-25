#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"

// 回调函数定义，处理接收到的压缩图像消息
void imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    try
    {
        // 将压缩图像消息解码为cv::Mat格式
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        // 在这里可以对接收到的图像进行处理
        cv::Mat image = cv_ptr->image;

        // 在这里添加你的图像处理代码

        // 获取当前时间
        auto current_time = rclcpp::Clock().now();

        // 计算帧率
        static int frame_count = 0;
        static auto last_time = current_time;
        auto elapsed_time = current_time - last_time;
        double frame_rate = 1.0 / elapsed_time.seconds();
        last_time = current_time;
        frame_count++;

        // 在图像右上角显示帧率
        std::stringstream ss;
        ss << "Frame Rate: " << frame_rate << " fps";
        cv::putText(image, ss.str(), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

        // 使用OpenCV窗口显示图像
        cv::imshow("Received Image", image);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("image_subscriber"), "cv_bridge exception: " << e.what());
        return;
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("image_subscriber");

    // 创建图像消息的订阅者，订阅话题为"/hk_camera/rgb/compressed"
    auto subscriber = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        "/hk_camera/rgb/compressed",
        100,
        imageCallback // 回调函数处理接收到的消息
    );

    // 初始化OpenCV窗口
    cv::namedWindow("Received Image", cv::WINDOW_NORMAL);
    cv::resizeWindow("Received Image", 640, 480);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
