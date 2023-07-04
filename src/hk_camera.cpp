#include <iostream>
#include "opencv2/opencv.hpp"
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "hk_camera.hpp"

// 剪裁掉照片和雷达没有重合的视角，去除多余像素可以使rosbag包变小
#define FIT_LIDAR_CUT_IMAGE false
#if FIT_LIDAR_CUT_IMAGE
    #define FIT_min_x 420
    #define FIT_min_y 70
    #define FIT_max_x 2450
    #define FIT_max_y 2000
#endif 

using namespace std;
using namespace cv;


int main(int argc, char **argv)
{
    //********** variables    **********/
    cv::Mat src;
    //string src = "",image_pub = "";
    //********** rosnode init **********/
    rclcpp::init(argc, argv);
    auto hk_camera = std::make_shared<rclcpp::Node>("hk_camera");
    camera::Camera MVS_cap(*hk_camera);
    //********** rosnode init **********/
    image_transport::ImageTransport main_cam_image(hk_camera);
    image_transport::CameraPublisher image_pub = main_cam_image.advertiseCamera("/hk_camera/rgb/compressed", 1000);

    sensor_msgs::msg::Image image_msg;
    sensor_msgs::msg::CameraInfo camera_info_msg;
    // cv_bridge::CvImagePtr cv_ptr = boost::make_shared<cv_bridge::CvImage>();
    cv_bridge::CvImagePtr cv_ptr = std::make_shared<cv_bridge::CvImage>();
    cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;  // 就是rgb格式 
    
    //********** 10 Hz        **********/
    rclcpp::Rate loop_rate(10);

    while (rclcpp::ok())
    {

        loop_rate.sleep();
        rclcpp::spin_some(hk_camera);

        MVS_cap.ReadImg(src);
        if (src.empty())
        {
            continue;
        }
#if FIT_LIDAR_CUT_IMAGE
        cv::Rect area(FIT_min_x,FIT_min_y,FIT_max_x-FIT_min_x,FIT_max_y-FIT_min_y); // cut区域：从左上角像素坐标x，y，宽，高
        cv::Mat src_new = src(area);
        cv_ptr->image = src_new;
#else
        cv_ptr->image = src;
#endif
        image_msg = *(cv_ptr->toImageMsg());
        image_msg.header.stamp = hk_camera->get_clock()->now();  // ros发出的时间不是快门时间
        image_msg.header.frame_id = "hk_camera";

        camera_info_msg.header.frame_id = image_msg.header.frame_id;
	    camera_info_msg.header.stamp = image_msg.header.stamp;
        image_pub.publish(image_msg, camera_info_msg);

        //*******************************************************************************************************************/
    }
    rclcpp::shutdown();
    
    return 0;
}
