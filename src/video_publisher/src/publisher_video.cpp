#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <functional>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

using namespace std::chrono_literals;

class VideoPublisher : public rclcpp::Node {
public:
  VideoPublisher() : Node("video_publisher"), count_(0) {
    publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>("video_frames", 10);

    video_cap_.open(
        "/home/cw/opencv_learning/ArmorDetector_ros2/R2.mp4");

    if (!video_cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "无法打开视频文件！");
      return;
    }
    //获取视频帧率
    double fps = video_cap_.get(cv::CAP_PROP_FPS);
    auto frame_duration = std::chrono::duration<double>(1.0 / fps);

    // 创建定时器，根据视频帧率发布帧
    timer_ = this->create_wall_timer(
        frame_duration, std::bind(&VideoPublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    cv::Mat frame;
    video_cap_ >> frame; // 读取下一帧

    // 循环播放
    if (frame.empty()) {
      video_cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
      video_cap_ >> frame;
    }

    if (!frame.empty()) {
      // 将opencv矩阵转换为ros2图像消息
      auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame)
                     .toImageMsg();

      msg->header.stamp = this->now();
      msg->header.frame_id = "video_frame";

      RCLCPP_INFO(this->get_logger(), "发布视频帧 %zu", count_++);
      publisher_->publish(*msg);
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  cv::VideoCapture video_cap_;
  size_t count_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VideoPublisher>());
  rclcpp::shutdown();
  return 0;
}