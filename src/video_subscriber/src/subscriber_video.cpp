#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
using std::placeholders::_1;

// 灯条描述类
class LightDescriptor {
public:
  float width, length, angle, area;
  cv::Point2f center;

  LightDescriptor(){};
  LightDescriptor(const cv::RotatedRect &light) {
    width = light.size.width;
    length = light.size.height;
    center = light.center;
    angle = light.angle;
    area = light.size.area();
  }
};

class VideoSubscriber : public rclcpp::Node {
public:
  VideoSubscriber() : Node("video_subscriber") {
    // 订阅原始视频帧
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "video_frames", 10,
        std::bind(&VideoSubscriber::image_callback, this, _1));

    // 发布处理后的视频帧
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "processed_video_frames", 10);

    RCLCPP_INFO(this->get_logger(), "视频订阅节点已启动");
  }

private:
  // 对点进行排序（左上、右上、右下、左下）
  void sortPoints(vector<Point2f> &points) {
    if (points.size() != 4)
      return;

    // 计算中心点
    Point2f center(0, 0);
    for (const auto &p : points) {
      center += p;
    }
    center /= 4.0;

    // 排序点
    vector<Point2f> sorted(4);
    for (const auto &p : points) {
      if (p.x < center.x && p.y < center.y)
        sorted[0] = p; // 左上
      else if (p.x > center.x && p.y < center.y)
        sorted[1] = p; // 右上
      else if (p.x > center.x && p.y > center.y)
        sorted[2] = p; // 右下
      else
        sorted[3] = p; // 左下
    }

    points = sorted;
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      // 将ROS图像消息转换为OpenCV图像
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      Mat frame = cv_ptr->image;

      if (frame.empty()) {
        RCLCPP_WARN(this->get_logger(), "接收到空帧");
        return;
      }

      Mat hsv, red_mask1, red_mask2, red_mask, binary, Gaussian, dilatee;
      Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
      vector<vector<Point>> contours;
      vector<Vec4i> hierarchy;

      // 转换为HSV颜色空间，更好地识别红色
      cvtColor(frame, hsv, COLOR_BGR2HSV);

      // 定义红色在HSV空间中的范围（红色有两个范围，因为红色在HSV色环的两端）
      Scalar lower_red1(0, 100, 100);   // 较低范围的红色
      Scalar upper_red1(10, 255, 255);  // 较高范围的红色
      Scalar lower_red2(160, 100, 100); // 较高范围的红色
      Scalar upper_red2(179, 255, 255); // 较高范围的红色

      // 创建红色掩码
      inRange(hsv, lower_red1, upper_red1, red_mask1);
      inRange(hsv, lower_red2, upper_red2, red_mask2);

      // 合并两个红色掩码
      red_mask = red_mask1 | red_mask2;

      // 形态学操作，去除噪声并连接红色区域
      GaussianBlur(red_mask, Gaussian, Size(5, 5), 0);
      dilate(Gaussian, dilatee, element);

      findContours(dilatee, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
      vector<LightDescriptor> lightInfos;

      // 筛选灯条
      for (int i = 0; i < contours.size(); i++) {
        double area = contourArea(contours[i]);
        if (area < 5 || contours[i].size() <= 1)
          continue;

        RotatedRect Light_Rec = fitEllipse(contours[i]);
        if (Light_Rec.size.width / Light_Rec.size.height > 4)
          continue;

        lightInfos.push_back(LightDescriptor(Light_Rec));
      }

      // 二重循环多条件匹配灯条
      for (size_t i = 0; i < lightInfos.size(); i++) {
        for (size_t j = i + 1; (j < lightInfos.size()); j++) {
          LightDescriptor &leftLight = lightInfos[i];
          LightDescriptor &rightLight = lightInfos[j];

          float angleGap_ = abs(leftLight.angle - rightLight.angle);
          float LenGap_ratio = abs(leftLight.length - rightLight.length) /
                               max(leftLight.length, rightLight.length);
          float dis =
              pow(pow((leftLight.center.x - rightLight.center.x), 2) +
                      pow((leftLight.center.y - rightLight.center.y), 2),
                  0.5);
          float meanLen = (leftLight.length + rightLight.length) / 2;
          float lengap_ratio =
              abs(leftLight.length - rightLight.length) / meanLen;
          float yGap = abs(leftLight.center.y - rightLight.center.y);
          float yGap_ratio = yGap / meanLen;
          float xGap = abs(leftLight.center.x - rightLight.center.x);
          float xGap_ratio = xGap / meanLen;
          float ratio = dis / meanLen;

          // 匹配条件
          if (angleGap_ > 15 || LenGap_ratio > 1.0 || lengap_ratio > 0.8 ||
              yGap_ratio > 1.5 || xGap_ratio > 2.2 || xGap_ratio < 0.8 ||
              ratio > 3 || ratio < 0.8) {
            continue;
          }

          // 计算装甲板角点
          Point center = Point((leftLight.center.x + rightLight.center.x) / 2,
                               (leftLight.center.y + rightLight.center.y) / 2);
          RotatedRect rect =
              RotatedRect(center, Size(dis, meanLen),
                          (leftLight.angle + rightLight.angle) / 2);

          Point2f vertices[4];
          rect.points(vertices);

          // 将点转换为向量并排序
          vector<Point2f> points(vertices, vertices + 4);
          sortPoints(points);

          // 绘制装甲板边界
          for (int k = 0; k < 4; k++) {
            line(frame, points[k], points[(k + 1) % 4], Scalar(0, 0, 255), 2);
          }

          // 可视化角点并标注序号
          for (int k = 0; k < 4; k++) {
            // 绘制角点
            circle(frame, points[k], 6, Scalar(0, 255, 0), -1);

            // 标注角点序号 (1, 2, 3, 4)
            string point_num = to_string(k + 1);
            putText(frame, point_num, points[k] + Point2f(-5, 5),
                    FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0), 2);
          }
        }
      }

      // 将处理后的图像转换回ROS消息并发布
      auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame)
                         .toImageMsg();

      img_msg->header.stamp = this->now();
      img_msg->header.frame_id = "processed_frame";

      publisher_->publish(*img_msg);
      RCLCPP_INFO(this->get_logger(), "发布处理后的视频帧");

    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge异常: %s", e.what());
      return;
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VideoSubscriber>());
  rclcpp::shutdown();
  return 0;
}