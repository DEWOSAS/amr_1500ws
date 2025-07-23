#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class MapAsImageProvider : public rclcpp::Node
{
public:
  MapAsImageProvider()
  : Node("map_to_image_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // Subscribers
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 10, std::bind(&MapAsImageProvider::mapCallback, this, std::placeholders::_1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&MapAsImageProvider::scanCallback, this, std::placeholders::_1));

    // Image publisher
    image_pub_ = image_transport::create_publisher(this, "map_image/full");

    // Timer
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&MapAsImageProvider::processAndPublish, this));

    RCLCPP_INFO(this->get_logger(), "Map to image node started.");
  }

private:
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  sensor_msgs::msg::LaserScan::SharedPtr scan_;
  geometry_msgs::msg::TransformStamped tf_map_robot_;
  bool recv_tf_ = false;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  image_transport::Publisher image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_ = msg;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    scan_ = msg;
  }

  void readTransform()
  {
    try
    {
      rclcpp::Time time;
      if (map_) {
        time = map_->header.stamp;
      } else if (scan_) {
        time = scan_->header.stamp;
      } else {
        RCLCPP_WARN(this->get_logger(), "No timestamp available for TF lookup.");
        return;
      }

      tf_map_robot_ = tf_buffer_.lookupTransform(
        "map", "base_link", time, tf2::durationFromSec(0.3));
      recv_tf_ = true;
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
    }
  }

  void processAndPublish()
  {
    if (!map_ || !scan_)
      return;

    readTransform();
    if (!recv_tf_)
      return;

    const auto &info = map_->info;
    cv::Mat image(info.height, info.width, CV_8UC3, cv::Scalar(255, 255, 255));

    for (size_t y = 0; y < info.height; ++y)
    {
      for (size_t x = 0; x < info.width; ++x)
      {
        int i = x + (info.height - y - 1) * info.width;
        int8_t val = map_->data[i];

        if (val == 0)
          image.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);
        else if (val == 100)
          image.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
        else
          image.at<cv::Vec3b>(y, x) = cv::Vec3b(128, 128, 128);
      }
    }

    // พล็อตตำแหน่งหุ่นยนต์
    float robot_x = tf_map_robot_.transform.translation.x;
    float robot_y = tf_map_robot_.transform.translation.y;

    int robot_img_x = (robot_x - info.origin.position.x) / info.resolution;
    int robot_img_y = info.height - (robot_y - info.origin.position.y) / info.resolution;

    if (robot_img_x >= 0 && robot_img_x < image.cols &&
        robot_img_y >= 0 && robot_img_y < image.rows)
    {
      cv::circle(image, cv::Point(robot_img_x, robot_img_y), 5, cv::Scalar(0, 0, 255), -1);
    }

    // พล็อต laser scan
    float angle = scan_->angle_min;
    for (const auto &r : scan_->ranges)
    {
      if (std::isfinite(r))
      {
        float x = r * cos(angle);
        float y = r * sin(angle);

        tf2::Vector3 pt(x, y, 0.0);
        tf2::Transform tf;
        tf.setOrigin(tf2::Vector3(tf_map_robot_.transform.translation.x,
                                  tf_map_robot_.transform.translation.y,
                                  0.0));
        tf.setRotation(tf2::Quaternion(
          tf_map_robot_.transform.rotation.x,
          tf_map_robot_.transform.rotation.y,
          tf_map_robot_.transform.rotation.z,
          tf_map_robot_.transform.rotation.w));

        tf2::Vector3 map_point = tf * pt;

        int px = (map_point.x() - info.origin.position.x) / info.resolution;
        int py = info.height - (map_point.y() - info.origin.position.y) / info.resolution;

        if (px >= 0 && px < image.cols && py >= 0 && py < image.rows)
        {
          image.at<cv::Vec3b>(py, px) = cv::Vec3b(0, 255, 0);
        }
      }
      angle += scan_->angle_increment;
    }

    // แปลงเป็น Image message และ publish
    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = "map";

    sensor_msgs::msg::Image::SharedPtr img_msg =
      cv_bridge::CvImage(header, "bgr8", image).toImageMsg();

    image_pub_.publish(img_msg);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapAsImageProvider>());
  rclcpp::shutdown();
  return 0;
}
