#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <opencv2/opencv.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

using namespace std;
using namespace cv;

class MapToImageNode : public rclcpp::Node {
public:
  MapToImageNode() : Node("map_to_image_node"),
                    tf_buffer_(this->get_clock()),
                    tf_listener_(tf_buffer_) {

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "map", 10, std::bind(&MapToImageNode::mapCallback, this, std::placeholders::_1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "scan", rclcpp::SensorDataQoS(),
                std::bind(&MapToImageNode::lasercallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      55ms, std::bind(&MapToImageNode::publishMapWithScan, this));

    image_pub_ = image_transport::create_publisher(this, "map_image/full");
    
    RCLCPP_INFO(this->get_logger(), "MapToImageNode started.");
  }

void lasercallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    scan_ = msg;
    RCLCPP_INFO(this->get_logger(), "LaserScan received: %ld ranges", msg->ranges.size());
  }

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
    current_map_ = map;
    RCLCPP_WARN(this->get_logger(), "Received map data");
  }

  void publishMapWithScan() {
    // พยายามดึง transform
    if (!current_map_) {
      RCLCPP_WARN(this->get_logger(), "No map data received yet.");
      return;
    }
    try {
      transform_stamped_ = tf_buffer_.lookupTransform(
          "map", "base_link", tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
      return;
    }

    // สร้างภาพแผนที่จากข้อมูล map
    int width = current_map_->info.width;
    int height = current_map_->info.height;
    cv::Mat map_image(height, width, CV_8UC3);

    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        int idx = (height - y - 1) * width + x;
        int8_t val = current_map_->data[idx];
        uchar pixel;
        if (val == -1) pixel = 127;
        else if (val == 0) pixel = 255;
        else pixel = 0;
        map_image.at<cv::Vec3b>(y, x) = cv::Vec3b(pixel, pixel, pixel);
      }
    }

    // วาดตำแหน่ง robot
    double x = transform_stamped_.transform.translation.x;
    double y = transform_stamped_.transform.translation.y;
    int px = static_cast<int>((x - current_map_->info.origin.position.x) / current_map_->info.resolution);
    int py = static_cast<int>((y - current_map_->info.origin.position.y) / current_map_->info.resolution);
    py = height - py;

    if (px < 0 || px >= width || py < 0 || py >= height) {
      RCLCPP_WARN(this->get_logger(), "TF pose is out of map bounds: (px=%d, py=%d)", px, py);
      return;
    }

    tf2::Quaternion q(
      transform_stamped_.transform.rotation.x,
      transform_stamped_.transform.rotation.y,
      transform_stamped_.transform.rotation.z,
      transform_stamped_.transform.rotation.w);
    double yaw = tf2::getYaw(q);

    int radius = width / 100;
    cv::circle(map_image, cv::Point(px, py), radius, cv::Scalar(255, 0, 0), 2);
    cv::line(map_image, cv::Point(px, py),
         cv::Point(px + radius * cos(yaw), py - radius * sin(yaw)),
         cv::Scalar(255, 0, 0), 2);

    // วาด LaserScan จุดสีเขียว
    float angle = scan_->angle_min;
    for (const auto &r : scan_->ranges)
    {
      if (std::isfinite(r))
      {
        float lx = r * cos(angle);
        float ly = r * sin(angle);

        tf2::Vector3 pt_laser(lx, ly, 0.0);

        tf2::Quaternion q(
          transform_stamped_.transform.rotation.x,
          transform_stamped_.transform.rotation.y,
          transform_stamped_.transform.rotation.z,
          transform_stamped_.transform.rotation.w);
        tf2::Transform tf_map_base(q,
          tf2::Vector3(transform_stamped_.transform.translation.x,
                       transform_stamped_.transform.translation.y,
                       transform_stamped_.transform.translation.z));

        tf2::Vector3 pt_map = tf_map_base * pt_laser;

        int px = static_cast<int>((pt_map.x() - current_map_->info.origin.position.x) / current_map_->info.resolution);
        int py = static_cast<int>((pt_map.y() - current_map_->info.origin.position.y) / current_map_->info.resolution);
        py = height - py;

        if (px >= 0 && px < width && py >= 0 && py < height)
        {
            cv::circle(map_image, cv::Point(px, py), 2, cv::Scalar(0, 255, 0), -1);  // จุดสีเขียว
        }
      }
      angle += scan_->angle_increment;
    }

    // แปลงภาพเป็น ROS message และ publish
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", map_image).toImageMsg();
    msg->header.frame_id = "map";
    msg->header.stamp = this->get_clock()->now();
    image_pub_.publish(msg);

    RCLCPP_INFO(this->get_logger(), "Published map image with TF pose.");
  }

  image_transport::Publisher image_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  sensor_msgs::msg::LaserScan::SharedPtr scan_;
  nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;

  geometry_msgs::msg::TransformStamped transform_stamped_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapToImageNode>());
  rclcpp::shutdown();
  return 0;
}
