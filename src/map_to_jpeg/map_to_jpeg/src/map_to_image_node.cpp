#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class MapToImageNode : public rclcpp::Node {
public:
  MapToImageNode() : Node("map_to_image_node") {
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "pose", 10, std::bind(&MapToImageNode::poseCallback, this, std::placeholders::_1));

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 10, std::bind(&MapToImageNode::mapCallback, this, std::placeholders::_1));

    image_pub_ = image_transport::create_publisher(this, "map_image/full");

    RCLCPP_INFO(this->get_logger(), "MapToImageNode started.");
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    pose_ = msg;
  }

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
    if (!pose_) return;

    int width = map->info.width;
    int height = map->info.height;

    cv::Mat map_image(height, width, CV_8UC3);
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        int idx = (height - y - 1) * width + x;
        int8_t val = map->data[idx];
        uchar pixel;
        if (val == -1) pixel = 127;
        else if (val == 0) pixel = 255;
        else pixel = 0;
        map_image.at<Vec3b>(y, x) = Vec3b(pixel, pixel, pixel);
      }
    }

    int px = static_cast<int>((pose_->pose.position.x - map->info.origin.position.x) / map->info.resolution);
    int py = static_cast<int>((pose_->pose.position.y - map->info.origin.position.y) / map->info.resolution);
    py = height - py;

    double yaw = tf2::getYaw(pose_->pose.orientation);
    int radius = width / 100;
    circle(map_image, Point(px, py), radius, Scalar(255, 0, 0), 2);
    line(map_image, Point(px, py),
         Point(px + radius * cos(yaw), py - radius * sin(yaw)),
         Scalar(255, 0, 0), 2);

    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", map_image).toImageMsg();
    msg->header.frame_id = "map";
    image_pub_.publish(msg);
  }

  image_transport::Publisher image_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  geometry_msgs::msg::PoseStamped::SharedPtr pose_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapToImageNode>());
  rclcpp::shutdown();
  return 0;
}