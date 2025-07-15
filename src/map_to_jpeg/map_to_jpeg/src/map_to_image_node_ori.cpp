#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include <cmath>

using std::placeholders::_1;

class MapAsImageProvider : public rclcpp::Node
{
public:
  MapAsImageProvider() : Node("map_to_image_node"),
                          tf_buffer_(this->get_clock()),
                          tf_listener_(tf_buffer_)
  {
    image_transport::ImageTransport it(shared_from_this());
    image_pub_ = it.advertise("map_image/full", 1);

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 1, std::bind(&MapAsImageProvider::mapCallback, this, _1));
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan_multi", 1, std::bind(&MapAsImageProvider::laserCallback, this, _1));

    cv_img_full_.header.frame_id = "map_image";
    cv_img_full_.encoding = sensor_msgs::image_encodings::RGB8;

    RCLCPP_INFO(this->get_logger(), "Map to Image node started.");
  }

private:
  Eigen::Vector2i worldToMapIndex(const Eigen::Vector2f& world)
  {
    float origin_x = map_->info.origin.position.x;
    float origin_y = map_->info.origin.position.y;
    float res = map_->info.resolution;
    int x = static_cast<int>((world.x() - origin_x) / res);
    int y = static_cast<int>((world.y() - origin_y) / res);
    return Eigen::Vector2i(x, y);
  }

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_ = msg;
    recv_map_ = true;
    size_x_ = map_->info.width;
    size_y_ = map_->info.height;
    map_resolution_ = map_->info.resolution;

    if (size_x_ < 3 || size_y_ < 3) return;

    if (map_ori_.rows != static_cast<int>(size_y_) || map_ori_.cols != static_cast<int>(size_x_)) {
      map_ori_ = cv::Mat(size_y_, size_x_, CV_8UC3);
    }

    auto & data = map_->data;
    unsigned char* map_ori_data_p = map_ori_.data;
    int size_y_rev = size_y_ - 1;

    for (int y = size_y_rev; y >= 0; --y) {
      int idx_map_y = size_x_ * (size_y_ - y);
      int idx_img_y = size_x_ * y;
      for (int x = 0; x < static_cast<int>(size_x_); ++x) {
        int idx = idx_img_y + x;
        switch (data[idx_map_y + x]) {
          case -1:
            map_ori_data_p[idx*3] = 127;
            map_ori_data_p[idx*3+1] = 127;
            map_ori_data_p[idx*3+2] = 127;
            break;
          case 0:
            map_ori_data_p[idx*3] = 255;
            map_ori_data_p[idx*3+1] = 255;
            map_ori_data_p[idx*3+2] = 255;
            break;
          case 100:
            map_ori_data_p[idx*3] = 0;
            map_ori_data_p[idx*3+1] = 0;
            map_ori_data_p[idx*3+2] = 0;
            break;
        }
      }
    }
  }

  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    scan_ = msg;
    recv_scan_ = true;

    if (image_pub_.getNumSubscribers() > 0) {
      clearMap();
      readTransform();
      addPose();
      addLaser();
      pubMap();
    }
  }

  void readTransform()
  {
    try {
      tf_map_robot_ = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
      recv_tf_ = true;
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    }
  }

  void addPose()
  {
    if (!recv_tf_ || !recv_map_) return;
    recv_tf_ = false;

    double x = tf_map_robot_.transform.translation.x;
    double y = tf_map_robot_.transform.translation.y;
    Eigen::Vector2f rob_pos(x, y);
    Eigen::Vector2i rob_pos_pix = worldToMapIndex(rob_pos);

    tf2::Quaternion q;
    tf2::fromMsg(tf_map_robot_.transform.rotation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    pose_yaw_ = yaw;
    pix_pose_x_ = rob_pos_pix[0];
    pix_pose_y_ = rob_pos_pix[1];

    double radius = 0.5 / map_resolution_;
    cv::circle(map_mat_, cv::Point(pix_pose_x_, size_y_ - pix_pose_y_), radius, cv::Scalar(255,0,0), 2);
    cv::line(map_mat_, cv::Point(pix_pose_x_, size_y_ - pix_pose_y_),
             cv::Point(pix_pose_x_ + (radius+4)*cos(pose_yaw_), size_y_ - pix_pose_y_ - (radius+4)*sin(pose_yaw_)),
             cv::Scalar(0,0,255), 2);
  }

  void addLaser()
  {
    if (!recv_scan_ || !recv_map_) return;
    recv_scan_ = false;

    double angle = scan_->angle_min;
    double angle_increment = scan_->angle_increment;
    int laser_size = static_cast<int>((scan_->angle_max - angle) / angle_increment);

    for (int i = 0; i < laser_size; ++i) {
      int laser_x = scan_->ranges[i]*cos(angle+pose_yaw_)/map_resolution_;
      int laser_y = scan_->ranges[i]*sin(angle+pose_yaw_)/map_resolution_;
      cv::circle(map_mat_, cv::Point(pix_pose_x_ + laser_x, size_y_ - (pix_pose_y_ + laser_y)), 0, cv::Scalar(0,255,0), 2);
      angle += angle_increment;
    }
  }

  void pubMap()
  {
    if (!recv_map_) return;
    map_ori_.copyTo(map_mat_);
    cv_img_full_.image = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0,0,0));
    cv::resize(map_mat_, map_mat_, cv::Size(640, 480));
    map_mat_.copyTo(cv_img_full_.image);
    image_pub_.publish(cv_img_full_.toImageMsg());
  }

  void clearMap()
  {
    map_ori_.copyTo(map_mat_);
    cv_img_full_.image = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0,0,0));
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  image_transport::Publisher image_pub_;

  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  sensor_msgs::msg::LaserScan::SharedPtr scan_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  geometry_msgs::msg::TransformStamped tf_map_robot_;

  cv_bridge::CvImage cv_img_full_;
  cv::Mat map_mat_;
  cv::Mat map_ori_;

  bool recv_map_ = false;
  bool recv_scan_ = false;
  bool recv_tf_ = false;

  int size_x_;
  int size_y_;
  double map_resolution_;

  int pix_pose_x_;
  int pix_pose_y_;
  double pose_yaw_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapAsImageProvider>());
  rclcpp::shutdown();
  return 0;
}
