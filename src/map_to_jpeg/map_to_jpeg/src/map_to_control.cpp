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
#include <nav_msgs/msg/path.hpp>

using namespace std;
using namespace cv;

class Map2ControlNode : public rclcpp::Node {
public:
  Map2ControlNode() : Node("map_to_control_node"),
                    tf_buffer_(this->get_clock()),
                    tf_listener_(tf_buffer_) {
                    


    auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
              .reliable()
              .transient_local();
    
    this->declare_parameter<int>("crop_width", 640);
    this->declare_parameter<int>("crop_height", 480);

    
     
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", qos, std::bind(&Map2ControlNode::mapCallback, this, std::placeholders::_1));


    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "scan", rclcpp::SensorDataQoS(),
                std::bind(&Map2ControlNode::lasercallback, this, std::placeholders::_1));

    plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
     "plan",
    rclcpp::QoS(10),                                  // ✅ QoS ปกติ (Volatile)
    std::bind(&Map2ControlNode::planCallback, this, std::placeholders::_1));

             
    local_costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/local_costmap/costmap",qos , std::bind(&Map2ControlNode::localCostmapCallback, this, std::placeholders::_1));
    
    global_costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/global_costmap/costmap", qos, std::bind(&Map2ControlNode::globalCostmapCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      55ms, std::bind(&Map2ControlNode::publishMapWithScan, this));

    image_pub_ = image_transport::create_publisher(this, "map_to_page_control/full");
    
    RCLCPP_INFO(this->get_logger(), "map_to_control_node started.");
  }
void localCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    local_costmap_ = msg;
    RCLCPP_INFO(this->get_logger(), "Local costmap received.");

  }

void planCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    current_plan_ = msg;
    RCLCPP_INFO(this->get_logger(), "Plan received with %zu poses", msg->poses.size());
}

void globalCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    global_costmap_ = msg;
    RCLCPP_INFO(this->get_logger(), "Global costmap received.");
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
  rclcpp::Time stamp = scan_ ? rclcpp::Time(scan_->header.stamp) : this->now();
    if (!current_map_) {
      RCLCPP_WARN(this->get_logger(), "No map data received yet.");
      return;
    }

    // --- Lookup robot pose ---
    try {
      transform_stamped_ = tf_buffer_.lookupTransform(
        "map",
        "base_link",
        stamp,
      tf2::durationFromSec(0.05));
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
      return;
    }


    int width = current_map_->info.width;
    int height = current_map_->info.height;
    cv::Mat map_image(height, width, CV_8UC3);

    // วาดแผนที่
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
    // วาด local และ global costmap
auto draw_costmap = [&](const nav_msgs::msg::OccupancyGrid::SharedPtr &costmap, 
                        const cv::Scalar &color, double alpha) {
    if (!costmap) return;

    int w = costmap->info.width;
    int h = costmap->info.height;

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            int idx = y * w + x; 
            int8_t val = costmap->data[idx];

            if (val >= 50) { // pixel ที่มีค่า obstacle หรือ inflated
                // world coordinates ของจุดนี้
                double wx = costmap->info.origin.position.x + x * costmap->info.resolution;
                double wy = costmap->info.origin.position.y + y * costmap->info.resolution;

                // แปลงเป็น pixel ของ map หลัก
                int mx = static_cast<int>((wx - current_map_->info.origin.position.x) / current_map_->info.resolution);
                int my = current_map_->info.height - static_cast<int>((wy - current_map_->info.origin.position.y) / current_map_->info.resolution);

                if (mx >= 0 && mx < current_map_->info.width && my >= 0 && my < current_map_->info.height) {
                    cv::Vec3b &pixel = map_image.at<cv::Vec3b>(my, mx);
                          pixel = (1.0 - alpha) * pixel + alpha * cv::Vec3b(color[0], color[1], color[2]);
                      }
                  }
              }
            }
};



  // Global costmap สีแดง
  //draw_costmap(global_costmap_, cv::Scalar(255, 0, 0  ), 0.3);
  // Local costmap สีเหลือง
  draw_costmap(local_costmap_, cv::Scalar(0, 255, 255), 0.1);


    // --- วาด robot ---
    double rx = transform_stamped_.transform.translation.x;
    double ry = transform_stamped_.transform.translation.y;
    int px = static_cast<int>((rx - current_map_->info.origin.position.x) / current_map_->info.resolution);
    int py = static_cast<int>((ry - current_map_->info.origin.position.y) / current_map_->info.resolution);
    py = height - py;

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

    // --- วาดเลเซอร์  ---
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

    // --- วาดเส้นทาง (Path) ---
    if (current_plan_) {
      for (size_t i = 1; i < current_plan_->poses.size(); ++i) {
          auto p1 = current_plan_->poses[i-1].pose.position;
          auto p2 = current_plan_->poses[i].pose.position;

          int x1 = static_cast<int>((p1.x - current_map_->info.origin.position.x) / current_map_->info.resolution);
          int y1 = height - static_cast<int>((p1.y - current_map_->info.origin.position.y) / current_map_->info.resolution);

          int x2 = static_cast<int>((p2.x - current_map_->info.origin.position.x) / current_map_->info.resolution);
          int y2 = height - static_cast<int>((p2.y - current_map_->info.origin.position.y) / current_map_->info.resolution);

          if ((x1 >= 0 && x1 < width && y1 >= 0 && y1 < height) &&
              (x2 >= 0 && x2 < width && y2 >= 0 && y2 < height)) {
              cv::line(map_image, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 0, 255), 2);
          }
      }
    }

    // --- Crop ภาพให้หุ่นยนต์อยู่ตรงกลาง ---
    
   
    int crop_width  = this->get_parameter("crop_width").as_int();
    int crop_height = this->get_parameter("crop_height").as_int();


    int half_crop_w = crop_width/2;
    int half_crop_h = crop_height/2;
      
    int x_start = std::max(0, px - half_crop_w);
    int y_start = std::max(0, py - half_crop_h);


    if (map_image.cols < 640 || map_image.rows < 480) {
    RCLCPP_WARN(this->get_logger(), "Map too small to crop: %dx%d", map_image.cols, map_image.rows);
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", map_image).toImageMsg();
    msg->header.frame_id = "map";
    msg->header.stamp = this->get_clock()->now();
    image_pub_.publish(msg);
    return;
    }
    // ตรวจไม่ให้ crop ล้นขอบภาพ
    if (x_start + crop_width > width) {
        x_start = width - crop_width;
    }
    if (y_start + crop_height > height) {
        y_start = height - crop_height;
    }

    // ตรวจอีกครั้ง (เพื่อไม่ให้ติดค่าติดลบ)
    x_start = std::max(0, x_start);
    y_start = std::max(0, y_start);

    cv::Rect roi(x_start, y_start, crop_width, crop_height);
    cv::Mat cropped = map_image(roi);

    // เปลี่ยนภาพที่จะ publish เป็น cropped
    cv::Mat output_image;
    cv::resize(cropped, output_image, cv::Size(crop_width, crop_height)); // Optional: ขยายภาพ



    // แปลงภาพเป็น ROS message และ publish
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", output_image).toImageMsg();
    msg->header.frame_id = "map";
    msg->header.stamp = this->get_clock()->now();
    image_pub_.publish(msg);

}

  // create subscriptions
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  

  // for 
  nav_msgs::msg::OccupancyGrid::SharedPtr local_costmap_;
  nav_msgs::msg::OccupancyGrid::SharedPtr global_costmap_;
  nav_msgs::msg::Path::SharedPtr current_plan_;

  sensor_msgs::msg::LaserScan::SharedPtr scan_;
  nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
  geometry_msgs::msg::TransformStamped transform_stamped_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  image_transport::Publisher image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Map2ControlNode>());
  rclcpp::shutdown();
  return 0;
}
