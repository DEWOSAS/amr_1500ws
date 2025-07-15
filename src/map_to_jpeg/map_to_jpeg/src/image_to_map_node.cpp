#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>

using std::placeholders::_1;

class ImageToMapNode : public rclcpp::Node {
public:
  ImageToMapNode() : Node("image_to_map_node") {
    this->declare_parameter("resolution", 0.05);
    this->get_parameter("resolution", resolution_);

    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_from_jpeg", 10);
    image_sub_ = image_transport::create_subscription(
      this, "map_image_raw",
      std::bind(&ImageToMapNode::imageCallback, this, _1),
      "raw");

    RCLCPP_INFO(this->get_logger(), "ImageToMapNode started.");
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    nav_msgs::msg::OccupancyGrid map;
    map.header = msg->header;
    map.header.frame_id = "map";
    map.info.resolution = resolution_;
    map.info.width = msg->width;
    map.info.height = msg->height;
    map.info.origin.orientation.w = 1.0;
    map.info.origin.position.x = -((msg->width + 1) * resolution_ * 0.5f);
    map.info.origin.position.y = -((msg->height + 1) * resolution_ * 0.5f);

    for (int y = msg->height - 1; y >= 0; --y) {
      for (unsigned int x = 0; x < msg->width; ++x) {
        int data = msg->data[y * msg->width + x];
        if (data >= 123 && data <= 131)
          map.data.push_back(-1);
        else if (data >= 251 && data <= 259)
          map.data.push_back(0);
        else
          map.data.push_back(100);
      }
    }

    map_pub_->publish(map);
  }

  double resolution_;
  image_transport::Subscriber image_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageToMapNode>());
  rclcpp::shutdown();
  return 0;
}