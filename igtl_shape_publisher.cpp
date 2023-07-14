#include <memory>
#include <chrono>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros2_igtl_bridge/msg/string.hpp"
#include "ros2_igtl_bridge/msg/point.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class IGTLPublisher : public rclcpp::Node
{
public:
    // using PoseArray = geometry_msgs::msg::PoseArray;
    IGTLPublisher(): Node("shape_publisher"), count_(0)
    {
        // Publisher: Send to bridge
        point_publisher_  = this->create_publisher<ros2_igtl_bridge::msg::Point>("IGTL_POINT_OUT", 10);
        string_publisher_ = this->create_publisher<ros2_igtl_bridge::msg::String>("IGTL_STRING_OUT", 10);

        // Subscriber:
        needle_shape_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>("/needle/state/current_shape", 10, std::bind(&IGTLPublisher::needle_shape_callback, this, _1)); 
    }

private:

    void needle_shape_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {    
        size_t nposes = msg->poses.size();

        RCLCPP_INFO(this->get_logger(), "Received Needle Shape");
        RCLCPP_INFO(this->get_logger(), "Total number of points: %d",  nposes);

    // Receives needle shape from /needle/state/current_shape
        // Pushes to IGTL ROS2 Bridge
        auto string_msg = ros2_igtl_bridge::msg::String();
        auto point_msg = ros2_igtl_bridge::msg::Point();

    // Point messages - Shape points
        for (int i = 0; i < nposes; i ++)
        {
            point_msg.name = "NEEDLESHAPE_"+std::to_string(i);
            point_msg.pointdata.x = msg->poses[i].position.x;
            point_msg.pointdata.y = msg->poses[i].position.y;
            point_msg.pointdata.z = msg->poses[i].position.z;
            point_publisher_->publish(point_msg);
            RCLCPP_INFO(this->get_logger(), "Point: '%s", point_msg.name.c_str());
        }

    // String message - Shape Header
        // Convert timestamp to a readable format
        rclcpp::Time timestamp = msg->header.stamp;
        auto now = std::chrono::system_clock::now();
        auto timestampDuration = std::chrono::nanoseconds(timestamp.nanoseconds());
        auto durationSinceEpoch = std::chrono::duration_cast<std::chrono::seconds>(timestampDuration);
        std::time_t timeT = std::chrono::system_clock::to_time_t(now - std::chrono::seconds(durationSinceEpoch.count()));
        std::stringstream ss;
        ss << std::put_time(std::localtime(&timeT), "%Y-%m-%d %H:%M:%S")
        << "." << std::setw(3) << std::setfill('0') << timestampDuration.count() % 1'000'000'000 / 1'000'000;
        std::string formattedTimestamp = ss.str();
        std::string frame_id = msg->header.frame_id;
        string_msg.name = "NeedleShapeHeader";
        string_msg.data = formattedTimestamp + ";" + std::to_string(count_++) + ";" + std::to_string(nposes) + ";" + frame_id;
        RCLCPP_INFO(this->get_logger(), "Publishing Needle Shape: %d total points", nposes);
        RCLCPP_INFO(this->get_logger(), "Data: %s", string_msg.data.c_str());
    	string_publisher_->publish(string_msg);
    }

    size_t count_;
    rclcpp::Publisher<ros2_igtl_bridge::msg::String>::SharedPtr      string_publisher_;
    rclcpp::Publisher<ros2_igtl_bridge::msg::Point>::SharedPtr       point_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr   needle_shape_subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IGTLPublisher>());
  rclcpp::shutdown();
  return 0;
}
