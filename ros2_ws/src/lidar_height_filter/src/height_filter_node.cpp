/*
 * Author: Ali Mohamed
 * Date: 17/07/2025
 * Description: 
 * This code implements a ROS2 node named HeightFilterNode that subscribes to 
 * raw LiDAR point cloud data (PointCloud2), filters the points based on a specified 
 * minimum and maximum height (z-coordinate) range provided as parameters, and 
 * publishes the filtered point cloud to a new topic. The node uses PCL (Point Cloud 
 * Library) for point cloud manipulation and allows dynamic parameter setting 
 * for height filtering at runtime.
 */


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class HeightFilterNode : public rclcpp::Node
{
public:
    HeightFilterNode()
        : Node("height_filter_node")
    {
        // Declare parameters with default values
        this->declare_parameter<double>("min_height", 0.0);
        this->declare_parameter<double>("max_height", 1.5);

        // Retrieve them
        this->get_parameter("min_height", min_height_);
        this->get_parameter("max_height", max_height_);

        RCLCPP_INFO(this->get_logger(), "Filtering points between %.2f and %.2f", min_height_, max_height_);

        // Publisher and subscriber
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("points_filtered", 10);
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "points_raw", 10,
            std::bind(&HeightFilterNode::pointcloud_callback, this, std::placeholders::_1));
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud_in;
    pcl::fromROSMsg(*msg, cloud_in);

    RCLCPP_INFO(this->get_logger(), "Received pointcloud with %zu points", cloud_in.points.size());

    pcl::PointCloud<pcl::PointXYZ> cloud_out;

    // Prints raw Z heights of first 10 points (or fewer)
    size_t count = std::min(cloud_in.points.size(), size_t(10));
    RCLCPP_INFO(this->get_logger(), "Raw Z heights (first %zu points):", count);
    for (size_t i = 0; i < count; ++i)
        RCLCPP_INFO(this->get_logger(), "  Z = %.3f", cloud_in.points[i].z);

    for (const auto& pt : cloud_in.points)
    {
        if (pt.z >= min_height_ && pt.z <= max_height_)
            cloud_out.points.push_back(pt);
    }

    cloud_out.header = cloud_in.header;

    // Print filtered Z heights of first 10 filtered points (or fewer)
    count = std::min(cloud_out.points.size(), size_t(10));
    RCLCPP_INFO(this->get_logger(), "Filtered Z heights (first %zu points):", count);
    for (size_t i = 0; i < count; ++i)
        RCLCPP_INFO(this->get_logger(), "  Z = %.3f", cloud_out.points[i].z);

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(cloud_out, output);
    pub_->publish(output);
}


    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    double min_height_, max_height_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeightFilterNode>());
    rclcpp::shutdown();
    return 0;
}
