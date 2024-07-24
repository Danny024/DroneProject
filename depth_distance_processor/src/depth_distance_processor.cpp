#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class DepthImageProcessor : public rclcpp::Node
{
public:
    DepthImageProcessor()
        : Node("depth_image_processor"), max_depth_range_(15.0)  // Set max depth range to 10 meters (adjust as needed)
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/depth", 10, std::bind(&DepthImageProcessor::image_callback, this, std::placeholders::_1));
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // Convert ROS Image message to OpenCV image
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            cv::Mat depth_image = cv_ptr->image;

            // Example: Convert the entire depth image from relative depth to metric depth
            cv::Mat metric_depth_image = depth_image * max_depth_range_;

            // Get the metric depth value at the center of the image
            int center_x = metric_depth_image.cols / 2;
            int center_y = metric_depth_image.rows / 2;
            float distance_in_meters = metric_depth_image.at<float>(center_y, center_x);

            RCLCPP_INFO(this->get_logger(), "Distance at center: %.2f meters", distance_in_meters);
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    float max_depth_range_;  // Maximum range of the depth sensor in meters
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthImageProcessor>());
    rclcpp::shutdown();
    return 0;
}
