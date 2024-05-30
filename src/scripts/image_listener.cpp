#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::imshow("Image Window", cv_ptr->image);
        cv::waitKey(1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Failed to convert image: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    std::string image_topic = "/realsense/color/image_raw";
    ros::Subscriber image_sub = nh.subscribe(image_topic, 1, imageCallback);

    ros::spin();

    cv::destroyAllWindows();
    return 0;
}
