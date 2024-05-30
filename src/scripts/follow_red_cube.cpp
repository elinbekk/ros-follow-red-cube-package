#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher velocity_pub;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr depth_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

void imageCallback(const sensor_msgs::ImageConstPtr& image_msg) {   
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat image = cv_ptr->image;
    cv::Mat hsv_image;
    cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

    cv::Mat mask1, mask2;
    cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), mask1);
    cv::inRange(hsv_image, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), mask2);
    cv::Mat mask = mask1 | mask2;

    //находим контура
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        ROS_WARN("No red cube detected.");
        cv::imshow("Red Cube Detection", image);
        cv::waitKey(1);
        return;
    }

    // Находим больший контур
    std::vector<cv::Point> largest_contour = contours[0];
    double largest_area = cv::contourArea(largest_contour);
    for (size_t i = 1; i < contours.size(); ++i) {
        double area = cv::contourArea(contours[i]);
        if (area > largest_area) {
            largest_area = area;
            largest_contour = contours[i];
        }
    }

    cv::Moments m = cv::moments(largest_contour);
    int cX = static_cast<int>(m.m10 / m.m00);
    int cY = static_cast<int>(m.m01 / m.m00);


    // Рисуем зелёный круг в центре
    cv::drawContours(image, std::vector<std::vector<cv::Point>>{largest_contour}, -1, cv::Scalar(0, 255, 0), 2);
    cv::circle(image, cv::Point(cX, cY), 5, cv::Scalar(0, 255, 0), -1);

    cv::imshow("Red Cube Detection", image);
    cv::waitKey(1);

    if (depth_cloud->empty()) {
        ROS_WARN("Depth cloud is empty.");
        return;
    }

    // Получаем расстояние до круг в центре контура
    pcl::PointXYZRGB point;
    if (cX >= 0 && cX < image.cols && cY >= 0 && cY < image.rows) {
        int idx = cY * depth_cloud->width + cX;
        if (idx < 0 || idx >= depth_cloud->points.size()) {
            ROS_ERROR("Index out of bounds: %d", idx);
            return;
        }
        point = depth_cloud->points[idx];
    } else {
        ROS_WARN("Centroid out of image bounds.");
        return;
    }

    if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
        ROS_WARN("Invalid point detected.");
        return;
    }

    float distance = point.z;
    ROS_INFO("Distance to the red cube: %f", distance);

    geometry_msgs::Twist cmd_vel;
    float linear_speed = 0.0;
    float angular_speed = 0.0;


    if (distance > 1.0) {
        linear_speed = 0.5;
    } else {
        linear_speed = 0.0; 
    }

    angular_speed = -0.002 * (cX - (image.cols / 2));

    cmd_vel.linear.x = linear_speed;
    cmd_vel.angular.z = angular_speed;

    velocity_pub.publish(cmd_vel);
}

void depthCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    pcl::fromROSMsg(*cloud_msg, *depth_cloud);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "cube_follower");
    ros::NodeHandle nh;

    ros::Subscriber color_image_sub = nh.subscribe("/realsense/color/image_raw", 1, imageCallback);
    ros::Subscriber depth_sub = nh.subscribe("/realsense/depth/color/points", 1, depthCallback);
    velocity_pub = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 10);

    cv::namedWindow("Red Cube Detection", cv::WINDOW_AUTOSIZE);

    ros::spin();

    cv::destroyAllWindows();
    return 0;
}
