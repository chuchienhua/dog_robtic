#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <fstream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <ctime>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <my_tf_listener_pkg/RobotState.h>

class CameraCapture
{
public:
    CameraCapture()
        : it_(nh_)
    {
        image_subscriber_ = it_.subscribe("/usb_cam/image_raw", 1, &CameraCapture::callback, this);
        capture_enabled_subscriber_ = nh_.subscribe("/tdk_base_controller/change_IMU_mode", 1, &CameraCapture::capture_enabled_callback, this);
        global_plan_sub_ = nh_.subscribe("/move_base/TebLocalPlannerROS/global_plan", 10, &CameraCapture::globalPlanCallback, this);
        local_plan_sub_ = nh_.subscribe("/move_base/TebLocalPlannerROS/local_plan", 10, &CameraCapture::localPlanCallback, this);
        robot_state_sub_ = nh_.subscribe("/robot_state", 10, &CameraCapture::robotStateCallback, this);
    }

    void globalPlanCallback(const nav_msgs::Path::ConstPtr &msg)
    {
        if (!msg->poses.empty())
        {
            global_pose_ = msg->poses.back().pose.position;
        }
    }

    void localPlanCallback(const nav_msgs::Path::ConstPtr &msg)
    {
        if (!msg->poses.empty())
        {
            local_pose_ = msg->poses.back().pose.position;
        }
    }

    void robotStateCallback(const my_tf_listener_pkg::RobotState::ConstPtr &msg)
    {
        robot_yaw_ = msg->yaw;
        robot_x_ = msg->x;
        robot_y_ = msg->y;
        total_distance_moved_ = msg->total_distance_moved;
    }

    void capture_enabled_callback(const std_msgs::Int8::ConstPtr &msg)
    {
        // ROS_INFO_STREAM("capture _ data= " << msg->data);
        if (msg->data == 9)
        {
            capture_enabled = true;
        }
    }

    void callback(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        // std::string timestamp = std::to_string(ros::Time::now().toSec());
        std::time_t raw_time = std::time(0);
        std::tm *timeinfo = std::localtime(&raw_time);
        char buffer[80];
        std::strftime(buffer, 80, "%Y-%m-%d-%H-%M-%S", timeinfo);
        std::string timestamp(buffer);
        cv::putText(cv_ptr->image, timestamp, cv::Point(5, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1.9, cv::LINE_AA);

        std::stringstream robot_pose_ss;
        robot_pose_ss << "( X, Y , Yaw) : "
                      << "( " << robot_x_ << ", " << robot_y_ << ", " << robot_yaw_ << ")";
        cv::putText(cv_ptr->image, robot_pose_ss.str(), cv::Point(5, 45), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1.9, cv::LINE_AA);

        std::stringstream global_pose_ss;
        global_pose_ss << "Global: (" << global_pose_.x << ", " << global_pose_.y << ", " << global_pose_.z << ")";
        cv::putText(cv_ptr->image, global_pose_ss.str(), cv::Point(5, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1.9, cv::LINE_AA);

        std::stringstream local_pose_ss;
        local_pose_ss << "Local: (" << local_pose_.x << ", " << local_pose_.y << ", " << local_pose_.z << ")";
        cv::putText(cv_ptr->image, local_pose_ss.str(), cv::Point(5, 75), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1.9, cv::LINE_AA);

        std::stringstream total_distance_moved_ss;
        total_distance_moved_ss << "Total distance moved: " << total_distance_moved_;

        if (capture_enabled)
        {
            cv::putText(cv_ptr->image, total_distance_moved_ss.str(), cv::Point(5, 90), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1.9, cv::LINE_AA);
            std::string folder_path = "/home/eddie123/test_idiot_ws/src/camera_capture_node/image/";
            std::string img_name = folder_path + "camera_image_" + std::to_string(ros::Time::now().toSec()) + ".png";
            cv::imwrite(img_name, cv_ptr->image);
            ROS_INFO("Saved image: %s", img_name.c_str());
            capture_enabled = false;
        }

        cv::imshow("image_view", cv_ptr->image);
        cv::waitKey(3);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber capture_enabled_subscriber_;
    ros::Subscriber global_plan_sub_;
    ros::Subscriber local_plan_sub_;
    ros::Subscriber robot_state_sub_;

    image_transport::ImageTransport it_;
    image_transport::Subscriber image_subscriber_;

    geometry_msgs::Point global_pose_;
    geometry_msgs::Point local_pose_;

    double robot_yaw_;
    double robot_x_;
    double robot_y_;
    double total_distance_moved_;
    bool capture_enabled;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_capture_node");

    CameraCapture camera_capture;

    ros::spin();

    return 0;
}
