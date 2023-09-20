#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <fstream>
#include <string>

std::string getTimeStampedFilename(const std::string& prefix, const std::string& extension) {
    std::stringstream ss;
    auto now = std::time(nullptr);
    auto tm = *std::localtime(&now);
    std::string folder_path = "/home/eddie123/test_idiot_ws/src/path_listener_node/path_listener/";
    ss << folder_path <<prefix << "_" << std::put_time(&tm, "%Y%m%d_%H%M%S") << extension;
    return ss.str();
}

double calculatePathLength(const nav_msgs::Path::ConstPtr& msg) {
    double path_length = 0.0;

    for (size_t i = 1; i < msg->poses.size(); ++i) {
        double dx = msg->poses[i].pose.position.x - msg->poses[i - 1].pose.position.x;
        double dy = msg->poses[i].pose.position.y - msg->poses[i - 1].pose.position.y;
        double dz = msg->poses[i].pose.position.z - msg->poses[i - 1].pose.position.z;

        path_length += std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    return path_length;
}

std::ofstream global_plan_file;
std::ofstream local_plan_file;
std::ofstream path_length_file;

void globalPlanCallback(const nav_msgs::Path::ConstPtr& msg) {
    ROS_INFO("Received global plan");

    double path_length = calculatePathLength(msg);
    ROS_INFO("Global plan length: %f", path_length);
    path_length_file << path_length << ",";

    for (const auto& pose : msg->poses) {
        global_plan_file << pose.pose.position.x << ","
                         << pose.pose.position.y << ","
                         << pose.pose.position.z << std::endl;
    }
    global_plan_file.flush();
}

void localPlanCallback(const nav_msgs::Path::ConstPtr& msg) {
    ROS_INFO("Received local plan");

    double path_length = calculatePathLength(msg);
    ROS_INFO("Local plan length: %f", path_length);
    path_length_file << path_length << std::endl;


    for (const auto& pose : msg->poses) {
        local_plan_file << pose.pose.position.x << ","
                        << pose.pose.position.y << ","
                        << pose.pose.position.z << std::endl;
    }
    local_plan_file.flush();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_listener_node");
    ros::NodeHandle nh;

    std::string global_filename = getTimeStampedFilename("global_plan", ".csv");
    std::string local_filename = getTimeStampedFilename("local_plan", ".csv");
    std::string path_length_filename = getTimeStampedFilename("path_length", ".csv");

    global_plan_file.open(global_filename, std::ios::out | std::ios::app);
    local_plan_file.open(local_filename, std::ios::out | std::ios::app);
    path_length_file.open(path_length_filename, std::ios::out | std::ios::app);

    global_plan_file << "x,y,z" << std::endl;
    local_plan_file << "x,y,z" << std::endl;
    path_length_file << "global_path_length,local_path_length" << std::endl;

    ros::Subscriber global_plan_sub = nh.subscribe("/move_base/TebLocalPlannerROS/global_plan", 10, globalPlanCallback);
    ros::Subscriber local_plan_sub = nh.subscribe("/move_base/TebLocalPlannerROS/local_plan", 10, localPlanCallback);

    ros::spin();

    global_plan_file.close();
    local_plan_file.close();
    path_length_file.close();

    return 0;
}
