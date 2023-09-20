#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <vector>

#include <std_msgs/Int8.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>

#define VX_AXIS 6      //Left Vertical Stick
#define VY_AXIS 7      //Left Horizonal Stick
#define W_AXIS 3         // 蘑菇頭 Right Horizonal Stick contorl left/right rotate
#define EMERG_BTN 4      // L1
#define JS_CTRL_BTN 5    // R1
#define HOME_BTN 2       // Triangle
#define ADD_POINT_BTN 1  // Circle
#define DEL_POINT_BTN 0  // Cross
#define SAVE_POINT_BTN 3 // Square
#define turn_zero 6 // back
#define motor_turnon 7 // start
#define mushroomhead_VX 0 //VX
#define mushroomhead_VY 1 //VY
#define change_IMU_mode 8 //Logitech

#define BTN_DEBOUNCE_TIME 0.2 // 100ms

void joyStickCallback(const sensor_msgs::Joy::ConstPtr &msg);
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);
void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
void savedPosePubTimerCallback(const ros::TimerEvent &);
void _dector_objectCallback(const std_msgs::Bool::ConstPtr &msg);
void goHome();
void addPoint();
void delPoint();
void savePointsToFile();

double vx_sacler;
double vy_sacler;
double w_sacler;
bool emergencyStopFlag = false;
bool last_emergencyStopFlag = false;
bool jsCmdVel = false;
bool lastAddBtnState = false;
bool lastDelBtnState = false;
bool lastSaveBtnState = false;
bool Logitech_mode = false;
bool detect_mode = false;

//20230204 add turn right/left and turnz_zero
bool lastChangeIMUmode =false;
bool change_IMU_state =false;
bool auto_move =false;
bool last_change_zero =false;
bool change_zero =false;
bool last_change_motoron =false;
bool change_motoron =false;


geometry_msgs::Pose currentPose;
std::vector<geometry_msgs::Pose> savedPose;

ros::Publisher cmdVelPub;
ros::Publisher emergencyStopPub;
ros::Publisher goalPub;
ros::Publisher savedPosePub;
ros::Time lastAddTime, lastDelTime, lastSaveTime,lastChangeIMUmodeTime,lastautomove;
ros::Publisher change_IMU_mode_Pub;
ros::Publisher change_right_Pub;
ros::Publisher change_zero_Pub;
ros::Publisher change_motoron_Pub;
ros::Publisher object_send_Pub;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tdk_js_control");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    ros::Subscriber joyStickSub = nh.subscribe("/joy", 1000, joyStickCallback);
    ros::Subscriber cmdVelSub = nh.subscribe("/cmd_vel", 1000, cmdVelCallback);
    ros::Subscriber amclPoseSub = nh.subscribe("/amcl_pose", 1000, amclPoseCallback);
    ros::Subscriber _dector_objectPub = nh.subscribe("/dector_object_stop", 10, _dector_objectCallback);

    cmdVelPub = nh.advertise<geometry_msgs::Twist>("tdk_base_controller/cmd_vel", 1);
    emergencyStopPub = nh.advertise<std_msgs::Bool>("tdk_base_controller/emergency_stop", 1);
    change_IMU_mode_Pub = nh.advertise<std_msgs::Int8>("tdk_base_controller/change_IMU_mode", 10);
    change_right_Pub = nh.advertise<std_msgs::Int8>("tdk_base_controller/change_right", 10);
    change_zero_Pub = nh.advertise<std_msgs::Int8>("tdk_base_controller/change_zero", 10);
    change_motoron_Pub = nh.advertise<std_msgs::Int8>("tdk_base_controller/change_motoron", 10);
    goalPub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    savedPosePub = nh.advertise<visualization_msgs::Marker>("tdk_js_control/saved_point", 1);
    object_send_Pub = nh.advertise<std_msgs::Bool>("tdk_base_controller/object_send_stop", 10);

    ros::Timer savedPosePubTimer = nh.createTimer(ros::Duration(0.1), savedPosePubTimerCallback);

    nh_private.param<double>("vx_sacler", vx_sacler, 0.5);
    nh_private.param<double>("vy_sacler", vy_sacler, 0.5);
    nh_private.param<double>("w_sacler", w_sacler, 3.14 / 2);

    ros::spin();
    return 0;
}

void _dector_objectCallback(const std_msgs::Bool::ConstPtr &msg)
{
    detect_mode = msg->data;
    // ROS_INFO_STREAM("Detect Mode : " << detect_mode);
    std_msgs::Bool object_detect_msg;
    object_detect_msg.data = detect_mode;
    object_send_Pub.publish(object_detect_msg);
}

void joyStickCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
    std_msgs::Bool boolMsg;
    std_msgs::Int8 IMU_mode_msg;
    std_msgs::Int8 rightmsg;
    std_msgs::Int8 zeromsg;
    std_msgs::Int8 motoronmsg;

    if (msg->buttons[EMERG_BTN] != last_emergencyStopFlag)
    {
        // emergencyStopFlag = true;
        // boolMsg.data = true;
        emergencyStopFlag = !emergencyStopFlag;
        boolMsg.data = emergencyStopFlag;
        emergencyStopPub.publish(boolMsg);
        ROS_INFO_STREAM("Emergency Stop!" << boolMsg);
        
    }

    if (msg->buttons[JS_CTRL_BTN])
    {
        jsCmdVel = !jsCmdVel;
        ROS_INFO_STREAM("Joystick control = " << jsCmdVel);
    }

    if (msg->buttons[HOME_BTN])
    {
        goHome();
    }

    if((msg->buttons[motor_turnon] != last_change_motoron) )
    {
        motoronmsg.data = 34;
        motoronmsg.data = (motoronmsg.data | last_change_motoron) ;
        change_motoron_Pub.publish(motoronmsg);
        last_change_motoron = msg->buttons[motor_turnon]; 
        // ROS_INFO_STREAM("motoronmsg.data= " << motoronmsg);
    }
    

    ros::Time now = ros::Time::now();
    if (msg->buttons[ADD_POINT_BTN] != lastAddBtnState)
    {
        if (msg->buttons[ADD_POINT_BTN])
        {
            if ((now - lastAddTime).toSec() < BTN_DEBOUNCE_TIME)
                return; // Last add time < 100ms
            addPoint();
            lastAddTime = now;
        }
        lastAddBtnState = msg->buttons[ADD_POINT_BTN];
    }

    if (msg->buttons[DEL_POINT_BTN] != lastDelBtnState)
    {
        if (msg->buttons[DEL_POINT_BTN])
        {
            if ((now - lastDelTime).toSec() < BTN_DEBOUNCE_TIME)
                return; // Last delete time < 100ms
            delPoint();
            lastDelTime = now;
        }
        lastDelBtnState = msg->buttons[DEL_POINT_BTN];
    }

    if (msg->buttons[SAVE_POINT_BTN] != lastSaveBtnState)
    {
        if ((now - lastSaveTime).toSec() < BTN_DEBOUNCE_TIME * 10)
            return; // Last save time < 100ms
        savePointsToFile();
        lastSaveTime = now;
    }
    lastSaveBtnState = msg->buttons[SAVE_POINT_BTN];

    if(msg->buttons[change_IMU_mode] != lastChangeIMUmode)
    {
        if(msg->buttons[change_IMU_mode])
        {
            if((now - lastChangeIMUmodeTime).toSec() < BTN_DEBOUNCE_TIME)
                return;
            change_IMU_state = !change_IMU_state;
            IMU_mode_msg.data = 8;
            IMU_mode_msg.data = (IMU_mode_msg.data | change_IMU_state) ;
            change_IMU_mode_Pub.publish(IMU_mode_msg);
            lastChangeIMUmodeTime = now;
        }
        lastChangeIMUmode = msg->buttons[change_IMU_mode];
        // ROS_INFO_STREAM("change_IMU_state= " << change_IMU_state);
    }

    if(msg->buttons[turn_zero] != last_change_zero )
    {
        if(msg->buttons[turn_zero])
        {
            if((now - lastautomove).toSec() < BTN_DEBOUNCE_TIME)
                return;
            auto_move = !auto_move;
            zeromsg.data = 20;           
            zeromsg.data = (zeromsg.data | auto_move) ;
            change_zero_Pub.publish(zeromsg);
            lastautomove = now;
        }
        last_change_zero = msg->buttons[turn_zero]; 
        // ROS_INFO_STREAM("automove= " << auto_move);
    }

    if (!jsCmdVel)
        return; // Return if not controlling with joystick

    geometry_msgs::Twist twistMsg;

    if(change_IMU_state == false && detect_mode == false){
        // twistMsg.linear.x = msg->axes[VX_AXIS] * vx_sacler;
        // twistMsg.linear.y = msg->axes[VY_AXIS] * vy_sacler;
        twistMsg.linear.x = (msg->axes[mushroomhead_VX] * vx_sacler);
        twistMsg.linear.y = -(msg->axes[mushroomhead_VY] * vy_sacler);
        twistMsg.angular.z = msg->axes[W_AXIS] * w_sacler;
    }else{
        twistMsg.linear.x = msg->axes[mushroomhead_VX] * vx_sacler;
        twistMsg.linear.y = msg->axes[mushroomhead_VY] * vy_sacler;
    }
    cmdVelPub.publish(twistMsg);
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    if (jsCmdVel)
        return;
    cmdVelPub.publish(msg);
}

void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    currentPose.position = msg->pose.pose.position;
    currentPose.orientation = msg->pose.pose.orientation;
}

void savedPosePubTimerCallback(const ros::TimerEvent &)
{
    visualization_msgs::Marker points;
    points.header.frame_id = "map";
    points.header.stamp = ros::Time::now();
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    points.color.r = 1.0f;
    points.color.g = 1.0f;
    points.color.b = 0.0f;
    points.color.a = 1.0f;
    points.scale.x = 0.25f;
    points.scale.y = 0.25f;

    for (size_t i = 0; i < savedPose.size(); i++)
    {
        geometry_msgs::Point p;
        p.x = savedPose[i].position.x;
        p.y = savedPose[i].position.y;
        p.z = 0;
        points.points.push_back(p);
    }

    savedPosePub.publish(points);
}

void goHome()
{
    ROS_INFO("Going Home!");
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.pose.position.x = 0;
    msg.pose.position.y = 0;
    msg.pose.position.z = 0;
    msg.pose.orientation.w = 1;
    msg.pose.orientation.x = 0;
    msg.pose.orientation.y = 0;
    msg.pose.orientation.z = 0;

    jsCmdVel = false;
    goalPub.publish(msg);
}

void addPoint()
{
    ROS_INFO("Adding Node %ld!", savedPose.size());
    savedPose.push_back(currentPose);
}

void delPoint()
{
    ROS_INFO("Deleting Point!");
    if (savedPose.size() <= 0)
        return;
    savedPose.pop_back();
}

void savePointsToFile()
{
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d-%H-%M-%S");
    auto str = oss.str();

    std::string fileName = ros::package::getPath("tdk_navigation") + "/saved_points/" + str + ".csv";
    ROS_INFO_STREAM("Saving Points to " << fileName);
    std::ofstream file;
    file.open(fileName);
    file << "ID,X,Y,W,X,Y,Z\n";
    for (size_t i = 0; i < savedPose.size(); i++)
    {
        file << i << "," << savedPose[i].position.x << "," << savedPose[i].position.y << ","
             << savedPose[i].orientation.x << "," << savedPose[i].orientation.y << "," << savedPose[i].orientation.z << ","
             << savedPose[i].orientation.w
             << std::endl;
    }
    file.close();
}