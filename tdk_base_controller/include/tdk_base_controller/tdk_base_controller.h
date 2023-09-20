#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <vector>
#include <mutex> 

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <serial/serial.h> 
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>

#define FRAME_LEN 16 // (including SOF, EOF and CRC16)
#define DATA_LEN 12  // Length of bytes of the data field

using namespace std;

// Message Frame Declaration
typedef struct __attribute__((__packed__))
{
    uint8_t sof;
    union
    {
        uint8_t data[DATA_LEN];
        struct
        {
            float vx;
            float vy;
            float w;
        };
    };
    uint16_t checksum;
    uint8_t eof;
} CtrlMsg;

class TDKBaseController
{
public:
    TDKBaseController(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // Subscribers
    ros::Subscriber cmdVelSub_;
    ros::Subscriber emergencyStopSub_;
    ros::Subscriber joyStickSub;
    ros::Subscriber change_IMU_mode_Pub_;
    ros::Subscriber change_right_Pub_;
    ros::Subscriber change_zero_Pub_;
    ros::Subscriber change_motoron_Pub_;
    ros::Subscriber object_send_Pub_;



    // Publishers
    ros::Publisher odomPub_;
    tf::TransformBroadcaster odomTfBroad_;

    // Timers
    ros::Timer connectionCheckTimer_;
    ros::Timer communicationTimer_;
    ros::Timer odomTimer_;

    // Parameters
    string port_name_ = "teensy4";

    // Subscriber Callbacks
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);
    void emergencyStopCallback(const std_msgs::Bool::ConstPtr &msg);
    void joyStickCallback(const sensor_msgs::Joy::ConstPtr &msg);
    void change_IMU_modeCallback(const std_msgs::Int8::ConstPtr &msg);
    void change_rightCallback(const std_msgs::Int8::ConstPtr &msg);
    void change_zeroCallback(const std_msgs::Int8::ConstPtr &msg);
    void change_motoronCallback(const std_msgs::Int8::ConstPtr &msg);
    void object_send_stop_Callback(const std_msgs::Bool::ConstPtr &msg);

    // Timer Callbacks
    void connectionCheckCallback(const ros::TimerEvent &event);
    void communicationCallback(const ros::TimerEvent &event);
    void odomCallback(const ros::TimerEvent &event);


    // Local Variables
    serial::Serial serialPort;
    bool emergencyStopFlag;
    bool object_send_stop_flag;
    float vx, vy, w;
    float odomX, odomY, odomYaw;
    bool mutexFlag;

    // Mutex
    std::mutex stopFlagMutex;
    
    //communication variables
    uint8_t fake_eof;
    uint8_t fake_right_eof;
    uint8_t fake_left_eof;
    uint8_t fake_zero_eof;
    int8_t fake_motoron_eof;

    // Private Functions
    inline uint16_t calculateChecksum(const uint8_t *data, int len)
    {
        uint16_t sum = 0;
        for (int i = 0; i < len / 2; i++)
        {
            sum ^= *(((uint16_t *)data) + i);
        }
        return sum;
    }
};