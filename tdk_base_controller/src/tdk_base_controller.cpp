#include "tdk_base_controller/tdk_base_controller.h"

TDKBaseController::TDKBaseController(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) : nh_(nh), nh_private_(nh_private)
{

    // Initialize subscribers
    cmdVelSub_ = nh_.subscribe("tdk_base_controller/cmd_vel", 1, &TDKBaseController::cmdVelCallback, this, ros::TransportHints().tcpNoDelay());
    emergencyStopSub_ = nh_.subscribe("tdk_base_controller/emergency_stop", 1, &TDKBaseController::emergencyStopCallback, this, ros::TransportHints().tcpNoDelay());
    change_IMU_mode_Pub_= nh_.subscribe("tdk_base_controller/change_IMU_mode", 10, &TDKBaseController::change_IMU_modeCallback, this, ros::TransportHints().tcpNoDelay());
    change_right_Pub_= nh_.subscribe("tdk_base_controller/change_right", 10, &TDKBaseController::change_rightCallback, this, ros::TransportHints().tcpNoDelay());
    change_zero_Pub_= nh_.subscribe("tdk_base_controller/change_zero", 10, &TDKBaseController::change_zeroCallback, this, ros::TransportHints().tcpNoDelay());
    change_motoron_Pub_= nh_.subscribe("tdk_base_controller/change_motoron", 10, &TDKBaseController::change_motoronCallback, this, ros::TransportHints().tcpNoDelay());
    object_send_Pub_= nh_.subscribe("tdk_base_controller/object_send_stop", 10, &TDKBaseController::object_send_stop_Callback, this, ros::TransportHints().tcpNoDelay());
    // Initialize publishers
    odomPub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 1);

    // Initialize Parameters
    nh_private_.param<string>("device_port", port_name_, "/dev/ttyACM0");

    // Initialize Timers
    connectionCheckTimer_ = nh_.createTimer(ros::Duration(1.0), &TDKBaseController::connectionCheckCallback, this); // timer for checking connection
    communicationTimer_ = nh_.createTimer(ros::Duration(0.010), &TDKBaseController::communicationCallback, this);   // timer for sending control msgs
    // odomTimer_ = nh_.createTimer(ros::Duration(0.01), &TDKBaseController::odomCallback, this);             // timer for calculating odometry

    vx = 0;
    vy = 0;
    w = 0;
    odomX = 0;
    odomY = 0;
    odomYaw = 0;
    fake_eof = 0 ;
    emergencyStopFlag = false;
    mutexFlag =false;
    
}

void TDKBaseController::object_send_stop_Callback(const std_msgs::Bool::ConstPtr &msg){
    // std::lock_guard<std::mutex> lock(stopFlagMutex);
    object_send_stop_flag = msg->data;
    if(object_send_stop_flag == true && mutexFlag == false){
        emergencyStopFlag = true;
    }
    if(object_send_stop_flag == false && mutexFlag == false){
        emergencyStopFlag = false;
    }
  
    // ROS_INFO("object_send_stop_flag = %d", object_send_stop_flag);
}


void TDKBaseController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    vx = (msg->linear.x);
    vy = -(msg->linear.y);
    w = (msg->angular.z);
    ROS_INFO("vx = %.2f, vy = %.2f, w = %.2f EOF = %2X", vx, vy, w, fake_eof);
}

void TDKBaseController::emergencyStopCallback(const std_msgs::Bool::ConstPtr &msg)
{
    // std::lock_guard<std::mutex> lock(stopFlagMutex);
    mutexFlag = msg->data;
    emergencyStopFlag = msg->data;
}

void TDKBaseController::change_IMU_modeCallback(const std_msgs::Int8::ConstPtr &msg)
{
    fake_left_eof = msg->data;
    fake_eof =fake_left_eof;
}
void TDKBaseController::change_rightCallback(const std_msgs::Int8::ConstPtr &msg)
{
    fake_right_eof = msg->data;
    fake_eof = fake_right_eof;
}

void TDKBaseController::change_zeroCallback(const std_msgs::Int8::ConstPtr &msg)
{
    fake_zero_eof = msg->data;
    fake_eof = fake_zero_eof;
}

void TDKBaseController::change_motoronCallback(const std_msgs::Int8::ConstPtr &msg)
{
    fake_motoron_eof = msg->data;
    fake_eof = fake_motoron_eof;
}


void TDKBaseController::connectionCheckCallback(const ros::TimerEvent &event)
{
    if (!serialPort.isOpen())
    {
        try
        {
            serialPort.setPort(port_name_);
            serialPort.setBaudrate(115200);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(10); // Set read timeout to 10ms
            serialPort.setTimeout(timeout);
            serialPort.open();
            serialPort.flushInput();
            serialPort.flushOutput();
            vx = 0;
            vy = 0;
            w = 0;
            fake_eof = 0x00;
            ROS_INFO("Connected to teensy successfully!");
        }
        catch (serial::IOException &e)
        {
            ROS_ERROR_STREAM("Unable to open port " << port_name_ << ", " << e.what());
        }
    }
}

void TDKBaseController::communicationCallback(const ros::TimerEvent &event)
{
    if (!serialPort.isOpen())
        return;
    CtrlMsg ctrlMsg;
    ctrlMsg.sof = 0xAA + emergencyStopFlag;
    ctrlMsg.eof = fake_eof; //ctrlMsg.eof = 0xCD;
    ctrlMsg.vx = vx;
    ctrlMsg.vy = vy;
    ctrlMsg.w = w;

    ctrlMsg.checksum = calculateChecksum(ctrlMsg.data, DATA_LEN);
    try
    {
        serialPort.write((uint8_t *)&ctrlMsg, FRAME_LEN);
        uint8_t rxBuf[FRAME_LEN];
        serialPort.read(rxBuf, FRAME_LEN);
        CtrlMsg *rxMsg = (CtrlMsg *)rxBuf;
        
        if (((rxMsg->sof & 0xFF) == 0xAA))
        {
            // TODO: handle response
        }
        else
        {
            ROS_ERROR("Error response! SOF = %2X, EOF = %2X", rxMsg->sof, rxMsg->eof);
        }
    }
    catch (serial::SerialException &e)
    {
        ROS_ERROR_STREAM("Error when communicating to teensy: " << e.what());
        serialPort.close();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Error when communicating to teensy: " << e.what());
        serialPort.close();
    }
}


void TDKBaseController::odomCallback(const ros::TimerEvent &event)
{
    double dt = 0.001;
    double deltaX = (vy * cos(odomYaw) - vx * sin(odomYaw)) * dt;
    double deltaY = (vy * sin(odomYaw) + vx * cos(odomYaw)) * dt;
    double deltaYaw = w * dt;   // rad to degree
    odomX += deltaX;
    odomY += deltaY;
    odomYaw += deltaYaw;

    ros::Time current_time = ros::Time::now();

    // since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(odomYaw);

    // first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odomTf;
    odomTf.header.stamp = current_time;
    odomTf.header.frame_id = "odom";
    odomTf.child_frame_id = "base_link";

    odomTf.transform.translation.x = odomX;
    odomTf.transform.translation.y = odomY;
    odomTf.transform.translation.z = 0.0;
    odomTf.transform.rotation = odomQuat;

    odomTfBroad_.sendTransform(odomTf);

    // next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odomMsg;
    odomMsg.header.frame_id = "odom";
    odomMsg.header.stamp = current_time;
    odomMsg.child_frame_id = "base_link";

    // set the position
    odomMsg.pose.pose.position.x = odomX;
    odomMsg.pose.pose.position.y = odomY;
    odomMsg.pose.pose.position.z = 0.0;
    odomMsg.pose.pose.orientation = odomQuat;
    

    // set the velocity
    odomMsg.child_frame_id = "base_link";
    odomMsg.twist.twist.linear.x = vy;
    odomMsg.twist.twist.linear.y = vx;
    odomMsg.twist.twist.angular.z = w;

    odomPub_.publish(odomMsg);
}
