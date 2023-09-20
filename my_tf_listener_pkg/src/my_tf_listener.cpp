#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <my_tf_listener_pkg/RobotState.h> 

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  tf::TransformListener listener;

  ros::Publisher state_pub = node.advertise<my_tf_listener_pkg::RobotState>("robot_state", 10);

  double last_x = 0.0, last_y = 0.0;
  bool first_iteration = true;
  double total_distance_moved = 0.0;

  ros::Rate rate(8.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    // Quaternion to RPY conversion
    tf::Quaternion q(
      transform.getRotation().getX(), 
      transform.getRotation().getY(), 
      transform.getRotation().getZ(), 
      transform.getRotation().getW()
    );
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Get x and y coordinates
    double x = transform.getOrigin().x();
    double y = transform.getOrigin().y();

    // Calculate the distance moved
    double distance_moved = 0.0;
    if (!first_iteration) {
        distance_moved = sqrt(pow(x - last_x, 2) + pow(y - last_y, 2));
        if (fabs(x - last_x) > 0.0025 || fabs(y - last_y) > 0.0025) {
            total_distance_moved += distance_moved;
        }
    } else {
        first_iteration = false;
    }

    // Store current position for next iteration
    last_x = x;
    last_y = y;
    

    my_tf_listener_pkg::RobotState state_msg;
    state_msg.yaw = yaw;
    state_msg.x = x;
    state_msg.y = y;
    state_msg.distance_moved = distance_moved;
    state_msg.total_distance_moved = total_distance_moved;

    state_pub.publish(state_msg);
  }
  return 0;
}

