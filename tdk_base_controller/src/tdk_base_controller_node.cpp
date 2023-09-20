#include "tdk_base_controller/tdk_base_controller.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tdk_base_controller");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  TDKBaseController tdkMoveBase (nh, nh_private);
  
  ros::spin();
  return 0;
}