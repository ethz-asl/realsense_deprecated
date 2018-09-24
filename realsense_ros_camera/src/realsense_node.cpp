#include <nodelet/loader.h>
#include "ros/ros.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "realsense_ros_camera");
  nodelet::Loader manager;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  std::string nodelet_name = ros::this_node::getName();
  manager.load(nodelet_name, "realsense_ros_camera/RealSenseNodeFactory", remap, nargv);
  ros::spin();
  return 0;
}