#include <nodelet/loader.h>
#include "ros/ros.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "realsense_ros_camera");

  nodelet::Loader manager;
  nodelet::V_string nargv;
  nodelet::M_string remap(ros::names::getRemappings());

  std::string nodelet_name = ros::this_node::getName();

  manager.load(nodelet_name + "/RealSenseNodeFactory",
               "realsense_ros_camera/RealSenseNodeFactory", remap, nargv);
  ROS_INFO_STREAM("Started " << nodelet_name << "/RealSenseNodeFactory"
                             << " nodelet.");

  ros::spin();
  return 0;
}