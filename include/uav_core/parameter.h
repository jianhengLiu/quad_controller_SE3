/*
 * @Author: Jianheng Liu
 * @Date: 2022-01-19 20:22:18
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2022-01-19 20:48:03
 * @Description: Description
 */
#pragma once
#include "geometry_msgs/Pose.h"
#include "ros/ros.h"

class Parameter_t {
public:
  Parameter_t(){};
  void readParam(const ros::NodeHandle &nh);

  int ctrl_freq_max;
  geometry_msgs::Pose takeoff_pose;

private:
  template <typename TName, typename TVal>
  void read_essential_param(const ros::NodeHandle &nh, const TName &name,
                            TVal &val) {
    if (nh.getParam(name, val)) {
      // pass
    } else {
      ROS_ERROR_STREAM("Read param: " << name << " failed.");
      ROS_BREAK();
    }
  };
};

inline void Parameter_t::readParam(const ros::NodeHandle &nh) {
  read_essential_param(nh, "ctrl_freq_max", ctrl_freq_max);
  read_essential_param(nh, "takeoff_pose/pos_x", takeoff_pose.position.x);
  read_essential_param(nh, "takeoff_pose/pos_y", takeoff_pose.position.y);
  read_essential_param(nh, "takeoff_pose/pos_z", takeoff_pose.position.z);
  read_essential_param(nh, "takeoff_pose/att_x", takeoff_pose.orientation.x);
  read_essential_param(nh, "takeoff_pose/att_y", takeoff_pose.orientation.y);
  read_essential_param(nh, "takeoff_pose/att_z", takeoff_pose.orientation.z);
  read_essential_param(nh, "takeoff_pose/att_w", takeoff_pose.orientation.w);
}