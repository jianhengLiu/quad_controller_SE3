/*
 * @Author: Jianheng Liu
 * @Date: 2022-01-17 18:30:45
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2022-01-19 20:49:20
 * @Description: Description
 */
#pragma once

#include "geometry_msgs/PoseStamped.h"
#include "rc.h"
#include "uav_core/parameter.h"
#include <ros/ros.h>

namespace nros {

class UAVCore {

public:
  UAVCore(Parameter_t &_param) { param = _param; };

  ~UAVCore(){};
  Parameter_t param;
  RC_t rc;

  ros::Publisher pub_ctrl_setpos, pub_ctrl_setatt;

  void process();

private:
};

} // namespace nros
