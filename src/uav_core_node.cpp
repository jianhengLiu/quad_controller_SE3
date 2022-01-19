/*
 * @Author: Jianheng Liu
 * @Date: 2022-01-17 14:57:48
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2022-01-19 20:48:57
 * @Description: Description
 */

#include "uav_core/parameter.h"
#include "uav_core/uav_core.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "mav_fsm");
  ros::NodeHandle nh("~");

  Parameter_t param;
  param.readParam(nh);
  nros::UAVCore uav_core(param);

  ros::Subscriber sub_rc = nh.subscribe<mavros_msgs::RCIn>(
      "/mavros/rc/in", 10, boost::bind(&RC_t::input, &uav_core.rc, _1));

  uav_core.pub_ctrl_setpos = nh.advertise<geometry_msgs::PoseStamped>(
      "/mavros/setpoint_position/local", 10);

  ros::Rate rate(param.ctrl_freq_max);
  while (ros::ok()) {
    rate.sleep();
    ros::spinOnce();
    uav_core.process();
  }
  return 0;
}
