/*
 * @Author: Jianheng Liu
 * @Date: 2022-01-17 19:08:16
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2022-01-19 20:37:59
 * @Description: 遥控器数据
 */
#pragma once
#include <mavros_msgs/RCIn.h>
class RC_t {
public:
  RC_t();
  void input(mavros_msgs::RCInConstPtr rc_msg);
};

inline void RC_t::input(mavros_msgs::RCInConstPtr rc_msg) {
}
