//
// Created by kevin on 2020/8/4.
//

#ifndef TRACKER_COMMON_INCLUDE_H
#define TRACKER_COMMON_INCLUDE_H

#include "iostream"

// ROS
#include <ros/ros.h>


#include "mavros_msgs/RCIn.h"
#include "geometry_msgs/TwistStamped.h"
// ROS Msg Type
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include "geometry_msgs/Pose.h"
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
//Tf
#include <tf/transform_datatypes.h>

//Eigen
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>

#define PI 3.1415926535



// typedefs for eigen
// double matricies
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatXX;
typedef Eigen::Matrix<double, 10, 10> Mat1010;
typedef Eigen::Matrix<double, 13, 13> Mat1313;
typedef Eigen::Matrix<double, 8, 10> Mat810;
typedef Eigen::Matrix<double, 8, 3> Mat83;
typedef Eigen::Matrix<double, 6, 6> Mat66;
typedef Eigen::Matrix<double, 5, 3> Mat53;
typedef Eigen::Matrix<double, 4, 3> Mat43;
typedef Eigen::Matrix<double, 4, 2> Mat42;
typedef Eigen::Matrix<double, 3, 3> Mat33;
typedef Eigen::Matrix<double, 2, 2> Mat22;
typedef Eigen::Matrix<double, 8, 8> Mat88;
typedef Eigen::Matrix<double, 7, 7> Mat77;
typedef Eigen::Matrix<double, 4, 9> Mat49;
typedef Eigen::Matrix<double, 8, 9> Mat89;
typedef Eigen::Matrix<double, 9, 4> Mat94;
typedef Eigen::Matrix<double, 9, 8> Mat98;
typedef Eigen::Matrix<double, 8, 1> Mat81;
typedef Eigen::Matrix<double, 1, 8> Mat18;
typedef Eigen::Matrix<double, 9, 1> Mat91;
typedef Eigen::Matrix<double, 1, 9> Mat19;
typedef Eigen::Matrix<double, 8, 4> Mat84;
typedef Eigen::Matrix<double, 4, 8> Mat48;
typedef Eigen::Matrix<double, 4, 4> Mat44;
typedef Eigen::Matrix<double, 3, 4> Mat34;
typedef Eigen::Matrix<double, 14, 14> Mat1414;

// float matricies
typedef Eigen::Matrix<float, 3, 3> Mat33f;
typedef Eigen::Matrix<float, 10, 3> Mat103f;
typedef Eigen::Matrix<float, 2, 2> Mat22f;
typedef Eigen::Matrix<float, 3, 1> Vec3f;
typedef Eigen::Matrix<float, 2, 1> Vec2f;
typedef Eigen::Matrix<float, 6, 1> Vec6f;
typedef Eigen::Matrix<float, 1, 8> Mat18f;
typedef Eigen::Matrix<float, 6, 6> Mat66f;
typedef Eigen::Matrix<float, 8, 8> Mat88f;
typedef Eigen::Matrix<float, 8, 4> Mat84f;
typedef Eigen::Matrix<float, 6, 6> Mat66f;
typedef Eigen::Matrix<float, 4, 4> Mat44f;
typedef Eigen::Matrix<float, 12, 12> Mat1212f;
typedef Eigen::Matrix<float, 13, 13> Mat1313f;
typedef Eigen::Matrix<float, 10, 10> Mat1010f;
typedef Eigen::Matrix<float, 9, 9> Mat99f;
typedef Eigen::Matrix<float, 4, 2> Mat42f;
typedef Eigen::Matrix<float, 6, 2> Mat62f;
typedef Eigen::Matrix<float, 1, 2> Mat12f;
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> MatXXf;
typedef Eigen::Matrix<float, 14, 14> Mat1414f;

// double vectors
typedef Eigen::Matrix<double, 14, 1> Vec14;
typedef Eigen::Matrix<double, 13, 1> Vec13;
typedef Eigen::Matrix<double, 10, 1> Vec10;
typedef Eigen::Matrix<double, 9, 1> Vec9;
typedef Eigen::Matrix<double, 8, 1> Vec8;
typedef Eigen::Matrix<double, 7, 1> Vec7;
typedef Eigen::Matrix<double, 6, 1> Vec6;
typedef Eigen::Matrix<double, 5, 1> Vec5;
typedef Eigen::Matrix<double, 4, 1> Vec4;
typedef Eigen::Matrix<double, 3, 1> Vec3;
typedef Eigen::Matrix<double, 2, 1> Vec2;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VecX;

// float vectors
typedef Eigen::Matrix<float, 12, 1> Vec12f;
typedef Eigen::Matrix<float, 8, 1> Vec8f;
typedef Eigen::Matrix<float, 10, 1> Vec10f;
typedef Eigen::Matrix<float, 4, 1> Vec4f;
typedef Eigen::Matrix<float, 12, 1> Vec12f;
typedef Eigen::Matrix<float, 13, 1> Vec13f;
typedef Eigen::Matrix<float, 9, 1> Vec9f;
typedef Eigen::Matrix<float, Eigen::Dynamic, 1> VecXf;
typedef Eigen::Matrix<float, 14, 1> Vec14f;


#endif //TRACKER_COMMON_INCLUDE_H
