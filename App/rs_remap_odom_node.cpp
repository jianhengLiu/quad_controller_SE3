//
// Created by yunfan on 8/9/20.
//

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Eigen>

ros::Publisher remap_odom,remap_to_px4;
double PI = 3.14159265358;
int cnt = 0;
nav_msgs::Odometry pubOdom;
geometry_msgs::PoseStamped pubPose;
void rsCallback(const nav_msgs::Odometry &msg){
    if(cnt<8)
    {
        cnt++;
        return;
    }

    cnt=0;

    pubOdom.header = msg.header;
//    pubOdom.header.frame_id = "map";
    pubOdom.header.frame_id = "odom";
    pubOdom.child_frame_id = "base_link";

    Eigen::Vector3d vins_position;
    vins_position.x() = msg.pose.pose.position.x;
    vins_position.y() = msg.pose.pose.position.y;
    vins_position.z() = msg.pose.pose.position.z;

    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(-PI / 2, Eigen::Vector3d::UnitZ()));

    vins_position = yawAngle * vins_position;

    pubOdom.pose.pose.position.x = vins_position.x();
    pubOdom.pose.pose.position.y = vins_position.y();
    pubOdom.pose.pose.position.z = vins_position.z();


    Eigen::Vector3d vins_vel;
    vins_vel.x() = msg.twist.twist.linear.x;
    vins_vel.y() = msg.twist.twist.linear.y;
    vins_vel.z() = msg.twist.twist.linear.z;

    vins_vel = yawAngle * vins_vel;

    pubOdom.twist.twist.linear.x = vins_vel.x();
    pubOdom.twist.twist.linear.y = vins_vel.y();
    pubOdom.twist.twist.linear.z = vins_vel.z();


    Eigen::Quaterniond vins_quat;
    vins_quat.x() = msg.pose.pose.orientation.x;
    vins_quat.y() = msg.pose.pose.orientation.y;
    vins_quat.z() = msg.pose.pose.orientation.z;
    vins_quat.w() = msg.pose.pose.orientation.w;

    auto vins_R = vins_quat.toRotationMatrix();
    auto vins_R_temp = vins_R;
    vins_R.col(0) = vins_R_temp.col(2);
    vins_R.col(1) = -vins_R_temp.col(0);
    vins_R.col(2) = -vins_R_temp.col(1);

    vins_quat = yawAngle * vins_R;

    pubOdom.pose.pose.orientation.x = vins_quat.x();
    pubOdom.pose.pose.orientation.y = vins_quat.y();
    pubOdom.pose.pose.orientation.z = vins_quat.z();
    pubOdom.pose.pose.orientation.w = vins_quat.w();

    Eigen::Vector3d vins_angular;
    vins_angular.x() = msg.twist.twist.angular.x;
    vins_angular.y() = msg.twist.twist.angular.z;
    vins_angular.z() = -msg.twist.twist.angular.y;

    vins_angular = yawAngle * vins_angular;

    pubOdom.twist.twist.angular.x = vins_angular.x();
    pubOdom.twist.twist.angular.y = vins_angular.y();
    pubOdom.twist.twist.angular.z = vins_angular.z();

    pubPose.header = msg.header;
    pubPose.header.frame_id = "map";

    pubPose.pose = pubOdom.pose.pose;
//    pubOdom.pose = msg.pose;
//    pubOdom.header.stamp = msg.header.stamp;
//    pubOdom.header.frame_id = "odom";
//    pubOdom.child_frame_id = "base_link";
    remap_odom.publish(pubOdom);
    remap_to_px4.publish(pubPose);

}

int main(int argc, char ** argv){
    ros::init(argc,argv,"rsRemap");
    ros::NodeHandle nh;
    ros::Subscriber joySub;
    joySub = nh.subscribe("/vins_estimator/imu_propagate", 400,rsCallback);
    remap_odom = nh.advertise<nav_msgs::Odometry>("/vins_remap_odom",400);///mavros/odometry/out
    remap_to_px4 = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",400);///mavros/odometry/out
    ros::spin();
    return 0;
}