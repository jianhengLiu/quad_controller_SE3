//
// Created by yunfan on 8/9/20.
//

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <eigen3/Eigen/Eigen>

ros::Publisher remap;
geometry_msgs::PoseStamped pubPose;
double PI = 3.14159265358;
int cnt = 0;
void rsCallback(const nav_msgs::Odometry &msg) {
    if(cnt<8)
    {
	    cnt++;
	    return;
    }

    cnt=0;

    pubPose.header = msg.header;
    pubPose.header.frame_id = "map";

    Eigen::Vector3d vins_position;
    vins_position.x() = msg.pose.pose.position.x;
    vins_position.y() = msg.pose.pose.position.y;
    vins_position.z() = msg.pose.pose.position.z;

    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(-PI / 2, Eigen::Vector3d::UnitZ()));

    vins_position = yawAngle * vins_position;

    pubPose.pose.position.x = vins_position.x();
    pubPose.pose.position.y = vins_position.y();
    pubPose.pose.position.z = vins_position.z();


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

    pubPose.pose.orientation.x = vins_quat.x();
    pubPose.pose.orientation.y = vins_quat.y();
    pubPose.pose.orientation.z = vins_quat.z();
    pubPose.pose.orientation.w = vins_quat.w();

    remap.publish(pubPose);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rsRemap");
    ros::NodeHandle nh;
    ros::Subscriber joySub;
    joySub = nh.subscribe("/vins_estimator/imu_propagate", 400, rsCallback);///vins_estimator/odometry", 1,rsCallback);/camera/odom/sample
    remap = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 400);
    ros::spin();
    return 0;
}
