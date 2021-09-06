//
// Created by yunfan on 8/9/20.
//

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <eigen3/Eigen/Eigen>

ros::Publisher remap;
geometry_msgs::PoseStamped pubPose;
double PI = 3.14159265358;
int cnt = 0;
void rsCallback(const geometry_msgs::TransformStamped &msg) {
    pubPose.header = msg.header;
    pubPose.header.frame_id = "map";

    pubPose.pose.position.x = msg.transform.translation.x;
    pubPose.pose.position.y = msg.transform.translation.y;
    pubPose.pose.position.z = msg.transform.translation.z;

    pubPose.pose.orientation.x = msg.transform.rotation.x;
    pubPose.pose.orientation.y = msg.transform.rotation.y;
    pubPose.pose.orientation.z = msg.transform.rotation.z;
    pubPose.pose.orientation.w = msg.transform.rotation.w;

    remap.publish(pubPose);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rsRemap");
    ros::NodeHandle nh;
    ros::Subscriber joySub;
    joySub = nh.subscribe("/vicon/uav/uav", 400, rsCallback);///vins_estimator/odometry", 1,rsCallback);/camera/odom/sample
    remap = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 400);
    ros::spin();
    return 0;
}
