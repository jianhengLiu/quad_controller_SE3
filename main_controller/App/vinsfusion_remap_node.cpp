#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <eigen3/Eigen/Eigen>

ros::Publisher remap;
geometry_msgs::PoseStamped pubPose;
double PI = 3.14159265358;
int cnt = 0;
void rsCallback(const nav_msgs::Odometry &msg) {
    if(cnt<4)
    {
	    cnt++;
	    return;
    }

    cnt=0;

    pubPose.header = msg.header;
    pubPose.header.frame_id = "map";

    pubPose.pose.position.x = msg.pose.pose.position.x;
    pubPose.pose.position.y = msg.pose.pose.position.y;
    pubPose.pose.position.z = msg.pose.pose.position.z;

    pubPose.pose.orientation.x = msg.pose.pose.orientation.x;
    pubPose.pose.orientation.y = msg.pose.pose.orientation.y;
    pubPose.pose.orientation.z = msg.pose.pose.orientation.z;
    pubPose.pose.orientation.w = msg.pose.pose.orientation.w;

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
