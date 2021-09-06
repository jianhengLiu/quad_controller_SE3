#include "main_controller/common_include.h"
#include "cstring"
#include "sensor_msgs/BatteryState.h"
#include "cstdio"
#include "cstdlib"

using namespace std;
string mode="MANULE";
double voltage;
Eigen::Vector3d mavros_pos,vicon_pos;
void staCallback(const mavros_msgs::StateConstPtr &msg){
    mode =  msg->mode;
}
void visCallback(const geometry_msgs::PoseStampedConstPtr &msg){
    vicon_pos.x() = msg->pose.position.x;
    vicon_pos.y() = msg->pose.position.y;
    vicon_pos.z() = msg->pose.position.z;
}
void posCallback(const geometry_msgs::PoseStampedConstPtr &msg){
    mavros_pos.x() = msg->pose.position.x;
    mavros_pos.y() = msg->pose.position.y;
    mavros_pos.z() = msg->pose.position.z;
}
void batCallback(const sensor_msgs::BatteryStateConstPtr &msg){
    voltage =  msg->voltage;
}

double dis(Eigen::Vector3d a,Eigen::Vector3d b){
    return (a-b).norm();
}

int main(int argc, char ** argv){
//    printf("\x1b[%d;%dm%s\x1b[%dm", 1, 2, 3);
    ros::init(argc,argv,"showStatus");
    ros::NodeHandle nh;
    ros::Subscriber state_sub, vision_sub, pose_sub, battery_sub;
    state_sub = nh.subscribe("/mavros/state", 1,staCallback);
    vision_sub = nh.subscribe("/mavros/vision_pose/pose", 1,visCallback);
    pose_sub = nh.subscribe("/mavros/local_position/pose", 1,posCallback);
    battery_sub = nh.subscribe("/mavros/battery", 1,batCallback);
    ros::Rate loopRate(1);
    int i, j;
    voltage = 14.8;
    while(ros::ok()){

        system("clear");
        printf("\033[0m");
        printf("\033[45;37m--------------------------------------------------------------------------------\n");
        printf("\033[45;37m                       WTR UAV with Suspended Payload                           \n");
        printf("\033[45;37m--------------------------------------------------------------------------------\n");
        printf("\033[45;37m|  Flight Mode: %s                                                              \n", mode.c_str());
//        printf("\033[45;37m|  Vicon      : (%.4lf,%.4lf,%.4lf)                                               \n", vicon_pos.x(),vicon_pos.y(),vicon_pos.z());

        if(vicon_pos.norm() < 1){
            printf("\033[45;32m|  VIO     : (%.4lf,%.4lf,%.4lf)                                               \n", (double)vicon_pos.x(),(double)vicon_pos.y(),(double)vicon_pos.z());
        }

        else{
            printf("\033[45;31m|  VIO     : (%.4lf,%.4lf,%.4lf)                                               \n", (double)vicon_pos.x(),(double)vicon_pos.y(),(double)vicon_pos.z());

        }

        if(voltage>15.5 && voltage < 16.9){
            printf("\033[45;32m|  Battery    : %.4lf                                                          \n", voltage);
        }

        else{
            printf("\033[45;31m|  Battery    : %.4lf                                                           \n", voltage);
        }

        printf("\033[45;37m--------------------------------------------------------------------------------\n");
        printf("\033[0m\n");

        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
