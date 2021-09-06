//
// Created by yunfan on 2021/3/19.
//
#include "poly_traj_utils/obvp_solver.hpp"
#include "poly_traj_utils/poly_visual_utils.hpp"
#include "ros/ros.h"
#include "poly_traj_utils/bvp_solver.h"

Vec3 points[2];
Vec3 vels[2];
ros::Publisher traj_snap_pub, traj_opt_pub, end_point_state_pub, start_point_state_pub,
        obvp_opt_pub, obvp_heu_pub, vel_pub2, vel_pub1;
int idx = 0;
ObvpSolver solve_;
BVPSolver::IntegratorBVP bvp_;

void wayPoint_callback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    Mat33 R = MsgUtils::poseToEigenRotation(msg->pose);
    points[idx] = Vec3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    ROS_INFO("Get target at %lf,%lf,%lf", points[idx].x(), points[idx].y(), points[idx].z());


    if (points[idx].z() < 1e-3)
        points[idx].z() = 2.5;
    Vec3 W(3, 0, 0);
    vels[idx] = R * W;
    if (idx) {
        Piece cur_p2;
        visualization_msgs::Marker list1, list2, acc_1, acc_2;
        geometry_msgs::PoseStamped point;
        point.pose = msg->pose;
        point.pose.position.z = points[idx].z();
        point.header.frame_id = "world";
        point.header.stamp = ros::Time::now();
        end_point_state_pub.publish(point);
        TimeConsuming t__("optimal");
        StatePVA start_state, end_state;
        start_state << points[0], vels[0], 0, 0, 0;
        end_state << points[1], vels[1], 0, 0, 0;
        if (bvp_.solve(start_state, end_state)) {
            DynamicMat coeff;
            bvp_.getCoeff(coeff);
            cur_p2 = Piece(bvp_.getTauStar(), coeff);
        }

        t__.stop();
        PolyVisual::publishPieceToPath(obvp_opt_pub, cur_p2);
        PolyVisual::publishPieceVelToMarker(vel_pub1, cur_p2, list1);
        PolyVisual::publishPieceAccToMarker(vel_pub1, cur_p2, acc_2);


        Piece cur_p1;
        TimeConsuming t_("heru");

        cur_p1 = solve_.genObvpMinSnapTraj_HeuEnd(points[0],vels[0],Vec3(0, 0, 0),Vec3(0, 0, 0),
                                           points[1],5, 3,true);

        t_.stop();
        PolyVisual::publishPieceToPath(obvp_heu_pub, cur_p1);
        PolyVisual::publishPieceVelToMarker(vel_pub2, cur_p1, list2);;
        PolyVisual::publishPieceAccToMarker(vel_pub2, cur_p1, acc_1);

        idx = 0;
    } else {
        geometry_msgs::PoseStamped point;
        point.pose = msg->pose;
        point.pose.position.z = points[idx].z();
        point.header.frame_id = "world";
        point.header.stamp = ros::Time::now();
        start_point_state_pub.publish(point);
        idx = 1;
    }

}

bool is_map = false;

int main(int argc, char **argv) {

    ros::init(argc, argv, "example");
    ros::NodeHandle nh("~");
    obvp_opt_pub = nh.advertise<nav_msgs::Path>("obvp_path_opt", 1);
    obvp_heu_pub = nh.advertise<nav_msgs::Path>("obvp_path_heu", 1);
    vel_pub1 = nh.advertise<visualization_msgs::Marker>("vel_and_acc_opt", 1);
    vel_pub2 = nh.advertise<visualization_msgs::Marker>("vel_and_acc_heu", 1);
    ros::Subscriber waypoints_sub = nh.subscribe("/goal", 1, wayPoint_callback);
    start_point_state_pub = nh.advertise<geometry_msgs::PoseStamped>("/start_point_vis", 10);
    end_point_state_pub = nh.advertise<geometry_msgs::PoseStamped>("/end_point_vis", 10);

    solve_.init(1000, 1, 1, 1.0);
    bvp_.init(TRIPLE_INTEGRATOR);
    bvp_.setRho(0.01);
    ros::Duration(1).sleep();
//    TimeConsuming t__("optimal");
//    Piece cur_p2= solve_.genObvpMinSnapTraj(start_state,end_state);
//    t__.stop();



    ros::spin();
    return 0;
}

