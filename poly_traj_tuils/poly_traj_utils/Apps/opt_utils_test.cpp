//
// Created by yunfan on 2021/3/6.
//
#include "vector"
#include "ros/ros.h"
#include "poly_traj_utils/am_traj_plus.hpp"
#include "poly_traj_utils/am_traj.hpp"
#include "poly_traj_utils/poly_visual_utils.hpp"

using namespace std;
typedef Eigen::Matrix<double, 3, 3> Mat33;
typedef Eigen::Matrix<double, 3, 1> Vec3;
nav_msgs::Odometry odom_;
order7::AmTraj::Ptr am_ptr_7;
order5::AmTraj::Ptr am_ptr_5;
Vec3 vels[2];
int idx = 0;
double init_v = 2.0;
ros::Publisher traj_jerk_pub, traj_snap_pub, opt_traj_pub,
        order_5_pub, order_7_pub, end_point_state_pub, vel_marker_pub, way_pts_pub,
        order_5_vel_pub, order_7_vel_pub;
Trajectory traj_5,traj_7;

/*  opt parameters */
struct OptParameters {
    /*
wT: Weight for the time regularization
wA: Weight for the integrated squared norm of acceleration
wJ: Weight for the integrated squared norm of jerk
mVr: Maximum velocity rate
mAr: Maximum acceleration rate
mIts: Maximum number of iterations in optimization
eps: Relative tolerance
*/
    double weight_T, weight_acc, weight_jerk, weight_snap, max_v, max_a, eps;
    int max_it;
    vector<Vec3> way_pts;
    bool use_order_7;
    bool use_order_5;
} op_7,op_5;


vector<Vec3> waypts;

void wayPoint_callback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    Vec3 cur_point;
    cur_point = Vec3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    ROS_INFO("Get target at %lf,%lf,%lf", cur_point.x(), cur_point.y(), cur_point.z());


    if (msg->pose.position.z > -0.1) {
        odom_.pose.pose = msg->pose;
        odom_.header.frame_id = "world";
        odom_.header.stamp = ros::Time::now();
        way_pts_pub.publish(odom_);
        waypts.push_back(cur_point);
    } else {
        if (op_5.use_order_7) {
            ROS_INFO("=======ORDER_7==================");
            Trajectory traj_7;vector<double> durations;
            {
                TimeConsuming t_71("order71");
                traj_7=  am_ptr_5->genOptimalTrajDTC(waypts, Vec3(0, 0, 0), Vec3(0, 0, 0), Vec3(0, 0, 0),
                                                     Vec3(0, 0, 0));
                traj_7 = am_ptr_7->genOptimalTrajDTCFromOrder5(traj_7, Vec3(0, 0, 0), Vec3(0, 0, 0), Vec3(0, 0, 0),
                                                               Vec3(0, 0, 0),Vec3(0, 0, 0),Vec3(0, 0, 0));

            }
            visualization_msgs::Marker vel_list;
            visualization_msgs::Marker acc_list;
            visualization_msgs::Marker jer_list;
            PolyVisual::publishTrajToPath(order_7_pub, traj_7);
            PolyVisual::publishTrajVelToMarker(order_7_vel_pub, traj_7, vel_list);
            PolyVisual::publishTrajAccToMarker(order_7_vel_pub, traj_7, acc_list);
            double total_time = traj_7.getTotalDuration();
            double total_length = traj_7.getTotalLength();
            printf("TRAJ TOTAL TIME   : \033[32m %f s\033[0m\n", (double) (total_time));
            printf("TRAJ TOTAL LENGTH : \033[32m %f m \033[0m\n", total_length);
            printf("AVERAGE SPEED  : \033[32m %lf m/s \033[0m \n", (double) total_length / total_time);
        }

        if (op_5.use_order_5) {
            ROS_INFO("=======ORDER_5==================");
            Trajectory traj_5;
            {
                TimeConsuming t_5("order5");
                traj_5 = am_ptr_5->genOptimalTrajDTC(waypts, Vec3(0, 0, 0), Vec3(0, 0, 0), Vec3(0, 0, 0),
                                                     Vec3(0, 0, 0));
            }
            visualization_msgs::Marker vel_list;
            visualization_msgs::Marker acc_list;
            visualization_msgs::Marker jer_list;
            PolyVisual::publishTrajToPath(order_5_pub, traj_5);
            PolyVisual::publishTrajVelToMarker(order_5_vel_pub, traj_5, vel_list);
            PolyVisual::publishTrajAccToMarker(order_5_vel_pub, traj_5, acc_list);
            double total_time = traj_5.getTotalDuration();
            double total_length = traj_5.getTotalLength();
            printf("TRAJ TOTAL TIME   : \033[32m %f s\033[0m\n", (double) (total_time));
            printf("TRAJ TOTAL LENGTH : \033[32m %f m \033[0m\n", total_length);
            printf("AVERAGE SPEED  : \033[32m %lf m/s \033[0m \n", (double) total_length / total_time);
        }
        waypts.clear();
    }

}


int main(int argc, char **argv) {

    ros::init(argc, argv, "opt_test");
    ros::NodeHandle nh("~");
    ros::Subscriber waypoints_sub = nh.subscribe("/goal", 1, wayPoint_callback);
    order_5_pub = nh.advertise<nav_msgs::Path>("/order_5/path", 1);
    order_7_pub = nh.advertise<nav_msgs::Path>("/order_7/path", 1);
    way_pts_pub = nh.advertise<nav_msgs::Odometry>("/waypoints", 10);
    order_5_vel_pub = nh.advertise<visualization_msgs::Marker>("/order_5/vel", 10);
    order_7_vel_pub = nh.advertise<visualization_msgs::Marker>("/order_7/vel", 10);
    am_ptr_5.reset(new order5::AmTraj);
    am_ptr_7.reset(new order7::AmTraj);

    nh.param<double>("/optimizer/order5/weight_T", op_5.weight_T, 1024.0);
    nh.param<double>("/optimizer/order5/weight_acc", op_5.weight_acc, 16.0);
    nh.param<double>("/optimizer/order5/weight_jerk", op_5.weight_jerk, 0.4);
    nh.param<double>("/optimizer/order5/weight_snap", op_5.weight_snap, 0.4);
    nh.param<double>("/optimizer/order5/max_a", op_5.max_a, 7.00);
    nh.param<double>("/optimizer/order5/max_v", op_5.max_v, 4.00);
    nh.param<int>("/optimizer/order5/max_it", op_5.max_it, 23);
    nh.param<double>("/optimizer/order5/eps", op_5.eps, 0.02);

    nh.param<double>("/optimizer/order7/weight_T", op_7.weight_T, 1024.0);
    nh.param<double>("/optimizer/order7/weight_acc", op_7.weight_acc, 16.0);
    nh.param<double>("/optimizer/order7/weight_jerk", op_7.weight_jerk, 0.4);
    nh.param<double>("/optimizer/order7/weight_snap", op_7.weight_snap, 0.4);
    nh.param<double>("/optimizer/order7/max_a", op_7.max_a, 7.00);
    nh.param<double>("/optimizer/order7/max_v", op_7.max_v, 4.00);
    nh.param<int>("/optimizer/order7/max_it", op_7.max_it, 23);
    nh.param<double>("/optimizer/order7/eps", op_7.eps, 0.02);

    nh.param<bool>("/manager/use_order_5", op_5.use_order_5, false);
    nh.param<bool>("/manager/use_order_7", op_5.use_order_7, false);



    am_ptr_7->init(op_7.weight_T, op_7.weight_acc, op_7.weight_jerk, op_7.weight_snap, op_7.max_v, op_7.max_a, op_7.max_it,
                   op_7.eps);
    am_ptr_5->init(op_5.weight_T, op_5.weight_acc, op_5.weight_jerk, op_5.max_v, op_5.max_a, op_5.max_it, op_5.eps);

    ros::Duration(1.0).sleep();

//    double global_path_x[] = {0, 0.910425186157, 2.03150987625, 3.07404398918, 4.07676744461, 5.05325651169, 5.02002382278, 4.13872098923, 3.17305445671, 1.87991642952, 0.852029323578, 1.59692049026, 0.995668888092, 2.2674677372, 3.76688718796, 5.30879402161, 5.054251194, 3.95133209229, 2.68733453751, 1.28811109066, -0.0613968372345, };
//    double global_path_y[] = {0, -0.0251655578613, 0.047304391861, 0.0137655735016, 0.019578576088, -0.00114870071411, -1.09645807743, -1.56336319447, -1.13355362415, -1.11466181278, -1.33172726631, -2.3697488308, -3.34960317612, -4.23943424225, -3.42589712143, -3.11349153519, -4.72465658188, -5.65464258194, -6.11057138443, -5.99993562698, -5.36124515533, };
//
//    for(int i = 0 ; i < 21 ; i++){
//
//        waypts.push_back(Vec3(global_path_x[i],global_path_y[i],0));
//    }
    waypts.push_back(Vec3(0, 0, 0));
    waypts.push_back(Vec3(0, 4, 0));
    waypts.push_back(Vec3(0, 5, 0));
    waypts.push_back(Vec3(0, 9, 0));
//    waypts.push_back(Vec3(2,4,0));
//    waypts.push_back(Vec3(2,2,0));
//    waypts.push_back(Vec3(2,0,0));
//    waypts.push_back(Vec3(3,4,0));
//    waypts.push_back(Vec3(3,6,0));
//    waypts.push_back(Vec3(3,8,0));
//    waypts.push_back(Vec3(4,6,0));
//    waypts.push_back(Vec3(4,0,0));

    if (op_5.use_order_7) {
        ROS_INFO("=======ORDER_7==================");
        Trajectory traj_7;vector<double> durations;
        {
            TimeConsuming t_71("order71");
            traj_7=  am_ptr_7->genOptimalTrajDTC(waypts, Vec3(0, 0, 0), Vec3(0, 0, 0), Vec3(0, 0, 0),
                                                 Vec3(0, 0, 0), Vec3(0, 0, 0), Vec3(0, 0, 0));
        }
//            {
//                TimeConsuming t_71("order71");
//                durations= am_ptr_5->getOptimalDurations(waypts, Vec3(0, 0, 0), Vec3(0, 0, 0),
//                                                         Vec3(0, 0, 0),
//                                                         Vec3(0, 0, 0));
//            }
//            {
//                TimeConsuming t_72("opt2");
//                traj_7=  am_ptr_7->genOptimalFast(waypts,durations, Vec3(0, 0, 0), Vec3(0, 0, 0), Vec3(0, 0, 0),
//                                                  Vec3(0, 0, 0), Vec3(0, 0, 0), Vec3(0, 0, 0));
//            }
        visualization_msgs::Marker vel_list;
        visualization_msgs::Marker acc_list;
        visualization_msgs::Marker jer_list;
        PolyVisual::publishTrajToPath(order_7_pub, traj_7);
        PolyVisual::publishTrajVelToMarker(order_7_vel_pub, traj_7, vel_list);
        PolyVisual::publishTrajAccToMarker(order_7_vel_pub, traj_7, acc_list);
        double total_time = traj_7.getTotalDuration();
        double total_length = traj_7.getTotalLength();
        printf("TRAJ TOTAL TIME   : \033[32m %f s\033[0m\n", (double) (total_time));
        printf("TRAJ TOTAL LENGTH : \033[32m %f m \033[0m\n", total_length);
        printf("AVERAGE SPEED  : \033[32m %lf m/s \033[0m \n", (double) total_length / total_time);
    }

    if (op_5.use_order_5) {
        ROS_INFO("=======ORDER_5==================");
        Trajectory traj_5;
        {
            TimeConsuming t_5("order5");
            traj_5 = am_ptr_5->genOptimalTrajDTC(waypts, Vec3(0, 0, 0), Vec3(0, 0, 0), Vec3(0, 0, 0),
                                                 Vec3(0, 0, 0));
        }
        visualization_msgs::Marker vel_list;
        visualization_msgs::Marker acc_list;
        visualization_msgs::Marker jer_list;
        PolyVisual::publishTrajToPath(order_5_pub, traj_5);
        PolyVisual::publishTrajVelToMarker(order_5_vel_pub, traj_5, vel_list);
        PolyVisual::publishTrajAccToMarker(order_5_vel_pub, traj_5, acc_list);
        double total_time = traj_5.getTotalDuration();
        double total_length = traj_5.getTotalLength();
        printf("TRAJ TOTAL TIME   : \033[32m %f s\033[0m\n", (double) (total_time));
        printf("TRAJ TOTAL LENGTH : \033[32m %f m \033[0m\n", total_length);
        printf("AVERAGE SPEED  : \033[32m %lf m/s \033[0m \n", (double) total_length / total_time);
    }

    waypts.clear();
    while (ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}

