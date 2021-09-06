#ifndef MAIN_CONTROLLER_MAV_FSM_H
#define MAIN_CONTROLLER_MAV_FSM_H

#include "main_controller/common_include.h"
#include "SE3Control/SE3Control.h"
#include "main_controller/calculate.h"

#include "poly_traj_utils/am_traj_plus.hpp"
#include "poly_traj_utils/am_traj.hpp"
#include "poly_traj_utils/poly_visual_utils.hpp"

#include "mavros_msgs/AttitudeTarget.h"
#include "geometry_msgs/Quaternion.h"

#include "dynamic_reconfigure/server.h"
#include "main_controller/ControllerParamsConfig.h"

#include "quadrotor_msgs/PositionCommand.h"

namespace wtr {

    class MavFsmNode {
    private:
        int cnt;
        int sent_rate;
        ros::NodeHandle node_;
        ros::Subscriber rc_sub_, odom_sub_, imu_sub_, restart_sub_, cmd_sub_;
        ros::Publisher local_pos_pub_, speed_pub_, att_pub_, desi_odom_pub_, trigger_pub_;
        ros::Timer mav_fsm_;

        dynamic_reconfigure::Server<dynamic_params::ControllerParamsConfig> srv;
        dynamic_reconfigure::Server<dynamic_params::ControllerParamsConfig>::CallbackType f;

        geometry_msgs::PoseStamped takeoff_pose_;
        geometry_msgs::TwistStamped land_signal_, vel_ctl_signal_;

        bool is_init_pose_;

        SO3Control controller_;
        cal_c cal_;

        /* 5-th order poly traj generator */
        order5::AmTraj::Ptr am_ptr_5;
        Trajectory traj_5;

        bool allow_traj = true;

        double t_start, t_now;

        double mass_;

        struct vel_mode_t {
            int vel_x;
            int vel_y;
            int vel_z;
            double rotate;
        } vel_param;

        struct desired_t {
            Vec3 pos;
            Vec3 pos_dot;
            Vec3 pos_ddot;
            Vec3 pos_dddot;

            Mat33 rotation;
            Vec3 bodyRate;
            double thrust;
            double yaw;
            double yawD;
        } desired;


        struct feedback_t {
            Vec3 cPosition;
            Vec3 cPositionD;
            Vec3 cPositionDD;

            Mat33 rotation;
            Vec3 bodyRate;
            double thrust;
            double yaw;
            double yawD;
        } feedback;

        struct output_t {
            double force;
            Eigen::Quaterniond quad;
            geometry_msgs::Quaternion orientation;
            geometry_msgs::Vector3 body_rate;
        } output;

        struct fly_params_t {
            Vec3 k_position, k_velocity;
            double k_integral;
            double k_thrust;
        } fp;

        struct command_t {
            Eigen::Vector3d pos = Eigen::Vector3d(0, 0, 0);
            Eigen::Vector3d vel = Eigen::Vector3d(0, 0, 0);
            Eigen::Vector3d acc = Eigen::Vector3d(0, 0, 0);

            double yaw = 0.0;
        } cmd;

        typedef enum {
            takeoff,
            attctr,
            land,
            waiting,
            trajectory
        } workstate_t;
        workstate_t work_state_;

        /**
        std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
        uint8 rssi
        uint16[] channels
     * @param msg
     */

        void dynParamsCallback(dynamic_params::ControllerParamsConfig &config, uint32_t level);

        void rcCallback(const mavros_msgs::RCInConstPtr &msg);

        void positionCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr &cmd_msg);

        void imuCallback(const sensor_msgs::ImuConstPtr &imu_msg);

        void odomCallback(const nav_msgs::OdometryConstPtr &odom_msg);

        void restartCallback(const std_msgs::BoolConstPtr &restart_msg);

        void main_FSM(const ros::TimerEvent & /*event*/);

        void callSE3ControlOnce();

        /*  Define the mode callback functions  */
        void att_mode();

        void land_mode();

        void takeoff_mode();

        void wait_mode();

        void traj_mode();

    public:
        MavFsmNode(ros::NodeHandle &nh);

        ~MavFsmNode() {};
        typedef std::shared_ptr<MavFsmNode> Ptr;

    };

}


#endif //MAIN_CONTROLLER_MAV_FSM_H
