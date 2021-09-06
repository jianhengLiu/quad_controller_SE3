#include "main_controller/mav_fsm.h"

using namespace wtr;
using namespace std;

bool is_land = false;
bool is_takeoff = false;
bool is_takeoff_traj = false;
int traj_n = 0;
vector<vector<Vec3>> wayptsArray;

void MavFsmNode::att_mode() {
    if (cnt++ > 50)// make the information sent_rate reduce to 1Hz.
    {
        cnt = 0;
        ROS_INFO("[WTR MAV MAIN FSM][ATT]:-------------- \033[33m ATT CONTROL WORKING \033[0m-------------------");
    }

    if (!is_takeoff_traj) {
        am_ptr_5->init(1024.0, 16.0, 0.4, 0.20, 0.10, 23, 0.02);

        vector<Vec3> waypts;
        waypts.push_back(Vec3(0, 0, 0));
        waypts.push_back(
                Vec3(0, 0, takeoff_pose_.pose.position.z));

        traj_5 = am_ptr_5->genOptimalTrajDTC(waypts, Vec3(0, 0, 0), Vec3(0, 0, 0),
                                             Vec3(0, 0, 0), Vec3(0, 0, 0));
        is_takeoff_traj = true;
        t_start = ros::Time::now().toSec();
    }

    t_now = ros::Time::now().toSec();
    double dt = t_now - t_start;

    if (dt >= traj_5.getTotalDuration() && !is_takeoff) {
        is_takeoff = true;
        t_start = 0;
        cmd.vel = Eigen::Vector3d::Zero();
        cmd.acc = Eigen::Vector3d::Zero();
        cout << "Reset t_start!" << endl;
    }

    if (!is_takeoff && is_takeoff_traj) {
        cmd.pos = traj_5.getPos(dt);
        cmd.vel = traj_5.getVel(dt);
        cmd.acc = traj_5.getAcc(dt);
        callSE3ControlOnce();
    } else {
//        cmd.pos.x() = takeoff_pose_.pose.position.x;
//        cmd.pos.y() = takeoff_pose_.pose.position.y;
//        cmd.pos.z() = takeoff_pose_.pose.position.z;
//        cmd.vel = Eigen::Vector3d::Zero();
//        cmd.acc = Eigen::Vector3d::Zero();
        callSE3ControlOnce();
    }
}

void MavFsmNode::traj_mode() {
    if (cnt++ > 50)// make the information sent_rate reduce to 1Hz.
    {
        cnt = 0;
        ROS_INFO("[WTR MAV MAIN FSM][TRAJECTORY]:--------------\033[35m TRAJECTORY \033[0m-------------------");
    }

//    if (t_start == 0) {
//        am_ptr_5->init(1024.0, 16.0, 0.4, 1.30, 1.30, 23, 0.02);
//        traj_5 = am_ptr_5->genOptimalTrajDTC(wayptsArray[traj_n], Vec3(0, 0, 0), Vec3(0, 0, 0),
//                                             Vec3(0, 0, 0), Vec3(0, 0, 0));
//        allow_traj = true;
//        t_start = ros::Time::now().toSec();
//    }
//    t_now = ros::Time::now().toSec();
//    double dt = t_now - t_start;
//
//    if (dt >= traj_5.getTotalDuration()) {
//        takeoff_pose_.pose.position.x = wayptsArray[traj_n].back().x();
//        takeoff_pose_.pose.position.y = wayptsArray[traj_n].back().y();
//        takeoff_pose_.pose.position.z = wayptsArray[traj_n].back().z();
//
//        allow_traj = false;
//        work_state_ = attctr;
//        t_start = 0;
//        ROS_INFO("Reset t_start!");
//
//        traj_n++;
//        if (traj_n < wayptsArray.size()) {
//            traj_5 = am_ptr_5->genOptimalTrajDTC(wayptsArray[traj_n], Vec3(0, 0, 0), Vec3(0, 0, 0),
//                                                 Vec3(0, 0, 0), Vec3(0, 0, 0));
//        } else {
//            traj_n = 0;
//            traj_5 = am_ptr_5->genOptimalTrajDTC(wayptsArray[traj_n], Vec3(0, 0, 0), Vec3(0, 0, 0),
//                                                 Vec3(0, 0, 0), Vec3(0, 0, 0));
//        }
//    }
//
//    if (allow_traj) {
//        cmd.pos = traj_5.getPos(dt);
//        cmd.vel = traj_5.getVel(dt);
//        cmd.acc = traj_5.getAcc(dt);
//        callSE3ControlOnce();
//    }

    geometry_msgs::PoseStamped pose_stamped_temp;
    pose_stamped_temp.header.stamp = ros::Time::now();
    trigger_pub_.publish(pose_stamped_temp);

        allow_traj = false;
        work_state_ = attctr;
}

void MavFsmNode::takeoff_mode() {
    if (cnt++ > 50)// make the information sent_rate reduce to 1Hz.
    {
        cnt = 0;
        ROS_INFO("[WTR MAV MAIN FSM][TAKEOFF]:--------------\033[36m TAKEOFF \033[0m-------------------");
    }
    takeoff_pose_.header.stamp = ros::Time::now();

    local_pos_pub_.publish(takeoff_pose_);
}

void MavFsmNode::land_mode() {
    if (cnt++ > 50)// make the information sent_rate reduce to 1Hz.
    {
        cnt = 0;
        ROS_INFO("[WTR MAV MAIN FSM][LAND]:--------------\033[32m LANDING \033[0m-------------------");
    }
    land_signal_.header.stamp = ros::Time::now();
    land_signal_.header.frame_id = "body";
    speed_pub_.publish(land_signal_);

}

void MavFsmNode::wait_mode() {
    if (cnt++ > 50)// make the information sent_rate reduce to 1Hz.
    {
        cnt = 0;
        ROS_INFO("[WTR MAV MAIN FSM][WAITING]:--------------Waiting for command.-------------------");
    }
}

void MavFsmNode::main_FSM(const ros::TimerEvent &) {
//    if (!is_init_pose_) work_state_ = waiting;

    switch (work_state_) {
        case waiting: {
            wait_mode();
            break;
        }
        case takeoff: {
            takeoff_mode();
            break;
        }
        case land: {
            land_mode();
            break;
        }
        case attctr: {
            att_mode();
            break;
        }
        case trajectory: {
            traj_mode();
            break;
        }
        default: {
            wait_mode();
        }
    }
    ros::spinOnce();
}

void MavFsmNode::rcCallback(const mavros_msgs::RCInConstPtr &msg) {
    //WFT09II遥控器
    if (!is_land) {
        if (msg->channels[7] < 1500) {    // E开关向上 降落
            work_state_ = land;
        } else if (msg->channels[5] < 1500) { // 开关C向下
            work_state_ = takeoff;
        } else if (msg->channels[5] > 1500) { // 开关C向shang 使用姿态控制器
            if (work_state_ == takeoff) {
                work_state_ = attctr;
            }
            if (work_state_ == attctr) {
                if (msg->channels[6] > 1900 && allow_traj) {
                    work_state_ = trajectory;
                } else if (msg->channels[6] < 1500) {
                    allow_traj = true;
                }
            }
        }
    }
}

void MavFsmNode::imuCallback(const sensor_msgs::ImuConstPtr &imu_msg) {

    feedback.cPositionDD = Vec3(imu_msg->linear_acceleration.x,
                                imu_msg->linear_acceleration.y,
                                imu_msg->linear_acceleration.z);

    feedback.bodyRate = Vec3(imu_msg->angular_velocity.x,
                             imu_msg->angular_velocity.y,
                             imu_msg->angular_velocity.z);

    controller_.setAcc(feedback.cPositionDD);
}

void MavFsmNode::odomCallback(const nav_msgs::OdometryConstPtr &odom_msg) {
    feedback.cPosition = Vec3(
            odom_msg->pose.pose.position.x,
            odom_msg->pose.pose.position.y,
            odom_msg->pose.pose.position.z
    );
    feedback.cPositionD = Vec3(
            odom_msg->twist.twist.linear.x,
            odom_msg->twist.twist.linear.y,
            odom_msg->twist.twist.linear.z
    );

    Eigen::Quaterniond q(
            odom_msg->pose.pose.orientation.w,
            odom_msg->pose.pose.orientation.x,
            odom_msg->pose.pose.orientation.y,
            odom_msg->pose.pose.orientation.z
    );

    feedback.rotation = q.toRotationMatrix();

    controller_.setPosition(feedback.cPosition);
    controller_.setVelocity(feedback.cPositionD);
    controller_.setRotation(feedback.rotation);

    if (feedback.cPosition.z() > 2 || feedback.cPosition.z() < -1) {
//        work_state_ = land;
    }
    is_init_pose_ = true;
}

void MavFsmNode::restartCallback(const std_msgs::BoolConstPtr &restart_msg) {
    if (restart_msg->data == true) {
        work_state_ = land;
        is_land = true;
    }
}

void MavFsmNode::dynParamsCallback(dynamic_params::ControllerParamsConfig &config, uint32_t level) {
    ROS_INFO("\n"
             "Reconfigure Request: \n"
             "takeoff_pose_:[%f, %f, %f] \n"
             "k_position:[%f, %f, %f] \n"
             "k_velocity:[%f, %f, %f] \n"
             "k_integral:[%f] \n"
             "k_thrust:[%f]",
             config.takeoff_pose_x, config.takeoff_pose_y, config.takeoff_pose_z,
             config.k_position_x, config.k_position_y, config.k_position_z,
             config.k_velocity_x, config.k_velocity_y, config.k_velocity_z,
             config.k_integral,
             config.k_thrust);

    takeoff_pose_.pose.position.x = config.takeoff_pose_x;
    takeoff_pose_.pose.position.y = config.takeoff_pose_y;
    takeoff_pose_.pose.position.z = config.takeoff_pose_z;

    fp.k_position.x() = config.k_position_x;
    fp.k_position.y() = config.k_position_y;
    fp.k_position.z() = config.k_position_z;

    fp.k_velocity.x() = config.k_velocity_x;
    fp.k_velocity.y() = config.k_velocity_y;
    fp.k_velocity.z() = config.k_velocity_z;

    fp.k_integral = config.k_integral;

    fp.k_thrust = config.k_thrust;

    work_state_ = workstate_t(config.work_state);
}

void MavFsmNode::positionCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr &cmd_msg) {
    cmd.pos = Eigen::Vector3d(cmd_msg->position.x, cmd_msg->position.y, cmd_msg->position.z);
    cmd.vel = Eigen::Vector3d(cmd_msg->velocity.x, cmd_msg->velocity.y, cmd_msg->velocity.z);
    cmd.acc = Eigen::Vector3d(cmd_msg->acceleration.x, cmd_msg->acceleration.y, cmd_msg->acceleration.z);
    cmd.yaw = cmd_msg->yaw;


//    position_cmd_updated_ = true;
}

MavFsmNode::MavFsmNode(ros::NodeHandle &nh) {
    node_ = nh;

    cnt = 0;
    is_init_pose_ = false;
    t_start = 0;
    t_now = 0;
    work_state_ = waiting;

    /*  Define the pulisher of point and velocity.  */
    local_pos_pub_ = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 10);
    speed_pub_ = nh.advertise<geometry_msgs::TwistStamped>
            ("/mavros/setpoint_velocity/cmd_vel", 10);
    att_pub_ = nh.advertise<mavros_msgs::AttitudeTarget>
            ("/mavros/setpoint_raw/attitude", 10);
    desi_odom_pub_ = nh.advertise<nav_msgs::Odometry>("/desired/odom", 10);
    trigger_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 1);

    /*  Define the subscriber of mocap system or realsense. */
    odom_sub_ = nh.subscribe(
            "/mavros/local_position/odom", 1000, &MavFsmNode::odomCallback, this);///vins_estimator/imu_propagate
    imu_sub_ = nh.subscribe(
            "/mavros/imu/data", 1000, &MavFsmNode::imuCallback, this);
    restart_sub_ = nh.subscribe(
            "/feature_tracker/restart", 10, &MavFsmNode::restartCallback, this);
    /*  Define the mavros RC callback.*/
    rc_sub_ = nh.subscribe
            ("/mavros/rc/in", 10, &MavFsmNode::rcCallback, this);
    cmd_sub_ = nh.subscribe("/planning/pos_cmd", 10, &MavFsmNode::positionCmdCallback, this);


    mav_fsm_ = node_.createTimer(ros::Duration(0.02), &MavFsmNode::main_FSM, this);

    ROS_INFO("INIT FSM TIMER SUCCESS!");

    /*  Set the dynamic params */
    f = boost::bind(&MavFsmNode::dynParamsCallback, this, _1, _2);
    srv.setCallback(f);

    /*  Get the parameters */
    nh.param<double>("/k_position/x", fp.k_position.x(), 0.0);
    nh.param<double>("/k_position/y", fp.k_position.y(), 0.0);
    nh.param<double>("/k_position/z", fp.k_position.z(), 0.0);

    nh.param<double>("/k_velocity/x", fp.k_velocity.x(), 0.0);
    nh.param<double>("/k_velocity/y", fp.k_velocity.y(), 0.0);
    nh.param<double>("/k_velocity/z", fp.k_velocity.z(), 0.0);

    nh.param<double>("/k_velocity/x", fp.k_velocity.x(), 0.0);
    nh.param<double>("/k_velocity/y", fp.k_velocity.y(), 0.0);
    nh.param<double>("/k_velocity/z", fp.k_velocity.z(), 0.0);

    nh.param<double>("/k_integral", fp.k_integral, 0.0);

    nh.param<double>("/k_thrust", fp.k_thrust, 0.0);

    nh.param<double>("/takeoff_pose/x", takeoff_pose_.pose.position.x, 0.0);
    nh.param<double>("/takeoff_pose/y", takeoff_pose_.pose.position.y, 0.0);
    nh.param<double>("/takeoff_pose/z", takeoff_pose_.pose.position.z, 0.0);

    nh.param<double>("/mass", mass_, 0.0);

    controller_.setMass(mass_);
    controller_.setGravity(9.8);

    am_ptr_5.reset(new order5::AmTraj);
    am_ptr_5->init(1024.0, 16.0, 0.4, 0.30, 0.30, 23, 0.02);

    /* triangle */
//    vector<Vec3> waypts1;
//    waypts1.push_back(
//            Vec3(takeoff_pose_.pose.position.x, takeoff_pose_.pose.position.y, takeoff_pose_.pose.position.z));
//    waypts1.push_back(Vec3(0.5, 0.5, takeoff_pose_.pose.position.z));
//
//    wayptsArray.push_back(waypts1);
//    vector<Vec3> waypts2;
//    waypts2.push_back(Vec3(0.5, 0.5, takeoff_pose_.pose.position.z));
//    waypts2.push_back(
//            Vec3(0.5, -0.5, takeoff_pose_.pose.position.z));
//    wayptsArray.push_back(waypts2);
//    vector<Vec3> waypts3;
//    waypts3.push_back(Vec3(0.5, -0.5, takeoff_pose_.pose.position.z));
//    waypts3.push_back(
//            Vec3(takeoff_pose_.pose.position.x, takeoff_pose_.pose.position.y, takeoff_pose_.pose.position.z));
//    wayptsArray.push_back(waypts3);

    /*vector<Vec3> waypts1;
    waypts1.push_back(
            Vec3(takeoff_pose_.pose.position.x, takeoff_pose_.pose.position.y, takeoff_pose_.pose.position.z));
    waypts1.push_back(Vec3(1, 0, takeoff_pose_.pose.position.z));

    wayptsArray.push_back(waypts1);
    vector<Vec3> waypts2;
    waypts2.push_back(Vec3(1, 0, takeoff_pose_.pose.position.z));
    waypts2.push_back(
            Vec3(1, 4, takeoff_pose_.pose.position.z));
    wayptsArray.push_back(waypts2);
    vector<Vec3> waypts3;
    waypts3.push_back(Vec3(1, 4, takeoff_pose_.pose.position.z));
    waypts3.push_back(
            Vec3(1,-2, takeoff_pose_.pose.position.z));
    wayptsArray.push_back(waypts3);

    vector<Vec3> waypts4;
    waypts4.push_back(Vec3(1, -2, takeoff_pose_.pose.position.z));
    waypts4.push_back(
            Vec3(takeoff_pose_.pose.position.x,takeoff_pose_.pose.position.y, takeoff_pose_.pose.position.z));
    wayptsArray.push_back(waypts4);*/

    vector<Vec3> waypts;
    waypts.push_back(
            Vec3(takeoff_pose_.pose.position.x, takeoff_pose_.pose.position.y, takeoff_pose_.pose.position.z));
    waypts.push_back(Vec3(0.5, 0.5, takeoff_pose_.pose.position.z));
    waypts.push_back(Vec3(1, 0, takeoff_pose_.pose.position.z));
    waypts.push_back(Vec3(1.5, -0.5, takeoff_pose_.pose.position.z));
    waypts.push_back(Vec3(2, 0, takeoff_pose_.pose.position.z));
    waypts.push_back(Vec3(1.5, 0.5, takeoff_pose_.pose.position.z));
    waypts.push_back(Vec3(1, 0, takeoff_pose_.pose.position.z));
    waypts.push_back(Vec3(0.5, -0.5, takeoff_pose_.pose.position.z));
    waypts.push_back(
            Vec3(takeoff_pose_.pose.position.x, takeoff_pose_.pose.position.y, takeoff_pose_.pose.position.z));
    wayptsArray.push_back(waypts);

    memset(&land_signal_, 0, sizeof(land_signal_));
    land_signal_.twist.linear.z = -0.5;
    land_signal_.twist.linear.x = 0;
    land_signal_.twist.linear.y = 0;

    /*  Waiting for the pose initialization  */
    while (!is_init_pose_) {
        ros::spinOnce();
    }


    takeoff_pose_.pose.orientation.x = 0;
    takeoff_pose_.pose.orientation.y = 0;
    takeoff_pose_.pose.orientation.z = 0;
    takeoff_pose_.pose.orientation.w = 1;

    takeoff_pose_.header.frame_id = "body";

    ROS_INFO("MAV INIT SUCCESS!");

}

void MavFsmNode::callSE3ControlOnce() {
    if (fp.k_position.norm() < 1e-3) {
        ROS_WARN("PID parameters has not been set, force return!");
        return;
    }

//    controller_.calculateControl(cmd.pos, cmd.vel, cmd.acc,
//                                 fp.k_position.asDiagonal(), fp.k_velocity.asDiagonal(), fp.k_integral,
//                                 fp.k_thrust);

    controller_.calculateControlYaw(cmd.pos, cmd.vel, cmd.acc,
                                    cmd.yaw,
                                    fp.k_position.asDiagonal(), fp.k_velocity.asDiagonal(),
                                    fp.k_integral,
                                    fp.k_thrust);

    mavros_msgs::AttitudeTarget cur_att_target;
    geometry_msgs::PoseStamped cur_pose;
    nav_msgs::Odometry cur_odom;
    cur_att_target.thrust = controller_.getPX4Thrust();
    cur_att_target.orientation = controller_.getComputedTFOrientation();
    cur_att_target.header.stamp = ros::Time::now();
    cur_att_target.header.frame_id = "map";


    cur_odom.pose.pose.position.x = cmd.pos.x();
    cur_odom.pose.pose.position.y = cmd.pos.y();
    cur_odom.pose.pose.position.z = cmd.pos.z();
    cur_odom.pose.pose.orientation = controller_.getComputedTFOrientation();

    Eigen::Vector3d vel_b = feedback.rotation.transpose() * cmd.vel;

    cur_odom.twist.twist.linear.x = vel_b.x();
    cur_odom.twist.twist.linear.y = vel_b.y();
    cur_odom.twist.twist.linear.z = vel_b.z();
    cur_odom.header.frame_id = "map";
    cur_odom.header.stamp = ros::Time::now();
    desi_odom_pub_.publish(cur_odom);
    att_pub_.publish(cur_att_target);
}
