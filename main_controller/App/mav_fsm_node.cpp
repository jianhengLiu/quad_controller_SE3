//
// Created by yunfan on 2020/12/22.
//

#include "main_controller/common_include.h"
#include "main_controller/mav_fsm.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mav_fsm");
    ros::NodeHandle nh("~");
    wtr::MavFsmNode mav(nh);

    ros::spin();
    return 0;
}

