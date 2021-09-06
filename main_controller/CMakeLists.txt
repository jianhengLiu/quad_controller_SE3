cmake_minimum_required(VERSION 2.8.3)
project(main_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11 -03)

#set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3") #c++11
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g -march=native --fast-math -ffast-math -mtune=native -funroll-loops -DNDEBUG -DBOOST_DISABLE_ASSERTS" CACHE STRING COMPILE_FLAGS FORCE)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -mtune=native -funroll-loops -DNDEBUG -DBOOST_DISABLE_ASSERTS" CACHE STRING COMPILE_FLAGS FORCE)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        image_transport
        cv_bridge
        geometry_msgs
        nav_msgs
        mavros_msgs
        tf
        dynamic_reconfigure
        poly_traj_utils
		quadrotor_msgs
        )

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
        cfg/ControllerParams.cfg
)

set(THIRD_PARTY_LIBS
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
catkin_package(
)

add_executable(main_fsm_node
        App/mav_fsm_node.cpp
        src/mav_fsm.cpp
        src/SE3Control/SE3Control.cpp
        src/SE3Control/matrix_utils.cpp)
target_link_libraries(main_fsm_node
        ${THIRD_PARTY_LIBS}
        poly_traj_utils
        )
add_dependencies(main_fsm_node ${PROJECT_NAME}_gencfg)

add_executable(rs_remap
        App/realsenseRemap.cpp)
target_link_libraries(rs_remap
        ${THIRD_PARTY_LIBS}
        )

add_executable(rs_remap_odom_node
        App/rs_remap_odom_node.cpp)
target_link_libraries(rs_remap_odom_node
        ${THIRD_PARTY_LIBS}
        )

add_executable(viconRemap
	App/viconRemap.cpp)
target_link_libraries(viconRemap
        ${THIRD_PARTY_LIBS}
        )

add_executable(vinsfusion_remap_node
		App/vinsfusion_remap_node.cpp)
target_link_libraries(vinsfusion_remap_node
		${THIRD_PARTY_LIBS}
		)

add_executable(show_status_node
        App/show_status_node.cpp)
target_link_libraries(show_status_node
        ${THIRD_PARTY_LIBS}
        )


