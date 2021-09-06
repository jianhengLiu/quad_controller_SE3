# poly_traj_utils

> Yunfan REN
>
> renyunfan@outlook.com

This work is an extension of [AM_traj](https://github.com/ZJU-FAST-Lab/am_traj) from Fei Gao et al. Due to the large number of constant matrices in the calculation of Alternating Minimization optimization, it is difficult to complete an efficient and general algorithm. So this package just realize the 5-th order and 7-th order optimization algorithm.

# 1 Build and run examples

```bash
mkdir -p catkin_ws/src && cd catkin_ws/src
git clone https://github.com/RENyunfan/poly_traj_tuils
cd ..
catkin_make
```

To run the examples

```bash
source devel/setup.bash
# Run the opt utils
roslaunch poly_traj_utils opt_test.launch
# Run the obvp ttils
roslaunch poly_traj_utils obvp_test.launch
```

# 2 Usage

Add library to your `package.xml`

```xml
<build_depend>poly_traj_utils</build_depend>
<exec_depend>poly_traj_utils</exec_depend>
```

Add library to your `CMakeLists.txt`

```cmake
find_package(poly_traj_utils)
catkin_package(
        CATKIN_DEPENDS roscpp rospy std_msgs poly_traj_utils 
)
```

Then just include the header file in your `cpp`  files

```cpp
#include "poly_traj_utils/am_traj_plus.hpp"
#include "poly_traj_utils/am_traj.hpp"
#include "poly_traj_utils/poly_visual_utils.hpp"
```

You can freely use the `shared_ptr` in your code and classes.

```cpp
// Creat the shared_ptr
order7::AmTraj::Ptr am_ptr_7;
order5::AmTraj::Ptr am_ptr_5;
// Reset memory
am_ptr_5.reset(new order5::AmTraj);
am_ptr_7.reset(new order7::AmTraj);
// Init parameters
am_ptr_7->init(op_7.weight_T, op_7.weight_acc, op_7.weight_jerk, op_7.weight_snap, op_7.max_v, op_7.max_a, op_7.max_it, op_7.eps);
am_ptr_5->init(op_5.weight_T, op_5.weight_acc,
               op_5.weight_jerk, op_5.max_v, op_5.max_a, op_5.max_it, op_5.eps);

Trajectory traj_5,traj_7;
// Generate constrained opt_traj
traj_7=  am_ptr_7->genOptimalTrajDTC(waypts, Vec3(0, 0, 0), Vec3(0, 0, 0), Vec3(0, 0, 0), Vec3(0, 0, 0), Vec3(0, 0, 0), Vec3(0, 0, 0));
```

# 3 Tips

* We found that  the weight of minimum snap will cause the entire optimizer to fall into a local minimum. We suggest the `wSnap` to be a small value, even 0.

* Some tricks: the 5-th order can calculate relative fast, I found that the solution of 5-th order durations can work well in 7-th order. 

* There may be bugs in this package, please contact me with email~ Thanks! n(\*≧▽≦\*)n

  