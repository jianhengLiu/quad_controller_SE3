#ifndef __SE3_CONTROL_H__
#define __SE3_CONTROL_H__


#include "eigen3/Eigen/Geometry"
#include <tf/transform_datatypes.h>

#include "SE3Control/matrix_utils.h"

class SO3Control {
public:

    SO3Control();

    void setMass(const double mass);

    void setGravity(const double g);

    void setPosition(const Eigen::Vector3d &position);

    void setVelocity(const Eigen::Vector3d &velocity);

    void setAcc(const Eigen::Vector3d &acc);

    void setRotation(const Eigen::Matrix3d &rotation);

    void calculateControl(const Eigen::Vector3d &des_pos,
                          const Eigen::Vector3d &des_vel,
                          const Eigen::Vector3d &des_acc,
                          const Eigen::Matrix3d &kX,
                          const Eigen::Matrix3d &kV,
                          const double &kI,
                          const double &kThrust);

    void calculateControlYaw(Eigen::Vector3d &des_pos,
                             Eigen::Vector3d &des_vel,
                             Eigen::Vector3d &des_acc,
                             double &des_yaw,
                             const Eigen::Matrix3d &kX,
                             const Eigen::Matrix3d &kV,
                             const double &kI,
                             const double &kThrust);

    const double &getThrust(void);

    const double &getPX4Thrust(void);

    const Eigen::Vector3d &getComputedForce(void);

    const Eigen::Quaterniond &getComputedOrientation(void);

    geometry_msgs::Quaternion getComputedTFOrientation();


    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    double dt = 1e-9;  /**< Time step size in seconds */
    bool use_integral = true;

    // Inputs for the controller
    double m_;
    double g_;
    Eigen::Vector3d pos_;
    Eigen::Vector3d vel_;
    Eigen::Vector3d acc_;

    Eigen::Matrix3d R_; /**< Attitude in SO(3) */

    Eigen::Vector3d b1p;

    // controller variables
    Eigen::Vector3d e1;
    Eigen::Vector3d e2;
    Eigen::Vector3d e3;

    struct integral_error_vec3 {
        Eigen::Vector3d error;
        Eigen::Vector3d integrand;

        integral_error_vec3(void) {
            set_zero();
        }


        /** \fn void integrate(const Vector3 current_integrand, const double dt)
         *
         * Integration of errors for a 3x1 state.
         *
         * @param current_integrand the value of the integrand right now
         * @param dt time step
         */
        void integrate(const Eigen::Vector3d current_integrand, const double dt) {
            error += (integrand + current_integrand) * dt / 2;
            integrand = current_integrand;
        }


        /** \fn void set_zero(void)
         * Set all the errors to zero.
         */
        void set_zero(void) {
            error.setZero();
            integrand.setZero();
        }
    } eIX;  // end of integral_error3 struct

    // Outputs of the controller
    double thrust_ = 0.0;  /**< Total propeller thrust */
    double px4_thrust_ = 0.0; /**< Mavros thrust [0,1] */
    Eigen::Vector3d force_;
    Eigen::Quaterniond orientation_;
};

#endif
