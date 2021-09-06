#include <iostream>
#include <SE3Control/SE3Control.h>

using namespace std;

SO3Control::SO3Control()
        : m_(0.0), g_(9.805) {
    e1 << 1.0, 0.0, 0.0;
    e2 << 0.0, 1.0, 0.0;
    e3 << 0.0, 0.0, 1.0;

    acc_.setZero();
    eIX.set_zero();
    b1p << 1.0, 0.0, 0.0;
}

void
SO3Control::setMass(const double mass) {
    m_ = mass;
}

void
SO3Control::setGravity(const double g) {
    g_ = g;
}

void
SO3Control::setPosition(const Eigen::Vector3d &position) {
    pos_ = position;
}

void
SO3Control::setVelocity(const Eigen::Vector3d &velocity) {
    vel_ = velocity;
}

void SO3Control::setAcc(const Eigen::Vector3d &acc) {
    acc_ = acc;
}

void SO3Control::setRotation(const Eigen::Matrix3d &rotation) {
    R_ = rotation;
}

void
SO3Control::calculateControl(const Eigen::Vector3d &des_pos,
                             const Eigen::Vector3d &des_vel,
                             const Eigen::Vector3d &des_acc,
                             const Eigen::Matrix3d &kX,
                             const Eigen::Matrix3d &kV,
                             const double &kI,
                             const double &kThrust) {
    /**
     * position control
     */
    // translational error functions
    Eigen::Vector3d eX = pos_ - des_pos;     // position error - eq (11)
    Eigen::Vector3d eV = vel_ - des_vel; // velocity error - eq (12)

    // position integral terms
    if (use_integral) {
        eIX.integrate(eX + eV, dt); // eq (13),

        double sat_sigma = 1;//1.8;
        saturate(eIX.error, -sat_sigma, sat_sigma);
    } else {
        eIX.set_zero();
    }

    auto yaw_cur = R_.eulerAngles(2, 1, 0)[0];
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(yaw_cur, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    auto Rz_inv = Rz.transpose();
    Eigen::Vector3d A = -Rz * kX * Rz_inv * eX
                        - Rz * kV * Rz_inv * eV
                        - Rz * kI * Rz_inv * eIX.error
                        + m_ * g_ * e3
                        + m_ * des_acc;

    if (A(2) < 0.5 * m_ * g_) {
        std::cout << "thrust too low A(2) = " << A(2) << std::endl;
        A = A / A(2) * (0.5 * m_ * g_);
    } else if (A(2) > 2 * m_ * g_) {
        std::cout << "thrust too high A(2) = " << A(2) << std::endl;
        A = A / A(2) * (2 * m_ * g_);
    }

    if (std::fabs(A(0) / A(2)) > std::tan(M_PI / 4)) {
        std::cout << "roll(x) too tilt atan2(Ax/Az) = " << std::atan2(A(0), A(2)) << std::endl;
        A(0) = A(0) / std::fabs(A(0)) * A(2) * std::tan(M_PI / 6);
    }

    if (std::fabs(A(1) / A(2)) > std::tan(M_PI / 4)) {
        std::cout << "pitch(y) too tilt atan2(Ay/Az) = " << std::atan2(A(1), A(2)) << std::endl;
        A(1) = A(1) / std::fabs(A(1)) * A(2) * std::tan(M_PI / 6);
    }

    px4_thrust_ = A.norm() * kThrust;
    if (px4_thrust_ > 0.8) {
        A = A * 0.8 / px4_thrust_;
    }

    Eigen::Vector3d b3 = R_ * e3;
    thrust_ = A.dot(b3);// eq (32)
    px4_thrust_ = thrust_ * kThrust;

    Eigen::Vector3d b3c = A.normalized();

    Eigen::Vector3d b1d(1, 0, 0);
    Eigen::Vector3d b1d_dot(0, 0, 0);

    Eigen::Vector3d A2 = -hat(b1d) * b3c;
    Eigen::Vector3d b2c = A2.normalized();

    Eigen::Vector3d b1c = hat(b2c) * b3c;

    Eigen::Matrix3d Rd;
    Rd << b1c, b2c, b3c;

    orientation_ = Eigen::Quaterniond(Rd);
}

void
SO3Control::calculateControlYaw(Eigen::Vector3d &des_pos,
                                Eigen::Vector3d &des_vel,
                                Eigen::Vector3d &des_acc,
                                double &des_yaw,
                                const Eigen::Matrix3d &kX,
                                const Eigen::Matrix3d &kV,
                                const double &kI,
                                const double &kThrust) {

    /**
     * position control
     */
    // translational error functions
    Eigen::Vector3d eX = pos_ - des_pos;     // position error - eq (11)
    Eigen::Vector3d eV = R_ * vel_ - des_vel; // velocity error - eq (12) R_ for mavros' odom

    // position integral terms
    if (use_integral) {
        eIX.integrate(eX + eV, dt); // eq (13),

        double sat_sigma = 1;//1.8;
        saturate(eIX.error, -sat_sigma, sat_sigma);
    } else {
        eIX.set_zero();
    }

    auto yaw_cur = R_.eulerAngles(2, 1, 0)[0];
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(yaw_cur, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    auto Rz_inv = Rz.transpose();
    Eigen::Vector3d A = -Rz * kX * Rz_inv * eX
                        - Rz * kV * Rz_inv * eV
                        - Rz * kI * Rz_inv * eIX.error
                        + m_ * g_ * e3
                        + m_ * des_acc;

    if (A(2) < 0.5 * m_ * g_) {
        std::cout << "thrust too low A(2) = " << A(2) << std::endl;
        A = A / A(2) * (0.5 * m_ * g_);
    } else if (A(2) > 2 * m_ * g_) {
        std::cout << "thrust too high A(2) = " << A(2) << std::endl;
        A = A / A(2) * (2 * m_ * g_);
    }

    if (std::fabs(A(0) / A(2)) > std::tan(M_PI / 4)) {
        std::cout << "roll(x) too tilt atan2(Ax/Az) = " << std::atan2(A(0), A(2)) << std::endl;
        A(0) = A(0) / std::fabs(A(0)) * A(2) * std::tan(M_PI / 6);
    }

    if (std::fabs(A(1) / A(2)) > std::tan(M_PI / 4)) {
        std::cout << "pitch(y) too tilt atan2(Ay/Az) = " << std::atan2(A(1), A(2)) << std::endl;
        A(1) = A(1) / std::fabs(A(1)) * A(2) * std::tan(M_PI / 6);
    }

    px4_thrust_ = A.norm() * kThrust;
    if (px4_thrust_ > 0.8) {
        A = A / px4_thrust_ * 0.8;
    }

    Eigen::Vector3d b3 = R_ * e3;
    thrust_ = A.dot(b3);// eq (32)
    px4_thrust_ = thrust_ * kThrust;

    Eigen::Vector3d b3c = A.normalized();

    /*Eigen::Vector3d b1d(1, 0, 0);
    Eigen::Vector3d b1v = Eigen::Vector3d(des_vel.x(), des_vel.y(), 0);
    Eigen::Vector3d nb1v = b1v.normalized();
    if (nb1v.norm() <= 1 + 1e-3 && nb1v.norm() >= 1 - 1e-3) {
        double yaw_diff = acos(nb1v.dot(b1p)) * 57.3;
        if (yaw_diff > 5) {
            nb1v = (nb1v + 0.5 * yaw_diff * b1p).normalized();
            ROS_INFO("Yaw Diff too big !");
        }
        b1d = nb1v;
        b1p = b1d;
    } else if (b1p.norm() <= 1 + 1e-2 && b1p.norm() >= 1 - 1e-2) {
        b1d = b1p;
    }*/

    Eigen::Vector3d b1d(cos(des_yaw), sin(des_yaw), 0);

    Eigen::Vector3d A2 = -hat(b1d) * b3c;
    Eigen::Vector3d b2c = A2.normalized();

    Eigen::Vector3d b1c = hat(b2c) * b3c;

    Eigen::Matrix3d Rd;
    Rd << b1c, b2c, b3c;

    orientation_ = Eigen::Quaterniond(Rd);
}

const double &SO3Control::getThrust(void) {
    return thrust_;
}

const double &SO3Control::getPX4Thrust(void) {
    return px4_thrust_;
}

const Eigen::Vector3d &
SO3Control::getComputedForce(void) {
    return force_;
}

const Eigen::Quaterniond &
SO3Control::getComputedOrientation(void) {
    return orientation_;
}

geometry_msgs::Quaternion SO3Control::getComputedTFOrientation() {
    geometry_msgs::Quaternion q;
    q.x = orientation_.x();
    q.y = orientation_.y();
    q.z = orientation_.z();
    q.w = orientation_.w();
    return q;
}

