//
// Created by chrisliu on 2021/5/4.
//

#include "SE3Control/matrix_utils.h"

Eigen::Matrix3d hat(const Eigen::Vector3d v) {
    Eigen::Matrix3d V;
    V.setZero();

    V(2, 1) = v(0);
    V(1, 2) = -V(2, 1);
    V(0, 2) = v(1);
    V(2, 0) = -V(0, 2);
    V(1, 0) = v(2);
    V(0, 1) = -V(1, 0);

    return V;
}


Eigen::Vector3d vee(const Eigen::Matrix3d V) {
    // TODO: improve code by: https://codereview.stackexchange.com/questions/77546/multiply-vector-elements-by-a-scalar-value-using-stl-and-templates
    Eigen::Vector3d v;
    Eigen::Matrix3d E;

    v.setZero();
    E = V + V.transpose();

    if (E.norm() > 1.e-6) {
        std::cout << "VEE: E.norm() = " << E.norm() << std::endl;
    }

    v(0) = V(2, 1);
    v(1) = V(0, 2);
    v(2) = V(1, 0);

    return v;
}


void saturate(Eigen::Vector3d &x, const double x_min, const double x_max) {
    for (int i = 0; i < 3; i++) {
        if (x(i) > x_max) x(i) = x_max;
        else if (x(i) < x_min) x(i) = x_min;
    }
}


void deriv_unit_vector(const Eigen::Vector3d &A, const Eigen::Vector3d &A_dot,
                       Eigen::Vector3d &q, Eigen::Vector3d &q_dot
) {
    double nA = A.norm();
    double nA3 = pow(nA, 3);
    double nA5 = pow(nA, 5);

    q = A / nA;
    q_dot = A_dot / nA - A * A.dot(A_dot) / nA3;
}