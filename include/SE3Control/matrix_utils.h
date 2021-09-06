//
// Created by chrisliu on 2021/5/4.
//

#ifndef MAIN_CONTROLLER_MATRIX_UTILS_H
#define MAIN_CONTROLLER_MATRIX_UTILS_H

#include "iostream"
#include "eigen3/Eigen/Dense"

/** \fn Matrix3 hat(const Vector3 v)
* Returns the hat map of a given 3x1 vector. This is the inverse of vee map.
* @param v vector which the hat map is needed to be operated on
* @return hat map of the input vector
*/
Eigen::Matrix3d hat(const Eigen::Vector3d v);


/** \fn Vector3 vee(const Matrix3 V)
* Returns the vee map of a given 3x3 matrix. This is the inverse of hat map.
* @param V matrix which the vee map is needed to be operated on
* @return vee map of the input matrix
*/
Eigen::Vector3d vee(const Eigen::Matrix3d V);


/** \fn void saturate(Vector3 &x, const double x_min, const double x_max)
 * Saturate the elements of a given 3x1 vector between a minimum and a maximum
 * value.
 * @param x     vector which the elements needed to be saturated
 * @param x_min minimum value for each element
 * @param x_max maximum value for each element
 */
void saturate(Eigen::Vector3d &x, const double x_min, const double x_max);


/** \fn deriv_unit_vector(const Vector3 &A, const Vector3 &A_dot, \
 * const Vector3 A_ddot, Vector3 &q, Vector3 &q_dot, Vector3 &q_ddot)
 * Outputs the time derivatives of a vector after normalizing it.
 * @param A Non-normal vector
 * @param A_dot Time derivative of A
 * @param A_ddot Time derivative of A_dot
 * @param
 */
void deriv_unit_vector(
        const Eigen::Vector3d &A, const Eigen::Vector3d &A_dot,
        Eigen::Vector3d &q, Eigen::Vector3d &q_dot
);

#endif //MAIN_CONTROLLER_MATRIX_UTILS_H
