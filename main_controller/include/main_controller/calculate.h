//
// Created by kevin on 2020/11/23.
//

#ifndef FLIGHTCONTROLLER_CALCULATE_H
#define FLIGHTCONTROLLER_CALCULATE_H
#include "math.h"
using namespace Eigen;
class cal_c{
public:
    cal_c(){
        e3<< 0, 0, 1;
    };

    Vec3 e3;
    const double pi = 3.1415926535;
    double t_, t_last_;
    double getDt(double t){
        t_last_ = t_;
        t_ = t;
        return t_ - t_last_;
    }

    Vec3 getVectorD(Eigen::Vector3d cVector, Eigen::Vector3d pVector)
    {
        return cVector - pVector;
    }

    Vec3 getVectorD(Eigen::Vector3d cVector, Eigen::Vector3d pVector, double dt)
    {
        return (cVector - pVector) / dt;
    }


    Vec3 antisymmetricMatrixToVector(Eigen::Matrix3d antisymmetricMatrix)
    {
        Eigen::Vector3d vector(antisymmetricMatrix(7), antisymmetricMatrix(2), antisymmetricMatrix(3));

        return vector;
    }

    Vec4 getAllocatedRevs(double Force, Vec3 Moment)
    {
        Eigen::Matrix4d Minvese;
        double sqrt2 = sqrt(2);
        Minvese << 1, sqrt2, sqrt2, 1,
                1, -sqrt2, sqrt2, -1,
                1, -sqrt2, -sqrt2, 1,
                1, sqrt2, -sqrt2, -1;
        Minvese = 0.25 * Minvese;
//        Vec4 input(Force, 0,0,3);
        Eigen::Vector4d input(Force, Moment.x(), Moment.y(), Moment.z());
        Eigen::Vector4d revs = Minvese * input;
        if (revs.x() < 0)
        {
            revs.x() = 0;
        }
        if (revs.y() < 0)
        {
            revs.y() = 0;
        }
        if (revs.z() < 0)
        {
            revs.z() = 0;
        }
        if (revs.w() < 0)
        {
            revs.w() = 0;
        }
        revs.x() = sqrt(revs.x());
        revs.y() = sqrt(revs.y());
        revs.z() = sqrt(revs.z());
        revs.w() = sqrt(revs.w());
        return revs;
    }

    Eigen::Quaterniond rotation2quatern(Mat33 rotation){
        Eigen::Quaterniond quaternion(rotation);
        return quaternion;
    }

    Vec3 rotation2eulerAngle(Mat33 rotation){
        /*
         * from rotation matrix to Roll-Pitch-Yaw
         * */
        Eigen::Vector3d eulerAngle=rotation.eulerAngles(2,1,0);
        return eulerAngle;
    }

    Mat33 eulerAngle2rotation(Vec3 eulerAngle){
        Eigen::AngleAxisd rollAngle(AngleAxisd(eulerAngle(0),Vector3d::UnitX()));
        Eigen::AngleAxisd pitchAngle(AngleAxisd(eulerAngle(1),Vector3d::UnitY()));
        Eigen::AngleAxisd yawAngle(AngleAxisd(eulerAngle(2),Vector3d::UnitZ()));

        Eigen::Matrix3d rotation_matrix;
        rotation_matrix=yawAngle*pitchAngle*rollAngle;
        return rotation_matrix;
    }

    Eigen::Quaterniond eulerAngle2quad(Vec3 eulerAngle){
        Eigen::AngleAxisd rollAngle(AngleAxisd(eulerAngle(2),Vector3d::UnitX()));
        Eigen::AngleAxisd pitchAngle(AngleAxisd(eulerAngle(1),Vector3d::UnitY()));
        Eigen::AngleAxisd yawAngle(AngleAxisd(eulerAngle(0),Vector3d::UnitZ()));

        Eigen::Quaterniond quaternion;
        quaternion=yawAngle*pitchAngle*rollAngle;
        return quaternion;
    }


};


#endif //FLIGHTCONTROLLER_CALCULATE_H
