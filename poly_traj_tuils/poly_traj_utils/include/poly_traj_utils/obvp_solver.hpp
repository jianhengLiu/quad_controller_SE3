//
// Created by yunfan on 2021/3/19.
//

#ifndef SRC_OBVP_SOLVER_HPP
#define SRC_OBVP_SOLVER_HPP
#include "root_finder.hpp"
#include "traj_utils.hpp"
#include "Eigen/Dense"
#include "vector"
#include "poly_visual_utils.hpp"
using namespace Eigen;
using namespace std;

typedef Eigen::Matrix<double,12,1> StatePVAJ;

class ObvpSolver {
private:
    double rho_;
    double vd_,rho_mp_,rho_ae_;
    Vec3 mp_start_vel_, mp_end_vel_;

    inline double evaluateCost(double T, double order){
        double cost = 0;
        if(order == 7){

        }

        return cost;
    }

    inline double calcOptimalTime(const Eigen::Array3d &p_0, const Eigen::Array3d &v_0,
                                  const Eigen::Array3d &a_0, const Eigen::Array3d &j_0,
                                  const Eigen::Array3d &p_f, const Eigen::Array3d &v_f,
                                  const Eigen::Array3d &a_f, const Eigen::Array3d &j_f){

        VectorXd coeffsGradT(9);

        coeffsGradT(0) = rho_;
        coeffsGradT(2) = (- 8*j_0*j_f - 16*j_0.square() - 16*j_f.square()).sum();
        coeffsGradT(3) = (240*a_f*j_0 - 240*a_0*j_f - 480*a_0*j_0 + 480*a_f*j_f).sum();
        coeffsGradT(4) = (5040*a_0*a_f - 2880*j_0*v_0 - 2160*j_0*v_f - 2160*j_f*v_0 - 2880*j_f*v_f - 3600*a_0.square() - 3600*a_f.square()).sum();
        coeffsGradT(5) = (37440*a_f*v_0 - 37440*a_0*v_f - 43200*a_0*v_0 + 43200*a_f*v_f - 6720*j_0*p_0 + 6720*j_0*p_f - 6720*j_f*p_0 + 6720*j_f*p_f).sum();
        coeffsGradT(6) = (100800*a_0*p_f - 100800*a_0*p_0 + 100800*a_f*p_0 - 100800*a_f*p_f - 244800*v_0*v_f - 129600*v_0.square() - 129600*v_f.square()).sum();
        coeffsGradT(7) = (604800*p_f*v_0 - 604800*p_0*v_f - 604800*p_0*v_0 + 604800*p_f*v_f).sum();
        coeffsGradT(8) = (1411200*p_0*p_f - 705600*p_0.square() - 705600*p_f.square()).sum();

        std::set<double> roots = RootFinder::solvePolynomial(coeffsGradT, DBL_EPSILON, DBL_MAX, 1e-6);

        bool result = false;
        double tau = DBL_MAX;
        double cost = DBL_MAX;

        VectorXd coeffsSnapObjective(7);
        coeffsSnapObjective(0) =(8*j_0*j_f + 16*j_0.square() + 16*j_f.square()).sum();
        coeffsSnapObjective(1) =(240*a_0*j_0 + 120*a_0*j_f - 120*a_f*j_0 - 240*a_f*j_f).sum();
        coeffsSnapObjective(2) =(960*j_0*v_0 - 1680*a_0*a_f + 720*j_0*v_f + 720*j_f*v_0 + 960*j_f*v_f + 1200*a_0.square() + 1200*a_f.square()).sum();
        coeffsSnapObjective(3) =(10800*a_0*v_0 + 9360*a_0*v_f - 9360*a_f*v_0 - 10800*a_f*v_f + 1680*j_0*p_0 - 1680*j_0*p_f + 1680*j_f*p_0 - 1680*j_f*p_f).sum();
        coeffsSnapObjective(4) =(20160*a_0*p_0 - 20160*a_0*p_f - 20160*a_f*p_0 + 20160*a_f*p_f + 48960*v_0*v_f + 25920*v_0.square() + 25920*v_f.square()).sum();
        coeffsSnapObjective(5) =(100800*p_0*v_0 + 100800*p_0*v_f - 100800*p_f*v_0 - 100800*p_f*v_f).sum();
        coeffsSnapObjective(6) =(100800*p_0.square() - 201600*p_0*p_f + 100800*p_f.square()).sum();
        for (const double& root : roots)
        {
            double t5 = pow(root,5);
            double current = rho_ * root + RootFinder::polyVal(coeffsSnapObjective, root) / t5;
            if (current < cost)
            {
                tau = root;
                cost = current;
                result = true;
            }
        }
        return tau;
    }

    /*  minimize Inte(1 + acc^T * rho * acc, 0, tau) */
    inline double calTauStarDouble(const StatePVAJ & start_state, const StatePVAJ & end_state)
    {
        VectorXd p(5);
        p[0] = 1;
        p[1] = 0;
        p[2] = (start_state[3]*start_state[3] + start_state[3]*end_state[3] + end_state[3]*end_state[3]
                + start_state[4]*start_state[4] + start_state[4]*end_state[4] + end_state[4]*end_state[4]
                + start_state[5]*start_state[5] + start_state[5]*end_state[5] + end_state[5]*end_state[5]) * (-4.0) * rho_;
        p[3] =(- start_state[0]*start_state[3] - start_state[0]*end_state[3] + end_state[0]*start_state[3]
               + end_state[0]*end_state[3] - start_state[1]*start_state[4] - start_state[1]*end_state[4]
               + end_state[1]*start_state[4] + end_state[1]*end_state[4] - start_state[2]*start_state[5]
               - start_state[2]*end_state[5] + end_state[2]*start_state[5] + end_state[2]*end_state[5]) * 24.0 * rho_;
        p[4] =(- start_state[0]*start_state[0] + 2.0*start_state[0]*end_state[0] - end_state[0]*end_state[0]
               - start_state[1]*start_state[1] + 2.0*start_state[1]*end_state[1] - end_state[1]*end_state[1]
               - start_state[2]*start_state[2] + 2.0*start_state[2]*end_state[2] - end_state[2]*end_state[2]) * 36.0 * rho_;
        std::set<double> roots = RootFinder::solvePolynomial(p, DBL_EPSILON, DBL_MAX, 1e-6);

        bool result = false;
        double tau = DBL_MAX;
        double cost = DBL_MAX;
        for (const double& root : roots)
        {
            double t1 = start_state[0] - end_state[0];
            double t2 = start_state[1] - end_state[1];
            double t3 = start_state[2] - end_state[2];
            double t4 = start_state[3] + end_state[3];
            double t5 = start_state[4] + end_state[4];
            double t6 = start_state[5] + end_state[5];
            double t7 = t1*t1 + t2*t2 + t3*t3;
            double t8 = t4*t4 + t5*t5 + t6*t6 - start_state[3]*end_state[3] - start_state[4]*end_state[4] - start_state[5]*end_state[5];
            double t9 = t1*t4 + t2*t5 + t3*t6;

            double current = root + rho_*(t7*12/root/root/root + t8*4/root + t9*12/root/root);
            if (current < cost)
            {
                tau = root;
                cost = current;
                result = true;
            }
        }
        return tau;
    }


public:
    ObvpSolver(){};
    ~ObvpSolver(){};

    typedef std::shared_ptr<ObvpSolver> Ptr;
    void init(double rho, double rho_mp, double rho_ae, double vd){
        rho_ = rho;
        vd_ = vd;
        rho_mp_  = rho_mp;
        rho_ae_  = rho_ae;
        ROS_INFO("OBVP solver init success, rho_mp = %lf, rho_ae = %lf, vd = %lf.", rho_mp,rho_ae,vd);
    };


    inline Piece genObvpMinAccTraj(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
                                         const Eigen::Vector3d &end_pt, const Eigen::Vector3d &end_vel, double max_v,
                                         double max_a) {
        double t; DynamicMat out_mat(3,4);

        double vs = start_vel.norm() ;
        double ve = end_vel.norm() ;
        double left = (vd_ + vs) + (vd_ + ve)*(fabs(vd_ - ve) / max(1e-2,fabs(vd_ - vs)));
        double t1 = 2 * (end_pt-start_pt).norm() / left;
        t = t1 * (1 + (fabs(vd_ - ve) / max(1e-2,fabs(vd_ - vs)) ))* rho_mp_;

//        t = (end_pt - start_pt).norm() / vd_ * rho_ae_;
        if (t < 0 || isnan(t)) {
            t = (start_pt - end_pt).norm() / vd_ * 3;
        }

        for (size_t i = 0; i < 3; i++) {
            // calculate obvp in axis [i]
            Eigen::Vector2d delta_pv = Eigen::Vector2d(end_pt[i] - start_pt[i] - start_vel[i] * t,
                                                       end_vel[i] - start_vel[i]);

            Eigen::MatrixXd tmat(2, 2);
            tmat << -12.0f, 6 * t,
                    6.0f * t, -2 * t * t;

            Eigen::Vector2d ab = tmat * delta_pv / (pow(t, 3));
            double aa = ab[0], bb = ab[1];
            out_mat.row(i)[0] = aa / 6.0f;
            out_mat.row(i)[1] = bb / 2.0f;
            out_mat.row(i)[2] = start_vel[i];
            out_mat.row(i)[3] = start_pt[i];
        }
        Piece pie(t,out_mat);
        return pie;
    }

    inline Piece genObvpMinSnapTraj_HeuEnd(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
                                    const Eigen::Vector3d &start_acc, const Eigen::Vector3d &start_jerk,
                                    const Eigen::Vector3d &end_pt,const double max_v, const double max_a, bool use_optimal_time = false){
        /*  Generate fix vel*/
        double t; DynamicMat out_mat(3,8);Vec3 mp_start_vel,mp_end_vel;
        Vec3 mp_end_acc = Vec3(0, 0, 0);
        Vec3 mp_end_jerk = Vec3(0, 0, 0);
        double R, distan, v;
        /*  1) Check start vel
         *  --- when star_vel near to zero, set the direction of velocity from start point to end point.
         * */
        if (start_vel.norm() < 1e-3) {
            mp_start_vel = (end_pt - start_pt).normalized() * 0.01;
        } else {
            mp_start_vel = start_vel;
        }
        /*  2) Generate end_vel  */
        Vec3 v1 = mp_start_vel.normalized();
        Vec3 v2 = (end_pt - start_pt).normalized();
        double theta = acos(v1.dot(v2));
        if (theta > 1e-3) {
            // When target point is not on the line of start_vel
            R = (end_pt - start_pt).norm() / 2 / max(sin(theta), 1e-4);
            distan = theta * 2 * R;
            v = sqrt(max_a * R);
            v = min(v, vd_);
            Eigen::AngleAxisd rot_v(2 * theta, v1.cross(v2));
            mp_end_vel = (rot_v.toRotationMatrix() * mp_start_vel.normalized()) * vd_;
        } else {
            // When searching along the start_vel
            v = vd_;
            distan = (end_pt - start_pt).norm();
            mp_end_vel = mp_start_vel.normalized() * vd_;
        }
        v = min(vd_,v);
//        printf("Cur v = %lf\n",v);
        if(use_optimal_time){
            t = calcOptimalTime(start_pt,start_vel,start_acc,start_jerk,end_pt,mp_end_vel,Vec3(0,0,0),Vec3(0,0,0));
//            ROS_WARN("Cur t = %lf", t);
        }else{
            /* 3) heuristic time allocation */
//            double vs = start_vel.norm();
//            double ve = mp_end_vel.norm();
//            double t1 = fabs(v-vs) / max_a;
//            double t2 = fabs(v-ve) / max_a;
//            double d1 = (v+vs) * t1 / 2;
//            double d2 = (v + ve) * t2 /2;
//            double dis = distan - d1 - d2;
//            double t3 = dis / v;
//            t = (t1+t2+t3) * rho_mp_;
            double vs = start_vel.norm() ;
            double ve = mp_end_vel.norm() ;
            double left = (v + vs) + (v + ve)*(fabs(v - ve) / max(1e-2,fabs(vd_ - vs)));
            double t1 = 2 * distan / left;
            t = t1 * (1 + (fabs(v - ve) / max(1e-2,fabs(v - vs)) ))* rho_mp_;
        }
//        printf("current t = %lf\n", t);
        if (t < 0 || isnan(t)) {
            t = (end_pt - start_pt).norm() / v *1.0 * rho_mp_;
        }
//        printf("current t = %lf\n", t);
        double tvec[8];
        tvec[0] = 1;
        for (size_t i = 1; i < 8; i++) {
            tvec[i] = pow(t, i);
        }

        for (size_t i = 0; i < 3; i++) {
            // calculate obvp in axis [i]
            Eigen::Vector4d delta_pvaj = Eigen::Vector4d(
                    end_pt[i] - start_pt[i] - start_vel[i] * t - 1.0 / 2.0f * start_acc[i] * t * t - 1.0 / 6.0f * start_jerk[i] * t * t * t,
                    mp_end_vel[i] - start_vel[i] - start_acc[i] * t - 1.0 / 2.0f * start_jerk[i] * t * t,
                    mp_end_acc[i] - start_acc[i] - start_jerk[i] * t,
                    mp_end_jerk[i] - start_jerk[i]);

            Eigen::MatrixXd tmat(4, 4);
            tmat << -33600, 16800 * tvec[1], -3360 * tvec[2], 280 * tvec[3],
                    25200 * tvec[1], -12240 * tvec[2], 2340 * tvec[3], -180 * tvec[4],
                    -10080 * tvec[2], 4680 * tvec[3], -840 * tvec[4], 60 * tvec[5],
                    840 * tvec[3], -360 * tvec[4], 60 * tvec[5], -4 * tvec[6];

            Eigen::Vector4d abyr = tmat * delta_pvaj / tvec[7];
            double aa = abyr[0], bb = abyr[1], yy = abyr[2], rr = abyr[3];
            out_mat.row(i)[0] = aa / 1680.0f;
            out_mat.row(i)[1] = bb / 360.0f;
            out_mat.row(i)[2] = yy / 120.0f;
            out_mat.row(i)[3] = rr / 24.0f;
            out_mat.row(i)[4] = start_jerk[i] / 6.0f;
            out_mat.row(i)[5] = start_acc[i] / 2.0f;
            out_mat.row(i)[6] = start_vel[i];
            out_mat.row(i)[7] = start_pt[i];
        }
        Piece out_pie(t,out_mat);
        return out_pie;

    }


    inline Piece genObvpMinSnapTraj(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
                                    const Eigen::Vector3d &start_acc, const Eigen::Vector3d &start_jerk,
                                    const Eigen::Vector3d &end_pt, const Eigen::Vector3d &end_vel,
                                    const Eigen::Vector3d &end_acc, const Eigen::Vector3d &end_jerk,
                                    const double max_v, const double max_a, bool use_optimal_time = false){

        double t; DynamicMat out_mat(3,8); double R, distan, v;
        if(use_optimal_time){
            t = calcOptimalTime(start_pt,start_vel,start_acc,start_jerk,end_pt,end_vel,end_acc,end_jerk);
        }else{
            mp_end_vel_ = end_vel;
            if (start_vel.norm() < 1e-3) {
                // when star_vel near to zero,
                mp_start_vel_ = (end_pt - start_pt).normalized() * 0.01;

            } else {
                mp_start_vel_ = start_vel;
            }
//            Vec3 v1 = mp_start_vel_.normalized();
//            Vec3 v2 = (end_pt - start_pt).normalized();
//            double theta = acos(v1.dot(v2));
//            if (theta > 1e-3) {
//                // When target point is not on the line of star_vel
//                R = (end_pt - start_pt).norm() / 2 / max(sin(theta), 1e-4);
//                distan = theta * 2 * R;
//                v = sqrt(max_a * R);
//            } else {
//                // When searching along the start_vel
//                v = vd_;
//                distan = (end_pt - start_pt).norm();
//            }
//            v = min(v, vd_);
////            printf("Cur v = %lf\n",v);
//            /* 3) heuristic time allocation */
//            double vs = start_vel.norm();
//            double ve = mp_end_vel_.norm();
//            double t1 = fabs(v-vs) / max_a;
//            double t2 = fabs(v-ve) / max_a;
//            double d1 = (v+vs) * t1 / 2;
//            double d2 = (v + ve) * t2 /2;
//            double dis = distan - d1 - d2;
//            double t3 = dis / v;
            t = (start_pt - end_pt).norm() / vd_ * rho_ae_;
        }
        if (t < 0 || isnan(t)) {
            t = (end_pt - start_pt).norm() / v * 3 * rho_ae_;
        }
        double tvec[8];
        tvec[0] = 1;
        for (size_t i = 1; i < 8; i++) {
            tvec[i] = pow(t, i);
        }

        for (size_t i = 0; i < 3; i++) {
            Eigen::Vector4d delta_pvaj = Eigen::Vector4d(
                    end_pt[i] - start_pt[i] - start_vel[i] * t - 1.0 / 2.0f * start_acc[i] * t * t -
                    1.0 / 6.0f * start_jerk[i] * t * t * t,
                    end_vel[i] - start_vel[i] - start_acc[i] * t - 1.0 / 2.0f * start_jerk[i] * t * t,
                    end_acc[i] - start_acc[i] - start_jerk[i] * t,
                    end_jerk[i] - start_jerk[i]);

            Eigen::MatrixXd tmat(4, 4);
            tmat << -33600, 16800 * tvec[1], -3360 * tvec[2], 280 * tvec[3],
                    25200 * tvec[1], -12240 * tvec[2], 2340 * tvec[3], -180 * tvec[4],
                    -10080 * tvec[2], 4680 * tvec[3], -840 * tvec[4], 60 * tvec[5],
                    840 * tvec[3], -360 * tvec[4], 60 * tvec[5], -4 * tvec[6];

            Eigen::Vector4d abyr = tmat * delta_pvaj / tvec[7];
            double aa = abyr[0], bb = abyr[1], yy = abyr[2], rr = abyr[3];
            out_mat.row(i)[0] = aa / 1680.0f;
            out_mat.row(i)[1] = bb / 360.0f;
            out_mat.row(i)[2] = yy / 120.0f;
            out_mat.row(i)[3] = rr / 24.0f;
            out_mat.row(i)[4] = start_jerk[i] / 6.0f;
            out_mat.row(i)[5] = start_acc[i] / 2.0f;
            out_mat.row(i)[6] = start_vel[i];
            out_mat.row(i)[7] = start_pt[i];
        }
        Piece out_pie(t,out_mat);
        return out_pie;
    }
    inline Piece genObvpMinSnapTraj(const StatePVAJ & start_state, const StatePVAJ & end_state, bool optimal_time_allo = false){
        double t;DynamicMat out_mat(3,8);

        t = calTauStarDouble(start_state,end_state);

        double tvec[8];
        tvec[0] = 1;
        for (size_t i = 1; i < 8; i++) {
            tvec[i] = pow(t, i);
        }

        for (size_t i = 0; i < 3; i++) {
            Eigen::Vector4d delta_pvaj = Eigen::Vector4d(
                    end_state[i] - start_state[i] - start_state[i+3] * t - 1.0 / 2.0f * start_state[i+6] * t * t -
                    1.0 / 6.0f * start_state[i+9] * t * t * t,
                    end_state[i+3] - start_state[i+3] - start_state[i+6] * t - 1.0 / 2.0f * start_state[i+9] * t * t,
                    end_state[i+6] - start_state[i+6] - start_state[i+9] * t,
                    end_state[i+9] - start_state[i+9]);

            Eigen::MatrixXd tmat(4, 4);
            tmat << -33600, 16800 * tvec[1], -3360 * tvec[2], 280 * tvec[3],
                    25200 * tvec[1], -12240 * tvec[2], 2340 * tvec[3], -180 * tvec[4],
                    -10080 * tvec[2], 4680 * tvec[3], -840 * tvec[4], 60 * tvec[5],
                    840 * tvec[3], -360 * tvec[4], 60 * tvec[5], -4 * tvec[6];

            Eigen::Vector4d abyr = tmat * delta_pvaj / tvec[7];
            double aa = abyr[0], bb = abyr[1], yy = abyr[2], rr = abyr[3];
            out_mat.row(i)[0] = aa / 1680.0f;
            out_mat.row(i)[1] = bb / 360.0f;
            out_mat.row(i)[2] = yy / 120.0f;
            out_mat.row(i)[3] = rr / 24.0f;
            out_mat.row(i)[4] = start_state[i+9] / 6.0f;
            out_mat.row(i)[5] = start_state[i+6] / 2.0f;
            out_mat.row(i)[6] = start_state[i+3];
            out_mat.row(i)[7] = start_state[i];
        }

        Piece out_pie(t,out_mat);
        return out_pie;
    }


};


#endif //SRC_OBVP_SOLVER_HPP
