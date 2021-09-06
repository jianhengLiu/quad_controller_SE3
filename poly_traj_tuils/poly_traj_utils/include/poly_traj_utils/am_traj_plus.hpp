/*
 * Yunfan REN
 * renyunfan@outlook.com
 * version: 03-12
 *
 *
 * */


#ifndef AM_TRAJ_PLUS_HPP
#define AM_TRAJ_PLUS_HPP

#include <vector>
#include <memory>
#include <Eigen/Eigen>
#include "iostream"
#include "root_finder.hpp"
#include "traj_utils.hpp"
using namespace std;

namespace order7 {

// Polynomial order and trajectory dimension are fixed here
    constexpr int TrajOrder = 7;
    constexpr int TrajDim = 3;

// Type for piece boundary condition and coefficient matrix
    typedef Eigen::Matrix<double, TrajDim, TrajOrder + 1> BoundaryCond;
    typedef Eigen::Matrix<double, TrajDim, TrajOrder + 1> CoefficientMat;
    typedef Eigen::Matrix<double, TrajDim, TrajOrder> VelCoefficientMat;
    typedef Eigen::Matrix<double, TrajDim, TrajOrder - 1> AccCoefficientMat;
    typedef Eigen::Vector3d Vec3;

// The banded system class is used for solving
// banded linear system Ax=b efficiently.
// A is an N*N band matrix with lower band width lowerBw
// and upper band width upperBw.
// Banded LU factorization has O(N) time complexity.
    class BandedSystem {
    public:
        // The size of A, as well as the lower/upper
        // banded width p/q are needed
        BandedSystem(const int &n, const int &p, const int &q)
                : N(n), lowerBw(p), upperBw(q),
                  ptrData(nullptr), offset(nullptr) {
            int rows = lowerBw + upperBw + 1;
            int actualSize = N * rows;
            ptrData = new double[actualSize];
            std::fill_n(ptrData, actualSize, 0.0);
            offset = new double *[rows];
            double *ptrRow = ptrData;
            for (int i = 0; i < rows; i++) {
                offset[i] = ptrRow;
                ptrRow += N;
            }
        }

        ~BandedSystem() {
            if (ptrData != nullptr) {
                delete[] ptrData;
            }
            if (offset != nullptr) {
                delete[] offset;
            }
        }

    private:
        int N;
        int lowerBw;
        int upperBw;
        double *ptrData;
        double **offset;

    public:
        // The band matrix is stored as suggested in "Matrix Computation"
        inline const double &operator()(const int &i, const int &j) const {
            return offset[i - j + upperBw][j];
        }

        inline double &operator()(const int &i, const int &j) {
            return offset[i - j + upperBw][j];
        }

        // This function conducts banded LU factorization in place
        // Note that the matrix "A" MUST NOT HAVE ZERO DIAGONAL ENREIES !!!
        // Normally, this can be satisfied in most cases where no
        // redundant variables are in x.
        inline void factorizeLU() {
            int iM, jM;
            for (int k = 0; k <= N - 2; k++) {
                iM = std::min(k + lowerBw, N - 1);
                for (int i = k + 1; i <= iM; i++) {
                    operator()(i, k) /= operator()(k, k);
                }
                jM = std::min(k + upperBw, N - 1);
                for (int j = k + 1; j <= jM; j++) {
                    for (int i = k + 1; i <= iM; i++) {
                        operator()(i, j) -= operator()(i, k) * operator()(k, j);
                    }
                }
            }
        }

        // This function solves Ax=b, then stores x in b
        // The input b is required to be N*m, i.e.,
        // m vectors to be solved.
        inline void solve(Eigen::MatrixXd &b) const {
            int iM;
            for (int j = 0; j <= N - 1; j++) {
                iM = std::min(j + lowerBw, N - 1);
                for (int i = j + 1; i <= iM; i++) {
                    b.row(i) -= operator()(i, j) * b.row(j);
                }
            }
            for (int j = N - 1; j >= 0; j--) {
                b.row(j) /= operator()(j, j);
                iM = std::max(0, j - upperBw);
                for (int i = iM; i <= j - 1; i++) {
                    b.row(i) -= operator()(i, j) * b.row(j);
                }
            }
        }
    };

// The trajectory optimizer to get optimal coefficient and durations at the same time
    class AmTraj {
    private:


        // Weights for total duration, acceleration, and jerk
        double wTime;
        double wAcc;
        double wJerk;
        /*  Add snap */
        double wSnap;
        // Constraints on maximum velocity rate and acceleration rate
        double maxVelRate;
        double maxAccRate;

        // Maximum iterations for trajectory optimization
        int maxIterations;

        // Acceptable relative tolerance for everything
        double epsilon;

    private:
        // Allocate durations for all pieces heuristically
        // Trapezoidal time allocation using maximum vel rate and acc rate
        // The arg conservativeness is used to shrink maximum vel rate and acc rate used
        std::vector<double> allocateTime(const std::vector<Eigen::Vector3d> &wayPs,
                                         double conservativeness) const {
            int N = (int) (wayPs.size()) - 1;
            std::vector<double> durations;

            if (N > 0) {
                durations.reserve(N);
                durations.clear();

                double speed = maxVelRate / conservativeness;
                double accRate = maxAccRate / conservativeness;

                Eigen::Vector3d p0, p1;
                double dtxyz, D, acct, accd, dcct, dccd, t1, t2, t3;
                for (int k = 0; k < N; k++) {
                    p0 = wayPs[k];
                    p1 = wayPs[k + 1];
                    D = (p1 - p0).norm();

                    acct = speed / accRate;
                    accd = (accRate * acct * acct / 2);
                    dcct = speed / accRate;
                    dccd = accRate * dcct * dcct / 2;

                    if (D < accd + dccd) {
                        t1 = sqrt(accRate * D) / accRate;
                        t2 = (accRate * t1) / accRate;
                        dtxyz = t1 + t2;
                    } else {
                        t1 = acct;
                        t2 = (D - accd - dccd) / speed;
                        t3 = dcct;
                        dtxyz = t1 + t2 + t3;
                    }

                    durations.push_back(dtxyz);
                }
            }

            return durations;
        }


        // Compute optimal coefficient matrices for all pieces
        // Time allocation, waypoints, and head/tail conditions of traj should be provided
        std::vector<DynamicMat> optimizeCoeffs(const std::vector<Eigen::Vector3d> &wayPs,
                                                   const std::vector<double> &durations,
                                                   const Eigen::Vector3d &iniVel,
                                                   const Eigen::Vector3d &iniAcc,
                                                   const Eigen::Vector3d &iniJerk,
                                                   const Eigen::Vector3d &finVel,
                                                   const Eigen::Vector3d &finAcc,
                                                   const Eigen::Vector3d &finJerk) const {
//            TimeConsuming t__("optimizeCoeffs");
            std::vector<DynamicMat> trajCoeffs;
            int N = durations.size();

            Eigen::VectorXd t1(N);
            Eigen::VectorXd t1n(N);
            for (int i = 0; i < N; i++) {
                t1(i) = durations[i];
                t1n(i) = 1.0 / durations[i];
            }
            /*  1: max t order increase to 7 */
            Eigen::VectorXd t2(t1.cwiseProduct(t1)), t3(t2.cwiseProduct(t1));
            Eigen::VectorXd t4(t3.cwiseProduct(t1)), t5(t4.cwiseProduct(t1));
            Eigen::VectorXd t6(t5.cwiseProduct(t1)), t7(t6.cwiseProduct(t1));

            Eigen::VectorXd t2n(t1n.cwiseProduct(t1n)), t3n(t2n.cwiseProduct(t1n));
            Eigen::VectorXd t4n(t3n.cwiseProduct(t1n)), t5n(t4n.cwiseProduct(t1n));
            Eigen::VectorXd t6n(t5n.cwiseProduct(t1n)), t7n(t6n.cwiseProduct(t1n));

            /*  2: 7-order with 8 coefs in each axis */
            std::vector<Eigen::Matrix<double, 8, 8>> Minvs(N);
            /*  3: A new MinvT matrix */
            Eigen::Matrix<double, 8, 8> CMT;

            for (int i = 0; i < N; i++) {
                // Direct computing inverse mapping with no explicit matrix inverse


                CMT << t7n(i), t6n(i), t5n(i), t4n(i), 1.0, 1.0, 1.0, 1.0,
                        t6n(i), t5n(i), t4n(i), t3n(i), 1.0, 1.0, 1.0, 1.0,
                        t5n(i), t4n(i), t3n(i), t2n(i), 1.0, 1.0, 1.0, 1.0,
                        t4n(i), t3n(i), t2n(i), t1n(i), 1.0, 1.0, 1.0, 1.0,

                        t7n(i), t6n(i), t5n(i), t4n(i), 1.0, 1.0, 1.0, 1.0,
                        t6n(i), t5n(i), t4n(i), t3n(i), 1.0, 1.0, 1.0, 1.0,
                        t5n(i), t4n(i), t3n(i), t2n(i), 1.0, 1.0, 1.0, 1.0,
                        t4n(i), t3n(i), t2n(i), t1n(i), 1.0, 1.0, 1.0, 1.0;

                Minvs[i] = CMT.cwiseProduct(CM);

            }

            /* 4: creat new cv marix[4 x 3] and ca matrix [4 x 3]*/

            Eigen::Matrix<double, 4, 3> CVJ_T;
            Eigen::Matrix<double, 4, 3> CAJ_T;
            Eigen::Matrix<double, 4, 3> CJJ_T;

            Eigen::Matrix<double, 4, 3> CVA_T;
            Eigen::Matrix<double, 4, 3> CAA_T;
            Eigen::Matrix<double, 4, 3> CJA_T;

            Eigen::Matrix<double, 4, 3> CVS_T;
            Eigen::Matrix<double, 4, 3> CAS_T;
            Eigen::Matrix<double, 4, 3> CJS_T;

            vector<Eigen::Matrix<double, 4, 3>> cv(N - 1);
            vector<Eigen::Matrix<double, 4, 3>> ca(N - 1);
            vector<Eigen::Matrix<double, 4, 3>> cj(N - 1);

            // Computed nonzero entries in A and b for linear system Ax=b to be solved
            /*
             * 5: CV CA CJ矩阵都是4x3的矩阵，一共计算了N-1个，分别代表时间0到N-2。
             *
             * */
            for (int i = 0; i < N - 1; i++) {

                if (use_acc_) {
                    CVA_T << t2n(i), t2n(i + 1) - t2n(i), -t2n(i + 1),
                            t1n(i), t1n(i + 1) + t1n(i), t1n(i + 1),
                            1.0, 0.0, -1.0,
                            t1(i), t1(i + 1) + t1(i), t1(i + 1);

                    CAA_T << -t1n(i), t1n(i + 1) + t1n(i), -t1n(i + 1),
                            -1.0, 0.0, 1.0,
                            -t1(i), t1(i + 1) + t1(i), -t1(i + 1),
                            t2(i), t2(i + 1) - t2(i), -t2(i + 1);

                    CJA_T << 1.0, 0.0, -1.0,
                            t1(i), t1(i + 1) + t1(i), t1(i + 1),
                            -t2(i), t2(i + 1) - t2(i), t2(i + 1),
                            - t3(i), t3(i + 1) + t3(i), -t3(i + 1);

                    cv[i] = (CVA_T.cwiseProduct(CVA));
                    ca[i] = (CAA_T.cwiseProduct(CAA));
                    cj[i] = (CJA_T.cwiseProduct(CJA));
                }

                if (use_jerk_) {
                    CVJ_T << t4n(i), t4n(i + 1) - t4n(i), -t4n(i + 1),
                            t3n(i), t3n(i) + t3n(i + 1), t3n(i + 1),
                            t2n(i), t2n(i + 1) - t2n(i), -t2n(i + 1),
                            t1n(i), t1n(i) + t1n(i + 1), t1n(i + 1);

                    CAJ_T << -t3n(i), t3n(i) + t3n(i + 1), -t3n(i + 1),
                            -t2n(i), t2n(i + 1) - t2n(i), t2n(i + 1),
                            -t1n(i), t1n(i) + t1n(i + 1), -t1n(i + 1),
                            -1, 0, 1;

                    CJJ_T << t2n(i), t2n(i + 1) - t2n(i), -t2n(i + 1),
                            t1n(i), t1n(i) + t1n(i + 1), t1n(i + 1),
                            1, 0, -1,
                            -t1(i), t1(i) + t1(i + 1), -t1(i + 1);

                    if(use_acc_){
                        cv[i] += (CVJ_T.cwiseProduct(CVJ));
                        ca[i] += (CAJ_T.cwiseProduct(CAJ));
                        cj[i] += (CJJ_T.cwiseProduct(CJJ));
                    }else{
                        cv[i] = (CVJ_T.cwiseProduct(CVJ));
                        ca[i] = (CAJ_T.cwiseProduct(CAJ));
                        cj[i] = (CJJ_T.cwiseProduct(CJJ));
                    }

                }

                if (use_snap_) {
                    CVS_T << t6n(i), t6n(i + 1) - t6n(i), -t6n(i + 1),
                            t5n(i), t5n(i + 1) + t5n(i), t5n(i + 1),
                            t4n(i), t4n(i + 1) - t4n(i), -t4n(i + 1),
                            t3n(i), t3n(i + 1) + t3n(i), t3n(i + 1);

                    CAS_T << -t5n(i), t5n(i + 1) + t5n(i), -t5n(i + 1),
                            -t4n(i), t4n(i + 1) - t4n(i), t4n(i + 1),
                            -t3n(i), t3n(i + 1) + t3n(i), -t3n(i + 1),
                            -t2n(i), t2n(i + 1) - t2n(i), t2n(i + 1);

                    CJS_T << t4n(i), t4n(i + 1) - t4n(i), -t4n(i + 1),
                            t3n(i), t3n(i + 1) + t3n(i), t3n(i + 1),
                            t2n(i), t2n(i + 1) - t2n(i), -t2n(i + 1),
                            t1n(i), t1n(i + 1) + t1n(i), t1n(i + 1);

                    if(use_jerk_ || use_acc_){
                        cv[i] += (CVS_T.cwiseProduct(CVS));
                        ca[i] += (CAS_T.cwiseProduct(CAS));
                        cj[i] += (CJS_T.cwiseProduct(CJS));
                    }else
                    {
                        cv[i] = (CVS_T.cwiseProduct(CVS));
                        ca[i] = (CAS_T.cwiseProduct(CAS));
                        cj[i] = (CJS_T.cwiseProduct(CJS));
                    }

                }

            }

            /* 6 Add jerk as state varibale*/
            Eigen::MatrixXd VelsAccsJerks(3, 3 * N + 3);
            if (N == 1) {
                VelsAccsJerks << iniVel, iniAcc, iniJerk, finVel, finAcc, finJerk;
            }
            else if (N == 2)
            {
                Eigen::MatrixXd A(3, 3), b(3, 3);
                A.setZero();
                b.setZero();

// These lines are too long... Just let it go...
                A << cv[0](1, 1), cv[0](2, 1), cv[0](3, 1),
                        ca[0](1, 1), ca[0](2, 1), ca[0](3, 1),
                        cj[0](1, 1), cj[0](2, 1), cj[0](3, 1);

                b << (-cv[0](0, 0) * wayPs[0] -
                      cv[0](0, 1) * wayPs[1] -
                      cv[0](0, 2) * wayPs[2] -
                      cv[0](1, 0) * iniVel -
                      cv[0](2, 0) * iniAcc -
                      cv[0](3, 0) * iniJerk -
                      cv[0](1, 2) * finVel -
                      cv[0](2, 2) * finAcc -
                      cv[0](3, 2) * finJerk).transpose(),

                        (-ca[0](0, 0) * wayPs[0] -
                         ca[0](0, 1) * wayPs[1] -
                         ca[0](0, 2) * wayPs[2] -
                         ca[0](1, 0) * iniVel -
                         ca[0](2, 0) * iniAcc -
                         ca[0](3, 0) * iniJerk -
                         ca[0](1, 2) * finVel -
                         ca[0](2, 2) * finAcc -
                         ca[0](3, 2) * finJerk).transpose(),

                        (-cj[0](0, 0) * wayPs[0] -
                         cj[0](0, 1) * wayPs[1] -
                         cj[0](0, 2) * wayPs[2] -
                         cj[0](1, 0) * iniVel -
                         cj[0](2, 0) * iniAcc -
                         cj[0](3, 0) * iniJerk -
                         cj[0](1, 2) * finVel -
                         cj[0](2, 2) * finAcc -
                         cj[0](3, 2) * finJerk).transpose();

                VelsAccsJerks << iniVel, iniAcc, iniJerk, (A.inverse() * b).transpose(), finVel, finAcc, finJerk;
            }
            else
            {
                BandedSystem A(3 * N - 3, 5, 5);
                Eigen::MatrixXd b(3 * N - 3, 3);
                b.setZero();

                /*  起点 v */
                A(0, 0) = cv[0](1, 1);
                A(0, 1) = cv[0](2, 1);
                A(0, 2) = cv[0](3, 1);

                A(0, 3) = cv[0](1, 2);
                A(0, 4) = cv[0](2, 2);
                A(0, 5) = cv[0](3, 2);
                /*   起点 a */
                A(1, 0) = ca[0](1, 1);
                A(1, 1) = ca[0](2, 1);
                A(1, 2) = ca[0](3, 1);
                A(1, 3) = ca[0](1, 2);
                A(1, 4) = ca[0](2, 2);
                A(1, 5) = ca[0](3, 2);
                /*   起点 j */
                A(2, 0) = cj[0](1, 1);
                A(2, 1) = cj[0](2, 1);
                A(2, 2) = cj[0](3, 1);
                A(2, 3) = cj[0](1, 2);
                A(2, 4) = cj[0](2, 2);
                A(2, 5) = cj[0](3, 2);

/*  终点 v */
                A(3 * N - 6, 3 * N - 9) = cv[N - 2](1, 0);
                A(3 * N - 6, 3 * N - 8) = cv[N - 2](2, 0);
                A(3 * N - 6, 3 * N - 7) = cv[N - 2](3, 0);
                A(3 * N - 6, 3 * N - 6) = cv[N - 2](1, 1);
                A(3 * N - 6, 3 * N - 5) = cv[N - 2](2, 1);
                A(3 * N - 6, 3 * N - 4) = cv[N - 2](3, 1);
/*  终点 a */
                A(3 * N - 5, 3 * N - 9) = ca[N - 2](1, 0);
                A(3 * N - 5, 3 * N - 8) = ca[N - 2](2, 0);
                A(3 * N - 5, 3 * N - 7) = ca[N - 2](3, 0);
                A(3 * N - 5, 3 * N - 6) = ca[N - 2](1, 1);
                A(3 * N - 5, 3 * N - 5) = ca[N - 2](2, 1);
                A(3 * N - 5, 3 * N - 4) = ca[N - 2](3, 1);

/*  终点 j */
                A(3 * N - 4, 3 * N - 9) = cj[N - 2](1, 0);
                A(3 * N - 4, 3 * N - 8) = cj[N - 2](2, 0);
                A(3 * N - 4, 3 * N - 7) = cj[N - 2](3, 0);
                A(3 * N - 4, 3 * N - 6) = cj[N - 2](1, 1);
                A(3 * N - 4, 3 * N - 5) = cj[N - 2](2, 1);
                A(3 * N - 4, 3 * N - 4) = cj[N - 2](3, 1);


                b.topLeftCorner<3, 3>() <<
                                        (-cv[0](0, 0) * wayPs[0] -
                                         cv[0](0, 1) * wayPs[1] -
                                         cv[0](0, 2) * wayPs[2] -
                                         cv[0](1, 0) * iniVel -
                                         cv[0](2, 0) * iniAcc -
                                         cv[0](3, 0) * iniJerk).transpose(),

                        (-ca[0](0, 0) * wayPs[0] -
                         ca[0](0, 1) * wayPs[1] -
                         ca[0](0, 2) * wayPs[2] -
                         ca[0](1, 0) * iniVel -
                         ca[0](2, 0) * iniAcc -
                         ca[0](3, 0) * iniJerk).transpose(),

                        (-cj[0](0, 0) * wayPs[0] -
                         cj[0](0, 1) * wayPs[1] -
                         cj[0](0, 2) * wayPs[2] -
                         cj[0](1, 0) * iniVel -
                         cj[0](2, 0) * iniAcc -
                         cj[0](3, 0) * iniJerk).transpose();

                b.bottomRightCorner<3, 3>() <<
                                            (-cv[N - 2](0, 0) * wayPs[N - 2] -
                                             cv[N - 2](0, 1) * wayPs[N - 1] -
                                             cv[N - 2](0, 2) * wayPs[N] -
                                             cv[N - 2](1, 2) * finVel -
                                             cv[N - 2](2, 2) * finAcc -
                                             cv[N - 2](3, 2) * finJerk).transpose(),

                        (-ca[N - 2](0, 0) * wayPs[N - 2] -
                         ca[N - 2](0, 1) * wayPs[N - 1] -
                         ca[N - 2](0, 2) * wayPs[N] -
                         ca[N - 2](1, 2) * finVel -
                         ca[N - 2](2, 2) * finAcc -
                         ca[N - 2](3, 2) * finJerk).transpose(),

                        (-cj[N - 2](0, 0) * wayPs[N - 2] -
                         cj[N - 2](0, 1) * wayPs[N - 1] -
                         cj[N - 2](0, 2) * wayPs[N] -
                         cj[N - 2](1, 2) * finVel -
                         cj[N - 2](2, 2) * finAcc -
                         cj[N - 2](3, 2) * finJerk).transpose();


                for (int i = 1; i < N - 2; i++) {
/* V */
                    A(i * 3, i * 3 - 3) = cv[i](1, 0);
                    A(i * 3, i * 3 - 2) = cv[i](2, 0);
                    A(i * 3, i * 3 - 1) = cv[i](3, 0);

                    A(i * 3, i * 3) = cv[i](1, 1);
                    A(i * 3, i * 3 + 1) = cv[i](2, 1);
                    A(i * 3, i * 3 + 2) = cv[i](3, 1);

                    A(i * 3, i * 3 + 3) = cv[i](1, 2);
                    A(i * 3, i * 3 + 4) = cv[i](2, 2);
                    A(i * 3, i * 3 + 5) = cv[i](3, 2);
/* A */
                    A(i * 3 + 1, i * 3 - 3) = ca[i](1, 0);
                    A(i * 3 + 1, i * 3 - 2) = ca[i](2, 0);
                    A(i * 3 + 1, i * 3 - 1) = ca[i](3, 0);

                    A(i * 3 + 1, i * 3) = ca[i](1, 1);
                    A(i * 3 + 1, i * 3 + 1) = ca[i](2, 1);
                    A(i * 3 + 1, i * 3 + 2) = ca[i](3, 1);

                    A(i * 3 + 1, i * 3 + 3) = ca[i](1, 2);
                    A(i * 3 + 1, i * 3 + 4) = ca[i](2, 2);
                    A(i * 3 + 1, i * 3 + 5) = ca[i](3, 2);
/* J */
                    A(i * 3 + 2, i * 3 - 3) = cj[i](1, 0);
                    A(i * 3 + 2, i * 3 - 2) = cj[i](2, 0);
                    A(i * 3 + 2, i * 3 - 1) = cj[i](3, 0);

                    A(i * 3 + 2, i * 3) = cj[i](1, 1);
                    A(i * 3 + 2, i * 3 + 1) = cj[i](2, 1);
                    A(i * 3 + 2, i * 3 + 2) = cj[i](3, 1);

                    A(i * 3 + 2, i * 3 + 3) = cj[i](1, 2);
                    A(i * 3 + 2, i * 3 + 4) = cj[i](2, 2);
                    A(i * 3 + 2, i * 3 + 5) = cj[i](3, 2);

                    b.block<3, 3>(i * 3, 0) <<
                                            (-cv[i](0, 0) * wayPs[i] -
                                             cv[i](0, 1) * wayPs[i + 1] -
                                             cv[i](0, 2) * wayPs[i + 2]).transpose(),

                            (-ca[i](0, 0) * wayPs[i] -
                             ca[i](0, 1) * wayPs[i + 1] -
                             ca[i](0, 2) * wayPs[i + 2]).transpose(),

                            (-cj[i](0, 0) * wayPs[i] -
                             cj[i](0, 1) * wayPs[i + 1] -
                             cj[i](0, 2) * wayPs[i + 2]).transpose();
                }
//                 Solve Ax=b using banded LU factorization
                A.factorizeLU();
                // The solution is computed in place.
                A.solve(b);
                VelsAccsJerks << iniVel, iniAcc, iniJerk, b.transpose(), finVel, finAcc, finJerk;
//                VelsAccsJerks << iniVel, iniAcc, iniJerk,(A.inverse() * b).transpose(), finVel, finAcc, finJerk;
            }

            // Recover coefficient matrices for all pieces from their boundary conditions
            Eigen::MatrixXd PosVelAccJekrPair(3, 8);
            for (int i = 0; i < N; i++) {
                PosVelAccJekrPair << wayPs[i],
                        VelsAccsJerks.col(3 * i),
                        VelsAccsJerks.col(3 * i + 1),
                        VelsAccsJerks.col(3 * i + 2),
                        wayPs[i + 1],
                        VelsAccsJerks.col(3 * i + 3),
                        VelsAccsJerks.col(3 * i + 4),
                        VelsAccsJerks.col(3 * i + 5);
//                cout<<PosVelAccJekrPair.transpose()<<endl;
                trajCoeffs.push_back(PosVelAccJekrPair * Minvs[i]);
            }
            return trajCoeffs;
        }


        // Clip the norm of vec3D if it exceeds (1-eps)*maxNorm
        inline void clipNorm(Eigen::Vector3d &vec3D, double maxNorm, double eps) const {
            const double gamma = 1 - eps;
            double tempNorm = vec3D.norm();
            vec3D *= tempNorm > gamma * maxNorm ? gamma * maxNorm / tempNorm : 1.0;
        }

        // Make sure the head/tail conditions of the trajectory satisfy constraints
        void enforceBoundFeasibility(Eigen::Vector3d &iniVel, Eigen::Vector3d &iniAcc,
                                     Eigen::Vector3d &finVel, Eigen::Vector3d &finAcc) const {
            clipNorm(iniVel, maxVelRate, epsilon);
            clipNorm(iniAcc, maxAccRate, epsilon);
            clipNorm(finVel, maxVelRate, epsilon);
            clipNorm(finAcc, maxAccRate, epsilon);
        }

        // Make sure a given initial guess traj satisfies constraints
        bool enforceIniTrajFeasibility(Trajectory &traj, int tryOut) const {
            int N = traj.getPieceNum();
            if (N > 0) {
                std::vector<Eigen::Vector3d> posVec, velVec, accVec, jerkVec;
                std::vector<double> durVec;

                posVec.push_back(traj.getJuncPos(0));
                velVec.push_back(traj.getJuncVel(0));
                accVec.push_back(traj.getJuncAcc(0));
                jerkVec.push_back(traj.getJuncJerk(0));

                clipNorm(velVec.back(), maxVelRate, epsilon);
                clipNorm(accVec.back(), maxAccRate, epsilon);

                // Clip norms of all derivatives at waypoints
                for (int i = 0; i < N; i++) {
                    durVec.push_back(traj[i].getDuration());
                    posVec.push_back(traj.getJuncPos(i + 1));
                    velVec.push_back(traj.getJuncVel(i + 1));
                    accVec.push_back(traj.getJuncAcc(i + 1));
                    jerkVec.push_back(traj.getJuncJerk(i + 1));
                    clipNorm(velVec.back(), maxVelRate, epsilon);
                    clipNorm(accVec.back(), maxAccRate, epsilon);
                }

                // Violently scale all durations until all pieces satisfy constrains
                double adjustRatio = 2.0;
                Piece piece;
                bool feasible;
                for (int j = 0; j < tryOut; j++) {
                    feasible = true;
                    for (int i = 0; i < N; i++) {
                        if (!traj[i].checkMaxAccRate(maxAccRate) ||
                            !traj[i].checkMaxVelRate(maxVelRate)) {
                            /* Adjust the states and times, except the ini and fin states */
                            durVec[i] *= adjustRatio;
                            velVec[i] /= (i == 0 ? 1.0 : adjustRatio);
                            accVec[i] /= (i == 0 ? 1.0 : adjustRatio);
                            jerkVec[i] /= (i == 0 ? 1.0 : adjustRatio);

                            velVec[i + 1] /= (i == N - 2 ? 1.0 : adjustRatio);
                            accVec[i + 1] /= (i == N - 2 ? 1.0 : adjustRatio);
                            jerkVec[i + 1] /= (i == N - 2 ? 1.0 : adjustRatio);
                            feasible = false;
                        }
                    }

                    if (feasible) {
                        break;
                    } else {
                        traj.clear();
                    }

                    // Recover a feasibile trajectory
                    BoundaryCond boundCond;
                    for (int i = 0; i < N; i++) {
                        boundCond << posVec[i], velVec[i], accVec[i], jerkVec[i],
                                posVec[i + 1], velVec[i + 1], accVec[i + 1], jerkVec[i + 1];
                        traj.emplace_back(boundCond, durVec[i]);
                    }
                }
                if (!feasible) {
                    traj.clear();
                }
            }

            return traj.getPieceNum() != 0;
        }

        // Compute the objective of a single piece determined by boundCond and duration
        double evaluateObjective(const BoundaryCond &boundCond, double duration) const {
            Eigen::Array3d
                    p_0 = boundCond.col(0),
                    v_0 = boundCond.col(1),
                    a_0 = boundCond.col(2),
                    j_0 = boundCond.col(3),
                    p_f = boundCond.col(4),
                    v_f = boundCond.col(5),
                    a_f = boundCond.col(6),
                    j_f = boundCond.col(7);

            double t1 = duration, t2 = t1 * duration;
            double t3 = t2 * duration, t4 = t3 * duration;
            double t5 = t4 * duration , t6 = t5 * duration;
            double t7 = t6 * duration;
            Eigen::VectorXd coeffsAccObjective(7);
            Eigen::VectorXd coeffsJerkObjective(7);
            Eigen::VectorXd coeffsSnapObjective(7);
            double cost = 0.0;
            if (use_acc_) {
                coeffsAccObjective(0) = (4 * j_0.square() - 3 * j_0 * j_f + 4 * j_f.square()).sum();
                coeffsAccObjective(1) = (140 * a_0 * j_0 - 25 * a_0 * j_f + 25 * a_f * j_0 - 140 * a_f * j_f).sum();
                coeffsAccObjective(2) = (480 * j_0 * v_0 - 30 * a_0 * a_f + 150 * j_0 * v_f + 150 * j_f * v_0 +
                                         480 * j_f * v_f + 1500 * a_0.square() + 1500 * a_f.square()).sum();
                coeffsAccObjective(3) = (11370 * a_0 * v_0 + 5430 * a_0 * v_f - 5430 * a_f * v_0 - 11370 * a_f * v_f +
                                         630 * j_0 * p_0 - 630 * j_0 * p_f + 630 * j_f * p_0 - 630 * j_f * p_f).sum();
                coeffsAccObjective(4) = (16800 * a_0 * p_0 - 16800 * a_0 * p_f - 16800 * a_f * p_0 + 16800 * a_f * p_f +
                                         68400 * v_0 * v_f + 54000 * v_0.square() + 54000 * v_f.square()).sum();
                coeffsAccObjective(5) = (176400 * p_0 * v_0 + 176400 * p_0 * v_f - 176400 * p_f * v_0 -
                                         176400 * p_f * v_f).sum();
                coeffsAccObjective(6) = (176400 * p_0.square() - 352800 * p_0 * p_f + 176400 * p_f.square()).sum();

                cost += wAcc * RootFinder::polyVal(coeffsAccObjective, duration) / (6930.0 * t3);

            }
            if (use_jerk_) {
                coeffsJerkObjective(0) = (4 * j_0.square() - j_0 * j_f + 4 * j_f.square()).sum();
                coeffsJerkObjective(1) = (69 * a_0 * j_0 + 15 * a_0 * j_f - 15 * a_f * j_0 - 69 * a_f * j_f).sum();
                coeffsJerkObjective(2) = (264 * j_0 * v_0 - 720 * a_0 * a_f + 156 * j_0 * v_f + 156 * j_f * v_0 +
                                          264 * j_f * v_f + 900 * a_0.square() + 900 * a_f.square()).sum();
                coeffsJerkObjective(3) = (7380 * a_0 * v_0 + 5220 * a_0 * v_f - 5220 * a_f * v_0 - 7380 * a_f * v_f +
                                          420 * j_0 * p_0 - 420 * j_0 * p_f + 420 * j_f * p_0 - 420 * j_f * p_f).sum();
                coeffsJerkObjective(4) = (12600 * a_0 * p_0 - 12600 * a_0 * p_f - 12600 * a_f * p_0 +
                                          12600 * a_f * p_f + 33120 * v_0 * v_f + 18720 * v_0.square() +
                                          18720 * v_f.square()).sum();
                coeffsJerkObjective(5) = (70560 * p_0 * v_0 + 70560 * p_0 * v_f - 70560 * p_f * v_0 -
                                          70560 * p_f * v_f).sum();
                coeffsJerkObjective(6) = (70560 * p_0.square() - 141120 * p_0 * p_f + 70560 * p_f.square()).sum();

                cost += wJerk * RootFinder::polyVal(coeffsJerkObjective, duration) / (63.0 * t5);
            }
            if (use_snap_) {
                coeffsSnapObjective(0) =(8*j_0*j_f + 16*j_0.square() + 16*j_f.square()).sum();
                coeffsSnapObjective(1) =(240*a_0*j_0 + 120*a_0*j_f - 120*a_f*j_0 - 240*a_f*j_f).sum();
                coeffsSnapObjective(2) =(960*j_0*v_0 - 1680*a_0*a_f + 720*j_0*v_f + 720*j_f*v_0 + 960*j_f*v_f + 1200*a_0.square() + 1200*a_f.square()).sum();
                coeffsSnapObjective(3) =(10800*a_0*v_0 + 9360*a_0*v_f - 9360*a_f*v_0 - 10800*a_f*v_f + 1680*j_0*p_0 - 1680*j_0*p_f + 1680*j_f*p_0 - 1680*j_f*p_f).sum();
                coeffsSnapObjective(4) =(20160*a_0*p_0 - 20160*a_0*p_f - 20160*a_f*p_0 + 20160*a_f*p_f + 48960*v_0*v_f + 25920*v_0.square() + 25920*v_f.square()).sum();
                coeffsSnapObjective(5) =(100800*p_0*v_0 + 100800*p_0*v_f - 100800*p_f*v_0 - 100800*p_f*v_f).sum();
                coeffsSnapObjective(6) =(100800*p_0.square() - 201600*p_0*p_f + 100800*p_f.square()).sum();
                cost += wSnap * RootFinder::polyVal(coeffsSnapObjective, duration) / t7;
//                cost += wSnap * (16*t6*j_0.square() + 8*t6*j_0*j_f + 16*t6*j_f.square() + 240*t5*a_0*j_0 +
//                                120*t5*a_0*j_f - 120*t5*a_f*j_0 - 240*t5*a_f*j_f + 1200*t4*a_0.square() -
//                                1680*t4*a_0*a_f + 1200*t4*a_f.square() + 960*t4*j_0*v_0 + 720*t4*j_0*v_f +
//                                720*t4*j_f*v_0 + 960*t4*j_f*v_f + 10800*t3*a_0*v_0 + 9360*t3*a_0*v_f -
//                                9360*t3*a_f*v_0 - 10800*t3*a_f*v_f + 1680*t3*j_0*p_0 - 1680*t3*j_0*p_f +
//                                1680*t3*j_f*p_0 - 1680*t3*j_f*p_f + 20160*t2*a_0*p_0 - 20160*t2*a_0*p_f -
//                                20160*t2*a_f*p_0 + 20160*t2*a_f*p_f + 25920*t2*v_0.square() + 48960*t2*v_0*v_f +
//                                25920*t2*v_f.square() + 100800*t1*p_0*v_0 + 100800*t1*p_0*v_f - 100800*t1*p_f*v_0 - 100800*t1*p_f*v_f +
//                                100800*p_0.square() - 201600*p_0*p_f + 100800*p_f.square()
//                               ).sum() / t7;
            }
            cost += wTime * t1;
            return (cost);
        }

    public:
        // Compute the objective over a whole trajectory
        double evaluateObjective(const Trajectory &traj) const {
            double objective = 0.0;

            int N = traj.getPieceNum();

            for (int i = 0; i < N; i++) {
                objective += evaluateObjective(traj[i].getBoundCond(),
                                               traj[i].getDuration());
            }

            return objective;
        }

    private:
        // Optimize the coefficient matrices of a traj, satisfying constraints all the time
        // The index of piece stuck by constraints is updated for consequent optimization
        void optimizeCoeffsConstrained(Trajectory &traj, int &idxPieceStuck) const {
            int N = traj.getPieceNum();

            std::vector<double> durVec;
            std::vector<Eigen::Vector3d> posVec;
            Eigen::Vector3d velIni, velFin, jerkFin;
            Eigen::Vector3d accIni, accFin, jerkIni;
            Eigen::MatrixXd vels(3, N - 1);
            Eigen::MatrixXd accs(3, N - 1);
            Eigen::MatrixXd jers(3, N - 1);

            // Recover free boundary conditions Dpk
            durVec.push_back(traj[0].getDuration());
            posVec.push_back(traj.getJuncPos(0));
            velIni = traj.getJuncVel(0);
            accIni = traj.getJuncAcc(0);
            jerkIni = traj.getJuncJerk(0);
            for (int i = 0; i < N - 1; i++) {
                durVec.push_back(traj[i + 1].getDuration());
                posVec.push_back(traj.getJuncPos(i + 1));
                vels.col(i) = traj.getJuncVel(i + 1);
                accs.col(i) = traj.getJuncAcc(i + 1);
                jers.col(i) = traj.getJuncJerk(i + 1);
            }
            posVec.push_back(traj.getJuncPos(N));
            velFin = traj.getJuncVel(N);
            accFin = traj.getJuncAcc(N);
            jerkFin = traj.getJuncJerk(N);
            // Calculate optimal boundary conditions in absence of constraints
            std::vector<DynamicMat> targetCoeffsMats = optimizeCoeffs(posVec, durVec,
                                                                          velIni, accIni, jerkIni,
                                                                          velFin, accFin, jerkFin);

            traj = Trajectory(durVec, targetCoeffsMats);

            // Extract free part of optimal boundary conditions Dp*
            Eigen::MatrixXd velsTarget(3, N - 1);
            Eigen::MatrixXd accsTarget(3, N - 1);
            Eigen::MatrixXd jersTarget(3, N - 1);
            for (int i = 0; i < N - 1; i++) {
                velsTarget.col(i) = traj.getJuncVel(i + 1);
                accsTarget.col(i) = traj.getJuncAcc(i + 1);
                jersTarget.col(i) = traj.getJuncJerk(i + 1);
            }

            // Check whether convex combination (1 - lambda)*Dpk + lambda*Dp* is feasibile
            auto temporalFeasibilityCheck = [&](double lambda) {
                bool feasible = true;
                BoundaryCond boundCond;
                Piece piece;
                for (int i = 0; i < N; i++) {
                    boundCond << posVec[i],
                            (i == 0) ? velIni : ((1 - lambda) * vels.col(i - 1) + lambda * velsTarget.col(i - 1)),
                            (i == 0) ? accIni : ((1 - lambda) * accs.col(i - 1) + lambda * accsTarget.col(i - 1)),
                            (i == 0) ? jerkIni : ((1 - lambda) * jers.col(i - 1) + lambda * jersTarget.col(i - 1)),
                            posVec[i + 1],
                            (i == N - 1) ? velFin : (1 - lambda) * vels.col(i) + lambda * velsTarget.col(i),
                            (i == N - 1) ? accFin : (1 - lambda) * accs.col(i) + lambda * accsTarget.col(i),
                            (i == N - 1) ? jerkFin : (1 - lambda) * jers.col(i) + lambda * jersTarget.col(i);
                    piece = Piece(boundCond, durVec[i]);
                    if (!piece.checkMaxAccRate(maxAccRate) ||
                        !piece.checkMaxVelRate(maxVelRate)) {
                        idxPieceStuck = i;
                        feasible = false;
                        break;
                    }
                }
                return feasible;
            };

            // Locate the best lambda of convex combination by bisection
            double bestLambda = 0.0;
            if (temporalFeasibilityCheck(bestLambda)) {
                bestLambda = 1.0;
                if (!temporalFeasibilityCheck(bestLambda)) {
                    int maxIts = std::max(-(int) log2(epsilon), 0) + 1;

                    double lbound = 0.0, rbound = 1.0, mid;

                    for (int i = 0; i < maxIts; i++) {
                        mid = (lbound + rbound) / 2.0;

                        if (temporalFeasibilityCheck(mid)) {
                            lbound = mid;
                        } else {
                            rbound = mid;
                        }
                    }

                    bestLambda = lbound;
                }
            }

            traj.clear();
            BoundaryCond boundCond;
            for (int i = 0; i < N; i++) {
                boundCond << posVec[i],
                        (i == 0) ? velIni : ((1 - bestLambda) * vels.col(i - 1) + bestLambda * velsTarget.col(i - 1)),
                        (i == 0) ? accIni : ((1 - bestLambda) * accs.col(i - 1) + bestLambda * accsTarget.col(i - 1)),
                        (i == 0) ? jerkIni : ((1 - bestLambda) * jers.col(i - 1) + bestLambda * jersTarget.col(i - 1)),
                        posVec[i + 1],
                        (i == N - 1) ? velFin : (1 - bestLambda) * vels.col(i) + bestLambda * velsTarget.col(i),
                        (i == N - 1) ? accFin : (1 - bestLambda) * accs.col(i) + bestLambda * accsTarget.col(i),
                        (i == N - 1) ? jerkIni : (1 - bestLambda) * jers.col(i) + bestLambda * jersTarget.col(i);
                traj.emplace_back(boundCond, durVec[i]);
            }

            // Update the last index piece stuck by constraints
            idxPieceStuck = bestLambda == 0.0 ? idxPieceStuck : -1;
        }

        // Optimized durations for all pieces of a trajertory, with or without constraints
        void optimizeDurations(Trajectory &traj, bool constrained = true) const {

            int N = traj.getPieceNum();

            std::vector<BoundaryCond> boundConds;
            std::vector<double> initialDurations;

            // Backup boundary conditions as durations
            for (int i = 0; i < N; i++) {
                boundConds.push_back(traj[i].getBoundCond());
//                cout<<traj[i].getBoundCond().transpose()<<endl;
                initialDurations.push_back(traj[i].getDuration());
            }

            traj.clear();

            Piece piece;
            Eigen::VectorXd coeffsGradT(11);

            double tempAccTerm, tempJerkTerm, tempSnapTerm;
            Eigen::Array3d p_0, v_0, a_0, j_0, p_f, v_f, a_f, j_f;
            for (int i = 0; i < N; i++) {
                coeffsGradT.setZero();
                // Calculate the numerator of dJi(T)/dT
                p_0 << boundConds[i].col(0);
                v_0 << boundConds[i].col(1);
                a_0 << boundConds[i].col(2);
                j_0 << boundConds[i].col(3);
                p_f << boundConds[i].col(4);
                v_f << boundConds[i].col(5);
                a_f << boundConds[i].col(6);
                j_f << boundConds[i].col(7);

                if (use_acc_) {
                    coeffsGradT(0) += wAcc*((2*j_0.square())/1155 - (j_0*j_f)/770 + (2*j_f.square())/1155).sum();
                    coeffsGradT(1) += wAcc*((4*a_0*j_0)/99 - (5*a_0*j_f)/693 + (5*a_f*j_0)/693 - (4*a_f*j_f)/99).sum();
                    coeffsGradT(2) += wAcc*((16*j_0*v_0)/231 - (a_0*a_f)/231 + (5*j_0*v_f)/231 + (5*j_f*v_0)/231 + (16*j_f*v_f)/231 + (50*a_0.square())/231 + (50*a_f.square())/231).sum();
                    coeffsGradT(4) += wAcc*((80*a_0*p_f)/33 - (80*a_0*p_0)/33 + (80*a_f*p_0)/33 - (80*a_f*p_f)/33 - (760*v_0*v_f)/77 - (600*v_0.square())/77 - (600*v_f.square())/77).sum();
                    coeffsGradT(5) += wAcc*((560*p_f*v_0)/11 - (560*p_0*v_f)/11 - (560*p_0*v_0)/11 + (560*p_f*v_f)/11).sum();
                    coeffsGradT(6) += wAcc*((1680*p_0*p_f)/11 - (840*p_0.square())/11 - (840*p_f.square())/11).sum();

                }

                if (use_jerk_) {
                    coeffsGradT(2) += wJerk*((4*j_0.square())/63 - (j_0*j_f)/63 + (4*j_f.square())/63).sum();
                    coeffsGradT(4) += wJerk*((80*a_0*a_f)/7 - (88*j_0*v_0)/21 - (52*j_0*v_f)/21 - (52*j_f*v_0)/21 - (88*j_f*v_f)/21 - (100*a_0.square())/7 - (100*a_f.square())/7).sum();
                    coeffsGradT(5) += wJerk*((1160*a_f*v_0)/7 - (1160*a_0*v_f)/7 - (1640*a_0*v_0)/7 + (1640*a_f*v_f)/7 - (40*j_0*p_0)/3 + (40*j_0*p_f)/3 - (40*j_f*p_0)/3 + (40*j_f*p_f)/3).sum();
                    coeffsGradT(6) += wJerk*(600*a_0*p_f - 600*a_0*p_0 + 600*a_f*p_0 - 600*a_f*p_f - (11040*v_0*v_f)/7 - (6240*v_0.square())/7 - (6240*v_f.square())/7).sum();
                    coeffsGradT(7) += wJerk*(4480*p_f*v_0 - 4480*p_0*v_f - 4480*p_0*v_0 + 4480*p_f*v_f).sum();
                    coeffsGradT(8) += wJerk*(11200*p_0*p_f - 5600*p_0.square() - 5600*p_f.square()).sum();

                }

                if (use_snap_) {
                    coeffsGradT(4) += wSnap*(- 8*j_0*j_f - 16*j_0.square() - 16*j_f.square()).sum();
                    coeffsGradT(5) += wSnap*(240*a_f*j_0 - 240*a_0*j_f - 480*a_0*j_0 + 480*a_f*j_f).sum();
                    coeffsGradT(6) += wSnap*(5040*a_0*a_f - 2880*j_0*v_0 - 2160*j_0*v_f - 2160*j_f*v_0 - 2880*j_f*v_f - 3600*a_0.square() - 3600*a_f.square()).sum();
                    coeffsGradT(7) += wSnap*(37440*a_f*v_0 - 37440*a_0*v_f - 43200*a_0*v_0 + 43200*a_f*v_f - 6720*j_0*p_0 + 6720*j_0*p_f - 6720*j_f*p_0 + 6720*j_f*p_f).sum();
                    coeffsGradT(8) += wSnap*(100800*a_0*p_f - 100800*a_0*p_0 + 100800*a_f*p_0 - 100800*a_f*p_f - 244800*v_0*v_f - 129600*v_0.square() - 129600*v_f.square()).sum();
                    coeffsGradT(9) += wSnap*(604800*p_f*v_0 - 604800*p_0*v_f - 604800*p_0*v_0 + 604800*p_f*v_f).sum();
                    coeffsGradT(10) += wSnap*(1411200*p_0*p_f - 705600*p_0.square() - 705600*p_f.square()).sum();
                }
                coeffsGradT(2) += wTime;


//                coeffsGradT(0) = ((2*j_0.square()*wAcc)/1155 + (2*j_f.square()*wAcc)/1155 - (j_0*j_f*wAcc)/770).sum();
//                coeffsGradT(1) = ((4*a_0*j_0*wAcc)/99 - (5*a_0*j_f*wAcc)/693 + (5*a_f*j_0*wAcc)/693 - (4*a_f*j_f*wAcc)/99).sum();
//                coeffsGradT(2) = (wTime + (50*a_0.square()*wAcc)/231 + (50*a_f.square()*wAcc)/231 + (4*j_0.square()*wJerk)/63 + (4*j_f.square()*wJerk)/63 - (a_0*a_f*wAcc)/231 - (j_0*j_f*wJerk)/63 + (16*j_0*v_0*wAcc)/231 + (5*j_0*v_f*wAcc)/231 + (5*j_f*v_0*wAcc)/231 + (16*j_f*v_f*wAcc)/231).sum();
//                coeffsGradT(4) = ((80*a_0*a_f*wJerk)/7 - (100*a_f.square()*wJerk)/7 - 16*j_0.square()*wSnap - 16*j_f.square()*wSnap - (600*v_0.square()*wAcc)/77 - (600*v_f.square()*wAcc)/77 - (100*a_0.square()*wJerk)/7 - (80*a_0*p_0*wAcc)/33 + (80*a_0*p_f*wAcc)/33 + (80*a_f*p_0*wAcc)/33 - (80*a_f*p_f*wAcc)/33 - 8*j_0*j_f*wSnap - (88*j_0*v_0*wJerk)/21 - (52*j_0*v_f*wJerk)/21 - (52*j_f*v_0*wJerk)/21 - (88*j_f*v_f*wJerk)/21 - (760*v_0*v_f*wAcc)/77).sum();
//                coeffsGradT(5) = (240*a_f*j_0*wSnap - 240*a_0*j_f*wSnap - 480*a_0*j_0*wSnap + 480*a_f*j_f*wSnap - (1640*a_0*v_0*wJerk)/7 - (1160*a_0*v_f*wJerk)/7 + (1160*a_f*v_0*wJerk)/7 + (1640*a_f*v_f*wJerk)/7 - (40*j_0*p_0*wJerk)/3 + (40*j_0*p_f*wJerk)/3 - (40*j_f*p_0*wJerk)/3 + (40*j_f*p_f*wJerk)/3 - (560*p_0*v_0*wAcc)/11 - (560*p_0*v_f*wAcc)/11 + (560*p_f*v_0*wAcc)/11 + (560*p_f*v_f*wAcc)/11).sum();
//                coeffsGradT(6) = (5040*a_0*a_f*wSnap - 3600*a_f.square()*wSnap - (840*p_0.square()*wAcc)/11 - (840*p_f.square()*wAcc)/11 - (6240*v_0.square()*wJerk)/7 - (6240*v_f.square()*wJerk)/7 - 3600*a_0.square()*wSnap - 600*a_0*p_0*wJerk + 600*a_0*p_f*wJerk + 600*a_f*p_0*wJerk - 600*a_f*p_f*wJerk + (1680*p_0*p_f*wAcc)/11 - 2880*j_0*v_0*wSnap - 2160*j_0*v_f*wSnap - 2160*j_f*v_0*wSnap - 2880*j_f*v_f*wSnap - (11040*v_0*v_f*wJerk)/7).sum();
//                coeffsGradT(7) = (37440*a_f*v_0*wSnap - 37440*a_0*v_f*wSnap - 43200*a_0*v_0*wSnap + 43200*a_f*v_f*wSnap - 6720*j_0*p_0*wSnap + 6720*j_0*p_f*wSnap - 6720*j_f*p_0*wSnap + 6720*j_f*p_f*wSnap - 4480*p_0*v_0*wJerk - 4480*p_0*v_f*wJerk + 4480*p_f*v_0*wJerk + 4480*p_f*v_f*wJerk).sum();
//                coeffsGradT(8) = (100800*a_0*p_f*wSnap - 5600*p_f.square()*wJerk - 129600*v_0.square()*wSnap - 129600*v_f.square()*wSnap - 100800*a_0*p_0*wSnap - 5600*p_0.square()*wJerk + 100800*a_f*p_0*wSnap - 100800*a_f*p_f*wSnap + 11200*p_0*p_f*wJerk - 244800*v_0*v_f*wSnap).sum();
//                coeffsGradT(9) = (604800*p_f*v_0*wSnap - 604800*p_0*v_f*wSnap - 604800*p_0*v_0*wSnap + 604800*p_f*v_f*wSnap).sum();
//                coeffsGradT(10) = (1411200*p_0*p_f*wSnap - 705600*p_f.square()*wSnap - 705600*p_0.square()*wSnap).sum();

                // Compute all stationaries in which the optimal duration locates
                std::set<double> stationaries;
                stationaries = RootFinder::solvePolynomial(coeffsGradT, 0.0, INFINITY,
                                                               initialDurations[i] * epsilon * epsilon);
//
//                cout<<"Get n solutions:"<<stationaries.size()<<endl;
//
//                for (auto it = stationaries.begin(); it != stationaries.end(); it++)
//                    cout<<*it<<endl;
                std::set<double> candidates;
                if (constrained) {
                    // When constraints are considered, duration T~ should be found where some constraints are tight
                    std::set<double> infeasibleStationaries, feasibleStationaries;
                    for (auto it = stationaries.begin(); it != stationaries.end(); it++) {
                        piece = Piece(boundConds[i], *it);
                        if (piece.checkMaxAccRate(maxAccRate) &&
                            piece.checkMaxVelRate(maxVelRate)) {
                            feasibleStationaries.insert(*it);
                        } else {
                            infeasibleStationaries.insert(*it);
                        }
                    }

                    // T~ must be located between a feasible stationary and neighbouring feasible one
                    candidates = feasibleStationaries;
                    if (infeasibleStationaries.size() != 0) {
                        double lbound = *(--infeasibleStationaries.end());
                        double rbound;
                        if (feasibleStationaries.size() == 0) {
                            rbound = initialDurations[i];
                        } else {
                            rbound = *(feasibleStationaries.begin());
                        }

                        int maxIts = std::max(-(int) log2(epsilon), 0) + 1;
                        double mid;
                        piece = Piece(boundConds[i], rbound);
                        if (piece.checkMaxAccRate(maxAccRate) &&
                            piece.checkMaxVelRate(maxVelRate)) {
                            for (int j = 0; j < maxIts; j++) {
                                mid = (lbound + rbound) / 2.0;
                                piece = Piece(boundConds[i], mid);
                                if (piece.checkMaxAccRate(maxAccRate) &&
                                    piece.checkMaxVelRate(maxVelRate)) {
                                    rbound = mid;
                                } else {
                                    lbound = mid;
                                }
                            }
                            // T~ is also candidates when constraints exist
                            candidates.insert(rbound);
                        }
                    }
                } else {
                    // When constraints do not exist, only stationaris are candidates
                    candidates = stationaries;
                }
                // We have to compare all candidates, even when constraints do not exist
                // Because the rational function Ji(T) can have peaks and valleys,
                // especially when initial guess is bad or duration weight is relative low
                candidates.insert(initialDurations[i]);
                double curBestCost = INFINITY;
                double tempCost;
                double curBestDuration = initialDurations[i];
                for (auto it = candidates.begin(); it != candidates.end(); it++) {
//                    if(i == 1){
//                        static int cnt = 0;
//                        cout<<"cost"<<cnt++<<" = ["<<endl;
//                        for(double cur_t = 0.3; cur_t < 1.5;cur_t+=0.01){
//                            cout<<evaluateObjective(boundConds[i], cur_t)<<","<<cur_t<<","<<boundConds[i].col(1).y() <<";";
//
//                        }cout<<"];"<<endl;
//                    }

                    tempCost = evaluateObjective(boundConds[i], *it);
//                    double costInit = evaluateObjective(boundConds[i], initialDurations[i]);
//                    printf("==================SEG %d =================\n", i);
//                    printf("have %d solutions\n", stationaries.size());
//                    printf("time: %lf with cost: %lf\n", *it, tempCost);
//                    printf("init time: %lf with cost: %lf\n", initialDurations[i], costInit);
                    if (tempCost < curBestCost) {
                        curBestCost = tempCost;
                        curBestDuration = *it;
                    }
                }

                // Construct a new piece with the best duration
                traj.emplace_back(boundConds[i], curBestDuration);
            }
            return;
        }

        // Recursively optimized the initial feasible trajectory
        // Constraints are considered
        Trajectory recursiveOptimize(Trajectory traj)  {
            if (traj.getPieceNum() > 0) {
                bool inTol;
                int idxPieceStuck = -1;
                Trajectory lastTraj = traj;
                vector<double > durations = traj.getDurations();
//                cout << "Duration: ";
//                for (size_t k = 0; k < durations.size(); k++) {
//                    cout << durations[k] << " ";
//                }
//                cout << endl;

                for (int i = 0; i < maxIterations; i++) {
                    it_time ++;
                    // Constrained alternating minimization between durations and coeffMats
                    optimizeCoeffsConstrained(traj, idxPieceStuck);


                    optimizeDurations(traj, true);

                    durations = traj.getDurations();
//                    cout << "Duration: ";
//                    for (size_t k = 0; k < durations.size(); k++) {
//                        cout << durations[k] << " ";
//                    }
//                    cout << endl;
                    // Check if tol fulfilled
                    inTol = true;
                    double diffDuration;
                    for (int j = 0; j < traj.getPieceNum(); j++) {
                        diffDuration = fabs(traj[j].getDuration() - lastTraj[j].getDuration());
                        if (diffDuration > lastTraj[j].getDuration() * epsilon ) {
                            inTol = false;
                            break;
                        }
                        if (inTol) {
                            break;
                        }
                    }


                    lastTraj = traj;
                }

                // Although objectives are much the same, we find that the minimum
                // in "Coeffs Direction" is smoother than the one in "Durations Direction"
                // in most cases. Therefore we update Coeffs one last time.
                optimizeCoeffsConstrained(traj, idxPieceStuck);

                // When there is piece stuck, call this func on sub-trajectories
                if (idxPieceStuck != -1) {
                    Trajectory subTraj, tempTraj;
                    Eigen::Vector3d tempVel, tempAcc;

                    if (idxPieceStuck != 0) {
                        for (int i = 0; i < idxPieceStuck; i++) {
                            tempTraj.emplace_back(traj[i]);
                        }
                        tempTraj = recursiveOptimize(tempTraj);
                    }

                    tempTraj.emplace_back(traj[idxPieceStuck]);

                    if (idxPieceStuck != traj.getPieceNum() - 1) {
                        for (int i = idxPieceStuck + 1; i < traj.getPieceNum(); i++) {
                            subTraj.emplace_back(traj[i]);
                        }
                        subTraj = recursiveOptimize(subTraj);
                    }
                    tempTraj.append(subTraj);

                    return tempTraj;
                } else {
                    return traj;
                }
            } else {
                return traj;
            }
        }

        bool is_init = false;

    private:
        bool use_acc_ = false, use_jerk_ = false, use_snap_ = false;
        double cva0, cva1_13, cva1_2, cva2, cva3_13, cva3_2,
                caa0, caa1, caa2_13, caa2_2, caa3_13, caa3_2,
                cja0, cja1_13, cja1_2, cja2_13, cja2_2, cja3_13, cja3_2;
        Eigen::Matrix<double, 4, 3> CVA;
        Eigen::Matrix<double, 4, 3> CAA;
        Eigen::Matrix<double, 4, 3> CJA;
        Eigen::Matrix<double, 4, 3> CVJ;
        Eigen::Matrix<double, 4, 3> CAJ;
        Eigen::Matrix<double, 4, 3> CJJ;
        Eigen::Matrix<double, 4, 3> CVS;
        Eigen::Matrix<double, 4, 3> CAS;
        Eigen::Matrix<double, 4, 3> CJS;

        Eigen::Matrix<double, 8, 8> CM;
        size_t it_time = 0;
    public:
        // Compulsory constructor from all necessary parameters
        AmTraj() {}

        AmTraj(double wT, double wA, double wJ,
               double mVr, double mAr, int mIts, double eps) {
            wTime = (wT);
            wAcc = (wA);
            wJerk = (wJ);
            maxVelRate = (mVr);
            maxAccRate = (mAr);
            maxIterations = (mIts);
            epsilon = (eps);
            is_init = true;
        }

        /*
            wT: Weight for the time regularization
            wA: Weight for the integrated squared norm of acceleration
            wJ: Weight for the integrated squared norm of jerk
            mVr: Maximum velocity rate
            mAr: Maximum acceleration rate
            mIts: Maximum number of iterations in optimization
            eps: Relative tolerance
         */
        void init(double wT, double wA, double wJ, double wS,
                  double mVr, double mAr, int mIts, double eps) {

            wTime = (wT);
            wAcc = (wA);
            wJerk = (wJ);
            wSnap = (wS);
            maxVelRate = (mVr);
            maxAccRate = (mAr);
            maxIterations = (mIts);
            epsilon = (eps);
            printf("Optimizer 7 init success: wTime=%lf, wAcc=%lf, wJerk=%lf, wSnap=%lf.\n", wTime,wAcc,wJerk,wSnap);
            CM << 20.0, -70.0, 84.0, -35.0, 0.0, 0.0, 0.0, 1.0,
                    10.0, -36.0, 45.0, -20.0, 0.0, 0.0, 1.0, 0.0,
                    2.0, -15.0 / 2.0, 10.0, -5.0, 0.0, 0.5, 0.0, 0.0,
                    1.0 / 6.0, -2.0 / 3.0, 1.0, -2.0 / 3.0, 1.0 / 6.0, 0.0, 0.0, 0.0,

                    -20.0, 70.0, -84.0, 35.0, 0.0, 0.0, 0.0, 0.0,
                    10.0, -34.0, 39.0, -15.0, 0.0, 0.0, 0.0, 0.0,
                    -2.0, 13.0 / 2.0, -7.0, 5.0 / 2.0, 0.0, 0.0, 0.0, 0.0,
                    1.0 / 6.0, -1.0 / 2.0, 1.0 / 2.0, -1.0 / 6.0, 0.0, 0.0, 0.0, 0.0;


            if (wAcc > 1e-3) {
                CVA << 140.0 / 11.0, 140.0 / 11.0, 140.0 / 11.0,
                        380.0 / 77.0, 600.0 / 77.0, 380.0 / 77.0,
                        181.0 / 462.0, 0, 181.0 / 462.0,
                        5.0 / 462.0, 8.0 / 231.0, 5.0 / 462.0;

                CAA << 40.0 / 33.0, 40.0 / 33.0, 40.0 / 33.0,
                        181.0/462.0, 0, 181.0/462.0,
                        1.0 / 462.0, 50.0 / 231.0, 1.0 / 462.0,
                        5.0 / 2772.0, 1.0 / 99.0, 5.0 / 2772.0;

                CJA << 1.0 / 22.0, 0, 1.0 / 22.0,
                        5.0 / 462.0, 8.0 / 231.0, 5.0 / 462.0,
                        5.0 / 2772.0, 1.0 / 99.0, 5.0 / 2772.0,
                        1.0 / 4620.0, 2.0 / 3465.0, 1.0 / 4620.0;

                CVA = CVA * (wAcc);
                CAA = CAA * (wAcc);
                CJA = CJA * (wAcc);
                use_acc_ = true;
            }
            if (wJerk > 1e-3) {
                CVJ << 560.0, 560.0, 560.0,
                        1840.0 / 7.0, 2080.0 / 7.0, 1840.0 / 7.0,
                        290.0 / 7.0, 410.0 / 7.0, 290.0 / 7.0,
                        26.0 / 21.0, 44.0 / 21.0, 26.0 / 21.0;

                CAJ << 100.0, 100.0, 100.0,
                        290.0 / 7.0, 410.0 / 7.0, 290.0 / 7.0,
                        40.0 / 7.0, 100.0 / 7.0, 40.0 / 7.0,
                        5.0 / 42.0, 0.0, 5.0 / 42.0;

                CJJ << 10.0 / 3.0, 10.0 / 3.0, 10.0 / 3.0,
                        26.0 / 21.0, 44.0 / 21.0, 26.0 / 21.0,
                        5.0 / 42.0, 0.0, 5.0 / 42.0,
                        1.0 / 126.0, 4.0 / 63.0, 1.0 / 126.0;

                CVJ = CVJ * (wJerk);
                CAJ = CAJ * (wJerk);
                CJJ = CJJ * (wJerk);

                use_jerk_ = true;
            }
            if (wSnap > 1e-3) {
                CVS << 50400.0,     50400.0, 50400.0,
                        24480.0,    25920.0, 24480.0,
                        4680.0,     5400.0, 4680.0,
                        360.0,      480.0, 360.0;

                CAS << 10080.0, 10080.0, 10080.0,
                        4680.0, 5400.0, 4680.0,
                        840.0,  1200.0, 840.0,
                        60.0,   120.0, 60.0;

                CJS <<  840.0,   840.0, 840.0,
                        360.0,   480.0, 360.0,
                        60.0,    120.0, 60.0,
                        4.0,     16.0, 4.0;
                CVS = CVS * (wSnap);
                CAS = CAS * (wSnap);
                CJS = CJS * (wSnap);

                use_snap_ = true;
            }
            is_init = true;
        }

        typedef std::shared_ptr<AmTraj> Ptr;

        Trajectory closed_form_min_snap(vector<Eigen::Vector3d> way_pts) {
            std::vector<double> durations = allocateTime(way_pts, 1.0);
            Vec3 initV, initA, initJ, finA, finJ, finV;
            initV.setZero();initA.setZero();initJ.setZero();
            finV.setZero();finA.setZero();finJ.setZero();
            Trajectory traj;
            std::vector<DynamicMat> coeffMats;
            coeffMats = optimizeCoeffs(way_pts, durations,
                                       initV, initA, initJ,
                                       finV, finA, finJ);
            traj = Trajectory(durations, coeffMats);
            return traj;
        }

        Trajectory genOptimalTrajDT(vector<Eigen::Vector3d> way_pts,
                                  Vec3 iniVel, Vec3 iniAcc, Vec3 iniJerk,
                                  Vec3 finVel, Vec3 finAcc, Vec3 finJerk) {

            std::vector<double> durations = allocateTime(way_pts, 1.0);
            std::vector<DynamicMat> coeffMats;
            Trajectory traj;
            bool inTol;
            std::vector<double> lastDurations = durations;
            for (int i = 0; i < maxIterations; i++) {
                // Unconstrained alternating minimization between durations and coeffMats
                coeffMats = optimizeCoeffs(way_pts, durations,
                                           iniVel, iniAcc, iniJerk,
                                           finVel, finAcc, finJerk);
                traj = Trajectory(durations, coeffMats);
                BoundaryCond boundConds = traj[1].getBoundCond();
                std::vector<double> initialDurations;
                optimizeDurations(traj, false);
                durations = traj.getDurations();
                // Check if tol fulfilled
                inTol = true;
                double diffDuration;
                for (int j = 0; j < traj.getPieceNum(); j++) {
                    diffDuration = fabs(durations[j] - lastDurations[j]);
                    // Rel tol for each piece is used here
                    if (lastDurations[j] * epsilon < diffDuration) {
                        inTol = false;
                        break;
                    }
                }
                if (inTol) {
                    break;
                }

                lastDurations = durations;
            }
            // Although the unconstrained minimum can be reached in both diretions,
            // we find that the minimum in "Coeffs Direction" is smoother than the
            // minimum in "Durations Direction" in most cases. Therefore, we choose
            // the smoother one in the given relative tolerance.
            coeffMats = optimizeCoeffs(way_pts, durations,
                                       iniVel, iniAcc, iniJerk,
                                       finVel, finAcc, finJerk);
            traj = Trajectory(durations, coeffMats);
            return traj;

        };

        Trajectory genOptimalTrajDT(vector<Eigen::Vector3d> way_pts,vector<double> durations,
                                  Vec3 iniVel, Vec3 iniAcc, Vec3 iniJerk,
                                  Vec3 finVel, Vec3 finAcc, Vec3 finJerk) {
            std::vector<DynamicMat> coeffMats;
            Trajectory traj;
            bool inTol;
            std::vector<double> lastDurations = durations;
            for (int i = 0; i < maxIterations; i++) {
                // Unconstrained alternating minimization between durations and coeffMats
                coeffMats = optimizeCoeffs(way_pts, durations,
                                           iniVel, iniAcc, iniJerk,
                                           finVel, finAcc, finJerk);
                traj = Trajectory(durations, coeffMats);
                BoundaryCond boundConds = traj[1].getBoundCond();
                std::vector<double> initialDurations;
//                cout<<evaluateObjective(boundConds, durations[1])<<","<< durations[1]<<","<<boundConds.col(1).y() <<";"<<endl;
                optimizeDurations(traj, false);
                durations = traj.getDurations();
                // Check if tol fulfilled
                inTol = true;
                double diffDuration;
                for (int j = 0; j < traj.getPieceNum(); j++) {
                    diffDuration = fabs(durations[j] - lastDurations[j]);
                    // Rel tol for each piece is used here
                    if (lastDurations[j] * epsilon < diffDuration) {
                        inTol = false;
                        break;
                    }
                }
                if (inTol) {
                    break;
                }

                lastDurations = durations;
            }
            // Although the unconstrained minimum can be reached in both diretions,
            // we find that the minimum in "Coeffs Direction" is smoother than the
            // minimum in "Durations Direction" in most cases. Therefore, we choose
            // the smoother one in the given relative tolerance.
            coeffMats = optimizeCoeffs(way_pts, durations,
                                       iniVel, iniAcc, iniJerk,
                                       finVel, finAcc, finJerk);
            traj = Trajectory(durations, coeffMats);
            return traj;
        };

        // Generate trajectory with optimal coefficients
        // Durations are allocated heuristically and scaled to satisfy constraints
        // Only applies to rest-to-rest trajectories
        Trajectory genOptimalTrajDC(vector<Eigen::Vector3d> way_pts,
                                    Vec3 iniVel, Vec3 iniAcc, Vec3 iniJerk,
                                    Vec3 finVel, Vec3 finAcc, Vec3 finJerk) const {
            std::vector<double> durations = allocateTime(way_pts, 1.0);
            std::vector<DynamicMat> coeffMats = optimizeCoeffs(way_pts, durations,
                                                                   iniVel, iniAcc, iniJerk,
                                                                   finVel, finAcc, finJerk);
            Trajectory traj(durations, coeffMats);

            // Find the scaling ration such that some constraints are tight
            double ratio = std::max(traj.getMaxVelRate() / maxVelRate / (1.0 - epsilon * epsilon),
                                    sqrt(traj.getMaxAccRate() / maxAccRate / (1.0 - epsilon * epsilon)));

            // Scale the trajectory
            traj.scaleTime(1 / ratio);

            return traj;
        }


        // Generate trajectory with optimal coefficients and best durations
        // Constraints are satisfied all the time
        Trajectory genOptimalTrajDTC(vector<Eigen::Vector3d> way_pts,
                                   Vec3 iniVel, Vec3 iniAcc, Vec3 iniJerk,
                                   Vec3 finVel, Vec3 finAcc, Vec3 finJerk)  {
            /*  To make sure the init state feasiable */
            enforceBoundFeasibility(iniVel, iniAcc, finVel, finAcc);
            /*  Conservative inite time allocation */
            std::vector<double> durations = allocateTime(way_pts, 3.0);
            /*  Do unconstrained optimization to find the global minimum*/
            std::vector<DynamicMat> coeffMats;

            coeffMats = optimizeCoeffs(way_pts, durations,
                                       iniVel, iniAcc, iniJerk,
                                       finVel, finAcc, finJerk);


            /* Init trajectory */
            Trajectory traj(durations, coeffMats);

            if (enforceIniTrajFeasibility(traj, maxIterations)) {
                traj = recursiveOptimize(traj);

            }
            vector<double> duration = traj.getDurations();
            cout<<"Durations: ";
            for(size_t i = 0 ; i < durations.size() ; i++){
                cout<<durations[i]<<" ";
            }cout<<endl;
            return traj;
        }

        Trajectory genOptimalTrajDTC(Trajectory traj_in)  {
            Vec3 iniVel, iniAcc, iniJerk, finVel, finAcc, finJerk;
            iniVel =traj_in.getVel(0);iniAcc =traj_in.getAcc(0);iniJerk =traj_in.getJerk(0);
            finVel =traj_in.getVel(-1);finAcc =traj_in.getAcc(-1);finJerk =traj_in.getJerk(-1);
            vector<Vec3> way_pts = traj_in.getWaypoints();
            /*  To make sure the init state feasiable */
            enforceBoundFeasibility(iniVel, iniAcc, finVel, finAcc);
            /*  Conservative inite time allocation */
            std::vector<double> durations = allocateTime(way_pts, 3.0);
            /*  Do unconstrained optimization to find the global minimum*/
            std::vector<DynamicMat> coeffMats;

            coeffMats = optimizeCoeffs(way_pts, durations,
                                       iniVel, iniAcc, iniJerk,
                                       finVel, finAcc, finJerk);


            /* Init trajectory */
            Trajectory traj(durations, coeffMats);
            vector<double> duration = traj.getDurations();
            if (enforceIniTrajFeasibility(traj, maxIterations)) {
                traj = recursiveOptimize(traj);
                return traj;
            }
        }

        Trajectory genOptimalTrajDTCFromOrder5(Trajectory traj_in,Vec3 iniVel, Vec3 iniAcc, Vec3 iniJerk,
                                               Vec3 finVel, Vec3 finAcc, Vec3 finJerk){

            std::vector<double> durations = traj_in.getDurations();
            std::vector<Vec3> way_pts = traj_in.getWaypoints(true);
            /*  Do unconstrained optimization to find the global minimum*/
            std::vector<DynamicMat> coeffMats;
            coeffMats = optimizeCoeffs(way_pts, durations,
                                       iniVel, iniAcc, iniJerk,
                                       finVel, finAcc, finJerk);
            Trajectory traj(durations,coeffMats);
            return traj;
        }
    };

}

#endif
