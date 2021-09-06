#ifndef AM_TRAJ_HPP
#define AM_TRAJ_HPP

#include "root_finder.hpp"
#include "traj_utils.hpp"
#include <vector>
#include <memory>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <iostream>
using namespace std;
namespace order5{
    // Polynomial order and trajectory dimension are fixed here
    constexpr int TrajOrder = 5;
    constexpr int TrajDim = 3;

// Type for piece boundary condition and coefficient matrix
    typedef Eigen::Matrix<double, TrajDim, TrajOrder + 1> BoundaryCond;
    typedef Eigen::Matrix<double, TrajDim, TrajOrder + 1> CoefficientMat;
    typedef Eigen::Matrix<double, TrajDim, TrajOrder> VelCoefficientMat;
    typedef Eigen::Matrix<double, TrajDim, TrajOrder - 1> AccCoefficientMat;


    // The banded system class is used for solving
// banded linear system Ax=b efficiently.
// A is an N*N band matrix with lower band width lowerBw
// and upper band width upperBw.
// Banded LU factorization has O(N) time complexity.
    class BandedSystem
    {
    public:
        // The size of A, as well as the lower/upper
        // banded width p/q are needed
        BandedSystem(const int &n, const int &p, const int &q)
                : N(n), lowerBw(p), upperBw(q),
                  ptrData(nullptr), offset(nullptr)
        {
            int rows = lowerBw + upperBw + 1;
            int actualSize = N * rows;
            ptrData = new double[actualSize];
            std::fill_n(ptrData, actualSize, 0.0);
            offset = new double *[rows];
            double *ptrRow = ptrData;
            for (int i = 0; i < rows; i++)
            {
                offset[i] = ptrRow;
                ptrRow += N;
            }
        }

        ~BandedSystem()
        {
            if (ptrData != nullptr)
            {
                delete[] ptrData;
            }
            if (offset != nullptr)
            {
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
        inline const double &operator()(const int &i, const int &j) const
        {
            return offset[i - j + upperBw][j];
        }

        inline double &operator()(const int &i, const int &j)
        {
            return offset[i - j + upperBw][j];
        }

        // This function conducts banded LU factorization in place
        // Note that the matrix "A" MUST NOT HAVE ZERO DIAGONAL ENREIES !!!
        // Normally, this can be satisfied in most cases where no
        // redundant variables are in x.
        inline void factorizeLU()
        {
            int iM, jM;
            for (int k = 0; k <= N - 2; k++)
            {
                iM = std::min(k + lowerBw, N - 1);
                for (int i = k + 1; i <= iM; i++)
                {
                    operator()(i, k) /= operator()(k, k);
                }
                jM = std::min(k + upperBw, N - 1);
                for (int j = k + 1; j <= jM; j++)
                {
                    for (int i = k + 1; i <= iM; i++)
                    {
                        operator()(i, j) -= operator()(i, k) * operator()(k, j);
                    }
                }
            }
        }

        // This function solves Ax=b, then stores x in b
        // The input b is required to be N*m, i.e.,
        // m vectors to be solved.
        inline void solve(Eigen::MatrixXd &b) const
        {
            int iM;
            for (int j = 0; j <= N - 1; j++)
            {
                iM = std::min(j + lowerBw, N - 1);
                for (int i = j + 1; i <= iM; i++)
                {
                    b.row(i) -= operator()(i, j) * b.row(j);
                }
            }
            for (int j = N - 1; j >= 0; j--)
            {
                b.row(j) /= operator()(j, j);
                iM = std::max(0, j - upperBw);
                for (int i = iM; i <= j - 1; i++)
                {
                    b.row(i) -= operator()(i, j) * b.row(j);
                }
            }
        }
    };

// The trajectory optimizer to get optimal coefficient and durations at the same time

// The trajectory optimizer to get optimal coefficient and durations at the same time
    class AmTraj
    {
    private:
        // Weights for total duration, acceleration, and jerk
        double wTime;
        double wAcc;
        double wJerk;
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
                                         double conservativeness) const
        {
            int N = (int)(wayPs.size()) - 1;
            std::vector<double> durations;

            if (N > 0)
            {
                durations.reserve(N);
                durations.clear();

                double speed = maxVelRate / conservativeness;
                double accRate = maxAccRate / conservativeness;

                Eigen::Vector3d p0, p1;
                double dtxyz, D, acct, accd, dcct, dccd, t1, t2, t3;
                for (int k = 0; k < N; k++)
                {
                    p0 = wayPs[k];
                    p1 = wayPs[k + 1];
                    D = (p1 - p0).norm();

                    acct = speed / accRate;
                    accd = (accRate * acct * acct / 2);
                    dcct = speed / accRate;
                    dccd = accRate * dcct * dcct / 2;

                    if (D < accd + dccd)
                    {
                        t1 = sqrt(accRate * D) / accRate;
                        t2 = (accRate * t1) / accRate;
                        dtxyz = t1 + t2;
                    }
                    else
                    {
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
                                                   const Eigen::Vector3d &finVel,
                                                   const Eigen::Vector3d &finAcc) const
        {
//            TimeConsuming t__("optimizeCoeffs");
            std::vector<DynamicMat> trajCoeffs;

            int N = durations.size();

            Eigen::VectorXd t1(N);
            for (int i = 0; i < N; i++)
            {
                t1(i) = durations[i];
            }
            Eigen::VectorXd t2(t1.cwiseProduct(t1)), t3(t2.cwiseProduct(t1));
            Eigen::VectorXd t4(t3.cwiseProduct(t1)), t5(t4.cwiseProduct(t1));
            std::vector<Eigen::Matrix<double, 6, 6>> Minvs(N);

            for (int i = 0; i < N; i++)
            {
                // Direct computing inverse mapping with no explicit matrix inverse
                Minvs[i] << -6.0 / t5(i), 15.0 / t4(i), -10 / t3(i), 0.0, 0.0, 1.0,
                        -3.0 / t4(i), 8.0 / t3(i), -6.0 / t2(i), 0.0, 1.0, 0.0,
                        -1.0 / 2.0 / t3(i), 3.0 / 2.0 / t2(i), -3.0 / 2.0 / t1(i), 0.5, 0.0, 0.0,
                        6.0 / t5(i), -15.0 / t4(i), 10.0 / t3(i), 0.0, 0.0, 0.0,
                        -3.0 / t4(i), 7.0 / t3(i), -4.0 / t2(i), 0.0, 0.0, 0.0,
                        1.0 / 2.0 / t3(i), -1.0 / t2(i), 1.0 / 2.0 / t1(i), 0.0, 0.0, 0.0;
            }

            Eigen::VectorXd cv00(N - 1), cv01(N - 1), cv02(N - 1);
            Eigen::VectorXd cv10(N - 1), cv11(N - 1), cv12(N - 1);
            Eigen::VectorXd cv20(N - 1), cv21(N - 1), cv22(N - 1);
            Eigen::VectorXd ca00(N - 1), ca01(N - 1), ca02(N - 1);
            Eigen::VectorXd ca10(N - 1), ca11(N - 1), ca12(N - 1);
            Eigen::VectorXd ca20(N - 1), ca21(N - 1), ca22(N - 1);

            // Computed nonzero entries in A and b for linear system Ax=b to be solved
            for (int i = 0; i < N - 1; i++)
            {
                cv00(i) = wAcc * 120.0 / 7.0 / t2(i) +
                          wJerk * 720.0 / t4(i);
                cv01(i) = wAcc * -120.0 / 7.0 * (1.0 / t2(i) - 1.0 / t2(i + 1)) +
                          wJerk * 720.0 * (1.0 / t4(i + 1) - 1.0 / t4(i));
                cv02(i) = wAcc * -120.0 / 7.0 / t2(i + 1) +
                          wJerk * -720.0 / t4(i + 1);

                cv10(i) = wAcc * 216.0 / 35.0 / t1(i) +
                          wJerk * 336.0 / t3(i);
                cv11(i) = wAcc * 384.0 / 35.0 * (1.0 / t1(i) + 1.0 / t1(i + 1)) +
                          wJerk * 384.0 * (1.0 / t3(i + 1) + 1.0 / t3(i));
                cv12(i) = wAcc * 216.0 / 35.0 / t1(i + 1) +
                          wJerk * 336.0 / t3(i + 1);

                cv20(i) = wAcc * 8.0 / 35.0 +
                          wJerk * 48.0 / t2(i);
                cv21(i) = wAcc * 0.0 +
                          wJerk * 72.0 * (1.0 / t2(i + 1) - 1.0 / t2(i));
                cv22(i) = wAcc * -8.0 / 35.0 +
                          wJerk * -48.0 / t2(i + 1);

                ca00(i) = wAcc * -6.0 / 7.0 / t1(i) +
                          wJerk * -120.0 / t3(i);
                ca01(i) = wAcc * 6.0 / 7.0 * (1.0 / t1(i) + 1.0 / t1(i + 1)) +
                          wJerk * 120.0 * (1.0 / t3(i + 1) + 1.0 / t3(i));
                ca02(i) = wAcc * -6.0 / 7.0 / t1(i + 1) +
                          wJerk * -120.0 / t3(i + 1);
                ca10(i) = wAcc * -8.0 / 35.0 +
                          wJerk * -48.0 / t2(i);
                ca11(i) = wAcc * 0.0 +
                          wJerk * 72.0 * (1.0 / t2(i + 1) - 1.0 / t2(i));
                ca12(i) = wAcc * 8.0 / 35.0 +
                          wJerk * 48.0 / t2(i + 1);
                ca20(i) = wAcc * t1(i) / 35 +
                          wJerk * -6.0 / t1(i);
                ca21(i) = wAcc * 6.0 / 35.0 * (t1(i) + t1(i + 1)) +
                          wJerk * 18.0 * (1.0 / t1(i + 1) + 1.0 / t1(i));
                ca22(i) = wAcc * t1(i + 1) / 35.0 +
                          wJerk * -6.0 / t1(i + 1);
            }

            Eigen::MatrixXd VelsAccs(3, 2 * N + 2);

            if (N == 1)
            {
                VelsAccs << iniVel, iniAcc, finVel, finAcc;
            }
            else if (N == 2)
            {
                Eigen::MatrixXd A(2, 2), b(2, 3);
                A.setZero();
                b.setZero();

                // These lines are too long... Just let it go...
                A << cv11(0), cv21(0),
                        ca11(0), ca21(0);
                b << (
                        -cv00(0) * wayPs[0] -
                        cv01(0) * wayPs[1] -
                        cv02(0) * wayPs[2] -
                        cv10(0) * iniVel -
                        cv20(0) * iniAcc -
                        cv12(0) * finVel -
                        cv22(0) * finAcc).transpose(),
                        (-ca00(0) * wayPs[0] -
                         ca01(0) * wayPs[1] -
                         ca02(0) * wayPs[2] -
                         ca10(0) * iniVel -
                         ca20(0) * iniAcc -
                         ca12(0) * finVel -
                         ca22(0) * finAcc).transpose();

                VelsAccs << iniVel, iniAcc, (A.inverse() * b).transpose(), finVel, finAcc;
            }
            else
            {
                BandedSystem A(2 * N - 2, 3, 3);
                Eigen::MatrixXd b(2 * N - 2, 3);
                b.setZero();

                A(0, 0) = cv11(0);
                A(0, 1) = cv21(0);
                A(0, 2) = cv12(0);
                A(0, 3) = cv22(0);

                A(1, 0) = ca11(0);
                A(1, 1) = ca21(0);
                A(1, 2) = ca12(0);
                A(1, 3) = ca22(0);

                A(2 * N - 4, 2 * N - 6) = cv10(N - 2);
                A(2 * N - 4, 2 * N - 5) = cv20(N - 2);
                A(2 * N - 4, 2 * N - 4) = cv11(N - 2);
                A(2 * N - 4, 2 * N - 3) = cv21(N - 2);

                A(2 * N - 3, 2 * N - 6) = ca10(N - 2);
                A(2 * N - 3, 2 * N - 5) = ca20(N - 2);
                A(2 * N - 3, 2 * N - 4) = ca11(N - 2);
                A(2 * N - 3, 2 * N - 3) = ca21(N - 2);

                b.topLeftCorner<2, 3>() << (
                        -cv00(0) * wayPs[0] -
                        cv01(0) * wayPs[1] -
                        cv02(0) * wayPs[2] -
                        cv10(0) * iniVel -
                        cv20(0) * iniAcc).transpose(),

                        (-ca00(0) * wayPs[0] -
                         ca01(0) * wayPs[1] -
                         ca02(0) * wayPs[2] -
                         ca10(0) * iniVel -
                         ca20(0) * iniAcc).transpose();

                b.bottomRightCorner<2, 3>() << (-cv00(N - 2) * wayPs[N - 2] - cv01(N - 2) * wayPs[N - 1] - cv02(N - 2) * wayPs[N] - cv12(N - 2) * finVel - cv22(N - 2) * finAcc).transpose(), (-ca00(N - 2) * wayPs[N - 2] - ca01(N - 2) * wayPs[N - 1] - ca02(N - 2) * wayPs[N] - ca12(N - 2) * finVel - ca22(N - 2) * finAcc).transpose();
                for (int i = 1; i < N - 2; i++)
                {
                    A(i * 2, i * 2 - 2) = cv10(i);
                    A(i * 2, i * 2 - 1) = cv20(i);
                    A(i * 2, i * 2) = cv11(i);
                    A(i * 2, i * 2 + 1) = cv21(i);
                    A(i * 2, i * 2 + 2) = cv12(i);
                    A(i * 2, i * 2 + 3) = cv22(i);
                    A(i * 2 + 1, i * 2 - 2) = ca10(i);
                    A(i * 2 + 1, i * 2 - 1) = ca20(i);
                    A(i * 2 + 1, i * 2) = ca11(i);
                    A(i * 2 + 1, i * 2 + 1) = ca21(i);
                    A(i * 2 + 1, i * 2 + 2) = ca12(i);
                    A(i * 2 + 1, i * 2 + 3) = ca22(i);

                    b.block<2, 3>(i * 2, 0) << (-cv00(i) * wayPs[i] - cv01(i) * wayPs[i + 1] - cv02(i) * wayPs[i + 2]).transpose(), (-ca00(i) * wayPs[i] - ca01(i) * wayPs[i + 1] - ca02(i) * wayPs[i + 2]).transpose();
                }

                // Solve Ax=b using banded LU factorization
                A.factorizeLU();
                // The solution is computed in place.
                A.solve(b);

                VelsAccs << iniVel, iniAcc, b.transpose(), finVel, finAcc;
            }

            // Recover coefficient matrices for all pieces from their boundary conditions
            Eigen::MatrixXd PosVelAccPair(3, 6);
            for (int i = 0; i < N; i++)
            {
                PosVelAccPair << wayPs[i], VelsAccs.col(2 * i), VelsAccs.col(2 * i + 1),
                        wayPs[i + 1], VelsAccs.col(2 * i + 2), VelsAccs.col(2 * i + 3);
                trajCoeffs.push_back(PosVelAccPair * Minvs[i]);
            }

            return trajCoeffs;
        }

        // Clip the norm of vec3D if it exceeds (1-eps)*maxNorm
        inline void clipNorm(Eigen::Vector3d &vec3D, double maxNorm, double eps) const
        {
            const double gamma = 1 - eps;
            double tempNorm = vec3D.norm();
            vec3D *= tempNorm > gamma * maxNorm ? gamma * maxNorm / tempNorm : 1.0;
        }

        // Make sure the head/tail conditions of the trajectory satisfy constraints
        void enforceBoundFeasibility(Eigen::Vector3d &iniVel, Eigen::Vector3d &iniAcc,
                                     Eigen::Vector3d &finVel, Eigen::Vector3d &finAcc) const
        {
            clipNorm(iniVel, maxVelRate, epsilon);
            clipNorm(iniAcc, maxAccRate, epsilon);
            clipNorm(finVel, maxVelRate, epsilon);
            clipNorm(finAcc, maxAccRate, epsilon);
        }

        // Make sure a given initial guess traj satisfies constraints
        bool enforceIniTrajFeasibility(Trajectory &traj, int tryOut) const
        {
            int N = traj.getPieceNum();
            if (N > 0)
            {
                std::vector<Eigen::Vector3d> posVec, velVec, accVec;
                std::vector<double> durVec;

                posVec.push_back(traj.getJuncPos(0));
                velVec.push_back(traj.getJuncVel(0));
                accVec.push_back(traj.getJuncAcc(0));
                clipNorm(velVec.back(), maxVelRate, epsilon);
                clipNorm(accVec.back(), maxAccRate, epsilon);

                // Clip norms of all derivatives at waypoints
                for (int i = 0; i < N; i++)
                {
                    durVec.push_back(traj[i].getDuration());
                    posVec.push_back(traj.getJuncPos(i + 1));
                    velVec.push_back(traj.getJuncVel(i + 1));
                    accVec.push_back(traj.getJuncAcc(i + 1));
                    clipNorm(velVec.back(), maxVelRate, epsilon);
                    clipNorm(accVec.back(), maxAccRate, epsilon);
                }

                // Violently scale all durations until all pieces satisfy constrains
                double adjustRatio = 2.0;
                Piece piece;
                bool feasible;
                for (int j = 0; j < tryOut; j++)
                {
                    feasible = true;
                    for (int i = 0; i < N; i++)
                    {
                        if (!traj[i].checkMaxAccRate(maxAccRate) ||
                            !traj[i].checkMaxVelRate(maxVelRate))
                        {
                            durVec[i] *= adjustRatio;
                            velVec[i] /= i == 0 ? 1.0 : adjustRatio;
                            accVec[i] /= i == 0 ? 1.0 : adjustRatio;
                            velVec[i + 1] /= i == N - 2 ? 1.0 : adjustRatio;
                            accVec[i + 1] /= i == N - 2 ? 1.0 : adjustRatio;
                            feasible = false;
                        }
                    }

                    if (feasible)
                    {
                        break;
                    }
                    else
                    {
                        traj.clear();
                    }

                    // Recover a feasibile trajectory
                    BoundaryCond boundCond;
                    for (int i = 0; i < N; i++)
                    {
                        boundCond << posVec[i], velVec[i], accVec[i],
                                posVec[i + 1], velVec[i + 1], accVec[i + 1];
                        traj.emplace_back(boundCond, durVec[i]);
                    }
                }
                if (!feasible)
                {
                    traj.clear();
                }
            }

            return traj.getPieceNum() != 0;
        }

        // Compute the objective of a single piece determined by boundCond and duration
        double evaluateObjective(const BoundaryCond &boundCond, double duration) const
        {
            Eigen::Array3d iniPos = boundCond.col(0), iniVel = boundCond.col(1), iniAcc = boundCond.col(2);
            Eigen::Array3d finPos = boundCond.col(3), finVel = boundCond.col(4), finAcc = boundCond.col(5);

            Eigen::VectorXd coeffsAccObjective(5);
            coeffsAccObjective(0) = (3.0 * iniAcc.square() + iniAcc * finAcc + 3.0 * finAcc.square()).sum();
            coeffsAccObjective(1) = (22.0 * iniAcc * iniVel - 8.0 * finAcc * iniVel + 8.0 * iniAcc * finVel - 22.0 * finAcc * finVel).sum();
            coeffsAccObjective(2) = 6.0 * (32.0 * iniVel.square() + 36.0 * iniVel * finVel + 32.0 * finVel.square() + 5.0 * (iniAcc - finAcc) * (iniPos - finPos)).sum();
            coeffsAccObjective(3) = 600.0 * ((iniVel + finVel) * (iniPos - finPos)).sum();
            coeffsAccObjective(4) = 600.0 * (iniPos - finPos).square().sum();

            Eigen::VectorXd coeffsJerObjective(5);
            coeffsJerObjective(0) = (3.0 * iniAcc.square() - 2.0 * iniAcc * finAcc + 3.0 * finAcc.square()).sum();
            coeffsJerObjective(1) = 8.0 * (3.0 * iniAcc * iniVel - 2.0 * finAcc * iniVel + 2.0 * iniAcc * finVel - 3.0 * finAcc * finVel).sum();
            coeffsJerObjective(2) = 8.0 * (8.0 * iniVel.square() + 14.0 * iniVel * finVel + 8.0 * finVel.square() + 5.0 * (iniAcc - finAcc) * (iniPos - finPos)).sum();
            coeffsJerObjective(3) = 240.0 * ((iniVel + finVel) * (iniPos - finPos)).sum();
            coeffsJerObjective(4) = 240.0 * (iniPos - finPos).square().sum();

            double t3 = duration * duration * duration;
            double t5 = t3 * duration * duration;

            return (wTime * duration +
                    wAcc * RootFinder::polyVal(coeffsAccObjective, duration) / (35.0 * t3) +
                    wJerk * RootFinder::polyVal(coeffsJerObjective, duration) * 3.0 / t5);
        }

    public:
        // Compute the objective over a whole trajectory
        double evaluateObjective(const Trajectory &traj) const
        {
            double objective = 0.0;

            int N = traj.getPieceNum();

            for (int i = 0; i < N; i++)
            {
                objective += evaluateObjective(traj[i].getBoundCond(),
                                               traj[i].getDuration());
            }

            return objective;
        }

    private:
        // Optimize the coefficient matrices of a traj, satisfying constraints all the time
        // The index of piece stuck by constraints is updated for consequent optimization
        void optimizeCoeffsConstrained(Trajectory &traj, int &idxPieceStuck) const
        {
            int N = traj.getPieceNum();

            std::vector<double> durVec;
            std::vector<Eigen::Vector3d> posVec;
            Eigen::Vector3d velIni, velFin;
            Eigen::Vector3d accIni, accFin;
            Eigen::MatrixXd vels(3, N - 1);
            Eigen::MatrixXd accs(3, N - 1);

            // Recover free boundary conditions Dpk
            durVec.push_back(traj[0].getDuration());
            posVec.push_back(traj.getJuncPos(0));
            velIni = traj.getJuncVel(0);
            accIni = traj.getJuncAcc(0);
            for (int i = 0; i < N - 1; i++)
            {
                durVec.push_back(traj[i + 1].getDuration());
                posVec.push_back(traj.getJuncPos(i + 1));
                vels.col(i) = traj.getJuncVel(i + 1);
                accs.col(i) = traj.getJuncAcc(i + 1);
            }
            posVec.push_back(traj.getJuncPos(N));
            velFin = traj.getJuncVel(N);
            accFin = traj.getJuncAcc(N);

            // Calculate optimal boundary conditions in absence of constraints
            std::vector<DynamicMat> targetCoeffsMats = optimizeCoeffs(posVec, durVec, velIni,
                                                                          accIni, velFin, accFin);

            traj = Trajectory(durVec, targetCoeffsMats);

            // Extract free part of optimal boundary conditions Dp*
            Eigen::MatrixXd velsTarget(3, N - 1);
            Eigen::MatrixXd accsTarget(3, N - 1);
            for (int i = 0; i < N - 1; i++)
            {
                velsTarget.col(i) = traj.getJuncVel(i + 1);
                accsTarget.col(i) = traj.getJuncAcc(i + 1);
            }

            // Check whether convex combination (1 - lambda)*Dpk + lambda*Dp* is feasibile
            auto temporalFeasibilityCheck = [&](double lambda) {
                bool feasible = true;
                BoundaryCond boundCond;
                Piece piece;
                for (int i = 0; i < N; i++)
                {
                    boundCond << posVec[i],
                            (i == 0) ? velIni : ((1 - lambda) * vels.col(i - 1) + lambda * velsTarget.col(i - 1)),
                            (i == 0) ? accIni : ((1 - lambda) * accs.col(i - 1) + lambda * accsTarget.col(i - 1)),
                            posVec[i + 1],
                            (i == N - 1) ? velFin : (1 - lambda) * vels.col(i) + lambda * velsTarget.col(i),
                            (i == N - 1) ? accFin : (1 - lambda) * accs.col(i) + lambda * accsTarget.col(i);
                    piece = Piece(boundCond, durVec[i]);
                    if (!piece.checkMaxAccRate(maxAccRate) ||
                        !piece.checkMaxVelRate(maxVelRate))
                    {
                        idxPieceStuck = i;
                        feasible = false;
                        break;
                    }
                }
                return feasible;
            };

            // Locate the best lambda of convex combination by bisection
            double bestLambda = 0.0;
            if (temporalFeasibilityCheck(bestLambda))
            {
                bestLambda = 1.0;
                if (!temporalFeasibilityCheck(bestLambda))
                {
                    int maxIts = std::max(-(int)log2(epsilon), 0) + 1;

                    double lbound = 0.0, rbound = 1.0, mid;

                    for (int i = 0; i < maxIts; i++)
                    {
                        mid = (lbound + rbound) / 2.0;

                        if (temporalFeasibilityCheck(mid))
                        {
                            lbound = mid;
                        }
                        else
                        {
                            rbound = mid;
                        }
                    }

                    bestLambda = lbound;
                }
            }

            traj.clear();
            BoundaryCond boundCond;
            for (int i = 0; i < N; i++)
            {
                boundCond << posVec[i],
                        (i == 0) ? velIni : ((1 - bestLambda) * vels.col(i - 1) + bestLambda * velsTarget.col(i - 1)),
                        (i == 0) ? accIni : ((1 - bestLambda) * accs.col(i - 1) + bestLambda * accsTarget.col(i - 1)),
                        posVec[i + 1],
                        (i == N - 1) ? velFin : (1 - bestLambda) * vels.col(i) + bestLambda * velsTarget.col(i),
                        (i == N - 1) ? accFin : (1 - bestLambda) * accs.col(i) + bestLambda * accsTarget.col(i);
//                cout<<boundCond.transpose()<<endl;
                traj.emplace_back(boundCond, durVec[i]);
            }

            // Update the last index piece stuck by constraints
            idxPieceStuck = bestLambda == 0.0 ? idxPieceStuck : -1;
        }

        // Optimized durations for all pieces of a trajertory, with or without constraints
        void optimizeDurations(Trajectory &traj, bool constrained = true) const
        {
//            TimeConsuming t__("optimizeDurations");
            int N = traj.getPieceNum();

            std::vector<DynamicMat> boundConds;
            std::vector<double> initialDurations;

            // Backup boundary conditions as durations
            for (int i = 0; i < N; i++)
            {
                boundConds.push_back(traj[i].getBoundCond());
                initialDurations.push_back(traj[i].getDuration());
            }

            traj.clear();

            Piece piece;
            Eigen::VectorXd coeffsGradT(7);
            double tempAccTerm, tempJerkTerm;
            Eigen::Array3d posIni, velIni, accIni, posFin, velFin, accFin;
            for (int i = 0; i < N; i++)
            {
                // Calculate the numerator of dJi(T)/dT
                posIni << boundConds[i].col(0);
                velIni << boundConds[i].col(1);
                accIni << boundConds[i].col(2);
                posFin << boundConds[i].col(3);
                velFin << boundConds[i].col(4);
                accFin << boundConds[i].col(5);

                tempAccTerm = (3.0 * accIni.square() + accIni * accFin + 3.0 * accFin.square()).sum();
                coeffsGradT(0) = wTime * 35.0 + wAcc * tempAccTerm;
                coeffsGradT(1) = 0.0;
                tempAccTerm = (32.0 * velIni.square() + 36.0 * velIni * velFin + 32.0 * velFin.square()).sum() +
                              (5.0 * (accIni - accFin) * (posIni - posFin)).sum();

                tempJerkTerm = (3.0 * accIni.square() - 2.0 * accIni * accFin + 3.0 * accFin.square()).sum();
                coeffsGradT(2) = -3.0 * (2.0 * wAcc * tempAccTerm + 35.0 * wJerk * tempJerkTerm);
                tempAccTerm = ((velIni + velFin) * (posIni - posFin)).sum();

                tempJerkTerm = ((-3.0 * accIni + 2.0 * accFin) * velIni + (-2.0 * accIni + 3.0 * accFin) * velFin).sum();
                coeffsGradT(3) = 240.0 * (-5.0 * wAcc * tempAccTerm + 7.0 * wJerk * tempJerkTerm);
                tempAccTerm = (posIni - posFin).square().sum();

                tempJerkTerm = (8.0 * velIni.square() + 14.0 * velIni * velFin + 8.0 * velFin.square()).sum() +
                               (5.0 * (accIni - accFin) * (posIni - posFin)).sum();

                coeffsGradT(4) = -360.0 * (5.0 * wAcc * tempAccTerm + 7.0 * wJerk * tempJerkTerm);
                coeffsGradT(5) = -100800.0 * wJerk * ((velIni + velFin) * (posIni - posFin)).sum();
                coeffsGradT(6) = -126000.0 * wJerk * (posIni - posFin).square().sum();

                // Compute all stationaries in which the optimal duration locates
                std::set<double> stationaries;
                stationaries = RootFinder::solvePolynomial(coeffsGradT, 0.0, INFINITY,
                                                           initialDurations[i] * epsilon * epsilon);

                std::set<double> candidates;
                if (constrained)
                {
                    // When constraints are considered, duration T~ should be found where some constraints are tight
                    std::set<double> infeasibleStationaries, feasibleStationaries;
                    for (auto it = stationaries.begin(); it != stationaries.end(); it++)
                    {
                        piece = Piece(boundConds[i], *it);
                        if (piece.checkMaxAccRate(maxAccRate) &&
                            piece.checkMaxVelRate(maxVelRate))
                        {
                            feasibleStationaries.insert(*it);
                        }
                        else
                        {
                            infeasibleStationaries.insert(*it);
                        }
                    }

                    // T~ must be located between a feasible stationary and neighbouring feasible one
                    candidates = feasibleStationaries;
                    if (infeasibleStationaries.size() != 0)
                    {
                        double lbound = *(--infeasibleStationaries.end());
                        double rbound;
                        if (feasibleStationaries.size() == 0)
                        {
                            rbound = initialDurations[i];
                        }
                        else
                        {
                            rbound = *(feasibleStationaries.begin());
                        }

                        int maxIts = std::max(-(int)log2(epsilon), 0) + 1;
                        double mid;
                        piece = Piece(boundConds[i], rbound);
                        if (piece.checkMaxAccRate(maxAccRate) &&
                            piece.checkMaxVelRate(maxVelRate))
                        {
                            for (int j = 0; j < maxIts; j++)
                            {
                                mid = (lbound + rbound) / 2.0;
                                piece = Piece(boundConds[i], mid);
                                if (piece.checkMaxAccRate(maxAccRate) &&
                                    piece.checkMaxVelRate(maxVelRate))
                                {
                                    rbound = mid;
                                }
                                else
                                {
                                    lbound = mid;
                                }
                            }
                            // T~ is also candidates when constraints exist
                            candidates.insert(rbound);
                        }
                    }
                }
                else
                {
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
                for (auto it = candidates.begin(); it != candidates.end(); it++)
                {
                    tempCost = evaluateObjective(boundConds[i], *it);
                    if (tempCost < curBestCost)
                    {
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
        Trajectory recursiveOptimize(Trajectory traj) const
        {
            if (traj.getPieceNum() > 0)
            {
                bool inTol;
                int idxPieceStuck = -1;
                Trajectory lastTraj = traj;
                for (int i = 0; i < maxIterations; i++)
                {
                    // Constrained alternating minimization between durations and coeffMats
                    optimizeCoeffsConstrained(traj, idxPieceStuck);
                    optimizeDurations(traj, true);
                    vector<double> durations = traj.getDurations();
//                    cout<<"Durations: ";
//                    for(size_t i = 0 ; i < durations.size() ; i++){
//                        cout<<durations[i]<<" ";
//                    }cout<<endl;
                    // Check if tol fulfilled
                    inTol = true;
                    double diffDuration;
                    for (int j = 0; j < traj.getPieceNum(); j++)
                    {
                        diffDuration = fabs(traj[j].getDuration() - lastTraj[j].getDuration());
                        if (diffDuration > lastTraj[j].getDuration() * epsilon)
                        {
                            inTol = false;
                            break;
                        }
                    }
                    if (inTol)
                    {
                        break;
                    }

                    lastTraj = traj;
                }
                // Although objectives are much the same, we find that the minimum
                // in "Coeffs Direction" is smoother than the one in "Durations Direction"
                // in most cases. Therefore we update Coeffs one last time.
                optimizeCoeffsConstrained(traj, idxPieceStuck);

                // When there is piece stuck, call this func on sub-trajectories
                if (idxPieceStuck != -1)
                {
                    Trajectory subTraj, tempTraj;
                    Eigen::Vector3d tempVel, tempAcc;

                    if (idxPieceStuck != 0)
                    {
                        for (int i = 0; i < idxPieceStuck; i++)
                        {
                            tempTraj.emplace_back(traj[i]);
                        }
                        tempTraj = recursiveOptimize(tempTraj);
                    }

                    tempTraj.emplace_back(traj[idxPieceStuck]);

                    if (idxPieceStuck != traj.getPieceNum() - 1)
                    {
                        for (int i = idxPieceStuck + 1; i < traj.getPieceNum(); i++)
                        {
                            subTraj.emplace_back(traj[i]);
                        }
                        subTraj = recursiveOptimize(subTraj);
                    }
                    tempTraj.append(subTraj);

                    return tempTraj;
                }
                else
                {
                    return traj;
                }
            }
            else
            {
                return traj;
            }
        }

        bool is_init = false;
    public:
        // Compulsory constructor from all necessary parameters
        AmTraj(){}
        AmTraj(double wT, double wA, double wJ,
               double mVr, double mAr, int mIts, double eps){
            wTime=(wT); wAcc=(wA); wJerk=(wJ);
            maxVelRate=(mVr); maxAccRate=(mAr);
            maxIterations=(mIts); epsilon=(eps);
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
        void init(double wT, double wA, double wJ,
                  double mVr, double mAr, int mIts, double eps){
            wTime=(wT); wAcc=(wA); wJerk=(wJ);
            maxVelRate=(mVr); maxAccRate=(mAr);
            maxIterations=(mIts); epsilon=(eps);
            printf("Optimizer 5 init success: wTime=%lf, wAcc=%lf, wJerk=%lf.\n", wTime,wAcc,wJerk);
        }

        typedef std::shared_ptr<AmTraj> Ptr;
        // Generate trajectory with optimal coefficients
        // Durations are allocated heuristically and scaled to satisfy constraints
        // Only applies to rest-to-rest trajectories
        Trajectory genOptimalTrajDC(const std::vector<Eigen::Vector3d> &wayPs,
                                    Eigen::Vector3d iniVel, Eigen::Vector3d iniAcc,
                                    Eigen::Vector3d finVel, Eigen::Vector3d finAcc) const
        {
            std::vector<double> durations = allocateTime(wayPs, 1.0);
            std::vector<DynamicMat> coeffMats = optimizeCoeffs(wayPs, durations,
                                                                   iniVel, iniAcc,
                                                                   finVel, finAcc);
            Trajectory traj(durations, coeffMats);

            // Find the scaling ration such that some constraints are tight
            double ratio = std::max(traj.getMaxVelRate() / maxVelRate / (1.0 - epsilon * epsilon),
                                    sqrt(traj.getMaxAccRate() / maxAccRate / (1.0 - epsilon * epsilon)));

            // Scale the trajectory
            traj.scaleTime(1 / ratio);

            return traj;
        }

        // Generate trajectory with optimal coefficients and optimal durations
        // Constraints are not considered
        Trajectory genOptimalTrajDT(const std::vector<Eigen::Vector3d> &wayPs,
                                    Eigen::Vector3d iniVel, Eigen::Vector3d iniAcc,
                                    Eigen::Vector3d finVel, Eigen::Vector3d finAcc) const
        {
            std::vector<double> durations = allocateTime(wayPs, 1.0);
            for(size_t i = 0 ; i < durations.size() ; i++){
                cout<<durations[i]<<" ";
            }cout<<endl;
            std::vector<DynamicMat> coeffMats;
            Trajectory traj;
            bool inTol;
            std::vector<double> lastDurations = durations;
            for (int i = 0; i < maxIterations; i++)
            {
                // Unconstrained alternating minimization between durations and coeffMats
                coeffMats = optimizeCoeffs(wayPs, durations,
                                           iniVel, iniAcc,
                                           finVel, finAcc);
                traj = Trajectory(durations, coeffMats);
                optimizeDurations(traj, false);
                durations = traj.getDurations();

                // Check if tol fulfilled
                inTol = true;
                double diffDuration;
                for (int j = 0; j < traj.getPieceNum(); j++)
                {
                    diffDuration = fabs(durations[j] - lastDurations[j]);
                    // Rel tol for each piece is used here
                    if (lastDurations[j] * epsilon < diffDuration)
                    {
                        inTol = false;
                        break;
                    }
                }
                if (inTol)
                {
                    break;
                }

                lastDurations = durations;
            }
            // Although the unconstrained minimum can be reached in both diretions,
            // we find that the minimum in "Coeffs Direction" is smoother than the
            // minimum in "Durations Direction" in most cases. Therefore, we choose
            // the smoother one in the given relative tolerance.
            coeffMats = optimizeCoeffs(wayPs, durations,
                                       iniVel, iniAcc,
                                       finVel, finAcc);
            traj = Trajectory(durations, coeffMats);

            return traj;
        }

        // Generate trajectory with optimal coefficients and best durations
        // Constraints are satisfied all the time
        Trajectory genOptimalTrajDTC(const std::vector<Eigen::Vector3d> &wayPs,
                                     Eigen::Vector3d iniVel, Eigen::Vector3d iniAcc,
                                     Eigen::Vector3d finVel, Eigen::Vector3d finAcc) const
        {
            enforceBoundFeasibility(iniVel, iniAcc, finVel, finAcc);
            std::vector<double> durations = allocateTime(wayPs, 1.0);
//            cout<<"Durations: ";
//            for(size_t i = 0 ; i < durations.size() ; i++){
//                cout<<durations[i]<<" ";
//            }cout<<endl;
            std::vector<DynamicMat> coeffMats = optimizeCoeffs(wayPs, durations,
                                                                   iniVel, iniAcc,
                                                                   finVel, finAcc);
            Trajectory traj(durations, coeffMats);

            if (enforceIniTrajFeasibility(traj, maxIterations))
            {durations = traj.getDurations();
//                cout<<"Durations: ";
//                for(size_t i = 0 ; i < durations.size() ; i++){
//                    cout<<durations[i]<<" ";
//                }cout<<endl;
                traj = recursiveOptimize(traj);
            }
            durations = traj.getDurations();
//            cout<<"Durations: ";
//            for(size_t i = 0 ; i < durations.size() ; i++){
//                cout<<durations[i]<<" ";
//            }cout<<endl;
            return traj;
        }

        // Generate trajectory with optimal coefficients and best durations
        // Constraints are satisfied all the time
        Trajectory genOptimalTrajDTC(Trajectory traj_in)
        {
            Vec3 iniVel, iniAcc, iniJerk, finVel, finAcc, finJerk;
            iniVel =traj_in.getVel(0);iniAcc =traj_in.getAcc(0);
            finVel =traj_in.getVel(-1);finAcc =traj_in.getAcc(-1);
            vector<Vec3> way_pts = traj_in.getWaypoints();
            enforceBoundFeasibility(iniVel, iniAcc, finVel, finAcc);
            std::vector<double> durations = allocateTime(way_pts, 1.0);
            std::vector<DynamicMat> coeffMats = optimizeCoeffs(way_pts, durations,
                                                               iniVel, iniAcc,
                                                               finVel, finAcc);
            Trajectory traj(durations, coeffMats);

            if (enforceIniTrajFeasibility(traj, maxIterations))
            {
                traj = recursiveOptimize(traj);
            }
            durations = traj.getDurations();

            return traj;
        }

        // Generate trajectory with optimal coefficients and best durations
        // Constraints are satisfied all the time
        Trajectory genOptimalTrajDTC(const std::vector<Eigen::Vector3d> &wayPs, vector<double> durations,
                                   Eigen::Vector3d iniVel, Eigen::Vector3d iniAcc,
                                   Eigen::Vector3d finVel, Eigen::Vector3d finAcc) const
        {
            enforceBoundFeasibility(iniVel, iniAcc, finVel, finAcc);
            std::vector<DynamicMat> coeffMats = optimizeCoeffs(wayPs, durations,
                                                                   iniVel, iniAcc,
                                                                   finVel, finAcc);
            Trajectory traj(durations, coeffMats);

            if (enforceIniTrajFeasibility(traj, maxIterations))
            {
                traj = recursiveOptimize(traj);
            }
            durations = traj.getDurations();
            return traj;
        }

        // Generate trajectory with optimal coefficients and best durations
        // Constraints are satisfied all the time
        vector<double> getOptimalDurations(const std::vector<Eigen::Vector3d> &wayPs,
                                   Eigen::Vector3d iniVel, Eigen::Vector3d iniAcc,
                                   Eigen::Vector3d finVel, Eigen::Vector3d finAcc) const
        {
            enforceBoundFeasibility(iniVel, iniAcc, finVel, finAcc);
            std::vector<double> durations = allocateTime(wayPs, 1.0);
            std::vector<DynamicMat> coeffMats = optimizeCoeffs(wayPs, durations,
                                                                   iniVel, iniAcc,
                                                                   finVel, finAcc);
            Trajectory traj(durations, coeffMats);

            if (enforceIniTrajFeasibility(traj, maxIterations))
            {
                traj = recursiveOptimize(traj);
            }
            durations = traj.getDurations();
            return durations;
        }


    };
}
#endif

