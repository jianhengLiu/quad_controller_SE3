#ifndef _BVP_SOLVER_H_
#define _BVP_SOLVER_H_

#include "poly_traj_utils/traj_utils.hpp"
#include <Eigen/Eigen>

#define DOUBLE_INTEGRATOR 2
#define TRIPLE_INTEGRATOR 3
#define ACC_KNOWN 0
#define ACC_UNKNOWN 1

namespace BVPSolver
{

using Eigen::Vector3d;
using Eigen::Vector2d;
using Eigen::VectorXd;

class IntegratorBVP {
public:
  void init(int model)
  {
    model_ = model;
    if (model_ == DOUBLE_INTEGRATOR)
    {
      coeff_.resize(3,4);
      x0_ = Eigen::Matrix<double, 6, 1>::Zero();
      x1_ = Eigen::Matrix<double, 6, 1>::Zero(); 
    }
    else if (model_ == TRIPLE_INTEGRATOR) 
    {
      coeff_.resize(3,6);
      x0_ = Eigen::Matrix<double, 9, 1>::Zero();
      x1_ = Eigen::Matrix<double, 9, 1>::Zero(); 
    }
  };

  void setRho(double rho) 
  {
    rho_ = rho;
    printf("BVP solver init success, rho = %lf\n", rho);
  };

  bool solve(const VectorXd& start, const VectorXd& goal, int type = ACC_UNKNOWN)
  {
    setBoundaries(start, goal);
    if (model_ == DOUBLE_INTEGRATOR)
    {
      return solveDouble();
    }
    else if (model_ == TRIPLE_INTEGRATOR) 
    {
      if (type == ACC_UNKNOWN)
        return solveTripleAccUnknown();
      else
        return solveTriple();
    }
    else
    {
      printf("Input model is neither double integrator nor triple.");
      return false;
    }
  };

  double estimateHeuristic(const VectorXd& start, const VectorXd& goal)
  {
    setBoundaries(start, goal);
    calTauStarDouble();
    return cost_star_;
  };

  double getTauStar()
  {
    return tau_star_;
  };

  double getCostStar()
  {
    return cost_star_;
  };

  void getCoeff(DynamicMat &coeff)
  {
    coeff = coeff_;
  };


private:
  VectorXd x0_, x1_;
  double rho_;
  int model_;
  DynamicMat coeff_;
  double tau_star_, cost_star_;

  bool calTauStarDouble();
  bool solveDouble();
  bool calTauStarTriple();
  bool solveTriple();

  bool calTauStarTripleAccUnknown();
  bool solveTripleAccUnknown();

  bool calTauStarTripleVelAccUnknown();
  bool solveTripleVelAccUnknown();

  void setBoundaries(const VectorXd& start, const VectorXd& goal) 
  {
    x0_ = start;
    x1_ = goal;
  };

};

}
#endif