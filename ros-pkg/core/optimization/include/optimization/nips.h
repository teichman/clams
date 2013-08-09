#ifndef NIPS_H
#define NIPS_H

#include <vector>
#include <optimization/optimization.h>

class NesterovInteriorPointSolver
{
public:
  Eigen::VectorXd* optimal_solution_;
  
  NesterovInteriorPointSolver(ScalarFunction::ConstPtr objective,
                              VectorFunction::ConstPtr gradient,
                              double initial_mu,
                              double tol,
                              double alpha,
                              double beta,
                              int max_num_iters,
                              double initial_stepsize,
                              int restart,
                              int debug);
  void addConstraint(ScalarFunction::ConstPtr constraint,
                     VectorFunction::ConstPtr gradient);
  Eigen::VectorXd solve(const Eigen::VectorXd& init,
                        long int* num_steps = NULL);
  bool feasible(const Eigen::VectorXd& x);

protected:
  //! f_0(x)
  ScalarFunction::ConstPtr objective_;
  //! grad f_0(x)
  VectorFunction::ConstPtr gradient_;
  //! f_i(x) \leq 0
  std::vector<ScalarFunction::ConstPtr> constraints_;
  //! grad f_i(x)
  std::vector<VectorFunction::ConstPtr> grad_constraints_;
  double initial_mu_;
  double tol_;
  double alpha_;
  double beta_;
  int max_num_iters_;
  double initial_stepsize_;
  int restart_;
  int debug_;
  double mu_;

  double barrierObjective(const Eigen::VectorXd& x);
  Eigen::VectorXd barrierGradient(const Eigen::VectorXd& x);
  Eigen::VectorXd solveInner(const Eigen::VectorXd& init,
                             long int* num_steps);
  double backtracking(double t,
                      const Eigen::VectorXd& x,
                      const Eigen::VectorXd& grad,
                      const Eigen::VectorXd& direction,
                      double objective,
                      int* num_backtracks,
                      bool* precision_flag);
};

#endif // NIPS_H
