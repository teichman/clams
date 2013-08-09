#include <optimization/optimization.h>

using namespace Eigen;
using namespace std;

BisectionSolver::BisectionSolver(ScalarFunction* objective, VectorFunction* gradient,
                                 double tol, double min, double max,
                                 int max_num_iters, bool debug) :
  objective_(objective),
  gradient_(gradient),
  tol_(tol),
  min_(min),
  max_(max),
  max_num_iters_(max_num_iters),
  debug_(debug)
{
  assert(min_ < max_);
}

double BisectionSolver::solve()
{
  if(debug_) { 
    VectorXd tmp(1);
    tmp(0) = min_;
    assert(gradient_->eval(tmp).rows() == 1);
    double grad_min = gradient_->eval(tmp)(0);
    cout << "Left boundary: " << min_ << endl;
    cout << "Gradient at left boundary: " << grad_min << endl;
    tmp(0) = max_;
    double grad_max = gradient_->eval(tmp)(0);
    cout << "Right boundary: " << max_ << endl;
    cout << "Gradient at right boundary: " << grad_max << endl;
    assert(grad_min < 0);
    assert(grad_max > 0);
  }

  double left = min_;
  double right = max_;
  VectorXd x(1);
  x(0) = left + (right - left) / 2.0;
  double grad = gradient_->eval(x)(0);
  int step = 0;
  while(fabs(grad) > tol_) {
    if(grad > 0)
      right = x(0);
    else
      left = x(0);

    x(0) = left + (right - left) / 2.0;
    grad = gradient_->eval(x)(0);

    if(debug_) {
      double obj = objective_->eval(x);
      cout << "Step " << step << ", left = " << left << ", right = "
           << right << ", x = " << x.transpose() << ", gradient = " << grad
           << ", objective = " << obj
           << ", tol = " << tol_ << endl;
    }
    ++step;
  }

  return x(0);
}

NesterovGradientSolver::NesterovGradientSolver(ScalarFunction* objective,
                                               VectorFunction* gradient,
                                               double tol,
                                               double alpha,
                                               double beta,
                                               int max_num_iters,
                                               double initial_stepsize,
                                               bool debug) :
  objective_(objective),
  gradient_(gradient),
  hessian_(NULL),
  tol_(tol),
  alpha_(alpha),
  beta_(beta),
  max_num_iters_(max_num_iters),
  initial_stepsize_(initial_stepsize),
  debug_(debug)
{
}

double NesterovGradientSolver::backtracking(double t, const VectorXd& x, const VectorXd& grad, const VectorXd& direction, double objective, int* num_backtracks) {
  assert(beta_ < 1 && beta_ > 0);
  assert(alpha_ > 0 && alpha_ <= 0.5);

  if(num_backtracks)
    *num_backtracks = 0;
  
  while(objective_->eval(x + t*direction) > objective + alpha_ * t * grad.dot(direction)) {
    if(debug_ > 1)
      cout << "Backtracking: " << objective_->eval(x + t*direction) << " > " << objective + alpha_ * t * grad.dot(direction) << endl;
    t *= beta_;

    if(num_backtracks)
      *num_backtracks += 1;
  }

  return t;
}



// double NesterovGradientSolver::backtracking(double t, const VectorXd& x, const VectorXd& grad, double objective) {
//   assert(beta_ < 1 && beta_ > 0);
//   assert(alpha_ > 0 && alpha_ <= 0.5);
  
//   while(objective_->eval(x - t*grad) >= objective - alpha_ * t * grad.dot(grad))
//     t *= beta_;

//   return t;
// }

VectorXd NesterovGradientSolver::solve(const VectorXd& init) {
  VectorXd x = init;
  VectorXd y = init;
  VectorXd gradient_x, gradient_y;
  VectorXd x_prev;
  double min_objective = FLT_MAX;
  VectorXd best_x;
  double k = 1;
  int num_backtracks = 0;
  double mult = 0.5;
  
  gradient_x = gradient_->eval(x);
  if(debug_) {
    cout << "Step 0, gradient norm " << gradient_x.norm() << ", objective " << objective_->eval(x) << endl;
  }

  while(true) { 
    x_prev = x;

    // -- If we're given a hessian, check the condition number.
    if(hessian_) { 
      MatrixXd hess = hessian_->eval(x);
      double nonzeros = 0;
      for(int i = 0; i < hess.rows(); ++i)
        for(int j = 0; j < hess.cols(); ++j)
          if(fabs(hess(i, j)) > 1e-12)
            ++nonzeros;
      cout << "Sparsity of the Hessian: " << nonzeros / (double)(hess.rows() * hess.cols()) << endl;
         
      Eigen::JacobiSVD<MatrixXd> svd(hess);
      VectorXd vals = svd.singularValues();
      cout << "Singular values: " << endl << vals.transpose() << endl;
      cout << "Condition number: " << vals.maxCoeff() / vals.minCoeff() << endl;
    }

    double objective_y = objective_->eval(y);
    if(objective_y < min_objective) {
      min_objective = objective_y;
      best_x = y;
    }

    // -- Compute the gradient and step using backtracking.
    gradient_y = gradient_->eval(y);
    if(num_backtracks == 0)
      mult *= 2.0;
    else if(num_backtracks > 2)
      mult *= 2.0 / 3.0;
    double stepsize = backtracking(mult * initial_stepsize_, y, gradient_y, -gradient_y, objective_y, &num_backtracks);
    x = y - stepsize * gradient_y;
    y = x + (k / (k + 3.0)) * (x - x_prev);

    // -- Check to see how good this location is.
    double objective_x = objective_->eval(x);
    if(objective_x < min_objective) {
      min_objective = objective_x;
      best_x = x;
    }

    gradient_x = gradient_->eval(x);
    double norm = gradient_x.norm();
    assert(!isnan(norm));
    assert(!isinf(norm));

    if(debug_) {
      cout << "Nesterov Step " << k << ", gradient norm " << norm
           << ", objective " << setprecision(12) << objective_x
           << ", (x - x_prev).norm() " << setprecision(6) << (x - x_prev).norm()
           << ", " << num_backtracks << " backtracks." << endl;
    }

    // -- Stop if we're done.
    if(norm < tol_) {
      cout << "Breaking due to tolerance." << endl;
      break;
    }
     else if(max_num_iters_ > 0 && k == max_num_iters_) {
      cout << "Breaking because num_iters " << k << " = max_num_iters_ = " << max_num_iters_ << endl;
      break;
    }
    ++k;
  }

  if(debug_) {
    cout << "Solver complete, gradient norm " << gradient_x.norm() << " < tolerance " << tol_ << ", objective " << objective_->eval(x) << endl;
    cout << "Best x found has gradient norm " << setprecision(12) << gradient_->eval(best_x).norm() << ", objective " << objective_->eval(best_x) << endl;
  }

  return best_x;  
}


/****************************************
 * Newton Solver
 ****************************************/

NewtonSolver::NewtonSolver(ScalarFunction* objective,
                           VectorFunction* gradient,
                           MatrixFunction* hessian,
                           double tol,
                           double alpha,
                           double beta,
                           double stepsize,
                           int debug,
                           int max_iters) :
  objective_(objective),
  gradient_(gradient),
  hessian_(hessian),
  tol_(tol),
  alpha_(alpha),
  beta_(beta),
  stepsize_(stepsize),
  debug_(debug),
  max_iters_(max_iters)
{
}


string NewtonSolver::progressString(int k, const VectorXd& grad, const VectorXd& x, double lambda2, double objective, int num_backtracks) { 
  ostringstream oss;
  oss << "Newton Step " << k
      << ", gradient norm = " << setprecision(12) << grad.norm()
      << ", 1/2 lambda^2 = " << lambda2 / 2.0
      << ", tol = " << tol_
      << ", objective = " << objective
      << ", num_backtracks = " << num_backtracks;
    //<< ", x = " << x.transpose();
  return oss.str();
}

double NewtonSolver::backtracking(double t, const VectorXd& x, const VectorXd& grad, const VectorXd& direction, double objective, int* num_backtracks) {
  assert(beta_ < 1 && beta_ > 0);
  assert(alpha_ > 0 && alpha_ <= 0.5);

  if(num_backtracks)
    *num_backtracks = 0;
  
  while(objective_->eval(x + t*direction) > objective + alpha_ * t * grad.dot(direction)) {
    if(debug_ > 1)
      cout << "Backtracking: " << objective_->eval(x - t*grad) << " >= " << objective + alpha_ * t * grad.dot(direction) << endl;
    t *= beta_;

    if(num_backtracks)
      *num_backtracks += 1;
  }

  return t;
}

VectorXd NewtonSolver::solve(VectorXd x) {

  int k = 0;
  VectorXd grad = VectorXd::Zero(x.rows());
  MatrixXd hess = MatrixXd::Zero(x.rows(), x.rows());  
  VectorXd direction = VectorXd::Zero(x.rows());
  double obj = FLT_MAX;
  double lambda2 = FLT_MAX;
  double stepsize = 0;
  int num_backtracks = 0;
  double mult = 1.0;

  cout << "Initializing with: " << endl;
  cout << x.transpose() << endl;
  
  while(true) {
    grad = gradient_->eval(x);
    hess = hessian_->eval(x);
    Eigen::JacobiSVD<MatrixXd> svd(hess);
    VectorXd vals = svd.singularValues();
    cout << "Singular values: " << endl << vals.transpose() << endl;
    cout << "Condition number: " << vals.maxCoeff() / vals.minCoeff() << endl;
    direction = hess.ldlt().solve(-grad);

    double obj_prev = obj;
    obj = objective_->eval(x);
    lambda2 = grad.dot(-direction);

    if(debug_)
      cout << progressString(k, grad, x, lambda2, obj, num_backtracks) << endl;

    assert(!isinf(lambda2));
    assert(obj <= obj_prev);
    if(lambda2 / 2.0 < tol_)
      break;

    if(max_iters_ > 0 && k == max_iters_)
      break;
    
    if(num_backtracks == 0)
      mult *= 2;
    stepsize = backtracking(mult*stepsize_, x, grad, direction, obj, &num_backtracks);
    x = x + stepsize * direction;
    ++k;
  }

  if(debug_) {
    cout << "Newton solver complete." << endl;
  }

  return x;
}


/****************************************
 * Standard Gradient Solver
 ****************************************/


GradientSolver::GradientSolver(ScalarFunction* objective,
                               VectorFunction* gradient,
                               double tol,
                               double alpha,
                               double beta,
                               int max_num_iters,
                               double initial_stepsize,
                               bool debug) :
  objective_(objective),
  gradient_(gradient),
  tol_(tol),
  alpha_(alpha),
  beta_(beta),
  max_num_iters_(max_num_iters),
  initial_stepsize_(initial_stepsize),
  debug_(debug)
{  
}

double GradientSolver::backtracking(double t,
                                    const VectorXd& x,
                                    const VectorXd& grad,
                                    const VectorXd& direction,
                                    double objective,
                                    int* num_backtracks)
{
  assert(beta_ < 1 && beta_ > 0);
  assert(alpha_ > 0 && alpha_ <= 0.5);

  if(num_backtracks)
    *num_backtracks = 0;
  
  while(objective_->eval(x + t*direction) > objective + alpha_ * t * grad.dot(direction)) {
    if(debug_ > 1)
      cout << "Backtracking: " << objective_->eval(x + t*direction) << " > " << objective + alpha_ * t * grad.dot(direction) << endl;
    t *= beta_;

    if(num_backtracks)
      *num_backtracks += 1;
  }

  return t;
}

VectorXd GradientSolver::solve(const VectorXd& init) {
  VectorXd x = init;
  VectorXd y = init;
  VectorXd gradient;
  VectorXd x_prev;
  double k = 1;
  int num_backtracks = 0;
  double mult = 0.5;
  double objective = FLT_MAX;
  double prev_objective = FLT_MAX;
  
  gradient = gradient_->eval(x);
  if(debug_) {
    cout << "Gradient Step 0, gradient norm " << gradient.norm() << ", objective " << objective_->eval(x) << endl;
  }

  while(true) { 
    x_prev = x;
    prev_objective = objective;

    objective = objective_->eval(x);
    gradient = gradient_->eval(x);

    assert(objective <= prev_objective);

    if(gradient.norm() <= tol_) { 
      cout << "Breaking due to tolerance." << endl;
      break;
    }
     else if(max_num_iters_ > 0 && k == max_num_iters_) {
      cout << "Breaking because num_iters " << k << " = max_num_iters_ = " << max_num_iters_ << endl;
      break;
    }
    
    if(num_backtracks == 0)
      mult *= 2;
    double stepsize = backtracking(mult * initial_stepsize_,
                                   x,
                                   gradient,
                                   -gradient,
                                   objective,
                                   &num_backtracks);

    x += -stepsize * gradient;

    assert(!isnan(gradient.norm()));
    assert(!isinf(gradient.norm()));
    assert(!isnan(x.norm()));
    assert(!isinf(x.norm()));
    
    if(debug_) {
      //cout << "Step " << k << ", gradient norm " << norm << ", objective " << objective_x << endl;
      cout << "Gradient Step " << k << ", gradient norm " << gradient.norm()
           << ", objective " << setprecision(12) << objective
           << ", (x - x_prev).norm() " << setprecision(6) << (x - x_prev).norm() << endl;
        //<< ", x = " << x.transpose() << endl;
      //      cout << "Step " << k << ", gradient norm " << norm << ", objective " << objective_x << ", (x - x_prev).norm() " << (x - x_prev).norm() << endl;
    }

    ++k;
  }

  if(debug_) {
    cout << "Solver complete, gradient norm " << gradient.norm() << " < tolerance " << tol_ << ", objective " << objective_->eval(x) << endl;
  }

  return x;  
}
