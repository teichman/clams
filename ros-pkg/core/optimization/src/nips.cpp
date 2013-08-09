#include <optimization/nips.h>

using namespace std;
using namespace Eigen;

NesterovInteriorPointSolver::NesterovInteriorPointSolver(ScalarFunction::ConstPtr objective,
                                                         VectorFunction::ConstPtr gradient,
                                                         double initial_mu,
                                                         double tol,
                                                         double alpha,
                                                         double beta,
                                                         int max_num_iters,
                                                         double initial_stepsize,
                                                         int restart,
                                                         int debug) :
  
  optimal_solution_(NULL),
  objective_(objective),
  gradient_(gradient),
  initial_mu_(initial_mu),
  tol_(tol),
  alpha_(alpha),
  beta_(beta),
  max_num_iters_(max_num_iters),
  initial_stepsize_(initial_stepsize),
  restart_(restart),
  debug_(debug),
  mu_(initial_mu)
{
  if(debug_)
    cout << "Initializing with debugging level " << debug_ << endl;
}

void NesterovInteriorPointSolver::addConstraint(ScalarFunction::ConstPtr constraint,
                                                VectorFunction::ConstPtr gradient)
{
  constraints_.push_back(constraint);
  grad_constraints_.push_back(gradient);
}


double NesterovInteriorPointSolver::barrierObjective(const Eigen::VectorXd& x)
{
  double obj = objective_->eval(x);
  for(size_t i = 0; i < constraints_.size(); ++i)
    obj -= mu_ * log(-constraints_[i]->eval(x));

  return obj;
}

Eigen::VectorXd NesterovInteriorPointSolver::barrierGradient(const Eigen::VectorXd& x)
{
  VectorXd grad = gradient_->eval(x);
  for(size_t i = 0; i < grad_constraints_.size(); ++i)
    grad -= mu_ * grad_constraints_[i]->eval(x) / constraints_[i]->eval(x);

  return grad;
}

double NesterovInteriorPointSolver::backtracking(double t,
                                                 const VectorXd& x,
                                                 const VectorXd& grad,
                                                 const VectorXd& direction,
                                                 double objective,
                                                 int* num_backtracks,
                                                 bool* precision_flag)
{
  assert(beta_ < 1 && beta_ > 0);
  assert(alpha_ > 0 && alpha_ <= 0.5);
  *precision_flag = false;

  if(num_backtracks)
    *num_backtracks = 0;

  while(!feasible(x + t * direction) ||
        (barrierObjective(x + t * direction) > objective + alpha_ * t * grad.dot(direction)))
  {
    if(debug_ > 2) {
      VectorXd canditate = x + t * direction;
      if(!feasible(canditate))
        cout << "  Backtracking (Infeasible): " << setprecision(16) << canditate.transpose() << endl;
      else { 
        cout << "  Backtracking (condition not met): " << setprecision(16)
             << barrierObjective(canditate) << " > "
             << objective + alpha_ * t * grad.dot(direction) << endl;
        cout << "    t = " << t << endl;
        cout << "    x = " << x.transpose() << endl;
        cout << "    direction = " << direction.transpose() << endl;
        cout << "    t * direction = " << (t * direction).transpose() << endl;
        cout << "    x + t * direction = " << canditate.transpose() << endl;        
      }
    }

    // Perturb beta.
    double beta;
    while((beta = beta_ * (0.5 + (double)rand() / (double)RAND_MAX)), beta >= 1);
    t *= beta;
    
    if(num_backtracks)
      *num_backtracks += 1;
  }

  VectorXd canditate = x + t * direction;
  for(int i = 0; i < canditate.rows(); ++i) { 
    if(direction(i) != 0 && canditate(i) == x(i)) { 
      *precision_flag = true;
      if(debug_ > 2)
        cout << "    ==========  t * direction(i) = " << t * direction(i)
             << " is 0 when added to " << x(i) << "!" << endl;
    }
  }

  return t;
}

bool NesterovInteriorPointSolver::feasible(const VectorXd& x)
{
  for(size_t i = 0; i < constraints_.size(); ++i)
    if(constraints_[i]->eval(x) > 0)
      return false;

  return true;
}

VectorXd NesterovInteriorPointSolver::solve(const VectorXd& init,
                                            long int* num_steps)
{
  assert(feasible(init));

  mu_ = initial_mu_;
  VectorXd x = init;
  VectorXd x_prev;
  long int total_steps = 0;
  while(true) {
    if(debug_)
      cout << "Solving for mu = " << mu_ << endl;
    
    x_prev = x;
    long int ns;
    x = solveInner(x, &ns);
    total_steps += ns;

    if(debug_) { 
      cout << "Solved for mu = " << mu_ << endl;
      cout << "Took " << ns << " steps." << endl;
      cout << "Objective: " << objective_->eval(x) << endl;
      cout << "Gradient norm: " << gradient_->eval(x).norm() << endl;
      cout << "x = " << x.transpose() << endl;
    }

    if(mu_ < tol_ || (x - x_prev).norm() / x_prev.norm() < tol_)
      break;

    mu_ *= 0.5;
  }

  if(num_steps)
    *num_steps = total_steps;
  return x;
}

VectorXd NesterovInteriorPointSolver::solveInner(const VectorXd& init, long int* num_steps)
{
  assert(feasible(init));
  
  VectorXd x = init;
  VectorXd y = init;
  VectorXd gradient_x, gradient_y;
  VectorXd x_prev;
  double min_objective = FLT_MAX;
  VectorXd best_x;
  double k = 1;
  *num_steps = 0;
  int num_backtracks = 0;
  double mult = 0.5;
  double objective_x = std::numeric_limits<double>::max();
  double prev_objective_x = std::numeric_limits<double>::max();
  int num_bad_steps = 0;
  
  gradient_x = barrierGradient(x);
  if(debug_ > 1) {
    cout << "Nesterov step 0, mu = " << mu_ << ", gradient norm " << gradient_x.norm() << ", objective " << barrierObjective(x);
    if(optimal_solution_)
      cout << ", error norm: " << (x - *optimal_solution_).norm();
    cout << endl;

    cout << "  x = " << x.transpose() << endl;
  }

  while(true) {
    ++*num_steps;
    x_prev = x;

    double objective_y = barrierObjective(y);
    if(objective_y < min_objective) {
      min_objective = objective_y;
      best_x = y;
    }

    // -- Compute the gradient and step using backtracking.
    gradient_y = barrierGradient(y);

    // Adaptive stepsize choice.
    if(num_backtracks == 0)
      mult *= 2.0;
    else if(num_backtracks > 2)
      mult *= 2.0 / 3.0;
    bool precision_flag;
    double stepsize = backtracking(mult * initial_stepsize_, y, gradient_y,
                                   -gradient_y, objective_y, &num_backtracks,
                                   &precision_flag);

    // Regular backtracking.
    // double stepsize = backtracking(initial_stepsize_, y, gradient_y, -gradient_y,
    //                                    objective_y, &num_backtracks);

    // Fixed stepsize choice.
    // double beta = 0.5;
    // double stepsize = initial_stepsize_;
    // while(!feasible(y - stepsize * gradient_y))
    //   stepsize *= beta;
    // cout << "  y = " << y.transpose() << endl;
    // cout << "  Gradient at y = " << gradient_y.transpose() << endl;
    // cout << "  Using stepsize of " << stepsize << endl;

    // -- Nesterov step.
    x = y - stepsize * gradient_y;
    y = x + (k / (k + 3.0)) * (x - x_prev);

    // -- If y is infeasible, restart the algorithm.
    if(!feasible(y)) {
      if(debug_ > 1)
        cout << "  Infeasible y!  Restarting." << endl;
      k = 1;
      y = x;
    }
    
    // -- Check to see how good this location is.
    prev_objective_x = objective_x;
    objective_x = barrierObjective(x);
    if(objective_x < min_objective) {
      min_objective = objective_x;
      best_x = x;
    }

    // -- Restart if there is not a numerical precision issue
    //    and the situation calls for it.
    if(objective_x > prev_objective_x)
      ++num_bad_steps;
    else
      num_bad_steps = 0;
    
    if(!precision_flag &&
       ((restart_ < 0 && -restart_ < num_bad_steps) ||  
        (restart_ > 0 && (int)k % restart_ == 0)))
    { 
      if(debug_ > 1)
        cout << "Restarting Nesterov." << endl;
      return solveInner(x, num_steps);
    }

    gradient_x = barrierGradient(x);
    double norm = gradient_x.norm();
    
    if(debug_ > 1) {
      cout << setprecision(16)
           << "Nesterov Step " << *num_steps << ", mu = " << mu_ << ", gradient norm " << norm
           << ", objective " << setprecision(12) << objective_x
           << ", (x - x_prev).norm() " << setprecision(6) << (x - x_prev).norm()
           << ", " << num_backtracks << " backtracks, stepsize = " << stepsize;

      if(optimal_solution_)
        cout << ", error norm: " << (x - *optimal_solution_).norm();
      cout << endl;
    }

    if(debug_ > 2) {
      cout << "  x  = " << setprecision(16) << x.transpose() << endl;
      cout << "  y  = " << setprecision(16) << y.transpose() << endl;
      cout << "  dx = " << gradient_x.transpose() << endl;
      cout << "  dy = " << barrierGradient(y).transpose() << endl;
    }

    assert(feasible(x));
    assert(!isnan(norm));
    assert(!isinf(norm));
    cout.flush();
    
    if(norm < mu_) {
      if(debug_)
        cout << "Breaking due to tolerance." << endl;
      break;
    }
     else if(max_num_iters_ > 0 && *num_steps == max_num_iters_) {
       if(debug_)
         cout << "Breaking because num_iters " << *num_steps
              << " = max_num_iters_ = " << max_num_iters_ << endl;
      break;
    }
    ++k;
  }

  if(debug_) {
    cout << "Solver complete, gradient norm " << gradient_x.norm()
         << " < tolerance " << mu_ << ", objective " << barrierObjective(x);
    if(optimal_solution_)
      cout << ", error norm: " << (x - *optimal_solution_).norm();
    cout << endl;

    cout << "Best x found has gradient norm " << setprecision(12)
         << barrierGradient(best_x).norm() << ", objective "
         << barrierObjective(best_x) << endl;
  }

  return best_x;  
}

