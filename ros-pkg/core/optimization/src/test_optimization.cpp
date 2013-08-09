
#include <timer/timer.h>
#include <optimization/optimization.h>
#include <optimization/common_functions.h>
#include <optimization/nips.h>
#include <optimization/grid_search.h>
#include <gtest/gtest.h>

using namespace std;
using namespace Eigen;    

#define DEBUG (getenv("DEBUG") ? atoi(getenv("DEBUG")) : 0)
#define RESTART (getenv("RESTART") ? atoi(getenv("RESTART")) : 0)

// A must be symmetric.
class QuadraticFunction : public ScalarFunction
{
public:
  typedef boost::shared_ptr<QuadraticFunction> Ptr;
  
  QuadraticFunction(const Eigen::MatrixXd& A,
                    const Eigen::VectorXd& b,
                    double c) :
    ScalarFunction(),
    A_(A),
    b_(b),
    c_(c)
    {
    }
  
  double eval(const Eigen::VectorXd& x) const
    {
      return 0.5 * x.transpose() * A_ * x + b_.dot(x) + c_;
    }

protected:
  Eigen::MatrixXd A_;
  Eigen::VectorXd b_;
  double c_;
};

class QuadraticGradient : public VectorFunction
{
public:
  typedef boost::shared_ptr<QuadraticGradient> Ptr;
  
  QuadraticGradient(const Eigen::MatrixXd& A,
                    const Eigen::VectorXd& b) :
    VectorFunction(),
    A_(A),
    b_(b)
    {
    }
  
  VectorXd eval(const Eigen::VectorXd& x) const
    {
      return A_ * x + b_;
    }

protected:
  Eigen::MatrixXd A_;
  Eigen::VectorXd b_;
};

class QuadraticHessian : public MatrixFunction
{
public:
  QuadraticHessian(const Eigen::MatrixXd& A) :
    MatrixFunction(),
    A_(A)
    {
    }

  MatrixXd eval(const Eigen::VectorXd& x) const
    {
      return A_;
    }

protected:
  Eigen::MatrixXd A_;
};

TEST(Functions, SparseLinearFunction)
{
  SMPtr A(new SparseMatrix<double>(10, 3));
  A->startVec(0);
  A->insertBack(0, 0) = 10;
  A->insertBack(7, 0) = 1;
  A->startVec(1);
  A->insertBack(3, 1) = 12;
  A->insertBack(4, 1) = 13;
  A->startVec(2);
  A->insertBack(1, 2) = 1;
  A->insertBack(7, 2) = 1;
  A->finalize();

  cout << *A << endl;

  SVPtr b(new SparseVector<double>(10));
  b->startVec(0);
  for(int i = 0; i < b->rows(); ++i)
    b->insertBack(i) = 1;
  cout << "b is: " << endl;
  cout << *b << endl;
  
  SparseLinearFunction slf(A, b);
                           

  VectorXd x = VectorXd::Zero(3);
  cout << slf.eval(x).transpose() << endl;

  x(1) = 1;
  cout << slf.eval(x).transpose() << endl;

  x(2) = 1;
  cout << slf.eval(x).transpose() << endl;
}

TEST(Functions, SparseQuadraticFunction)
{
  SMPtr A(new SparseMatrix<double>(3, 3));
  A->startVec(0);
  A->insertBack(0, 0) = 10;
  A->insertBack(2, 0) = 1;
  A->startVec(1);
  A->startVec(2);
  A->insertBack(1, 2) = 1;
  A->finalize();

  cout << *A << endl;
  SVPtr b(new SparseVector<double>(3));
  b->startVec(0);
  b->insertBack(0) = 1.0;
  b->insertBack(1) = 1.0;
  b->insertBack(2) = 1.0;
  cout << *b << endl;

  SparseQuadraticFunction sqf(A, b, 0.5);

  VectorXd x = VectorXd::Zero(3);
  cout << sqf.eval(x) << endl;

  x(1) = 1;
  cout << sqf.eval(x) << endl;

  x(2) = 1;
  cout << sqf.eval(x) << endl;
  
}

//! Solution is at (0, 0), no constraints, condition number 1000.
TEST(NesterovInteriorPointSolver, SimpleProblem)
{
  MatrixXd A = MatrixXd::Identity(2, 2);
  A(0, 0) = 1000;
  VectorXd b = VectorXd::Zero(2);
  QuadraticFunction::Ptr obj(new QuadraticFunction(A, b, 0));
  QuadraticGradient::Ptr grad(new QuadraticGradient(A, b));
  double mu = 1e-7;
  double tol = 1e-6;
  double alpha = 0.3;
  double beta = 0.5;
  int max_num_iters = 0;
  double stepsize = 1e-8; // Oscillates badly without restarting.
  //double stepsize = 1e-3;
  NesterovInteriorPointSolver nips(obj, grad, mu, tol, alpha, beta,
                                   max_num_iters, stepsize, RESTART, DEBUG);
  VectorXd xstar = VectorXd::Zero(2);
  nips.optimal_solution_ = &xstar;
  VectorXd init = VectorXd::Ones(2) * 100;
  init(1) = 500;
  ScopedTimer st("Solver time");
  VectorXd nips_soln = nips.solve(init);
  cout << "NIPS solution:" << endl << nips_soln.transpose() << endl;

  for(int i = 0; i < nips_soln.rows(); ++i)
    EXPECT_NEAR(nips_soln(i), 0, 1e-5);
}


// Wants to be at 0, perfectly conditioned, constrained to be in
// the unit ball at (10, 10) and x0 >= 9.75.
TEST(NesterovInteriorPointSolver, ConstrainedWellConditionedProblem)
{
  MatrixXd A = MatrixXd::Identity(2, 2);
  VectorXd b = VectorXd::Zero(2);
  QuadraticFunction::Ptr obj(new QuadraticFunction(A, b, 0));
  QuadraticGradient::Ptr grad(new QuadraticGradient(A, b));
  double mu = 1.0;
  double tol = 1e-6;
  double alpha = 0.3;
  double beta = 0.5;
  int max_num_iters = 0;
  double stepsize = 100;
  NesterovInteriorPointSolver nips(obj, grad, mu, tol, alpha, beta,
                                   max_num_iters, stepsize, RESTART, DEBUG);

  // -- x0 \geq 9.75
  VectorXd b0 = VectorXd::Zero(2);
  b0(0) = -1;
  QuadraticFunction::Ptr c0(new QuadraticFunction(MatrixXd::Zero(2, 2), b0, 9.75));
  QuadraticGradient::Ptr gc0(new QuadraticGradient(MatrixXd::Zero(2, 2), b0));
  nips.addConstraint(c0, gc0);

  // -- x1 \leq 100
  VectorXd b1 = VectorXd::Zero(2);
  b1(1) = 1;
  QuadraticFunction::Ptr c1(new QuadraticFunction(MatrixXd::Zero(2, 2), b1, -100));
  QuadraticGradient::Ptr gc1(new QuadraticGradient(MatrixXd::Zero(2, 2), b1));
  nips.addConstraint(c1, gc1);

  // -- x1 \geq 5
  VectorXd b2 = VectorXd::Zero(2);
  b2(1) = -1;
  QuadraticFunction::Ptr c2(new QuadraticFunction(MatrixXd::Zero(2, 2), b2, 5));
  QuadraticGradient::Ptr gc2(new QuadraticGradient(MatrixXd::Zero(2, 2), b2));
  nips.addConstraint(c2, gc2);

  // -- unit ball constraint at x = (10, 10).
  QuadraticFunction::Ptr c3(new QuadraticFunction(2.0 * MatrixXd::Identity(2, 2), -20 * VectorXd::Ones(2), 199));
  QuadraticGradient::Ptr gc3(new QuadraticGradient(2.0 * MatrixXd::Identity(2, 2), -20 * VectorXd::Ones(2)));
  nips.addConstraint(c3, gc3);
  
  VectorXd init(2);
  init(0) = 10;
  init(1) = 10;
  ScopedTimer st("Solver time");
  VectorXd nips_soln = nips.solve(init);
  cout << "NIPS solution:" << endl << nips_soln.transpose() << endl;

  EXPECT_NEAR((nips_soln - 10.0 * VectorXd::Ones(2)).norm(), 1.0, 1e-3);
  EXPECT_NEAR(nips_soln(0), 9.75, 1e-3);
}

// Wants to be at 0, moderately badly conditioned, constrained to be in
// the unit ball at (10, 10) and x0 >= 9.75.
TEST(NesterovInteriorPointSolver, ConstrainedBadlyConditionedProblem)
{
  SMPtr A(new SparseMatrix<double>(2, 2));
  A->startVec(0);
  //A->insertBack(0, 0) = 1000;
  A->insertBack(0, 0) = 1;
  A->startVec(1);
  A->insertBack(1, 1) = 1;
  A->finalize();
  SVPtr b(new SparseVector<double>(2));
  b->finalize();
  
  SparseQuadraticFunction::Ptr obj(new SparseQuadraticFunction(A, b, 0));
  SparseLinearFunction::Ptr grad(new SparseLinearFunction(A, b));
  double mu = 1.0;
  double tol = 1e-6;
  double alpha = 0.3;
  double beta = 0.5;
  int max_num_iters = 0;
  double stepsize = 1;
  NesterovInteriorPointSolver nips(obj, grad, mu, tol, alpha, beta,
                                   max_num_iters, stepsize, RESTART, DEBUG);
  VectorXd xstar(2);
  xstar(0) = 9.75;
  xstar(1) = 9.031754163;
  nips.optimal_solution_ = &xstar;
  
  // -- x0 \geq 9.75
  VectorXd b0 = VectorXd::Zero(2);
  b0(0) = -1;
  QuadraticFunction::Ptr c0(new QuadraticFunction(MatrixXd::Zero(2, 2), b0, 9.75));
  QuadraticGradient::Ptr gc0(new QuadraticGradient(MatrixXd::Zero(2, 2), b0));
  nips.addConstraint(c0, gc0);

  // -- x1 \leq 100
  VectorXd b1 = VectorXd::Zero(2);
  b1(1) = 1;
  QuadraticFunction::Ptr c1(new QuadraticFunction(MatrixXd::Zero(2, 2), b1, -100));
  QuadraticGradient::Ptr gc1(new QuadraticGradient(MatrixXd::Zero(2, 2), b1));
  nips.addConstraint(c1, gc1);

  // -- x1 \geq 5
  VectorXd b2 = VectorXd::Zero(2);
  b2(1) = -1;
  QuadraticFunction::Ptr c2(new QuadraticFunction(MatrixXd::Zero(2, 2), b2, 5));
  QuadraticGradient::Ptr gc2(new QuadraticGradient(MatrixXd::Zero(2, 2), b2));
  nips.addConstraint(c2, gc2);

  // -- unit ball constraint at x = (10, 10).
  QuadraticFunction::Ptr c3(new QuadraticFunction(2.0 * MatrixXd::Identity(2, 2), -20 * VectorXd::Ones(2), 199));
  QuadraticGradient::Ptr gc3(new QuadraticGradient(2.0 * MatrixXd::Identity(2, 2), -20 * VectorXd::Ones(2)));
  nips.addConstraint(c3, gc3);
  
  VectorXd init(2);
  init(0) = 10;
  init(1) = 10;
  ScopedTimer st("Solve time");
  VectorXd nips_soln = nips.solve(init);
  cout << "NIPS solution:" << endl << nips_soln.transpose() << endl;

  EXPECT_NEAR((nips_soln - 10.0 * VectorXd::Ones(2)).norm(), 1.0, 1e-3);
  EXPECT_NEAR(nips_soln(0), 9.75, 1e-3);
}

// Wants to be at (*, 0), constrained to be in
// the unit ball at (10, 10) and x0 >= 9.75.
// Infinitely bad conditioning, i.e. no curvature for x0.
TEST(NesterovInteriorPointSolver, NoCurvature)
{
  MatrixXd A = MatrixXd::Identity(2, 2);
  VectorXd b = VectorXd::Zero(2);
  A(0, 0) = 0.0;
  QuadraticFunction::Ptr obj(new QuadraticFunction(A, b, 0));
  QuadraticGradient::Ptr grad(new QuadraticGradient(A, b));
  double mu = 1.0;
  double tol = 1e-6;
  double alpha = 0.3;
  double beta = 0.5;
  int max_num_iters = 0;
  double stepsize = 1e-3;
  NesterovInteriorPointSolver nips(obj, grad, mu, tol, alpha, beta,
                                   max_num_iters, stepsize, RESTART, DEBUG);
  
  // -- x0 \geq 9.75
  VectorXd b0 = VectorXd::Zero(2);
  b0(0) = -1;
  QuadraticFunction::Ptr c0(new QuadraticFunction(MatrixXd::Zero(2, 2), b0, 9.75));
  QuadraticGradient::Ptr gc0(new QuadraticGradient(MatrixXd::Zero(2, 2), b0));
  nips.addConstraint(c0, gc0);

  // -- x1 \leq 100
  VectorXd b1 = VectorXd::Zero(2);
  b1(1) = 1;
  QuadraticFunction::Ptr c1(new QuadraticFunction(MatrixXd::Zero(2, 2), b1, -100));
  QuadraticGradient::Ptr gc1(new QuadraticGradient(MatrixXd::Zero(2, 2), b1));
  nips.addConstraint(c1, gc1);

  // -- x1 \geq 5
  VectorXd b2 = VectorXd::Zero(2);
  b2(1) = -1;
  QuadraticFunction::Ptr c2(new QuadraticFunction(MatrixXd::Zero(2, 2), b2, 5));
  QuadraticGradient::Ptr gc2(new QuadraticGradient(MatrixXd::Zero(2, 2), b2));
  nips.addConstraint(c2, gc2);

  // -- unit ball constraint at x = (10, 10).
  QuadraticFunction::Ptr c3(new QuadraticFunction(2.0 * MatrixXd::Identity(2, 2), -20 * VectorXd::Ones(2), 199));
  QuadraticGradient::Ptr gc3(new QuadraticGradient(2.0 * MatrixXd::Identity(2, 2), -20 * VectorXd::Ones(2)));
  nips.addConstraint(c3, gc3);
  
  VectorXd init(2);
  init(0) = 10;
  init(1) = 10;
  ScopedTimer st("Solve time");
  VectorXd nips_soln = nips.solve(init);
  cout << "NIPS solution:" << endl << nips_soln.transpose() << endl;

  EXPECT_NEAR((nips_soln - 10.0 * VectorXd::Ones(2)).norm(), 1.0, 1e-3);
}

TEST(GridSearch, Easy)
{
  MatrixXd A = MatrixXd::Identity(2, 2);
  VectorXd b = VectorXd::Zero(2);
  //A(0, 0) = 0.0;
  QuadraticFunction::Ptr obj(new QuadraticFunction(A, b, 0));

  GridSearch gs(2);
  gs.objective_ = obj;
  gs.max_resolutions_ << 1, 1;
  gs.grid_radii_ << 2, 2;
  gs.scale_factors_ << 0.1, 0.1;
  gs.num_scalings_ = 2;
  
  ArrayXd init = ArrayXd::Ones(2) * 9.1;
  ArrayXd x = gs.search(init);

  cout << x.transpose() << endl;
  EXPECT_NEAR(x.matrix().norm(), 0, 1e-3);
}

TEST(GridSearch, Coupling)
{
  MatrixXd A = MatrixXd::Identity(3, 3);
  VectorXd b = VectorXd::Zero(3);
  QuadraticFunction::Ptr obj(new QuadraticFunction(A, b, 0));

  GridSearch gs(3);
  gs.objective_ = obj;
  gs.num_scalings_ = 2;
  gs.max_resolutions_ << 1, 1, 1;
  gs.grid_radii_ << 2, 2, 2;
  gs.scale_factors_ << 0.1, 0.1, 0.1;
  gs.couplings_ << 1, 0, 0; // Search the last two jointly.
  
  ArrayXd init = ArrayXd::Ones(3) * 9.1;
  ArrayXd x = gs.search(init);

  cout << x.transpose() << endl;
  EXPECT_NEAR(x.matrix().norm(), 0, 1e-3);

  cout << "Computed " << gs.num_evals_ << " evals in " << gs.time_ << " seconds." << endl;
  cout << gs.num_evals_ / gs.time_ << " evals / second." << endl;
}


// TEST(GridSearch, Long)
// {
//   MatrixXd A = MatrixXd::Identity(2, 2);
//   VectorXd b = VectorXd::Zero(2);
//   QuadraticFunction::Ptr obj(new QuadraticFunction(A, b, 0));

//   GridSearch gs(2);
//   gs.verbose_ = false;
//   gs.objective_ = obj;
//   gs.max_resolutions_ << 1, 1;
//   gs.grid_radii_ << 1e6, 1e6;
//   gs.scale_factors_ << 0.8, 0.8;
//   gs.num_scalings_ = 1e2;
  
//   ArrayXd init = ArrayXd::Ones(2) * 9.1;
//   ArrayXd x = gs.search(init);

//   cout << x.transpose() << endl;
// }


int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

