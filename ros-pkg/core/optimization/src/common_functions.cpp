#include <optimization/common_functions.h>

using namespace Eigen;
using namespace std;


SparseLinearFunction::SparseLinearFunction(SMConstPtr A, SVConstPtr b) :
  A_(A),
  b_(b)
{
}

Eigen::VectorXd SparseLinearFunction::eval(const Eigen::VectorXd& x) const
{
  Eigen::VectorXd result = (*A_) * x;
  for(SparseVector<double>::InnerIterator it(*b_); it; ++it)
    result(it.index()) += it.value();
  
  return result;
}

SparseQuadraticFunction::SparseQuadraticFunction(SMConstPtr A, SVConstPtr b, double c) : 
  A_(A),
  b_(b),
  c_(c)
{
}

double SparseQuadraticFunction::eval(const Eigen::VectorXd& x) const
{
  return 0.5 * x.transpose() * (*A_) * x + b_->dot(x) + c_;
}


double sigmoid(double z)
{
  long double big = exp(-z);
  if(isinf(1.0 + big))
    return 0.0;
  else
    return 1.0 / (1.0 + big);
}

double logsig(double z)
{
  long double big = exp(-z);
  if(isinf(1.0 + big))
    return z;
  else
    return -log(1.0 + big);
}

/************************************************************
 * Mean Exponential Loss, Sparse Implementation
 ************************************************************/

ObjectiveMELSparse::ObjectiveMELSparse(Eigen::SparseMatrix<double, Eigen::ColMajor> const* A,
                                       Eigen::VectorXd const* b) :
  A_(A),
  b_(b)
{
  assert(A_->cols() == b_->rows());
}

double ObjectiveMELSparse::eval(const VectorXd& x) const
{
  VectorXd vals = x.transpose() * (*A_);
  double obj = 0.0;
  for(int i = 0; i < vals.rows(); ++i)
    obj += exp(vals(i) + (*b_)(i)) / (double)A_->cols();

  if(isinf(obj)) {  // This is terrible.  Initialize with 0s when doing this optimization to avoid this.
    cout << "MEL Objective is inf!" << endl;
    assert(0);
    obj = FLT_MAX;
  }
  return obj;
}

GradientMELSparse::GradientMELSparse(Eigen::SparseMatrix<double, Eigen::ColMajor>* A,
                                     Eigen::VectorXd* b) :
  A_(A),
  b_(b)
{
  assert(A_->cols() == b_->rows());
}

VectorXd GradientMELSparse::eval(const VectorXd& x) const
{
  VectorXd grad = VectorXd::Zero(x.rows());
  VectorXd vals = x.transpose() * (*A_);

  for(int i = 0; i < A_->cols(); ++i) {
    double mult = exp(vals(i) + (*b_)(i)) / (double)A_->cols();
    for(Eigen::SparseMatrix<double, Eigen::ColMajor>::InnerIterator it(*A_, i); it; ++it) {  
      grad(it.row()) += mult * it.value();
    }
  }

  for(int i = 0; i < grad.rows(); ++i) {
    if(isinf(grad(i))) { // This should be avoided by initializing the optimization with 0s. (and making b not too large.)
      cout << "MEL gradient has an inf." << endl;
      assert(0);
    }
  }
  
  return grad;
}


/************************************************************
 * Mean Logistic Score, Sparse Implementation
 ************************************************************/

ObjectiveMLSSparse::ObjectiveMLSSparse(Eigen::SparseMatrix<double, Eigen::ColMajor> const* A,
                                       Eigen::VectorXd const* b) :
  A_(A),
  b_(b)
{
  assert(A_->cols() == b_->rows());
}

double ObjectiveMLSSparse::eval(const VectorXd& x) const
{
  VectorXd vals = -x.transpose() * (*A_);
  double obj = 0.0;
  for(int i = 0; i < vals.rows(); ++i)
    obj -= logsig(vals(i) - (*b_)(i));
  
  obj /= (double)A_->cols();
  return obj;
}

GradientMLSSparse::GradientMLSSparse(Eigen::SparseMatrix<double, Eigen::ColMajor>* A,
                                     Eigen::VectorXd* b) :
  A_(A),
  b_(b)
{
  assert(A_->cols() == b_->rows());
}

VectorXd GradientMLSSparse::eval(const VectorXd& x) const
{
  VectorXd grad = VectorXd::Zero(x.rows());
  VectorXd vals = x.transpose() * (*A_);

  for(int i = 0; i < A_->cols(); ++i) {
    double mult = sigmoid(vals(i) + (*b_)(i));
    for(Eigen::SparseMatrix<double, Eigen::ColMajor>::InnerIterator it(*A_, i); it; ++it) {  
      grad(it.row()) += mult * it.value();
    }
  }
        
  grad /= (double)A_->cols();
  return grad;
}

HessianMLSSparse::HessianMLSSparse(Eigen::SparseMatrix<double, Eigen::ColMajor>* A,
                                   Eigen::VectorXd* b) :
  A_(A),
  b_(b)
{
  assert(A_->cols() == b_->rows());
}

MatrixXd HessianMLSSparse::eval(const VectorXd& x) const
{
  MatrixXd hess = MatrixXd::Zero(x.rows(), x.rows());

  VectorXd vals = x.transpose() * (*A_);
  for(int i = 0; i < A_->cols(); ++i) {
    double mult = sigmoid(vals(i) + b_->coeffRef(i)) * sigmoid(-vals(i) - b_->coeffRef(i));
    for(Eigen::SparseMatrix<double, Eigen::ColMajor>::InnerIterator it1(*A_, i); it1; ++it1) {
      for(Eigen::SparseMatrix<double, Eigen::ColMajor>::InnerIterator it2(*A_, i); it2; ++it2) {  
        hess(it1.row(), it2.row()) += it1.value() * it2.value() * mult;
      }
    }
  }
    
  hess /= (double)A_->cols();
  return hess;
}


/************************************************************
 * Mean Logistic Score, non-optimized.
 ************************************************************/

ObjectiveMLS::ObjectiveMLS(const MatrixXd& a, const VectorXd& b) :
  a_(a),
  b_(b)
{
  assert(a_.cols() == b_.rows());
}

double ObjectiveMLS::eval(const VectorXd& x) const
{ 
  double obj = 0.0;
  for(int i = 0; i < a_.cols(); ++i)
    obj -= logsig(-x.dot(a_.col(i)) - b_(i));
  
  obj /= (double)a_.cols();
  return obj;
}

GradientMLS::GradientMLS(const MatrixXd& a, const VectorXd& b) :
  a_(a),
  b_(b)
{
  assert(a_.cols() == b_.rows());
}

VectorXd GradientMLS::eval(const VectorXd& x) const
{
  VectorXd grad = VectorXd::Zero(x.rows());
  for(int i = 0; i < a_.cols(); ++i)
    grad += a_.col(i) * sigmoid(x.dot(a_.col(i)) + b_(i));
  
  grad /= (double)a_.cols();
  return grad;
}

HessianMLS::HessianMLS(const MatrixXd& a, const VectorXd& b) :
  a_(a),
  b_(b)
{
  assert(a_.cols() == b_.rows());
}

MatrixXd HessianMLS::eval(const VectorXd& x) const
{
  MatrixXd hess = MatrixXd::Zero(x.rows(), x.rows());
  for(int i = 0; i < a_.cols(); ++i)
    hess += a_.col(i) * a_.col(i).transpose() * sigmoid(x.dot(a_.col(i)) + b_(i)) * sigmoid(-x.dot(a_.col(i)) - b_(i));
  
  hess /= (double)a_.cols();
  return hess;
}


/************************************************************
 * Mean Logistic Score, dense but no copy.
 ************************************************************/

ObjectiveMLSNoCopy::ObjectiveMLSNoCopy(boost::shared_ptr<const Eigen::MatrixXd> A, boost::shared_ptr<const Eigen::VectorXd> b) :
  A_(A),
  b_(b)
{
  assert(A_->cols() == b_->rows());
}

double ObjectiveMLSNoCopy::eval(const Eigen::VectorXd& x) const
{
  double obj = 0.0;
  for(int i = 0; i < A_->cols(); ++i)
    obj -= logsig(-x.dot(A_->col(i)) - b_->coeff(i));
  
  obj /= (double)A_->cols();
  return obj;
}

GradientMLSNoCopy::GradientMLSNoCopy(boost::shared_ptr<const Eigen::MatrixXd> A, boost::shared_ptr<const Eigen::VectorXd> b) :
  A_(A),
  b_(b)
{
  assert(A_->cols() == b_->rows());
}

VectorXd GradientMLSNoCopy::eval(const VectorXd& x) const
{
  VectorXd grad = VectorXd::Zero(x.rows());
  for(int i = 0; i < A_->cols(); ++i)
    grad += A_->col(i) * sigmoid(x.dot(A_->col(i)) + b_->coeff(i));
  
  grad /= (double)A_->cols();
  return grad;
}


/************************************************************
 *
 ************************************************************/

ObjectiveMEL::ObjectiveMEL(const MatrixXd& a, const VectorXd& b) :
  a_(a),
  b_(b)
{
  assert(a_.cols() == b_.rows());
}

double ObjectiveMEL::eval(const VectorXd& x) const
{
  double obj = 0.0;
  for(int i = 0; i < a_.cols(); ++i)
    obj += exp(x.dot(a_.col(i)) + b_(i));
  
  obj /= (double)a_.cols();
  return obj;
}

GradientMEL::GradientMEL(const MatrixXd& a, const VectorXd& b) :
  a_(a),
  b_(b)
{
  assert(a_.cols() == b_.rows());
}

VectorXd GradientMEL::eval(const VectorXd& x) const
{
  VectorXd grad = VectorXd::Zero(x.rows());
  for(int i = 0; i < a_.cols(); ++i)
    grad += a_.col(i) * exp(x.dot(a_.col(i)) + b_(i));
  
  grad /= (double)a_.cols();
    return grad;
}

HessianMEL::HessianMEL(const MatrixXd& a, const VectorXd& b) :
  a_(a),
  b_(b)
{
  assert(a_.cols() == b_.rows());
}

MatrixXd HessianMEL::eval(const VectorXd& x) const
{
  MatrixXd hess = MatrixXd::Zero(x.rows(), x.rows());
  for(int i = 0; i < a_.cols(); ++i)
    hess += a_.col(i) * a_.col(i).transpose() * exp(x.dot(a_.col(i)) + b_(i));
  
  hess /= (double)a_.cols();
  return hess;
}

