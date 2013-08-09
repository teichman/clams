#ifndef COMMON_FUNCTIONS_H
#define COMMON_FUNCTIONS_H

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

#include <optimization/optimization.h>
#include <Eigen/Sparse>
#include <boost/shared_ptr.hpp>

double sigmoid(double z);
double logsig(double z);

typedef boost::shared_ptr< const Eigen::SparseMatrix<double> > SMConstPtr;
typedef boost::shared_ptr< const Eigen::SparseVector<double> > SVConstPtr;
typedef boost::shared_ptr< Eigen::SparseMatrix<double> > SMPtr;
typedef boost::shared_ptr< Eigen::SparseVector<double> > SVPtr;

//! Ax + b
class SparseLinearFunction : public VectorFunction
{
public:
  typedef boost::shared_ptr<SparseLinearFunction> Ptr;
  
  SMConstPtr A_;
  SVConstPtr b_;
  SparseLinearFunction(SMConstPtr A, SVConstPtr b);
  Eigen::VectorXd eval(const Eigen::VectorXd& x) const;
};

//! 1/2 x^T A x + b^T x + c
class SparseQuadraticFunction : public ScalarFunction
{
public:
  typedef boost::shared_ptr<SparseQuadraticFunction> Ptr;
  
  SMConstPtr A_;
  SVConstPtr b_;
  double c_;

  SparseQuadraticFunction(SMConstPtr A, SVConstPtr b, double c);
  double eval(const Eigen::VectorXd& x) const;
};

//! 1/M sum_i exp(A_i^T x + b_i)
//! Implemented efficiently using sparse Eigen.
class ObjectiveMELSparse: public ScalarFunction
{
public:
  //! A_.col(i) is A_i.
  Eigen::SparseMatrix<double, Eigen::ColMajor> const* A_;
  Eigen::VectorXd const* b_;

  ObjectiveMELSparse(Eigen::SparseMatrix<double, Eigen::ColMajor> const* A,
                     Eigen::VectorXd const* b);
  double eval(const Eigen::VectorXd& x) const;
};

class GradientMELSparse : public VectorFunction
{
public:
  Eigen::SparseMatrix<double, Eigen::ColMajor>* A_;
  Eigen::VectorXd* b_;
  
  GradientMELSparse(Eigen::SparseMatrix<double, Eigen::ColMajor>* A,
                    Eigen::VectorXd* b);
  Eigen::VectorXd eval(const Eigen::VectorXd& x) const;
};

//! 1/M sum_i log(1 + exp(A_i^T x + b_i))
//! Implemented efficiently using sparse Eigen.
class ObjectiveMLSSparse : public ScalarFunction
{
public:
  //! A_.col(i) is A_i.
  Eigen::SparseMatrix<double, Eigen::ColMajor> const* A_;
  Eigen::VectorXd const* b_;

  ObjectiveMLSSparse(Eigen::SparseMatrix<double, Eigen::ColMajor> const* A,
                     Eigen::VectorXd const* b);
  double eval(const Eigen::VectorXd& x) const;
};

class GradientMLSSparse : public VectorFunction
{
public:
  Eigen::SparseMatrix<double, Eigen::ColMajor>* A_;
  Eigen::VectorXd* b_;
  
  GradientMLSSparse(Eigen::SparseMatrix<double, Eigen::ColMajor>* A,
                    Eigen::VectorXd* b);
  Eigen::VectorXd eval(const Eigen::VectorXd& x) const;
};

class HessianMLSSparse : public MatrixFunction
{
public:
  Eigen::SparseMatrix<double, Eigen::ColMajor>* A_;
  Eigen::VectorXd* b_;

  HessianMLSSparse(Eigen::SparseMatrix<double, Eigen::ColMajor>* A,
                   Eigen::VectorXd* b);
  Eigen::MatrixXd eval(const Eigen::VectorXd& x) const;
};

//! 1/M sum_i log(1 + exp(a_i^T x + b_i))
class ObjectiveMLS : public ScalarFunction
{
public:
  //! a_.col(i) is a_i.
  Eigen::MatrixXd a_;
  Eigen::VectorXd b_;

  ObjectiveMLS(const Eigen::MatrixXd& a, const Eigen::VectorXd& b);
  double eval(const Eigen::VectorXd& x) const;
};

class GradientMLS : public VectorFunction
{
public:
  Eigen::MatrixXd a_;
  Eigen::VectorXd b_;

  GradientMLS(const Eigen::MatrixXd& a, const Eigen::VectorXd& b);
  Eigen::VectorXd eval(const Eigen::VectorXd& x) const;
};

class HessianMLS : public MatrixFunction
{
public:
  Eigen::MatrixXd a_;
  Eigen::VectorXd b_;

  HessianMLS(const Eigen::MatrixXd& a, const Eigen::VectorXd& b);
  Eigen::MatrixXd eval(const Eigen::VectorXd& x) const;
};

//! 1/M sum_i log(1 + exp(A_{:,i}^T x + b_i))
//! TODO: This should replace ObjectiveMLS.
class ObjectiveMLSNoCopy : public ScalarFunction
{
public:
  //! a_.col(i) is a_i.
  boost::shared_ptr<const Eigen::MatrixXd> A_;
  boost::shared_ptr<const Eigen::VectorXd> b_;

  ObjectiveMLSNoCopy(boost::shared_ptr<const Eigen::MatrixXd> A, boost::shared_ptr<const Eigen::VectorXd> b);
  double eval(const Eigen::VectorXd& x) const;
};

//! TODO: This should replace GradientMLS.
class GradientMLSNoCopy : public VectorFunction
{
public:
  boost::shared_ptr<const Eigen::MatrixXd> A_;
  boost::shared_ptr<const Eigen::VectorXd> b_;

  GradientMLSNoCopy(boost::shared_ptr<const Eigen::MatrixXd> A, boost::shared_ptr<const Eigen::VectorXd> b);
  Eigen::VectorXd eval(const Eigen::VectorXd& x) const;
};


//! 1/M \sum_i exp(a_i^T x + b_i)
class ObjectiveMEL : public ScalarFunction
{
public:
  //! a_.col(i) is a_i.
  Eigen::MatrixXd a_;
  Eigen::VectorXd b_;

  ObjectiveMEL(const Eigen::MatrixXd& a, const Eigen::VectorXd& b);
  double eval(const Eigen::VectorXd& x) const;
};
    
class GradientMEL : public VectorFunction
{
public:
  Eigen::MatrixXd a_;
  Eigen::VectorXd b_;

  GradientMEL(const Eigen::MatrixXd& a, const Eigen::VectorXd& b);
  Eigen::VectorXd eval(const Eigen::VectorXd& x) const;
};

class HessianMEL : public MatrixFunction
{
public:
  Eigen::MatrixXd a_;
  Eigen::VectorXd b_;

  HessianMEL(const Eigen::MatrixXd& a, const Eigen::VectorXd& b);
  Eigen::MatrixXd eval(const Eigen::VectorXd& x) const;
};

#endif // COMMON_FUNCTIONS_H
