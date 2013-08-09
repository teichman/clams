#include <optimization/common_functions.h>
#include <timer/timer.h>

using namespace Eigen;
using namespace std;
using boost::shared_ptr;

int main(int argc, char** argv)
{

  int num_variables = 1e3;
  int num_training_examples = 1e3;
  
  shared_ptr<MatrixXd> A(new MatrixXd(num_variables, num_training_examples));
  for(int i = 0; i < num_training_examples; ++i) {
    for(int j = 0; j < num_variables; ++j) {
      A->coeffRef(j, i) = (double)rand() / (double)RAND_MAX;
    }
  }
  
  shared_ptr<VectorXd> b(new VectorXd(num_training_examples));
  for(int i = 0; i < num_training_examples; ++i) {
    b->coeffRef(i) = (double)rand() / (double)RAND_MAX;
  }
  
  ObjectiveMLSNoCopy obj(A, b);
  GradientMLSNoCopy grad(A, b);
  double tol = 1e-6;
  double alpha = 0.3;
  double beta = 0.5;
  VectorXd init = VectorXd::Zero(num_variables);

  NesterovGradientSolver ngs(&obj, &grad, tol, alpha, beta);
  cout << "Starting nesterov." << endl;
  HighResTimer hrt("Nesterov");
  hrt.start();
  VectorXd ngs_sol = ngs.solve(init);
  hrt.stop();
  cout << hrt.reportSeconds() << endl;

  GradientSolver gs(&obj, &grad, tol, alpha, beta);
  hrt.reset("Gradient");
  hrt.start();
  VectorXd gs_sol = gs.solve(init);
  hrt.stop();
  cout << hrt.reportSeconds() << endl;
    
  return 0;
}
