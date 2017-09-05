#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();
  
  // Struct for returning actuators and predicted points
  struct Results {
    vector<double> MPC_X = {};
    vector<double> MPC_Y = {};
    vector<double> ACTUATORS = {};
  };

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  Results Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
