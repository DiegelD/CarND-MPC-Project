#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

class MPC {
 public:
  MPC();

  virtual ~MPC();

      // Solve the model given an initial state and polynomial coefficients.
      // Return the first actuations.
  std::vector<double> Solve(const Eigen::VectorXd &state,
                                const Eigen::VectorXd &coeffs);
  double prev_delta{0};
  double prev_throttle{0.1};
};

#endif  // MPC_H
