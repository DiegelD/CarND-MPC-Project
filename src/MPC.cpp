#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using CppAD::AD;
using Eigen::VectorXd;

/**
 * TODO: Set the timestep length and duration
 */
size_t N = 10;
double dt = 0.1;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
//   simulator around in a circle with a constant steering angle and velocity on
//   a flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
//   presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;
const double ref_v = 40;


// TO DO - Checking where the x_start is coming from
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  VectorXd coeffs;
  FG_eval(VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    /**
     * TODO: implement MPC
     * `fg` is a vector of the cost constraints, `vars` is a vector of variable 
     *   values (state & actuators)
     *   The cost is stored is the first element of `fg`.
     * NOTE: You'll probably go back and forth between this function and
     *   the Solver function below.
     */
    size_t t;
    fg[0] = 0;

    // The part of the cost based on the reference state.
    for ( t = 0; t < N; t++)
    {
      fg[0] +=  500000 * CppAD::pow(vars[cte_start + t] , 2);     // CTE Error 
      fg[0] +=  50000 * CppAD::pow(vars[epsi_start + t], 2);             // EPSI Error
      fg[0] +=  10000 * CppAD::pow(vars[v_start + t] - ref_v, 2);
      /*
      // Calculating Speed Cost
      if ( vars[v_start + t] > speed_limit )   // Hard Stop if velocity is higher 
      {
        fg[0] += cost_high_speed;
        std::cout << "Highspeed Penaltiy " <<  std::endl;
      }
      else if ( (vars[v_start + t] > target_speed)  && (vars[v_start + t] < speed_limit) )  // Cost if Speed is in Buffer zone
      {
         std::cout << "Bufferzone " << std::endl;
        fg[0] +=   cost_stop_speed  * CppAD::pow( ((vars[v_start + t] - target_speed )/ v_buffer), 2);
      }
      else    // Otherwise calculate cost
      {
        std::cout << "Low Speed " << v0 << std::endl;
        fg[0] +=  cost_stop_speed * CppAD::pow( vars[v_start + t] - target_speed, 2); //
      }
      */

    }
    // Minimize the use of actuators.
    for ( t = 0; t < N - 1; t++)
    {
      fg[0] += 50* CppAD::pow(vars[delta_start + t], 2);     
      fg[0] +=  CppAD::pow(vars[a_start + t], 2);
    }

    // Cost that considers the acceleration and the rate of change for a smooths turn
    for ( t = 0; t < N - 2; t++)
    {
      fg[0] +=  1000 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);   // Stering Angle use delta
      fg[0] +=  CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);                    // Accerleration delta
    }

    // Setup Constraints
    //
    // NOTE: Setup the model constraints
    // Initial constraints
    //
    // Add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    for ( t = 1; t < N; t++)
    {
      /**
       *  Grab the rest of the states at t+1 and t.
       * 
       */
      // The state at time t+1
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // The state at time t
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      AD<double> f0;
      AD<double> psides0;

    
      f0 = coeffs[0] + x0*coeffs[1]  + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
      psides0 = CppAD::atan(coeffs[1] + 2*coeffs[2]*x0 + 3*coeffs[3]*CppAD::pow(x0,2));
  
  

      // NOTE: The use of `AD<double>` and use of `CppAD`!
      // CppAD can compute derivatives and pass these to the solver.

      // Recall the equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt

      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 / Lf * delta0 * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(psi0) * dt));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / (Lf * dt));
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

std::vector<double> MPC::Solve(const VectorXd &state, const VectorXd &coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;
  size_t i;
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  /**
   * TODO: Set the number of model variables (includes both states and inputs).
   * For example: If the state is a 4 element vector, the actuators is a 2
   *   element vector and there are 10 timesteps. The number of variables is:
   *   4 * 10 + 2 * 9
   */

  // number of independent variables
  // N timesteps == N - 1 actuations
  size_t n_vars = N * 6 + (N - 1) * 2;

  // Number of constraints
  size_t n_constraints = N * 6;


  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }

  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for ( i = 0; i < delta_start; i++)
  {
    vars_lowerbound[i] = std::numeric_limits<double>::lowest();
    vars_upperbound[i] = std::numeric_limits<double>::max();;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  vars_lowerbound[delta_start] = prev_delta;
  vars_upperbound[delta_start] = prev_delta;

  for ( i = delta_start + 1 ; i < a_start; i++)  //
  {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  vars_lowerbound[a_start] = prev_throttle;
  vars_upperbound[a_start] = prev_throttle;
  for ( i = a_start + 1 ; i < n_vars; i++)
  {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  /**
   * TODO: Set lower and upper limits for variables.
   */

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  // NOTE: You don't have to worry about these options
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  //   of sparse routines, this makes the computation MUCH FASTER. If you can
  //   uncomment 1 of these and see if it makes a difference or not but if you
  //   uncomment both the computation time should go up in orders of magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  prev_delta = solution.x[delta_start + 1];
  prev_throttle = solution.x[a_start + 1];


  std::vector<double> result = {solution.x[delta_start + 1], solution.x[a_start + 1]};
  for ( i = 0; i < N; ++i)
  {
    result.push_back(solution.x[x_start + i]);
    result.push_back(solution.x[y_start + i]);
  }
  return result;
}