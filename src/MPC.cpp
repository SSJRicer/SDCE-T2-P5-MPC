//#include "MPC.h"
//#include <cppad/cppad.hpp>
//#include <cppad/ipopt/solve.hpp>
//#include "Eigen-3.3/Eigen/Core"
//
//using CppAD::AD;
//
//// TODO: Set the timestep length and duration
//size_t N = 10;
//double dt = 0.1;
//
//// This value assumes the model presented in the classroom is used.
////
//// It was obtained by measuring the radius formed by running the vehicle in the
//// simulator around in a circle with a constant steering angle and velocity on a
//// flat terrain.
////
//// Lf was tuned until the the radius formed by the simulating the model
//// presented in the classroom matched the previous radius.
////
//// This is the length from front to CoG that has a similar radius.
//const double Lf = 2.67;
//
//const double ref_cte = 0;
//const double ref_epsi = 0;
//const double ref_v = 100;
//
//const size_t x_start = 0;
//const size_t y_start = x_start + N;
//const size_t psi_start = y_start + N;
//const size_t v_start = psi_start + N;
//const size_t cte_start = v_start + N;
//const size_t epsi_start = cte_start + N;
//const size_t delta_start = epsi_start + N;
//const size_t a_start = delta_start + N - 1;
//
//class FG_eval {
//public:
//    // Fitted polynomial coefficients
//    Eigen::VectorXd coeffs;
//    FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }
//
//    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
//    void operator()(ADvector& fg, const ADvector& vars) {
//      // TODO: implement MPC
//      // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
//      // NOTE: You'll probably go back and forth between this function and
//      // the Solver function below.
//
//      // The cost is stored is the first element of `fg`.
//      // Any additions to the cost should be added to `fg[0]`.
//      fg[0] = 0;
//
//      // The part of the cost based on the reference state.
//      for( int i = 0; i < N; i++ ) {
//        fg[0] += 1000*CppAD::pow(vars[cte_start + i] - ref_cte, 2);
//        fg[0] += 1000*CppAD::pow(vars[epsi_start + i] - ref_epsi, 2);
//        fg[0] += CppAD::pow(vars[v_start + i] - ref_v, 2);
//      }
//
//      // Minimize the use of actuators.
//      for (int i = 0; i< N - 1; i++) {
//        fg[0] += 50*CppAD::pow(vars[delta_start + i], 2);
//        fg[0] += 50*CppAD::pow(vars[a_start + i], 2);
//      }
//
//      // Minimize the value gap between sequential actuations.
//      // (how smooth the actuations are)
//      for (int i = 0; i < N - 2; i++) {
//        fg[0] += 250000*CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
//        fg[0] += 5000*CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
//      }
//
//      // Initial constraints.
//      fg[1 + x_start] = vars[x_start];
//      fg[1 + y_start] = vars[y_start];
//      fg[1 + psi_start] = vars[psi_start];
//      fg[1 + v_start] = vars[v_start];
//      fg[1 + cte_start] = vars[cte_start];
//      fg[1 + epsi_start] = vars[epsi_start];
//
//      for (int t = 1; t < N; t++) {
//        // The state at time t+1 .
//        AD<double> x1 = vars[x_start + t];
//        AD<double> y1 = vars[y_start + t];
//        AD<double> psi1 = vars[psi_start + t];
//        AD<double> v1 = vars[v_start + t];
//        AD<double> cte1 = vars[cte_start + t];
//        AD<double> epsi1 = vars[epsi_start + t];
//
//        // The state at time t.
//        AD<double> x0 = vars[x_start + t - 1];
//        AD<double> y0 = vars[y_start + t - 1];
//        AD<double> psi0 = vars[psi_start + t - 1];
//        AD<double> v0 = vars[v_start + t - 1];
//        AD<double> cte0 = vars[cte_start + t - 1];
//        AD<double> epsi0 = vars[epsi_start + t - 1];
//
//        // Only consider the actuation at time t.
//        AD<double> delta0 = vars[delta_start + t - 1];
//        AD<double> a0 = vars[a_start + t - 1];
//
//        AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
//        AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));
//
//        // Here's `x` to get you started.
//        // The idea here is to constraint this value to be 0.
//        //
//        // Recall the equations for the model:
//        // x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
//        // y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
//        // psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
//        // v_[t] = v[t-1] + a[t-1] * dt
//        // cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
//        // epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
//        fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
//        fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
//        fg[1 + psi_start + t] = psi1 - (psi0 - v0 / Lf * delta0 * dt);
//        fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
//        fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
//        fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) - v0 / Lf * delta0 * dt);
//      }
//    }
//};
//
////
//// MPC class definition implementation.
////
//MPC::MPC() {}
//MPC::~MPC() {}
//
//vector<double> MPC::Solve(Eigen::VectorXd &state, Eigen::VectorXd &coeffs) {
//  bool ok = true;
//  // size_t i;
//  typedef CPPAD_TESTVECTOR(double) Dvector;
//
//  const double x = state[0];
//  const double y = state[1];
//  const double psi = state[2];
//  const double v = state[3];
//  const double cte = state[4];
//  const double epsi = state[5];
//
//  // TODO: Set the number of model variables (includes both states and inputs).
//  // For example: If the state is a 4 element vector, the actuators is a 2
//  // element vector and there are 10 timesteps. The number of variables is:
//  //
//  // 4 * 10 + 2 * 9
//  const size_t n_vars = N * 6 + (N - 1) * 2;
//  // TODO: Set the number of constraints
//  const size_t n_constraints = N * 6;
//
//  // Initial value of the independent variables.
//  // SHOULD BE 0 besides initial state.
//  Dvector vars(n_vars);
//  for (int i = 0; i < n_vars; i++) {
//    vars[i] = 0;
//  }
//
//  Dvector vars_lowerbound(n_vars);
//  Dvector vars_upperbound(n_vars);
//  // TODO: Set lower and upper limits for variables.
//  // Set the initial variable values
//
//  // Set all non-actuators upper and lower limits
//  // to the max negative and positive values.
//  for ( int i = 0; i < delta_start; i++ ) {
//    vars_lowerbound[i] = -1.0e19;
//    vars_upperbound[i] = 1.0e19;
//  }
//
//  // The upper and lower limits of delta are set to -25 to 25
//  // degrees (values in radians).
//  for ( int i = delta_start; i < a_start; i++ ) {
//    vars_lowerbound[i] = -0.436332*Lf;
//    vars_upperbound[i] = 0.43632*Lf;
//  }
//
//  // Actuator limits.
//  for ( int i = a_start; i < n_vars; i++ ) {
//    vars_lowerbound[i] = -1.0;
//    vars_upperbound[i] = 1.0;
//  }
//
//  // Lower and upper limits for the constraints
//  // Should be 0 besides initial state.
//  Dvector constraints_lowerbound(n_constraints);
//  Dvector constraints_upperbound(n_constraints);
//  for (int i = 0; i < n_constraints; i++) {
//    constraints_lowerbound[i] = 0;
//    constraints_upperbound[i] = 0;
//  }
//
//  constraints_lowerbound[x_start] = x;
//  constraints_lowerbound[y_start] = y;
//  constraints_lowerbound[psi_start] = psi;
//  constraints_lowerbound[v_start] = v;
//  constraints_lowerbound[cte_start] = cte;
//  constraints_lowerbound[epsi_start] = epsi;
//
//  constraints_upperbound[x_start] = x;
//  constraints_upperbound[y_start] = y;
//  constraints_upperbound[psi_start] = psi;
//  constraints_upperbound[v_start] = v;
//  constraints_upperbound[cte_start] = cte;
//  constraints_upperbound[epsi_start] = epsi;
//
//  // object that computes objective and constraints
//  FG_eval fg_eval(coeffs);
//
//  //
//  // NOTE: You don't have to worry about these options
//  //
//  // options for IPOPT solver
//  std::string options;
//  // Uncomment this if you'd like more print information
//  options += "Integer print_level  0\n";
//  // NOTE: Setting sparse to true allows the solver to take advantage
//  // of sparse routines, this makes the computation MUCH FASTER. If you
//  // can uncomment 1 of these and see if it makes a difference or not but
//  // if you uncomment both the computation time should go up in orders of
//  // magnitude.
//  options += "Sparse  true        forward\n";
//  options += "Sparse  true        reverse\n";
//  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
//  // Change this as you see fit.
//  options += "Numeric max_cpu_time          0.5\n";
//
//  // place to return solution
//  CppAD::ipopt::solve_result<Dvector> solution;
//
//  // solve the problem
//  CppAD::ipopt::solve<Dvector, FG_eval>(
//          options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
//          constraints_upperbound, fg_eval, solution);
//
//  // Check some of the solution values
//  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
//
//  // Cost
//  // auto cost = solution.obj_value;
//  // std::cout << "Cost " << cost << std::endl;
//
//  // TODO: Return the first actuator values. The variables can be accessed with
//  // `solution.x[i]`.
//  //
//  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
//  // creates a 2 element double vector.
//  vector<double> result;
//
//  result.push_back(solution.x[delta_start]);
//  result.push_back(solution.x[a_start]);
//
//  for ( int i = 0; i < N - 2; i++ ) {
//    result.push_back(solution.x[x_start + i + 1]);
//    result.push_back(solution.x[y_start + i + 1]);
//  }
//  return result;
//}

#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;
using Eigen::VectorXd;

// Prediction Horizon (T) = 1 sec
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

// Vars vector starting indices for each element:
const size_t x_start = 0;
const size_t y_start = x_start + N;
const size_t psi_start = y_start + N;
const size_t v_start = psi_start + N;
const size_t cte_start = v_start + N;
const size_t epsi_start = cte_start + N;
const size_t delta_start = epsi_start + N;
const size_t a_start = delta_start + N - 1;

// Velocity reference for the cost function:
const double ref_v = 100;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  VectorXd coeffs;
  FG_eval(VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {

    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    // Based on the reference state (cte, psi error, velocity):
    // Setting the weights for each element:
    const int cte_w = 1000;
    const int epsi_w = 1000;
    const int v_w = 1;

    for (int t = 0; t < N; t++) {
        fg[0] += cte_w  * CppAD::pow(vars[cte_start + t], 2);
        fg[0] += epsi_w * CppAD::pow(vars[epsi_start + t], 2);
        fg[0] += v_w    * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Minimizing the use of actuators (steeing angle, accelration):
    // Setting the weights for each element:
    const int delta_w = 50;
    const int a_w = 50;

    for (int t = 0; t < N - 1; t++) {
        fg[0] += delta_w * CppAD::pow(vars[delta_start + t], 2);
        fg[0] += a_w     * CppAD::pow(vars[a_start + t], 2);
    }

    // Minimizing the value gap between sequential actuations:
    // Setting the weights for each element:
    const int delta_change_w = 250000;
    const int a_change_w = 5000;

    for (int t = 0; t < N - 2; t++) {
        fg[0] += delta_change_w * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
        fg[0] += a_change_w     * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (int t = 1; t < N; ++t) {
      // Grabbing each element in both time states (t & t+1):
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> x1 = vars[x_start + t];

      AD<double> y0 = vars[y_start + t - 1];
      AD<double> y1 = vars[y_start + t];

      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> psi1 = vars[psi_start + t];

      AD<double> v0 = vars[v_start + t - 1];
      AD<double> v1 = vars[v_start + t];

      // f( x(t) ):
      AD<double> f0 = coeffs[0] + coeffs[1]*x0 + coeffs[2]*pow(x0, 2) + coeffs[3]*pow(x0, 3);
      AD<double> cte0 = f0 - y0;
      AD<double> cte1 = vars[cte_start + t];

      // psi_des(t):
      AD<double> psi_des0 = CppAD::atan(coeffs[1] + 2*coeffs[2]*x0 + 3*coeffs[3]*pow(x0, 2));
      AD<double> epsi0 = psi0 - psi_des0;
      AD<double> epsi1 = vars[epsi_start + t];

      // For actuators, we only need time state t:
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      // Model equations:
      // x(t+1)    = x(t)   + v(t) * cos( psi(t) ) * dt
      // y(t+1)    = y(t)   + v(t) * sin( psi(t) ) * dt
      // psi(t+1)  = psi(t) + (v(t) / Lf) * delta(t) * dt
      // v(t+1)    = v(t)   + a(t) * dt
      // cte(t+1)  = cte(t) + v(t) * sin( epsi(t) ) * dt
      //     cte(t) = f( x(t) ) - y(t)
      //      f( x(t) ) = a + bx(t) + cx(t)^2 + dx(t)^3
      // epsi(t+1) = epsi(t) + (v(t) / Lf) * delta(t) * dt
      //    epsi(t) = psi(t) - psi_des(t)
      //     psi_des(t) = arctan( f'( x(t) ) ) = arctan( b + 2cx(t) + 3dx(t)^2 )

      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 - (v0 / Lf) * delta0 * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] = cte1 - (cte0 + v0 * CppAD::sin(epsi0) * dt);
      fg[1 + epsi_start + t] = epsi1 - (epsi0 - (v0 / Lf) * delta0 * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

std::vector<double> MPC::Solve(VectorXd &state, VectorXd &coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // The vars vector contains N samples of the state elements (6 in number)
  //  and N-1 samples of the actuators (steering angle and acceleration):
  size_t n_vars = N * 6 + (N - 1) * 2;

  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; ++i) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; ++i) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  for (int i = delta_start; i < a_start; ++i) {
    vars_lowerbound[i] = -0.436332 * Lf;
    vars_upperbound[i] = 0.436332 * Lf;
  }

  // Acceleration/decceleration upper and lower limits.
  for (int i = a_start; i < n_vars; ++i) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; ++i) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start] = state[0];
  constraints_lowerbound[y_start] = state[1];
  constraints_lowerbound[psi_start] = state[2];
  constraints_lowerbound[v_start] = state[3];
  constraints_lowerbound[cte_start] = state[4];
  constraints_lowerbound[epsi_start] = state[5];

  constraints_upperbound[x_start] = state[0];
  constraints_upperbound[y_start] = state[1];
  constraints_upperbound[psi_start] = state[2];
  constraints_upperbound[v_start] = state[3];
  constraints_upperbound[cte_start] = state[4];
  constraints_upperbound[epsi_start] = state[5];

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

  // Returning the 1st actuator values:
  vector<double> result;

  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);
  for (int i = 0; i < N - 2; i++) {
    result.push_back(solution.x[x_start + i + 1]);
    result.push_back(solution.x[y_start + i + 1]);
  }

  return result;
}