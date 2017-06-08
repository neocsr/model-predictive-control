#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 20;
double dt = 0.05;
//double dt = 0.05;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

double ref_cte = 0;
double ref_epsi = 0;
double ref_v = 40;

size_t x_idx = 0;
size_t y_idx = x_idx + N;
size_t psi_idx = y_idx + N;
size_t v_idx = psi_idx + N;
size_t cte_idx = v_idx + N;
size_t epsi_idx = cte_idx + N;    // N values
size_t delta_idx = epsi_idx + N;  // N-1 values
size_t a_idx = delta_idx + N - 1; // N-1 values

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // fg a vector of constraints, x is a vector of constraints.
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    // Cost
    fg[0] = 0;

    // Cost based on the reference state
    for (int i = 0; i < N; i++) {
      fg[0] += CppAD::pow(vars[cte_idx + i] - ref_cte, 2);
      fg[0] += CppAD::pow(vars[epsi_idx + i] - ref_epsi, 2);
      fg[0] += CppAD::pow(vars[v_idx + i] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (int i = 0; i < N - 1; i++) {
      fg[0] += CppAD::pow(vars[delta_idx + i], 2);
      fg[0] += CppAD::pow(vars[a_idx + i], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int i = 0; i < N - 2; i++) {
      fg[0] += CppAD::pow(vars[delta_idx + i + 1] - vars[delta_idx + i], 2);
      fg[0] += CppAD::pow(vars[a_idx + i + 1] - vars[a_idx + i], 2);
    }

    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_idx] = vars[x_idx];
    fg[1 + y_idx] = vars[y_idx];
    fg[1 + psi_idx] = vars[psi_idx];
    fg[1 + v_idx] = vars[v_idx];
    fg[1 + cte_idx] = vars[cte_idx];
    fg[1 + epsi_idx] = vars[epsi_idx];

    // The rest of the constraints
    for (int i = 0; i < N - 1; i++) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_idx + i + 1];
      AD<double> y1 = vars[y_idx + i + 1];
      AD<double> psi1 = vars[psi_idx + i + 1];
      AD<double> v1 = vars[v_idx + i + 1];
      AD<double> cte1 = vars[cte_idx + i + 1];
      AD<double> epsi1 = vars[epsi_idx + i + 1];

      // The state at time t.
      AD<double> x0 = vars[x_idx + i];
      AD<double> y0 = vars[y_idx + i];
      AD<double> psi0 = vars[psi_idx + i];
      AD<double> v0 = vars[v_idx + i];
      AD<double> cte0 = vars[cte_idx + i];
      AD<double> epsi0 = vars[epsi_idx + i];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_idx + i];
      AD<double> a0 = vars[a_idx + i];

      // f(x)
      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0;

      // f'(x)
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0);

      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      //
      // Recall the equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      fg[2 + x_idx + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[2 + y_idx + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[2 + psi_idx + i] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[2 + v_idx + i] = v1 - (v0 + a0 * dt);
      fg[2 + cte_idx + i] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[2 + epsi_idx + i] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

bool MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // State and Errors
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = N * 6 + (N - 1) * 2;
  // N = 10 => 20 * 6 + 19 * 2 = 120 + 38 = 158

  // TODO: Set the number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_idx; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (int i = delta_idx; i < a_idx; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (int i = a_idx; i < n_vars; i++) {
    vars_lowerbound[i] = -0.4;
    vars_upperbound[i] = 0.4;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_idx] = x;
  constraints_lowerbound[y_idx] = y;
  constraints_lowerbound[psi_idx] = psi;
  constraints_lowerbound[v_idx] = v;
  constraints_lowerbound[cte_idx] = cte;
  constraints_lowerbound[epsi_idx] = epsi;

  constraints_upperbound[x_idx] = x;
  constraints_upperbound[y_idx] = y;
  constraints_upperbound[psi_idx] = psi;
  constraints_upperbound[v_idx] = v;
  constraints_upperbound[cte_idx] = cte;
  constraints_upperbound[epsi_idx] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
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

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.

  this->first_delta = solution.x[delta_idx];
  this->first_a = solution.x[a_idx];
  this->predicted_points.resize(N);

  std::cout << "Solution.x length: " << solution.x.size() << std::endl;
  std::cout << "N: " << N << std::endl;
  std::cout << "Size predicted: " << this->predicted_points.size() << std::endl;

  for (int i = 0; i < N; ++i) {
    Point p;
    p.x = solution.x[x_idx + i];
    p.y = solution.x[y_idx + i];
    this->predicted_points[i] = p;
  }

  return ok;
  
//  return {solution.x[x_idx + 1],   solution.x[y_idx + 1],
//          solution.x[psi_idx + 1], solution.x[v_idx + 1],
//          solution.x[cte_idx + 1], solution.x[epsi_idx + 1],
//          solution.x[delta_idx],   solution.x[a_idx]};
}
