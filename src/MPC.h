#ifndef MPC_H
#define MPC_H

#define HAVE_CSTDDEF 1

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

struct Point {
  double x;
  double y;
};

class MPC {
public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  bool Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  double first_delta;
  double first_a;
  vector<Point> predicted_points;
};

#endif /* MPC_H */
