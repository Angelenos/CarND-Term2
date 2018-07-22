#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;
// for portability of M_PI (Vis Studio, MinGW, etc.)
#ifndef M_PI
const double M_PI = 3.14159265358979323846;
#endif

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  // Re-write deg-rad conversion functions into the MPC class so both main.cpp and MPC.cpp can access them
  double deg2rad(double x) { return x * M_PI / 180; }
  double rad2deg(double x) { return x * 180 / M_PI; }
};

#endif /* MPC_H */
