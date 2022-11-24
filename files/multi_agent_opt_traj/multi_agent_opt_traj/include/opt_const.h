#ifndef MPC_H
#define MPC_H

#include <vector>
#include <eigen3/Eigen/Core>

#define DT 0.32 // time step duration dt in s 
#define PI                      3.1415926

// Set weights parameters for the cost function
#define W_X 0.1
#define W_Y 0.1
#define W_DV 1.5
#define W_DOMEGA 3.0


// Set lower and upper limits for variables.
#define MAXOMEGA 1 // 25 deg in rad, used as delta bound
#define MAXV  1 // Maximal a value
#define BOUND 1.0e20 // Bound value for other variables

#endif /* MPC_H */
