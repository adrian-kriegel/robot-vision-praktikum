
#ifndef ADRIANK_UTIL_HPP
#define ADRIANK_UTIL_HPP

#include <Eigen/Dense>

#ifndef NULL
#define NULL   ((void *) 0)
#endif

#define PI 3.14

typedef unsigned int uint;

namespace util
{
  float clamp(float val, float minmax);

  // finds center of largest plateau at specified height
  // returns leftmost or rightmost point if the plateau is on the edge
  int plateau_center(
    Eigen::MatrixXf hist,
    double height,
    double tol,
    int last_index
  );
}; // namespace ak_util

#endif // ADRIAN_UTIL_HPP
