
#include "util.hpp"

#include <algorithm>

float util::clamp(float val, float minmax)
{
  return std::min(std::max(val, -minmax), minmax);
}

int util::plateau_center(
  Eigen::MatrixXf hist,
  double height,
  double tol,
  int last_index
)
{
  std::vector<uint> group_sizes(hist.size());

  std::fill(group_sizes.begin(), group_sizes.end(), 0);

  uint max_group_size = 0;
  uint max_group_start = 0;

  int current_group_start = -1;

  for (uint i = 0; i < hist.size(); i++)
  {
    bool is_max_dist = (std::abs(hist(i) - height) <= tol);

    if (current_group_start == -1 && is_max_dist)
    {
      current_group_start = i;
    }
    
    if (current_group_start != -1)
    {
      if (group_sizes.at(current_group_start) > max_group_size)
      {
        max_group_size = group_sizes.at(current_group_start);
        max_group_start = current_group_start;
      }

      if (is_max_dist)
      {
        group_sizes.at(current_group_start)++;
      }
      else
      {
        current_group_start = -1;
      }

    }
  }

  // if the last max. index is still "good enough"
  // prevents the path from flipping between two similarly good goal points
  if (
    last_index != -1 &&
    std::abs(hist(last_index) - height) <= tol
  )
  {
    // find the group the index now belongs to
    for (int i = last_index; i >= 0; i--)
    {
      if (group_sizes.at(i) != 0)
      {
        if (std::abs((int)group_sizes[i] - group_sizes[max_group_start]) <= 1)
        {
          max_group_start = i;
        }
        break;
      }
    }
  }

  // if the group starts to the left or ends on the right, return the outermost index
  if (
    max_group_start == 0
  )
  {
    return 0;
  }

  if (
    max_group_start + group_sizes[max_group_start] == hist.size() - 1
  )
  {
    return hist.size() - 1;
  }

  // else return the center of the largest group
  return max_group_start + group_sizes[max_group_start]/2;
}