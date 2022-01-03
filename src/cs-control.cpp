
#include "cs-control.hpp"

#include <algorithm>

double cs_control::calc_column_dist(
  std::function<uint(uint, uint)> img,
  uint width,
  uint height,
  double padding_top,
  double padding_bottom,
  double min_fill,
  uint col_width,
  uint x_start,
  uint x_end
)
{
  unsigned int y_start = height - 1 - padding_bottom*height;

  // number of "obstacle" pixels counted
  double obst_count = 0;

  // iterate image bottom to top
  for (
    uint y = y_start;
    y >= padding_top*height;
    y--
  )
  {
    // iterate column left to right
    for (
      uint x = x_start;
      x < x_end;
      x++
    )
    {
      if(img(x,y) < 255)
      {
        obst_count++;
      }
    }

    if(
      obst_count/(col_width*height) >= min_fill
    )
    {
      // current "height" in the image from bottom to y
      return (double)(1.0-(double)y/height);
    }
  }

  return 1.0 - padding_top - padding_bottom;
}

uint cs_control::hist_calc_distances(
  std::vector<double>& hist,
  std::function<uint(uint, uint)> img,
  unsigned int width,
  unsigned int height,
  double min_fill,
  double padding_left,
  double padding_right,
  double padding_top,
  double padding_bottom,
  std::vector<bool>* indexes
)
{
  unsigned int col_width = width*(1.0 - padding_left - padding_right) / hist.size();

  double max_dist = 0;
  
  // iterate through the histogram
  for (uint i = 0; i < hist.size(); i++)
  { 
    if (indexes != nullptr && !(indexes->at(i)))
      continue;

    hist.at(i) = cs_control::calc_column_dist(
      img,
      width, height, padding_top, padding_bottom,
      min_fill,
      col_width, 
      i*col_width + padding_left*width, // x start
      (i+1)*col_width + padding_left*width // x end
    );
    
    if (hist.at(i) > max_dist)
    {
      max_dist = hist.at(i);
    }
  }

  std::vector<uint> group_sizes(hist.size());

  std::fill(group_sizes.begin(), group_sizes.end(), 0);

  uint max_group_size = 0;
  uint max_group_start = 0;

  int current_group_start = -1;

  for (uint i = 0; i < hist.size(); i++)
  {
    bool is_max_dist = (std::abs(hist.at(i) - max_dist) <= 1.0/height);

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

void cs_control::calc_pursuit_point(
  std::vector<double>& path,
  uint hist_max_index,
  std::vector<double>& hist,
  std::vector<double>& hist_left,
  std::vector<double>& hist_right
)
{
  double max_distance = hist[hist_max_index];
  
  // find the largest local minimum 
  // or largest distance if there is none
  uint max_index = hist_max_index;
  /*
  for (uint i = 1; i < hist.size() - 1; i++)
  {
    if (
      // is local minimum
      hist[i-1] > hist[i] && hist[i] < hist[i+1] &&
      // is larger than the previous one
      hist[i] > max_distance
    )
    {
      max_distance = hist[i];
      max_index = i;
    }
  }
  */
  // point in the path is before the split
  for (uint i = 0; i < path.size(); i++)
  {
    // side histograms go from top to bottom
    // path should go from bottom to top
    uint path_index = path.size() - i - 1;

    path.at(path_index) = (double)hist_max_index/(hist.size() - 1);
    
    // distance on the path from bottom to top
    double pursuit_dist = (double)path_index/(path.size() + 1) * hist[hist_max_index];

    if (pursuit_dist < max_distance)
    {
      double factor = pursuit_dist/max_distance;

      path.at(path_index) *= factor;
      // center of the path
      path.at(path_index) += (1.0-factor)*0.5*(hist_left[i] + (1.0 - hist_right[i])); 
    }
  }

}
