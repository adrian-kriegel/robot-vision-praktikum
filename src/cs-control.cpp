
#include "cs-control.hpp"

#include <algorithm>

uint cs_control::hist_calc_distances(
  std::vector<double>& hist,
  std::function<uint(uint, uint)> img,
  unsigned int width,
  unsigned int height,
  double min_fill,
  double padding_left,
  double padding_right,
  double padding_top,
  double padding_bottom
)
{
  unsigned int col_width = (width - (padding_left+padding_right)*width)/ hist.size();

  unsigned int y_start = height - 1 - padding_bottom*height;

  double max_dist = 0;
  double max_dist_index = 0;
  
  // iterate through the histogram
  for (uint i = 0; i < hist.size(); i++)
  { 
    hist.at(i) = 1.0;

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
        uint x = i*col_width + padding_left*width;
        x < (i+1)*col_width + padding_left*width;
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
        double dist = (double)(1.0-(double)y/height);

        hist.at(i) = dist;

        if (dist > max_dist)
        {
          max_dist = dist;
          max_dist_index = i;
        }

        break;
      }
    }
  }
  
  return max_dist_index;
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
