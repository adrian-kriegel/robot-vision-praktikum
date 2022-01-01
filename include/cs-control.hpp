
#ifndef AK_CS_CONTROL_HPP
#define AK_CS_CONTROL_HPP

#include <vector>
#include <functional>

#include "util.hpp"

namespace cs_control
{
  double calc_column_dist(
    std::function<uint(uint, uint)> img,
    uint width,
    uint height,
    double padding_top,
    double padding_bottom,
    double min_fill,
    uint col_width,
    uint x_start,
    uint x_end
  );

  /**
  * @brief populates distance histogram
  * @param[out] hist histogram 
  * @param[in] img segmentation
  * @param[in] min_fill min. obstacle density before reporting a disance
  * @param[in] padding_x
  * @param[in] padding_top
  * @param[in] padding_bottom
  * @returns uint16 the index of the largest distance
  */
  uint hist_calc_distances(
    std::vector<double>& hist,
    std::function<uint(uint, uint)> img,
    unsigned int width,
    unsigned int height,
    double min_fill,
    double padding_left=0,
    double padding_right=0,
    double padding_top=0,
    double padding_bottom=0
  );

  /**
  * @brief calculates a pursuit point on a path from 3 distance histograms
  * @param[in] pursuit_dist the y coordinate of the persiut point 
  * @param[in] hist bottom to top distances
  * @param[in] hist_left left to right distances
  * @param[in] hist_right right to left distances
  * @returns the x coordinate of the pursuit point
  */
  void calc_pursuit_point(
    std::vector<double>& path,
    uint hist_max_index,
    std::vector<double>& hist,
    std::vector<double>& hist_left,
    std::vector<double>& hist_right
  );


  class PIDController
  {
  private:
    double int_, last_error_;

  public:
    double p_,i_,d_;

    PIDController(){ reset_state(); }

    void reset_state()
    {
      int_ = 0;
      last_error_ = 0;
    }

    double feed(double e)
    {
      int_ += e;

      double u = -(p_ * e + i_ * int_ + d_ * (e - last_error_)); 

      last_error_ = e;

      if (e < 0.3)
      {
        int_ = 0;
      }

      return u;
    }
  };

}; // namespace control

#endif // AK_CONTROL_PP
