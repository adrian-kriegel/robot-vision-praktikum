
#ifndef AK_CS_CONTROL_HPP
#define AK_CS_CONTROL_HPP

#include <vector>
#include <functional>
#include <cmath>
#include <limits>
#include <chrono>

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
    double padding_bottom=0,
    std::vector<bool>* indexes = nullptr
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

  #define PID_BASE_TIME 0.0024944

  class PIDController
  {
  private:
    double int_, last_error_;
    bool init_;

    std::chrono::time_point<std::chrono::system_clock> last_time_;
    
  public:
    double p_,i_,d_;
    double max_;

    PIDController(){ reset_state(); }
    PIDController(double max) { reset_state(); max_ = max; }

    void reset_state()
    {
      int_ = 0;
      last_error_ = 0;
      init_ = false;
      max_ = std::numeric_limits<double>::infinity();
    }

    double dt()
    {
      if (!init_)
        return 0;

      std::chrono::duration<double> diff = std::chrono::system_clock::now() - last_time_;
      return diff.count();
    }

    double feed(double e)
    {
      double d = 0;

      if (!init_)
      {
        init_ = true;
      }
      else 
      {
        d = (e - last_error_)/(dt() / PID_BASE_TIME);
        int_ += e * (dt() / PID_BASE_TIME);
      }

      double u = -(p_ * e + i_ * int_ + d_ * d); 

      last_error_ = e;

      if (std::abs(e) < 0.3)
      {
        int_ = 0;
      }
      
      last_time_ = std::chrono::system_clock::now();

      return std::min(u, max_);
    }
  };

  template<typename T>
  class LowPass
  {
  private:
    T last_val_;
    bool init_;
  public:

    double alpha_;

    void reset_state()
    {
      init_ = false;
    }

    T feed(T val)
    {
      if (!init_)
      {
        last_val_ = val;
        init_ = true;
      }
      else 
      {
        last_val_ *= alpha_;
        last_val_ += (1-alpha_)*val;
      }

      return last_val_;
    }

  };

}; // namespace control

#endif // AK_CONTROL_PP
