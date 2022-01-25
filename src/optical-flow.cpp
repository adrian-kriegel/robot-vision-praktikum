
#include <utility>
#include <iostream>
#include <queue>

#include <OpticalFlowTemplate.h>
#include <ReferenceOF.h>
#include <GDBPrint.h>

#include <factory/Factory.h>
#include <serialization/Serialization.h>
#include <serialization/DefaultInitializer.h>
#include <image/Img.h>

#include "cs-control.hpp"
#include "of-util.hpp"
#include "util.hpp"

using namespace std;
using namespace mira;
using namespace student;
using namespace cs_control;
using namespace of_util;
using namespace util;

///////////////////////////////////////////////////////////////////////////////

// start implementing by typing "opticalTemplate" and hitting
// CTRL+SPACE after the last letter.
// Step through the required parameters (class name etc.) using the tab key.

///////////////////////////////////////////////////////////////////////////////
/**
 * @brief Example implementation for steering the robot on the basis of
 * optical flow computation.
 */
class OpticalFlow_58941 : public OpticalFlowTemplate {
  MIRA_META_OBJECT( OpticalFlow_58941,
      ("StudentNumber","58941")
      ("Course","RV")
      ("Name","Kriegel")
      ("FirstName","Adrian")
      ("Description","Optical Flow"));

public:

  OpticalFlow_58941() :
    // set max. values for brake controllers to prevent them from speeding up the robot
    controller_brake_dist_(0.0),
    controller_brake_steer_(0.0)
  {
    // the following macro initializes all variables stated in the reflect() function
    // to their default values
    MIRA_INITIALIZE_THIS;

    // do initialization stuff

    // you might want to initialize some class members etc.
    last_angle_ = 0;

    low_pass_throttle_.reset_state();

    last_throttle_ = 0;
    last_angle_ = 0;

    last_max_index_ = -1;

    controller_brake_steer_.i_ = 0;
    controller_brake_dist_.i_ = 0;
  }

  /**
   * @brief The function should return salient features (features which are
   * good to track) for the given image.
   * For example you can apply some kind of edge detector to find good features.
   * @param[in] oldImage image which should be analyzed
   * @return Vector of features
   */
  vector<InterestPoint> interestOperator( Eigen::MatrixXf const& oldImage )
  {
    // Call the reference implementation
    // Remove or comment the following line to make use of your own implementation!!
    // The reference uses the moravec interest operator to find edges
    // The threshold is given in thousands
    // return OF::interestOperator( oldImage, mThreshold );

    // create a vector of features which will be returned to the caller
    typedef std::pair<InterestPoint, float> PointWithScore;

    auto cmp = [](PointWithScore l, PointWithScore r) { return l.second < r.second; };

    std::priority_queue<PointWithScore, std::deque<PointWithScore>, decltype(cmp)> interestPoints(cmp);

    // Please refer to the official documentation of Eigen if you do have
    // any questions on how to use the matrix class:
    // http://eigen.tuxfamily.org/dox/group__QuickRefPage.html

    // cycle through the image with a step width of 20 pixels
    // Of course, this is quite useless and should be replaced.

    int step_size = 5;
    int patch_size = step_size;
    int patch_delta = patch_size / 2;

    std::pair<int, int> dirs[] = {
      std::make_pair(-patch_delta, 0),
      std::make_pair(patch_delta, 0),
      std::make_pair(0, -patch_delta),
      std::make_pair(0, patch_delta)
    };

    // loop variables. pos. of the patch being compared
    float px, py;

    float err_min;

    int padding = std::max(patch_delta, mSearchWin) + 3;

    // ignore any points whose search windows cross outside the image
    // (x,y) is the top left corner of the patch
    for (int x = padding; x < oldImage.cols() - padding; x += step_size) 
    {
      for (int y = oldImage.rows()*0.45; y < oldImage.rows() - padding; y += step_size) 
      {
        Eigen::MatrixXf patch = oldImage.block(y, x, patch_size, patch_size);

        err_min = std::numeric_limits<float>::infinity();

        for (uint i = 0; i < 4; i++)
        {
          px = x + dirs[i].first;
          py = y + dirs[i].second;

          // if entire patch is within image bounds
          if (
            px >= 0 &&
            py >= 0 && 
            px + patch_size < oldImage.cols() &&
            py + patch_size < oldImage.rows()
          )
          {
            err_min = std::min(
              err_min,
              (patch - oldImage.block(py, px, patch_size, patch_size)).squaredNorm()
            );
          }

        }

        if (err_min/(patch_size*patch_size) > threshold_)
        {
          interestPoints.push(std::make_pair(
            // store the center of the patch
            InterestPoint(x + patch_size / 2, y + patch_size/2), 
            err_min
          ));
        }
      }
    }

    // return the vector of feature points
    vector<InterestPoint> res;
    res.reserve(max_interest_points_);

    while (!interestPoints.empty() && res.size() < max_interest_points_) 
    {
      res.push_back(interestPoints.top().first);
      interestPoints.pop();
    }

    return res;
  }

  /**
   * @brief calculate the apparent motion of the features at the given
   * positions between the two consecutive images.
   * E.g. you can use a correlation window and a search window to find the
   * Features detected in the oldImage (the interestPoints are given for the
   * locations in the old image) in the new one.
   * @param[in] oldImage the older image for which the interestPoints are defined
   * @param[in] newImage the new image, you have to search for the features in this image.
   * @param[in] interestPoints positions of the features detected in the old image.
   * @return vector of optical flow vectors
   */
  vector<OFVector> calculateOpticalFlow(
      Eigen::MatrixXf const& oldImage,
      Eigen::MatrixXf const& newImage,
      vector<InterestPoint> const& interestPoints )
  {
    image_width_ = oldImage.cols();
    // Call the reference implementation
    // Remove or comment the following line to make use of your own implementation!!
    // return OF::calculateOpticalFlow( oldImage, newImage, interestPoints, mSearchWin, mCorrWin );

    // create empty vector of optical flow vectors which will be return to the caller
    vector<OFVector> OFVectors;
    OFVectors.reserve(interestPoints.size());


    int factor = scaling_factor_;

    Eigen::MatrixXf old_scaled(oldImage.rows() / factor, oldImage.cols() / factor);
    Eigen::MatrixXf new_scaled(oldImage.rows() / factor, oldImage.cols() / factor);

    scale(old_scaled, oldImage);
    scale(new_scaled, newImage);

    old_scaled /= old_scaled.array().abs().maxCoeff();
    new_scaled /= new_scaled.array().abs().maxCoeff();

    /*
    lucas_kanade_.calc_optical_flow(
      OFVectors,
      old_scaled,
      new_scaled,
      interestPoints,
      mSearchWin,
      factor
    );
    */

    window_search_.calc_optical_flow(
      OFVectors,
      old_scaled,
      new_scaled,
      interestPoints,
      mSearchWin*2,
      mCorrWin*2
    );

    return OFVectors;
  }

  /**
   * @brief Calculate drive command depending on given segmentation
   * This function should calculate a reasonable drive command (steering angle,
   * and velocity) depending on the given segmentation.
   * @param[in] OFVectors Vectors of optical flow vectors
   * @return drive command as velocity
   */
  Velocity2 getDriveCommand( vector<OFVector> const& OFVectors )
  {
    double fov = M_PI/180.0 * fov_;

    std::vector<double> dist(OFVectors.size());
    
    int num_stripes = 12;

    // distance histogram of the image
    Eigen::VectorXf hist = Eigen::VectorXf::Zero(num_stripes);
    // number of OF-vectors per stripe
    Eigen::VectorXf density(hist);

    calc_distances(
      dist,
      OFVectors,
      image_width_,
      fov,
      last_angle_,
      last_throttle_
    );

    // max. allowed distance of the histogram
    double clipping_distance = 0;

    for (auto const & d : dist)
    {
      if (d > clipping_distance)
      {
        clipping_distance = d;
      }
    }

    clipping_distance += 5;
    // turn the distances into a histogram
    // the histogram must have the same (or larger) padding as the interest operator
    int padding = (mSearchWin + 3);

    for (int i = 0; i < dist.size(); i++)
    {
      if (std::isnan(dist[i]))
      {
        dist[i] = 0.6;
      }

      // distance < 0 may happen due to invalid flow vectors
      // this case should be ignored
      if (dist[i] >= 0)
      {
        double x = OFVectors.at(i).pos.x() - padding;
        int hist_index = std::max(std::min(
          (int)(x * (double)num_stripes / (image_width_-(2*padding))),
          num_stripes - 1
        ), 0);
        double x_hist = (image_width_-(2*padding));
        // points could only travel directly towards the camera if they are in the center
        if (
          (x_hist > 0.3 && x_hist < 0.7) ||
          OFVectors.at(i).dir.norm() > 1
        )
        {
          hist(hist_index) += dist[i];
          density(hist_index)++;
        }  
      }
    }

    // average out all distances
    hist.array() /= density.array();

    double max_hist = 0;

    // clip the histogram values and find the maximum
    for (int i = 0; i < num_stripes; i++)
    {
      // also clip any points without flow vectors
      if (density(i) <= 0.9 || hist(i) > clipping_distance)
      {
        hist(i) = clipping_distance;
      }

      if (hist(i) >= max_hist)
      {
        max_hist = hist(i);
      }
    }

    // put the histogram through a low pass
    // Eigen::VectorXf hist_cpy(hist);
    // hist = low_pass_hist_.feed(hist);
    
    // find the center of the highest plateau in the histogram
    double max_index = util::plateau_center(
      hist,
      max_hist,
      1,
      last_max_index_
    );

    // calculate the angle corresponding to the highest point
    double error = (max_index / (num_stripes-1) - 0.5) * fov*0.5;

    error += weight_pow_*std::pow(error, 3);

    error = clamp(error, fov*3.0);

    last_angle_ = controller_steer_.feed(error);

    last_throttle_ = low_pass_throttle_.feed(std::max(
      speed_max_ * (
        1.0 + 
        controller_brake_dist_.feed(hist.mean()) +
        controller_brake_steer_.feed(std::abs(last_angle_))
      ),
      0.0
    ));
    
    last_angle_ = clamp(last_angle_, steer_max_);

    // pass the velocity command to the robot
    // the second value should be zero since the robot cannot
    // move sideways while looking forward
    return Velocity2(last_throttle_, 0, last_angle_);
  }

  template <typename Reflector>
  void reflect( Reflector& r ) {
    OpticalFlowTemplate::reflect( r );

    // add your own member variables here
    // use
    // r.property( "ReadableNameOfVariable", <variable>, "Description", <default value>, <property hints> );
    // to add properties which can be edited. Use the Property hints to force the property to
    // stay within a certain range. E.g. you can use PropertyHints::limits(lower,upper) or PropertyHints::minimum(minimum)
    // And use
    // r.roproperty( "ReadableNameOfVariable", <variable>, "Description", <default value> );
    // to add read only members.

    // e.g. you can add a member variable which defines to height and width of your roi
    r.property( "threshold", threshold_, "", 100);
    r.property( "corrWin", mCorrWin, "size of the correlation window (multiplied by 2)", 3, PropertyHints::limits(1, 30) );
    r.property( "searchWin", mSearchWin, "size of the search window (multiplied by 2)",10, PropertyHints::limits(1, 30) );
    r.property( "search_step", search_step_, "", 3);
    
    r.property( "max_interest_points", max_interest_points_, "", 7, PropertyHints::limits(1, 100));

    r.property( "steer_max", steer_max_, "", 1);
    r.property( "P steer", controller_steer_.p_, "", 3);
    r.property( "I steer", controller_steer_.i_, "", 0.0);
    r.property( "D steer", controller_steer_.d_, "", 1);

    r.property("fov", fov_, "", 60);

    r.property( "speed_max_", speed_max_, "", 2.2);
    r.property( "lp throttle", low_pass_throttle_.alpha_, "", 0.9);

    r.property( "P br steer", controller_brake_steer_.p_, "", 1.2);
    r.property( "D br steer", controller_brake_steer_.d_, "", 40);

    r.property( "P br dist", controller_brake_dist_.p_, "", 0);
    r.property( "D br dist", controller_brake_dist_.d_, "", 1);

    r.property( "scaling", scaling_factor_, "", 1);
    r.property( "sad max", window_search_.sad_max_, "", 30);
    r.property( "lp hist", low_pass_hist_.alpha_, "", 0.02);


    r.property( "pow3", weight_pow_, "", 105);
  }

public:
  double threshold_;
  double deflect_;

  double fov_;

  uint max_interest_points_;
  int scaling_factor_;
  double correct_rotation_;

  double last_angle_, last_throttle_;
  double weight_pow_;

  double steer_max_;

  double obst_threshold_;

  int image_width_;

  int search_step_;

  double speed_max_;

  int last_max_index_;

  PIDController 
    controller_steer_,
    controller_brake_dist_,
    controller_brake_steer_;
  
  LowPass<double> low_pass_throttle_;
  LowPass<Eigen::VectorXf> low_pass_hist_;

  LucasKanade lucas_kanade_;
  WindowSearch window_search_;

  // Variables mSerachWin and mCorrWin are already declared in the parent
  // class. You should use these variables to define the size of the search
  // window and the correlation window

  // Here you can add member variables to your object
  // Please choose from one of the following types:
  // uint16 - unsigned integer in range of 0 .. 65535
  // float - signed floating point value
  // e.g. you can create a floating point variable b
  // with the following line:
  // float b;

};

MIRA_CLASS_SERIALIZATION( OpticalFlow_58941, student::OpticalFlowTemplate );

///////////////////////////////////////////////////////////////////////////////
