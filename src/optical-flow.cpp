
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
#include "util.hpp"

using namespace std;
using namespace mira;
using namespace student;
using namespace cs_control;
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

  OpticalFlow_58941()
  {
    // the following macro initializes all variables stated in the reflect() function
    // to their default values
    MIRA_INITIALIZE_THIS;

    // do initialization stuff

    // you might want to initialize some class members etc.
    last_angle_ = 0;

    controller_steer_.reset_state();
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

    int step_size = 20;
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

    for (int x = 0; x < oldImage.cols() - patch_delta; x += step_size) 
    {
      for (int y = oldImage.rows()*0.5; y < oldImage.rows() - patch_size; y += step_size) 
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
          interestPoints.push(std::make_pair(InterestPoint(x,y), err_min));
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

    int corr_win = 1 + mCorrWin*2;

    Eigen::MatrixXf origPatch(corr_win, corr_win);

    // process every Interest point given in the vector
    foreach ( InterestPoint const& p, interestPoints ) 
    {
      // The following lines provide a rough guideline on how to
      // implement image patch correlation

      // you can omit points that are so close to the image boundary that the
      // correlation window does not fit. e.g. by
      if ( 
        p.x() - mCorrWin >= 0 && p.x() + mCorrWin < oldImage.cols() &&
        p.y() - mCorrWin >= 0 && p.y() + mCorrWin < oldImage.rows()
      )
      {
        // After that you can extract the reference patch from the old image:
        origPatch = oldImage.block(
          p.y() - mCorrWin,
          p.x() - mCorrWin,
          1 + mCorrWin*2,
          1 + mCorrWin*2
        );

        // Now you need to "find" the patch in the new image by comparing it
        // at every position in the search window
        int x_min;
        int y_min;
        float sad_min = std::numeric_limits<float>::infinity();

        for ( int x = p.x() - mSearchWin; x + mSearchWin <= p.x(); x++)
        {
          for ( int y = p.y(); y <= p.y() + mSearchWin; y++)
          {
            // You need to make sure that you do not run out of the image boundaries
            if (
              y - mCorrWin >= 0 &&
              x - mCorrWin >= 0 &&
              y + 1 + mCorrWin < oldImage.rows() &&
              x + 1 + mCorrWin < oldImage.cols()
            )
            {
              // by extracting the image patches in the new image!
              // Compute the difference between the two patch positions:
              double sad = (
                newImage.block(
                  y - mCorrWin,
                  x - mCorrWin,
                  1 + mCorrWin*2,
                  1 + mCorrWin*2
                ) - origPatch
              ).array().abs().sum();

              // now you can compute e.g. the SAD value from the difference matrix
              // you need to find the minimum value and store the "displacement" values
              // if the SAD Values are low enough (and/or if the position can be found with high
              // confidence) you can add the optical flow vector to the list:

              if (sad < sad_min)
              {
                sad_min = sad;
                x_min = x;
                y_min = y;
              }
            } 
          }
        }

        if (sad_min < sad_max_ * (1 + mCorrWin*2)*(1 + mCorrWin*2))
        {
          OFVectors.push_back(
            OFVector(
              p.x(),  // x position
              p.y(),  // y position
              x_min - p.x(),     // x direction
              y_min - p.y()     // y direction
            )
          );
        }
      }
    }

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
    // Call the reference implementation
    // Remove or comment the following line to make use of your own implementation!!
    // return OF::getDriveCommand( OFVectors );

    // set throttle and steering angle depending on the segmented image
    double throttle = 0.8; 	// range 0 - 1 suggested

    // steering correction for the flow vectors
    Eigen::Vector2i correction(
      - correct_rotation_ * last_angle_,
      0
    );

    double avg_left = 0;
    double avg_right = 0;
    double total_center = 0;
    int num_left = 0;
    int num_right = 0;
    int num_center = 0;

    double wall_width = 0.25 * image_width_;

    foreach( OFVector const& v, OFVectors ) 
    {
      double length = (v.dir - correction).norm();

      if (v.pos.x() < wall_width)
      {
        avg_left += v.dir.x();
        num_left++;
      }
      else if(v.pos.x() >= image_width_-wall_width)
      {
        avg_right += v.dir.x();
        num_right++;
      }
      else
      {
        total_center += length;
        num_center++;
      }
    }
    
    last_angle_ = clamp(
      controller_steer_.feed(
        avg_right - avg_left
      ),
      steer_max_
    );

    // pass the velocity command to the robot
    // the second value should be zero since the robot cannot
    // move sideways while looking forward
    return Velocity2(throttle, 0, last_angle_);
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
    r.property( "corrWin", mCorrWin, "size of the correlation window (multiplied by 2)", 7, PropertyHints::limits(1, 30) );
    r.property( "searchWin", mSearchWin, "size of the search window (multiplied by 2)", 15, PropertyHints::limits(1, 30) );
    r.property( "max_interest_points", max_interest_points_, "", 7, PropertyHints::limits(1, 100));
    r.property( "sad_max_", sad_max_, "", 30);
    r.property( "correct_rotation_", correct_rotation_, "", 0.0);

    r.property( "steer_max", steer_max_, "", 0.3);
    r.property( "P steer", controller_steer_.p_, "", 0.1);
    r.property( "I steer", controller_steer_.i_, "", 0.0);
    r.property( "D steer", controller_steer_.d_, "", 0.0);
  }

public:
  double threshold_;
  double sad_max_;
  uint max_interest_points_;

  double correct_rotation_;

  double last_angle_;

  double steer_max_;

  int image_width_;

  PIDController controller_steer_;

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
