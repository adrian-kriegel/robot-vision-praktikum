
#include <mutex>

#include <ColorSegmentationTemplate.h>
#include <ReferenceCS.h>
#include <GDBPrint.h>

#include <factory/Factory.h>
#include <serialization/Serialization.h>
#include <serialization/DefaultInitializer.h>
#include <image/Img.h>

#include "util.hpp"
#include "cs-control.hpp"
#include "cs-debug.hpp"

using namespace std;
using namespace mira;
using namespace rv;
using namespace student;
using namespace util;

///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Example implementation for steering the robot on the basis of
 * ground color segmentation.
 */
class ColorSegmentationRG_58941 : public ColorSegmentationTemplateRG {

  // TODO: Add you personal and implementation details here
  MIRA_META_OBJECT( ColorSegmentationRG_58941,
      ("StudentNumber","58941")
      ("Course","RV")
      ("Name","Kriegel")
      ("FirstName","Adrian")
      ("Description","RG"));

  

  unsigned int mask_height_;
  unsigned int mask_width_top_;
  unsigned int mask_width_bottom_;

  bool interpolate_histogram_;

  float alpha_hist_;

  Histogram2D* last_hist_;

  GrayImage* segmentation_;

  std::mutex plot_mutex_;
  GrayImage* debug_plot_;

  // max. percentage of black pixels to count before an average distance is reported
  float max_stripe_fill_;

  float speed_;
  float pursuit_dist_;

  double steer_max_;

  float last_angle_;
  float alpha_angle_;

  float avoid_walls_;

  double alpha_pursuit_;

  double brake_;

  double max_speed_;

  double min_dist_;

  uint debug_hist_;

  cs_control::PIDController controller_steer_;

  // throttle controllers are using for braking, so their max. value should be 0
  cs_control::PIDController controller_throttle_error_;
  cs_control::PIDController controller_throttle_offset_;
  cs_control::PIDController controller_throttle_dist_;
  
  cs_control::LowPass<double> low_pass_throttle_;
  cs_control::LowPass<double> low_pass_pursuit_;

  bool debug_;

  double weight_offset_;

  double turning_speed_;

public:

  ColorSegmentationRG_58941()
  : controller_throttle_error_(0),
    controller_throttle_offset_(0),
    controller_throttle_dist_(0)
  {
    // the following macro initializes all variables stated in the reflect() function
    // to their default values
    MIRA_INITIALIZE_THIS;

    // do initialization stuff

    // you might want to initialize some class members etc.

    last_hist_ = NULL;
    segmentation_ = NULL;
    debug_plot_ = NULL;

    last_angle_ = 0.0;

    controller_steer_.reset_state();
    controller_throttle_error_.reset_state();
    controller_throttle_offset_.reset_state();
    controller_throttle_dist_.reset_state();

    low_pass_throttle_.reset_state();
    low_pass_pursuit_.reset_state();
  }

  /**
   * @brief Calculate image segmentation for a given input image.
   * Usually, you should segment the given input image into two classes (e.g.
   * 0 for obstacles and 255 for ground). However, you might want to pass
   * grey values to express uncertainty.
   * @param[in] inputImage the camera image from the robot (3-channel bgr values)
   * @return segmented image (one channel)
   */
  GrayImage segmentImage( BGRImage const& inputImage )
  {
    
    // create new segmented image
    if(segmentation_ == NULL)
    {
      segmentation_ = new GrayImage(inputImage.width(), inputImage.height());
    }

    // call function BRG_to_RG (have to be implemented properly)
    // to convert bgr image to rg
    // please note that the rg values are normalized between 0 and 1
    RGImage rgImage = BGR_to_RG( inputImage );

    // compute the histogram roi (mask) and the histogram of the image
    // this function will call computeMask() (have to be implemented properly)
    // and computeHistogram() with the given number for bins and the maximum
    // image value
    computeMaskAndHistogram( rgImage, mBins, 1.0 );

    // Call the reference implementation
    // Remove or comment the following line to make use of your own implementation!!
    //return CS_RG::segmentImage( rgImage, mHistogram, mThreshold );

    uint8 r,g;
    double bin_size = 1.01 / mBins;

    // process every single pixel of the input image to generate the segmented image
    for (int y=0; y < inputImage.height(); y++) 
    {
      // loop along y
      for (int x=0; x < inputImage.width(); x++) 
      {
        // loop along x

        // obtain rg values of current pixel
        r = rgImage(x,y)[0] / bin_size;
        g = rgImage(x,y)[1] / bin_size;

        // "compute" segmentation for current pixel
        // this segmentation have to be replaced
        // by a more sophisticated one
        (*segmentation_)(x,y) = mHistogram.at(r,g) > mThreshold ? 255 : 0;
      }
    }

    return debug_ ? debugPlot() : *segmentation_;
  }

  GrayImage& debugPlot()
  {
    if (debug_plot_ == NULL)
    {
      debug_plot_ = new GrayImage(segmentation_->width(), segmentation_->height());
    }

    return *debug_plot_;
  } 

  /**
   * @brief Generate the mask for the computation of the image histogram
   * You have to generate a mask image with the given size and set the
   * pixel values of the image to 255 if a pixel should belong the the mask
   * and to 0 otherwise.
   * @param[in] width width of the mask image that should be returned
   * @param[in] height height of the mask image that should be returned
   */
  GrayImage computeMask( int width, int height )
  {
    // generate the mask image
    GrayImage mask( width, height );
    // set all values to zero
    mask.clear();

    // slope of the trapezoid
    float slope = (float)mask_height_/(mask_width_bottom_ - mask_width_top_) * 2.0f;

    // start of the left edge
    uint16 slope_start = (width - mask_width_bottom_) / 2.0f;
    // end of the right edge
    uint16 slope_end = (width - slope_start);

    for (uint16 y = 1; y <= mask_height_; y++)
    {
      // start and stop values for x are derived from the following expressions
      // left slope: y <= slope*(x - slope_start)
      // right slope: y <= slope*(slope_end - x)
      for (uint16 x = y/slope + slope_start; x <= -y/slope + slope_end; x++)
      {
        mask(x - 1, height - y) = 255;
      }
    }

    return mask;
  }

  /**
   * @brief Convert the given BGR image to a RG Image
   */
  RGImage BGR_to_RG( BGRImage const &srcImage )
  {
    // Call the reference implementation
    // Remove or comment the following line to make use of your own implementation!!
    // return CS_RG::BGR_to_RG( srcImage );

    // create new image
    RGImage rgImage( srcImage.width(), srcImage.height() );

    for (uint x = 0; x < rgImage.width(); x++)
    {
      for (uint y = 0; y < rgImage.height(); y++)
      {
        float r,g,b;

        b = (float)srcImage(x,y)[0]/255;
        g = (float)srcImage(x,y)[1]/255;
        r = (float)srcImage(x,y)[2]/255;

        rgImage(x,y)[0] = (double)r /(r+g+b);
        rgImage(x,y)[1] = (double)g /(r+g+b);
      }
    }

    // access bgr values of srcImage using: srcImage(x,y)[0], srcImage(x,y)[1] and srcImage(x,y)[2]
    // bgr values are of type uint8 (char)
    // assign rg values to rgImage using: rgImage(x, y)[0] = <r-value>, rgImage(x, y)[1] = <g-value>
    // rg values are of type float

    return rgImage;
  }

  /**
   * @brief Generate the image histogram for the given image and mask
   * You should cycle through the image and build the histogram.
   * @param[in] srcImage The converted camera image (RG)
   * @param[in] mask The mask image computed in the computeMask() function
   * @param[in] nrOfBins The number of bins for each dimension of the histogram
   * @param[in] maxImgValue The maximum value that can occur in a single channel of
   * 			  the source image (e.g. 255 for RGB and 1.0 for RG)
   */
  virtual Histogram2D computeHistogram(
          RGImage const & srcImage,
          GrayImage& mask,
          int nrOfBins,
          float maxImgValue )
  {
    Histogram2D hist( nrOfBins );

    double bin_size = 1.01 / nrOfBins;

    // cycle through every single pixel of the source image to generate the histogram
    for (int y=0; y < srcImage.height(); ++y)
    {
      for (int x=0; x < srcImage.width(); ++x)
      {
        // regard the mask
        if ( mask(x,y) > 0 )
        {
          // obtain b,g,r values of image and compute
          // the associated bin in the histogram
          uint8 r,g;

          r = srcImage(x,y)[0]/bin_size;
          g = srcImage(x,y)[1]/bin_size;
          // transform b,g,r values to associated bins!
          // To get the number of bins of the histogram (equal for each dimension)
          // you can use mHistogram.bins()

          // increase the count for the histogram bin value
          hist.at(r,g) += 1;
        }
      }
    }

    // normalize the histogram value to be in range 0..1
    // you do not need to modify the following lines since
    // they already implement a valid normalization.
    double maxVal;
    cv::minMaxIdx( cv::InputArray(hist), 0, &maxVal );

    if ( maxVal > 0 ) {
      double sum = cv::sum( cv::InputArray(hist) )(0);
      if ( sum > 0 )
        hist /= sum;
    }

    if (last_hist_ == NULL)
    {
      last_hist_ = new Histogram2D(nrOfBins);
      *last_hist_ += hist;
    }

    *last_hist_ *= alpha_hist_;
    *last_hist_ += (1-alpha_hist_)*hist;

    return *last_hist_;
  }

  /**
   * @brief Calculate drive command depending on given segmentation
   * This function should calculate a reasonable drive command (steering angle,
   * and velocity) depending on the given segmentation.
   * @param[in] segmentedImage Image with the ground/obstacle segmentation
   * @return drive command as velocity
   */
  mira::Velocity2 getDriveCommand( GrayImage const& segmentedImage )
  {
    if (segmentation_ == NULL)
    {
      return mira::Velocity2(0,0,0);
    }

    // split the image into vertical columns
    uint8 num_stripes = 9;

    // distance histogram
    std::vector<double> dist(num_stripes);
    std::vector<double> dist_left(num_stripes);
    std::vector<double> dist_right(num_stripes);

    const GrayImage* img = debug_ ? segmentation_ : &segmentedImage;

    uint8 max_dist_index = cs_control::hist_calc_distances(
      dist,
      [img](uint16 x, uint16 y) { return (*img)(x,y); },
      segmentation_->width(),
      segmentation_->height(),
      max_stripe_fill_,
      0,0,
      // ignore the upper half of the image as distances cannot be greater than that anyway
      0.5, 0
    );

    std::vector<double> path(num_stripes);

    double path_end = std::max(dist[max_dist_index], 0.1);

    uint path_index = pursuit_dist_*(path.size() - 1);

    std::vector<bool> path_indexes(path.size());

    std::fill(path_indexes.begin(), path_indexes.end(), (debug_ && debug_hist_ == 4));

    // only calculate the root point and track point if not plotting
    path_indexes[0] = true;
    path_indexes[path_index] = true;

    // create a distance histogram from left to right (image flipped and inverted)
    cs_control::hist_calc_distances(
      dist_right,
      // flip and invert image
      [img](uint16 x, uint16 y) { return 255-(*img)(y,x); },
      segmentation_->height(),
      segmentation_->width(),
      max_stripe_fill_,
      // only build the histogram from the bottom to the max dist
      // using padding_left as image is flipped
      1.0 - path_end,
      0,0,0,
      &path_indexes
    );

    // create a distance histogram from right to left (image flipped and inverted)
    cs_control::hist_calc_distances(
      dist_left,
      // flip and invert image
      [img](uint16 x, uint16 y) { return 255-(*img)((*img).width() - y - 1,x); },
      segmentation_->height(),
      segmentation_->width(),
      max_stripe_fill_,
      1.0 - path_end,
      0,0,0,
      &path_indexes
    );

    cs_control::calc_pursuit_point(
      path,
      max_dist_index,
      dist, dist_left, dist_right
    );

    double pursuit_x = low_pass_pursuit_.feed(path[path_index]);
    double pursuit_y = ((double)path_index+0.5)/num_stripes * path_end;

    double track_error = std::atan2(pursuit_x - 0.5, pursuit_y);

    if(debug_)
    {
      cs_debug::draw_background(debug_plot_, segmentation_);

      for (uint i = 0; i < num_stripes; i++)
      {
        double pos = ((double)i+0.5)/num_stripes * path_end;
        
        if (debug_hist_ == 1)
        {
          // bottom
          cs_debug::draw_point(debug_plot_, ((double)i+0.5)/num_stripes, dist[i]);
        }
        else if (debug_hist_ == 2)
        {
          // left
          cs_debug::draw_point(debug_plot_, dist_left[i], path_end - pos);
        }
        else if (debug_hist_ == 3)
        {
          // right
          cs_debug::draw_point(debug_plot_, 1.0 - dist_right[i], path_end - pos);
        }
        else if(debug_hist_ == 4)
        {
          cs_debug::draw_point(debug_plot_, path[i], pos, path_index == i ? 255 : 100);
        }
      }

      if (debug_hist_ == 5)
      {
        cs_debug::draw_point(debug_plot_, pursuit_x, pursuit_y);
      }
    }

    double angle = clamp(
      controller_steer_.feed(track_error + weight_offset_*(path[0] - 0.5)),
      steer_max_
    );

    double total_distances = 0;

    for (auto d : dist)
    {
      total_distances += d;
    }

    double throttle =
      max_speed_ +
      controller_throttle_dist_.feed(
        std::min(dist.at(dist.size() / 2), min_dist_) - min_dist_
      ) +
      controller_throttle_error_.feed(
        std::abs(track_error)
      ) +
      controller_throttle_offset_.feed(
        std::abs(path[0] - 0.5) // absolute offset of the path
      )
    ;

    // target point is probably out of sight
    if (max_dist_index == 0 || max_dist_index == num_stripes-1)
    {
      throttle = std::min(throttle, turning_speed_);
    }

    auto end = std::chrono::system_clock::now();

    if (controller_steer_.dt() > PID_BASE_TIME*2)
    {
      throttle = 0;
    }

    return mira::Velocity2(
      std::max(std::min(low_pass_throttle_.feed(throttle), max_speed_), 0.0),
      0,
      angle
    );
  }

  /**
   * @brief You can use the reflect method to add members of your class
   * that are accessible in the visualization.
   */
  template <typename Reflector>
  void reflect( Reflector& r ) {
    ColorSegmentationTemplate::reflect( r );

    r.property( "threshold", mThreshold, "", 0.1, PropertyHints::limits(0, 1) );
    r.property( "bins", mBins, "", 9, PropertyHints::minimum(1) );
    r.property( "mask_height", mask_height_, "", 20, PropertyHints::limits(0, 100) );
    r.property( "mask_width_top", mask_width_top_, "", 30, PropertyHints::limits(0, 140) );
    r.property( "mask_width_bottom", mask_width_bottom_, "", 90, PropertyHints::limits(0, 200) );
    r.property( "alpha_hist", alpha_hist_, "", 0.98, PropertyHints::limits(0, 1) );
    
    r.property( "max_stripe_fill", max_stripe_fill_, "", 0.05, PropertyHints::limits(0.0, 1.0) );

    // r.property( "speed", speed_, "", 1.4);
    r.property("max_speed", max_speed_, "", 2.8);
    r.property("turning_speed", turning_speed_, "", 0.8);

    
    // r.property( "alpha_angle", alpha_angle_, "", 0.1, PropertyHints::limits(0, 1) );
    
    r.property( "alpha_pursuit", low_pass_pursuit_.alpha_, "", 0.0);

    r.property( "debug_hist", debug_hist_, "", 4);
    r.property( "debug", debug_, "", false);
    
    r.property( "lookahead", pursuit_dist_, "", 0.5, PropertyHints::limits(0.0, 1.0));
    
    r.property( "steer_max", steer_max_, "", 0.8);
    r.property( "weight offset", weight_offset_, "", 730);
    r.property( "P steer", controller_steer_.p_, "", 1.3);
    r.property( "I steer", controller_steer_.i_, "", 0.0);
    r.property( "D steer", controller_steer_.d_, "", 0.9);

    r.property( "min distance", min_dist_, "", 0.45);
    r.property( "alpha_throttle", low_pass_throttle_.alpha_, "", 0.8, PropertyHints::limits(0, 1) );
    
    r.property( "P thr. error", controller_throttle_error_.p_, "", 3.0);
    r.property( "I thr. error", controller_throttle_error_.i_, "", 0.0);
    r.property( "D thr. error", controller_throttle_error_.d_, "", 7.3);

    r.property( "P thr. offset", controller_throttle_offset_.p_, "", 4000);
    r.property( "I thr. offset", controller_throttle_offset_.i_, "", 0.0);
    r.property( "D thr. offset", controller_throttle_offset_.d_, "", 14000);

    r.property( "P thr. dist", controller_throttle_dist_.p_, "", -12);
    r.property( "I thr. dist", controller_throttle_dist_.i_, "", 0);
    r.property( "D thr. dist", controller_throttle_dist_.d_, "", -1200);
    
    // add your own member variables here
    // use
    // r.property( "ReadableNameOfVariable", <variable>, "Description", <default value>, <property hints> );
    // to add properties which can be edited. Use the Property hints to force the property to
    // stay within a certain range. E.g. you can use PropertyHints::limits(lower,upper) or PropertyHints::minimum(minimum)
    // And use
    // r.roproperty( "ReadableNameOfVariable", <variable>, "Description", <default value> );
    // to add read only members.

    // e.g. you can add a member variable which defines to height and width of your roi
  }

public:
  float mThreshold;
  uint16 mBins;

  // Here you can add member variables to your object
  // Please choose from one of the following types:
  // uint16 - unsigned integer in range of 0 .. 65535
  // float - signed floating point value
  // e.g. you can create a floating point variable b
  // with the following line:
  // float b;
};

MIRA_CLASS_SERIALIZATION( ColorSegmentationRG_58941, student::ColorSegmentationTemplateRG );

///////////////////////////////////////////////////////////////////////////////
