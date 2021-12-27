
#include <ColorSegmentationTemplate.h>
#include <ReferenceCS.h>
#include <GDBPrint.h>

#include <factory/Factory.h>
#include <serialization/Serialization.h>
#include <serialization/DefaultInitializer.h>
#include <image/Img.h>

#include "util.hpp"

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
class ClassName_StudentNumber : public ColorSegmentationTemplateRG {

  // TODO: Add you personal and implementation details here
  MIRA_META_OBJECT( ClassName_StudentNumber,
      ("StudentNumber","StudentNumber")
      ("Course","Course")
      ("Name","Name")
      ("FirstName","FirstName")
      ("Description","Description"));

  

  unsigned int mask_height_;
  unsigned int mask_width_top_;
  unsigned int mask_width_bottom_;

  bool interpolate_histogram_;

  float alpha_hist_;
  float alpha_segment_;

  Histogram2D* last_hist_;
  GrayImage* last_segmentation_;

  // max. percentage of black pixels to count before an average distance is reported
  float max_stripe_fill_;

  float speed_;
  float steer_linear_;
  float steer_quadratic_;

  float steer_max_;

  float last_angle_;
  float alpha_angle_;

  float avoid_walls_;

  float last_throttle_;

public:

  ClassName_StudentNumber()
  {
    // the following macro initializes all variables stated in the reflect() function
    // to their default values
    MIRA_INITIALIZE_THIS;

    // do initialization stuff

    // you might want to initialize some class members etc.

    last_hist_ = NULL;
    last_segmentation_ = NULL;

    last_angle_ = 0.0;
    last_throttle_ = 0.0;
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
    GrayImage segmentedImage( inputImage.width(), inputImage.height() );

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
    double bin_size = 1.0 / mBins;

    // process every single pixel of the input image to generate the segmented image
    for (int y=0; y < inputImage.height(); ++y) {
      // loop along y
      for (int x=0; x < inputImage.width(); ++x) {
        // loop along x

        // obtain rg values of current pixel
        r = rgImage(x,y)[0] / bin_size;
        g = rgImage(x,y)[1] / bin_size;

        // "compute" segmentation for current pixel
        // this segmentation have to be replaced
        // by a more sophisticated one
        segmentedImage(x,y) = mHistogram.at(r,g) > mThreshold ? 255 : 0;
      }
    }

    return segmentedImage;
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
    return CS_RG::BGR_to_RG( srcImage );

    // create new image
    RGImage rgImage( srcImage.width(), srcImage.height() );

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

    double bin_size = 1.0 / nrOfBins;

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
    // split the image into vertical stripes
    uint8 num_stripes = 5;

    float density[] = { 0,0,0,0,0 };
    float total_density = 0.0;

    // where to start counting for each stripe
    uint16 offsets[] = { 15, 30, 30, 30, 15 };

    uint16 stripe_width = segmentedImage.width() / num_stripes;

    for (int i = 0; i < num_stripes; i++) 
    {
      for (int y = segmentedImage.height() - 1 - offsets[i]; y >= 0; y--)
      {
        for (int x = i*stripe_width; x < (i+1)*stripe_width; x++)
        {
          density[i] += segmentedImage(x,y)/255.0/stripe_width/segmentedImage.width();
          total_density += density[i];
        }
      }
    }

    total_density /= segmentedImage.width()*segmentedImage.height();

    double angle = 2.0*PI*((avoid_walls_*density[0]+density[1])-(density[3]+avoid_walls_*density[4]))/total_density;

    last_angle_ *= alpha_angle_;
    last_angle_ += angle*(1.0-alpha_angle_);

    double angle_full = steer_linear_ * last_angle_ +
      steer_quadratic_ * last_angle_*std::abs(last_angle_);

    double angle_clamped = clamp(
      angle_full,
      steer_max_
    );

    double throttle = speed_*(total_density);

    if (std::abs(angle_full) > 1.5*steer_max_)
    {
      throttle = 0.0;
    }


    // obstacle in the center
    if (
      density[2] < density[1] && 
      density[2] < density[3] &&
      std::abs(angle_clamped) < steer_max_/0.5
    )
    {
      angle_clamped = std::copysign(steer_max_/0.5, angle_clamped);
    }

    last_throttle_ *= 0.9;
    last_throttle_ += (0.1)*throttle;

    // pass the velocity command to the robot
    // the second value should be zero since the robot cannot
    // move sideways while looking forward
    return mira::Velocity2(
      std::max(last_throttle_, 0.0f),
      0,
      angle_clamped
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
    r.property( "alpha_hist", alpha_hist_, "", 0.6, PropertyHints::limits(0, 1) );
    r.property( "alpha_segment", alpha_segment_, "", 0.2, PropertyHints::limits(0, 1) );
    
    r.property( "max_stripe_fill", max_stripe_fill_, "", 0.6, PropertyHints::limits(0.0, 1.0) );

    r.property( "speed", speed_, "", 15);



    r.property( "alpha_angle", alpha_angle_, "", 0.1, PropertyHints::limits(0, 1) );
    r.property( "steer_linear", steer_linear_, "", 0.35);
    r.property( "steer_quadratic", steer_quadratic_, "", 0.025);
    r.property( "steer_max", steer_max_, "", 1.0);
    r.property( "avoid_walls_", avoid_walls_, "", 0.3);
    


    //if (last_hist_ != NULL) delete last_hist_;
    last_hist_ = NULL;
    last_segmentation_ = NULL;
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

MIRA_CLASS_SERIALIZATION( ClassName_StudentNumber, student::ColorSegmentationTemplateRG );

///////////////////////////////////////////////////////////////////////////////
