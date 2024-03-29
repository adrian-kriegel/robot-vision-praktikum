
#include <ColorSegmentationTemplate.h>
#include <ReferenceCS.h>
#include <GDBPrint.h>

#include <factory/Factory.h>
#include <serialization/Serialization.h>
#include <serialization/DefaultInitializer.h>
#include <image/Img.h>
#include <array>

#include "util.hpp"

using namespace std;
using namespace mira;
using namespace rv;
using namespace student;
using namespace util;

///////////////////////////////////////////////////////////////////////////////

// start implementing by typing "colorTemplate" and hitting
// CTRL+SPACE after the last letter. Select the type of implementation (RG or RGB)
// and press enter. Step through the required parameters (class name etc.) using
// the tab key.
class ColorSegmentation_AdrianKriegel : public ColorSegmentationTemplateRGB {
  MIRA_META_OBJECT( ColorSegmentation_AdrianKriegel,
      ("StudentNumber","58941")
      ("Course","RV")
      ("Name","Kriegel")
      ("FirstName","Adrian")
      ("Description","Description"));


  unsigned int mask_height_;
  unsigned int mask_width_top_;
  unsigned int mask_width_bottom_;

  bool interpolate_histogram_;

  float alpha_hist_;
  float alpha_segment_;

  Histogram3D* last_hist_;
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

public:

  ColorSegmentation_AdrianKriegel()
  {
    // the following macro initializes all variables stated in the reflect() function
    // to their default values
    MIRA_INITIALIZE_THIS;

    // do initialization stuff

    // you might want to initialize some class members etc.

    last_hist_ = NULL;
    last_segmentation_ = NULL;

    last_angle_ = 0.0;
  }

  ~ColorSegmentation_AdrianKriegel()
  {
    //if(last_hist_ != NULL ) delete last_hist_;
    //if(last_segmentation_ != NULL) delete last_segmentation_;
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

    bool init = last_segmentation_ == NULL;
    // create new segmented image (do not change)
    if(init)
    {
      last_segmentation_ = new GrayImage( inputImage.width(), inputImage.height() );
    }

    GrayImage segmentedImage( inputImage.width(), inputImage.height() );

    // compute the histogram roi (mask) and the histogram of the image
    // this function will call computeMask() (have to be implemented properly)
    // and computeHistogram() with the given number for bins and the maximum
    // image value (do not change)
    // Important note: the resulting histogram will be stored in the variable mHistogram
    computeMaskAndHistogram( inputImage, mBins, 255.0 );

    // Call the reference implementation
    // Remove or comment the following line to make use of your own implementation!!
    // return CS_RGB::segmentImage( inputImage, mHistogram, mThreshold );

    // exact rgb values
    float r,g,b;
    // floored rgb values for hisogram lookup
    uint8 rf,gf,bf;

    // scaling factor for rgb values
    float f = (256.0/mBins);

    float hist_val;

    // process every single pixel of the input image to generate the segmented image
    // please note that images are stored as BGR images (channel 0 -> B, channel 1 -> G, channel 2 ->R)
    for (int y=0; y < inputImage.height(); ++y) {
      // loop along y
      for (int x=0; x < inputImage.width(); ++x) {
        // loop along x

        // obtain BGR values of current pixel
        b = (float)inputImage(x,y)[0] / f;
        g = (float)inputImage(x,y)[1] / f;
        r = (float)inputImage(x,y)[2] / f;

        if (interpolate_histogram_)
        {
          rf = r;
          gf = g;
          bf = b;

          // TODO: trilinear interp.
        }
        else
        {
          hist_val = mHistogram.at(b, g, r);
        }

        if (init)
        {
          (*last_segmentation_)(x,y) = hist_val*255.0;
        }
        else
        {
          (*last_segmentation_)(x,y) *= alpha_segment_;
          (*last_segmentation_)(x,y) += (1-alpha_segment_)*hist_val*255.0;
        }

        if ((*last_segmentation_)(x,y) > mThreshold*255.0)
        {
          segmentedImage(x,y) = 255;
        }
        else 
        {
          segmentedImage(x,y) = 0;
        }

        // You may wish to access the histogram you have computed at this point
        // The histogram is stored in the variable mHistogram
        // To get the number of bins (equal for each dimension) you can use mHistogram.bins()
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
   * @brief Generate the image histogram for the given image and mask
   * You should cycle through the image and build the histogram.
   * @param[in] srcImage The 3-channel camera image (BGR)
   * @param[in] mask The mask image computed in the computeMask() function
   * @param[in] nrOfBins The number of bins for each dimension of the histogram
   * @param[in] maxImgValue The maximum value that can occur in a single channel of
   * 			  the source image (e.g. 255 for RGB and 1.0 for RG)
   */
  virtual Histogram3D computeHistogram(
          BGRImage const & srcImage,
          GrayImage& mask,
          int nrOfBins,
          float maxImgValue )
  {
    // Call the reference implementation
    // Remove or comment the following line to make use of your own implementation!!
    // return CS_RGB::computeHistogram( srcImage, mask, nrOfBins, maxImgValue );

    // generate a new histogram object
    // the histogram will have three dimensions with nrOfBins bins for
    // each dimension. The type of the histogram is float.
    Histogram3D hist( nrOfBins );

    double bin_size = (maxImgValue + 1) / nrOfBins;

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
          uint8 b,g,r;

          b = srcImage(x,y)[0]/bin_size;
          g = srcImage(x,y)[1]/bin_size;
          r = srcImage(x,y)[2]/bin_size;
          // transform b,g,r values to associated bins!
          // To get the number of bins of the histogram (equal for each dimension)
          // you can use mHistogram.bins()

          // increase the count for the histogram bin value
          hist.at(b,g,r) += 1;
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
      last_hist_ = new Histogram3D(nrOfBins);
      *last_hist_ += hist;
    }

    *last_hist_ *= alpha_hist_;
    *last_hist_ += (1-alpha_hist_)*hist;

    return *last_hist_;
  }


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
          density[i] += (*last_segmentation_)(x,y)/255.0/stripe_width/segmentedImage.width();
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

    double throttle = speed_*(total_density + density[2]);

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

    // pass the velocity command to the robot
    // the second value should be zero since the robot cannot
    // move sideways while looking forward
    return mira::Velocity2(
      std::max(throttle, 0.0),
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

    r.property( "threshold", mThreshold, "", 0.15, PropertyHints::limits(0, 1) );
    r.property( "bins", mBins, "", 6, PropertyHints::minimum(1) );
    r.property( "mask_height", mask_height_, "", 20, PropertyHints::limits(0, 100) );
    r.property( "mask_width_top", mask_width_top_, "", 30, PropertyHints::limits(0, 140) );
    r.property( "mask_width_bottom", mask_width_bottom_, "", 90, PropertyHints::limits(0, 200) );
    r.property( "alpha_hist", alpha_hist_, "", 0.6, PropertyHints::limits(0, 1) );
    r.property( "alpha_segment", alpha_segment_, "", 0.2, PropertyHints::limits(0, 1) );
    
    r.property( "max_stripe_fill", max_stripe_fill_, "", 0.6, PropertyHints::limits(0.0, 1.0) );

    r.property( "speed", speed_, "", 9);

    r.property( "alpha_angle", alpha_angle_, "", 0.1, PropertyHints::limits(0, 1) );
    r.property( "steer_linear", steer_linear_, "", 0.09);
    r.property( "steer_quadratic", steer_quadratic_, "", 0.002);
    r.property( "steer_max", steer_max_, "", 1.0);
    r.property( "avoid_walls_", avoid_walls_, "", 0.3);
    


    //if (last_hist_ != NULL) delete last_hist_;
    last_hist_ = NULL;
    last_segmentation_ = NULL;

    // add your own member variables here
    // use0.03
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

MIRA_CLASS_SERIALIZATION(ColorSegmentation_AdrianKriegel, student::ColorSegmentationTemplateRGB );

///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////

