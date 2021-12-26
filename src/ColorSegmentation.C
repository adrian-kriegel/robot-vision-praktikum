
#include <ColorSegmentationTemplate.h>
#include <ReferenceCS.h>
#include <GDBPrint.h>

#include <factory/Factory.h>
#include <serialization/Serialization.h>
#include <serialization/DefaultInitializer.h>
#include <image/Img.h>
#include <array>

using namespace std;
using namespace mira;
using namespace rv;
using namespace student;

#ifndef NULL
#define NULL   ((void *) 0)
#endif

#define PI 3.14

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
  float alpha_hist_segment_;

  Histogram3D* last_hist_;
  GrayImage* last_segmentation_;

  // max. percentage of black pixels to count before an average distance is reported
  float max_stripe_fill_;

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
  }

  ~ColorSegmentation_AdrianKriegel()
  {
    if(last_hist_ != NULL ) delete last_hist_;
    if(last_segmentation_ != NULL) delete last_segmentation_;
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
    // create new segmented image (do not change)
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

        // "compute" segmentation for current pixel
        // this segmentation have to be replaced
        // by a more sophisticated one
        if (hist_val >= mThreshold)
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
    uint8 num_stripes = 7;

    uint16 distances[] = { 0,0,0,0,0,0,0 };

    // where to start counting
    uint16 offsets[] = { 0, 15, 25, 30, 25, 15, 0 };

    uint8 max_distance_index;
    uint16 max_distance = 0;

    uint16 stripe_width = segmentedImage.width() / num_stripes;

    uint16 start;

    // how many black pixels have been counted 
    uint16 counter;
    uint16 max_stripe_counter = stripe_width*segmentedImage.height()*max_stripe_fill_;

    // sum of all counted distances per stripe
    uint16 sum_distances;

    for (uint8 i = 0; i < num_stripes; i++)
    {
      start = i*stripe_width;
      counter = 0;
      sum_distances = 0;

      // iterate from bottom to top
      for (int y = segmentedImage.height() - 1 - offsets[i]; y >= 0; y--)
      {
        // iterate through the stripe left to right
        for (uint16 x = start; x < start+stripe_width; x++)
        {
          if (segmentedImage(x,y) < 255)
          {
            counter++;
            sum_distances += y;
          }
        }

        if (counter >= max_stripe_counter)
        {
          distances[i] = sum_distances / counter;

          if (distances[i] > max_distance)
          {
            max_distance = distances[i];
            max_distance_index = i;
          }

          break;
        }
      }
    }

    double max_angle = PI/3;

    // set throttle and steering angle depending on the segmented image
    double throttle = (double)max_distance / 600.0;	// range 0 - 1 suggested
    double angle = (float)(max_distance_index - 3) * max_angle/num_stripes;

    // pass the velocity command to the robot
    // the second value should be zero since the robot cannot
    // move sideways while looking forward
    return mira::Velocity2( throttle, 0, angle);
  }

  /**
   * @brief You can use the reflect method to add members of your class
   * that are accessible in the visualization.
   */
  template <typename Reflector>
  void reflect( Reflector& r ) {
    ColorSegmentationTemplate::reflect( r );

    r.property( "threshold", mThreshold, "", 0.2, PropertyHints::limits(0, 1) );
    r.property( "bins", mBins, "", 6, PropertyHints::minimum(1) );
    r.property( "interpolate_histogram", interpolate_histogram_, "", false);
    r.property( "mask_height", mask_height_, "", 30, PropertyHints::limits(0, 100) );
    r.property( "mask_width_top", mask_width_top_, "", 90, PropertyHints::limits(0, 140) );
    r.property( "mask_width_bottom", mask_width_bottom_, "", 200, PropertyHints::limits(0, 200) );
    r.property( "alpha", alpha_hist_, "", 0.5, PropertyHints::limits(0, 200) );
    r.property( "max_stripe_fill", max_stripe_fill_, "", 0.04, PropertyHints::limits(1, 300) );


    delete last_hist_;
    last_hist_ = NULL;

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

MIRA_CLASS_SERIALIZATION(ColorSegmentation_AdrianKriegel, student::ColorSegmentationTemplateRGB );

///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////

