
#include <OpticalFlowTemplate.h>
#include <ReferenceOF.h>
#include <GDBPrint.h>

#include <factory/Factory.h>
#include <serialization/Serialization.h>
#include <serialization/DefaultInitializer.h>
#include <image/Img.h>

using namespace std;
using namespace mira;
using namespace student;

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
		return OF::interestOperator( oldImage, mThreshold );

		// create a vector of features which will be returned to the caller
		vector<InterestPoint> interestPoints;

		// Please refer to the official documentation of Eigen if you do have
		// any questions on how to use the matrix class:
		// http://eigen.tuxfamily.org/dox/group__QuickRefPage.html

		// cycle through the image with a step width of 20 pixels
		// Of course, this is quite useless and should be replaced.
		for (int y = 0; y < oldImage.rows(); y+=20) {
			for (int x = 0; x < oldImage.cols(); x+=20) {
				// Hints:
				// - you can extract a "patch" from the image using the block function:
				//     Eigen::MatrixXf patch = oldImage.block(y,x,nrOfrows,nrOfcolumns);
				//     see: http://eigen.tuxfamily.org/dox/group__TutorialBlockOperations.html
				// - you can subtract two matrixes (also in combination with the block function):
				//     Eigen::MatrixXf diff = patch - oldImage.block(y,x,nrOfrows,nrOfcolumns);
				// - you can compute the squared norm of a matrix using the following function:
				//     float norm = diff.squaredNorm();

				// append the new feature point to the vector of features
				interestPoints.push_back( InterestPoint(x,y) );
			}
		}

		// return the vector of feature points
		return interestPoints;
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
		// Call the reference implementation
		// Remove or comment the following line to make use of your own implementation!!
		return OF::calculateOpticalFlow( oldImage, newImage, interestPoints, mSearchWin, mCorrWin );

		// create empty vector of optical flow vectors which will be return to the caller
		vector<OFVector> OFVectors;

		// process every Interest point given in the vector
		foreach ( InterestPoint const& p, interestPoints ) {
			// The following lines provide a rough guideline on how to
			// implement image patch correlation

			// you can omit points that are so close to the image boundary that the
			// correlation window does not fit. e.g. by
			// if ( p.x()-mCorrWin >= 0 && p.x()+mCorrWin < oldImage.cols()
			//	&& p.y()-mCorrWin >= 0 && p.y()+mCorrWin < oldImage.rows() ) {
			// After that you can extract the reference patch from the old image:
			// Eigen::MatrixXf origPatch = oldImage.block(
			//	p.y()-mCorrWin,
			//	p.x()-mCorrWin,
			//	1+mCorrWin*2,
			//	1+mCorrWin*2 );
			// Now you need to "find" the patch in the new image by comparing it
			// at every position in the search window
			// for ( int x=p.x()-mSearchWin; x < p.x()+mSearchWin;++x) {
			// 	for ( int y=p.y()-mSearchWin; y < p.y()+mSearchWin;++y) {
			// You need to make sure that you do not run out of the image boundaries
			// by extracting the image patches in the new image!
			// Compute the difference between the two patch positions:
			// Eigen::MatrixXf diff = newImage.block(y-mCorrWin,x-mCorrWin,1+mCorrWin*2,1+mCorrWin*2)-origPatch;
			// now you can compute e.g. the SAD value from the difference matrix
			// you need to find the minimum value and store the "displacement" values
			// if the SAD Values are low enough (and/or if the position can be found with high
			// confidence) you can add the optical flow vector to the list:

			// add a new optical flow vector at the position of the interest point
			// the direction of the vector will be zero
			OFVectors.push_back(
				OFVector(
					p.x(),  // x position
					p.y(),  // y position
					0,     // x direction
					0      // y direction
				)
			);
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
		return OF::getDriveCommand( OFVectors );

		// set throttle and steering angle depending on the segmented image
		double throttle=0.2; 	// range 0 - 1 suggested
		double angle=1;			// range -pi - +pi suggested


		// the OFVector structure consist of a position pos and a direction dir
		// Accessing the pos of a OFVector looks like:
		// Eigen::Vector2i position = p.pos;
		foreach( OFVector const& v, OFVectors ) {
			// you can obtain the position of the vector using:
			Eigen::Vector2i position = v.pos;
			// and the x and y values of this position by: position.x() and position.y()

			// you can access the direction of the vector using:
			Eigen::Vector2i direction = v.dir;
			// and the x and y components of the direction by: direction.x() and direction.y()
		}

		// pass the velocity command to the robot
		// the second value should be zero since the robot cannot
		// move sideways while looking forward
		return Velocity2( throttle, 0, angle );
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
		r.property( "threshold", mThreshold, "", 10 );
		r.property( "corrWin", mCorrWin, "size of the correlation window (multiplied by 2)", 5, PropertyHints::limits(1, 30) );
		r.property( "searchWin", mSearchWin, "size of the search window (multiplied by 2)", 15, PropertyHints::limits(1, 30) );
	}

public:
	int mThreshold;
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
