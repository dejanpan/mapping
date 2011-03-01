#include <highgui.h>
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "features_2d/features_2d.h"
#include <iostream>

using namespace cv;
using namespace std;
namespace f2d = features_2d;

#define DRAW_RICH_KEYPOINTS_MODE     0
#define DRAW_OUTLIERS_MODE           0

const string winName = "correspondences";

//Display function - DO NOT MODIFY
void drawMatchesOnly(const cv::Mat& image1, const std::vector<cv::KeyPoint>& keypoints1,
                 const cv::Mat& image2, const std::vector<cv::KeyPoint>& keypoints2,
                 const std::vector<f2d::Match>& matches, cv::Mat& display)
{
  // Set up composite image
  display = cv::Mat::zeros(std::max(image1.rows, image2.rows), image1.cols + image2.cols, CV_8UC3);
  cv::Mat sub_display1 = display( cv::Rect(0, 0, image1.cols, image1.rows) );
  cv::Mat sub_display2 = display( cv::Rect(image1.cols, 0, image2.cols, image2.rows) );
  
  cvtColor(image1, sub_display1, CV_GRAY2BGR);
  cvtColor(image2, sub_display2, CV_GRAY2BGR);

  // Draw lines between matches
  int shift_bits = 4;
  int multiplier = 1 << shift_bits;
  for (std::vector<f2d::Match>::const_iterator i = matches.begin(), ie = matches.end(); i != ie; ++i) {
    const cv::KeyPoint& keypt1 = keypoints1[i->index1];
    const cv::KeyPoint& keypt2 = keypoints2[i->index2];
    cv::Point center1(keypt1.pt.x * multiplier, keypt1.pt.y * multiplier);
    cv::Point center2((keypt2.pt.x + image1.cols) * multiplier, keypt2.pt.y * multiplier);
    cv::Scalar color(std::rand() % 256, std::rand() % 256, std::rand() % 256);
    cv::line(display, center1, center2, color, 1, CV_AA, shift_bits);
  }
}

int main(int argc, char** argv)
{
    if( argc != 4 )
    {
        cout << "Format:" << endl;
        cout << argv[0] << " [image1] [image2] [ransacReprojThreshold]" << endl;

        return -1;
    }
    double ransacReprojThreshold = atof(argv[3]);

    cout << "< Creating detector, descriptor extractor and descriptor matcher ..." << endl;
    //detector, descriptor and matcher objects
    f2d::SurfFeatureDetector detector;
    f2d::SurfDescriptorExtractor descriptorExtractor;
    f2d::BruteForceMatcher<f2d::L2<float> > descriptorMatcher;

    /*****************************************************
     *TODO 1: read in img1 and img1 from the command line
     */




    
    /*****************************************************
     *TODO 2: a)Extract keypoints and b)compute SURF descriptor for both images
     * For a) use "detector" object and detect function: https://code.ros.org/svn/wg-ros-pkg/branches/trunk_cturtle/stacks/visual_feature_detectors/features_2d/include/features_2d/detector.h
     * For b) use descriptorExtractor object and compute function: https://code.ros.org/svn/wg-ros-pkg/branches/trunk_cturtle/stacks/visual_feature_detectors/features_2d/include/features_2d/descriptor.h
     */




    
    /*****************************************************
     *TODO 3: Find matches in both images
     * Use descriptorMatcher object and matchWindowless function: https://code.ros.org/svn/wg-ros-pkg/branches/trunk_cturtle/stacks/visual_feature_detectors/features_2d/include/features_2d/matcher.h
     */
    vector<f2d::Match> matches;





    //push matching keypoints coordinates in points1, points2 structures
    vector<Point2f> points1, points2;
    for(size_t i = 0; i < matches.size(); i++)
    {
    	points1.push_back(keypoints1[matches[i].index1].pt);
    	points2.push_back(keypoints2[matches[i].index2].pt);
    }
    /*****************************************************
     *TODO 4: Find homography between the set of both matches
     * Use findHomography function from openCV
     */

    //estimate projection matrix
    Mat points1Projected;
    /*****************************************************
     *TODO 5: What does perspectiveTransform function do?
     */
    perspectiveTransform(Mat(points1), points1Projected, H);
    

    
    /*****************************************************
     *TODO : Filter out bad matches using ransacReprojThreshold
     * to do so iterate over points1 and check if the normalized distance 
     * between original and projected points is less than ransacReprojThreshold
     */
    vector<f2d::Match> matchesFiltered;
    for(size_t i = 0; i < points1.size(); i++)
    {
      /*todo*/
      if(/*todo*/)
    	{
    		matchesFiltered.push_back(matches[i]);
    	}
    }


    //draw matches
    namedWindow(winName, 1);
    Mat drawImg;
    drawMatchesOnly(img1, keypoints1, img2, keypoints2, matchesFiltered, drawImg);
    imshow(winName, drawImg);
    waitKey(0);

    return 0;
}
