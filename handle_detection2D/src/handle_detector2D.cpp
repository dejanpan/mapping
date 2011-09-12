#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>

using namespace cv;
using namespace std;

void help()
{
	cerr << "\nThis program demonstrates line finding with the Hough transform.\n"
    "Call:\n"
    "./houghlines <img1 img2 ...>" << endl;
}

float lineLength2D (int x1, int y1, int x2, int y2)
{
  float dist  = sqrt ((x1-x2) * (x1-x2) + (y1-y2) * (y1-y2));
  return dist;
}

int main(int argc, char** argv)
{

  if (argc == 1)
  {
    help();
    exit(2);
  }
  for (int image = 1; image < argc; image++)
  {
    const char* filename = argv[image];
    
    Mat src = imread(filename, 0);
    if(src.empty())
    {
      cerr << "Image %s " << filename << "read failed" << endl;
    }

    int min_line_length = 120;
    int max_line_gap = 10;
    int dilate_iter = 8;
    int erode_iter = 2;
  
    Mat dst, cdst, dilated, eroded, dilated2, harris;
    Canny(src, dst, 50, 200, 3);
    dilate(dst, dilated, Mat(), Point(-1,-1), dilate_iter);
    //  dilate(dst, dilated, Mat(), Point(-1,-1), 5);
    erode(dilated, eroded, Mat(), Point(-1,-1), erode_iter);
  
    //for visualization
    cvtColor(eroded, cdst, CV_GRAY2BGR);

    //harris.create(eroded.rows, eroded.cols, CV_32FC1);
    harris = Mat::zeros (eroded.rows, eroded.cols, CV_32FC1);
    cornerHarris(eroded, harris, 7, 3, 0.1);
    for (int i = 0; i < harris.rows; i++)
    {
      for (int j = 0; j < harris.cols; j++)
      {
        if (harris.at <float> (i, j) > 0.02)
        {
          cerr << i << "," << j << endl;
          //draw circle
          circle(cdst, cvPoint(j,  i), 5, cvScalar(0, 255, 0));
        }
      }
      //    cerr << endl;
    } 

    double * min_r = new double;
    double * max_r = new double;
    minMaxLoc (harris, min_r, max_r);

    cerr << "min: " << *min_r << " max: " << *max_r << endl;
    delete min_r;
    delete max_r;

    vector<Vec4i> lines;
    HoughLinesP(eroded, lines, 1, CV_PI/180, 50, min_line_length, max_line_gap );
    for( size_t i = 0; i < lines.size(); i++ )
    {
      Vec4i l = lines[i];
      float length = lineLength2D (l[0], l[1], l[2], l[3]);
      float angle = atan2 (l[3] - l[1], l[2] - l[0]) * 180 / CV_PI;
      if (length < 180 && (((80.0 < angle) && (angle < 110.0)) || ((-1.0 < angle) && (angle < 1.0))))
      {
        cv::Scalar color(std::rand() % 256, std::rand() % 256, std::rand() % 256);
        line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), color, 3, CV_AA);
        //TODO:
        //calculate orientation and check for it
        cerr << "angle: " << angle << endl;
        cerr << "length: " << length << endl;
      }
    }

    //  imshow("source", src);
    imshow("source", dilated);
    imshow("detected lines", cdst);
    //  imshow("detected lines", dilated);

    
    waitKey();
  }
  return 0;
}

