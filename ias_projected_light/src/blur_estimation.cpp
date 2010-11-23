/* 
 * Copyright (c) 2010, Florian Zacherl <Florian.Zacherl1860@mytum.de>, Dejan Pangercic <dejan.pangercic@cs.tum.edu>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Intelligent Autonomous Systems Group/
 *       Technische Universitaet Muenchen nor the names of its contributors 
 *       may be used to endorse or promote products derived from this software 
 *       without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <math.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <cv_bridge/CvBridge.h>

#include "ias_projected_light/cp.h"

/*If TEST_MODE is defined a test image is produced and published instead of the normal one
 * If NORMALIZED is defined to, it's the normalized image with non-roi illuminated pixels in green
 * Otherwise it shows the vertical derivations
 */

//#define TEST_MODE
//#define NORMALIZED

using namespace std;

class BlurEst
{
protected:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Subscriber sub_roi;
  ros::Publisher pub;
  ros::ServiceClient cl;
  sensor_msgs::CvBridge bridge;

  IplImage* white_image;
  IplImage* gray_image;

  string input_topic;
  string roi_input_topic;
  string output_topic;

  CvRect roi;
  bool roi_set;

  float max_blur;
  bool blocks;
  const static int threshold = 10;

  int pattern_number; //0 = white pattern, 1 = black pattern, 2 = normal pattern

  /*How long to wait while switching through the patterns
   * Should be be bigger than publishing delay of the image topic
   */
  const static int wait_time = 1000;

  void changeROI(const sensor_msgs::RegionOfInterestConstPtr& r)
  {
    CvRect roi_new;
    roi_new.x = r->x_offset;
    roi_new.y = r->y_offset;
    roi_new.width = r->width;
    roi_new.height = r->height;
    roi = roi_new;

    roi_set = true;
  }

  void detect_sharp_area(const sensor_msgs::ImageConstPtr& ms)
  {
    if (!roi_set)
      return;


    switch (pattern_number)
    {
      //Project white image:
      case 0:
      {
        ias_projected_light::cp srv;
        srv.request.pattern = 1;
        srv.request.block_size = 1;
        while (ros::ok() && !cl.call(srv))
        {
          ROS_INFO("Failed to show white pattern!");
          cvWaitKey(50);
        }
        ROS_INFO("%s pattern ready!", srv.response.name.c_str());
        cvWaitKey(wait_time);
      }
        break;

        //Save white image and project black image:
      case 1:
      {
        IplImage* image = bridge.imgMsgToCv(ms);
        white_image = cvCloneImage(image);

        ias_projected_light::cp srv;
        srv.request.pattern = 0;
        srv.request.block_size = 1;
        while (ros::ok() && !cl.call(srv))
        {
          ROS_INFO("Failed to show black pattern!");
          cvWaitKey(50);
        }

        ROS_INFO("%s pattern ready!", srv.response.name.c_str());
        cvWaitKey(wait_time);
      }
        break;

        //Save black image, project stripe/block pattern and compute ROI out of black and white image
      case 2:
      {
        IplImage* image = bridge.imgMsgToCv(ms);
        gray_image = cvCloneImage(image);

        ias_projected_light::cp srv;
        if (blocks)
        {
          srv.request.pattern = 2;
        }
        else
        {
          srv.request.pattern = 3;
        }
        srv.request.block_size = 40;

        while (ros::ok() && !cl.call(srv))
        {
          ROS_INFO("Failed to show %s pattern", (blocks ? "block" : "stripe"));
          cvWaitKey(50);
        }
        ROS_INFO("%s pattern ready!", srv.response.name.c_str());

        cvSetImageROI(white_image, roi);
        cvSetImageROI(gray_image, roi);
      }
        break;

        //Blur Estimation:
      default:
      {
        //Original image (without color):
        IplImage* image = bridge.imgMsgToCv(ms);

        //Color image to accent Region of Interest and sharp edges in color:
        IplImage* col = cvCreateImage(cvSize(image->width, image->height), IPL_DEPTH_8U, 4);
        cvCvtColor(image, col, CV_GRAY2BGR);

        cvSetImageROI(image, roi);
        cvSetImageROI(col, roi);

        unsigned int width = roi.width;
        unsigned int height = roi.height;

        //Normalized image and derivations in both directions:
        float norm[height][width];
        float derv[height][width];
        float hderv[height][width];

        for (unsigned int y = 0; y < height; y++)
        {
          for (unsigned int x = 0; x < width; x++)
          {
            //Compute normalized image:
            float c = cvGet2D(image, y, x).val[0] - cvGet2D(gray_image, y, x).val[0];
            float d = cvGet2D(white_image, y, x).val[0] - cvGet2D(gray_image, y, x).val[0];

            //NaN-value for pixels that are in the ROI but NOT illuminated:
            if (d < threshold)
            {
              norm[y][x] = numeric_limits<float>::quiet_NaN();
            }
            else
            {
              norm[y][x] = c / d;

              //Values should be between 0 and 1, all others are measurement errors:
              if (norm[y][x] < 0)
                norm[y][x] = 0;
              if (norm[y][x] > 1)
                norm[y][x] = 1;
            }

            //No derivations possible in the first row
            if (y == 0)
            {
              derv[y][x] = 0;
            }
            //No dervations for non-illuminated pixels:
            else if (isnan(norm[y][x]) || isnan(norm[y - 1][x]))
            {
              derv[y][x] = 0;
            }
            else
            {
              //Compute first derivative in vertical direction
              derv[y][x] = abs(norm[y][x] - norm[y - 1][x]);
            }
            if (blocks) //Block pattern
            {
              //No derivations possible in the first column
              if (x == 0)
              {
                hderv[y][x] = 0;
              }
              //No derivations for non-illuminated pixels:
              else if (isnan(norm[y][x]) || isnan(norm[y ][x - 1]))
              {
                hderv[y][x] = 0;
              }
              else
              {
                //Compute first derivative in horizontal direction
                hderv[y][x] = abs(norm[y][x] - norm[y][x - 1]);
              }

            }
          }
        }

#ifdef TEST_MODE
        IplImage* test = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);

        for (unsigned int y = 0; y < height; y++)
        {
          for (unsigned int x = 0; x < width; x++)
          {
#ifdef NORMALIZED
            if (isnan(norm[y][x]))
            {
              cvSet2D(test, y, x, cvScalar(0, 255, 0));
            }
            else
            {
              cvSet2D(test, y, x, cvScalar(norm[y][x] * 255, norm[y][x] * 255, norm[y][x] * 255));
            }
#else
            cvSet2D(test, y, x, cvScalar(derv[y][x] * 255,derv[y][x] * 255,derv[y][x] * 255));
#endif
          }
        }
#endif

        float blur[height][width];
        float hblur[height][width];

        //Blur in vertical direction:
        for (unsigned int x = 0; x < width; x++)
        {
          for (unsigned int y = 0; y < height; y++)
          {
            blur[y][x] = 1 / (derv[y][x] * sqrt(2 * M_PI));

            //Edge is sharp:
            if (blur[y][x] < max_blur)
            {
              cvSet2D(col, y, x, cvScalar(255, 0, 0, 0));
            }
          }

          if (blocks)
          {
            //Blur in horzontal direction:
            for (unsigned int x = 0; x < width; x++)
            {
              for (unsigned int y = 0; y < height; y++)
              {
                hblur[y][x] = 1 / (hderv[y][x] * sqrt(2 * M_PI));

                //Edge is sharp:
                if (hblur[y][x] < max_blur)
                  cvSet2D(col, y, x, cvScalar(0, 0, 255, 0));
              }
            }
          }
        }

        cvResetImageROI(col);
        cvRectangle(col, cvPoint(roi.x, roi.y), cvPoint(roi.x + roi.width, roi.y + roi.height), cvScalar(0, 255, 0), 2);

#ifdef TEST_MODE
        pub.publish(bridge.cvToImgMsg(test));
        cvReleaseImage(&test);
#else
        pub.publish(bridge.cvToImgMsg(col));
#endif

        ROS_INFO("Image published on topic  %s", output_topic.c_str());

        cvReleaseImage(&col);
      }
        break;
    }

    if (pattern_number < 3)
      pattern_number++;

  }

public:

  BlurEst(ros::NodeHandle& n, float m_blur, bool bls, string input_topic, string roi_topic) :
    nh(n), max_blur(m_blur),  input_topic(input_topic), output_topic("/sharp_region"), pattern_number(0),roi_set(false),roi_input_topic(roi_topic), blocks(bls)
  {
    sub = nh.subscribe(input_topic, 1, &BlurEst::detect_sharp_area, this);
    sub_roi = nh.subscribe(roi_input_topic, 1, &BlurEst::changeROI, this);
    pub = nh.advertise<sensor_msgs::Image> (output_topic, 1);
    cl = nh.serviceClient<ias_projected_light::cp> ("change_pattern");
    ROS_INFO("Parameter: %f , %s", max_blur, (blocks ? "true" : "false"));
  }

  ~BlurEst()
  {
    cvReleaseImage(&white_image);
    cvReleaseImage(&gray_image);
  }
};

int main(int argc, char** argv)
{
  if (argc < 4)
  {
    ROS_INFO(
             "This function expects three parameters:\n\t-Float value for the maximal blur value that should be treated as sharp (e.g. 1.0)\n\t-Bool value that declares if a block or a stripe pattern is used (Zero is stripe pattern)\n\t-Image input topic");
  }
  else
  {
    string roi_topic = "stereo/left/roi";
    if(argc > 4)
    {
      roi_topic = argv[4];
    }
    ros::init(argc, argv, "blur_estimation");
    ros::NodeHandle n;
    BlurEst be(n, atof(argv[1]), atoi(argv[2]), argv[3], roi_topic);
    ros::spin();
  }
}

