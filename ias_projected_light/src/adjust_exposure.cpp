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

#include <ros/ros.h>
#include <unistd.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <cv_bridge/CvBridge.h>
#include "ias_projected_light/cp.h"
#include <math.h>

//#include <find_roi.h>

using namespace std;

class AdjExposure
{
private:
  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Subscriber sub_roi;
  ros::ServiceClient cl;

  string prefix;
  string input_topic;
  string roi_input_topic;

  IplImage* image;
  IplImage* non_image; //Image without pattern

  CvRect roi;

  sensor_msgs::CvBridge bridge;

  int exposure_min, exposure_max, exposure;
  int mode;

  bool roi_set;

  //This constant equates to the average brightness reached by automatic exposure adjustion:
  const static int avg_brightness_auto = 120;

  //Pattern number (as in create_pattern), 4 is the random pattern:
  const static int pattern_number = 4;

  //Sets the camera exposure time to the value in exposure
  // Because a API for C++ doesn't exist, yet, this has to be done with a system call:
  void setExposure()
  {
    stringstream ss;
    ss << "rosrun dynamic_reconfigure dynparam set " << prefix << " exposure " << exposure;
    int s = system(ss.str().c_str());
    if (s != 0)
    {
      exit(s);
    }

  }

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

  void adjust(const sensor_msgs::ImageConstPtr& ms)
  {
    if (!roi_set)
      return;

    image = bridge.imgMsgToCv(ms);

    switch (mode)
    {
      case 0: //Project black pattern (no pattern)
      {
        ias_projected_light::cp srv;
        srv.request.pattern = 0;
        srv.request.block_size = 1;
        while (ros::ok() && !cl.call(srv))
        {
          ROS_INFO("Failed to show black pattern!");
          cvWaitKey(50);
        }
        ROS_INFO("Switched to black image!");
        sleep(1);
      }
        break;

      case 1: //Save image without pattern and project the real pattern:
      {
        non_image = cvCreateImage(cvSize(image->width, image->height), image->depth, 1);
        cvCopy(image, non_image);
        ias_projected_light::cp srv;
        srv.request.pattern = pattern_number;
        srv.request.block_size = 15;
        while (ros::ok() && !cl.call(srv))
        {
          ROS_INFO("Failed to show pattern!");

          cvWaitKey(50);
        }
        ROS_INFO("Switched to pattern!");
        sleep(1);
      }
        break;

      default: //Compute the exposure time:
      {
        int w = roi.width;
        int h = roi.height;

        //Compute sum of brightnesses for the whole image and the part that is not ROI, together with
        //the average brightness in the ROI:
        int avg_image = 0;
        int avg_roi = 0;

        for (int i = 0; i < image->width; i++)
        {
          for (int j = 0; j < image->height; j++)
          {
            if (i >= roi.x && i <= roi.x + roi.width && j >= roi.y && j <= roi.y + roi.height)
            {
              avg_roi += cvGet2D(image, j, i).val[0];
            }
            avg_image += cvGet2D(image, j, i).val[0];
          }
        }
        avg_roi /= (w * h);
        avg_image /= (image->width * image->height);

        int avg_brightness_wanted = avg_image + avg_brightness_auto - avg_roi;

        exposure = max(1, avg_brightness_wanted * exposure / avg_image);
        setExposure();
        ROS_INFO("Wanted Avg: %d Real Avg: %d, Avg ROI (should be %d):, %d", avg_brightness_wanted, avg_image,
            avg_brightness_auto, avg_roi);

      }
    }
    if (mode < 3)
      mode++;
  }

public:
  AdjExposure(ros::NodeHandle nh) :
    n(nh), roi_set(false)
  {
    prefix = nh.resolveName("prefix");
    if (strcmp(prefix.c_str(), "/prefix") == 0)
    {
      prefix = "/stereo/stereodcam2701/";
    }

    input_topic = nh.resolveName("input");
    if (strcmp(input_topic.c_str(), "/input") == 0)
    {
      input_topic = "/stereo/left/image_mono";
    }
    roi_input_topic = nh.resolveName("roi_input");
    if (strcmp(roi_input_topic.c_str(), "/roi_input") == 0)
    {
      roi_input_topic = "/stereo/left/roi";
    }
    mode = 0;

    //Get ROS Parameters for camera exposure:
    if (!nh.getParam(prefix + "exposure_min", exposure_min))
    {
      ROS_ERROR("Parameter %sexposure_min not found!", prefix.c_str());
      exit(-1);
    }
    if (!nh.getParam(prefix + "exposure_max", exposure_max))
    {
      ROS_ERROR("Parameter %sexposure_max not found!", prefix.c_str());
      exit(-1);
    }
    if (!nh.getParam(prefix + "exposure", exposure))
    {
      ROS_ERROR("Parameter %sexposure not found!", prefix.c_str());
      exit(-1);
    }

    ROS_INFO("Exposure parameters found!");

    exposure = (exposure_max - exposure_min) / 2; //Set a initial value for exposure time
    setExposure();

    sleep(2);

    sub = nh.subscribe(input_topic, 1, &AdjExposure::adjust, this);
    sub_roi = nh.subscribe(roi_input_topic, 1, &AdjExposure::changeROI, this);
    cl = nh.serviceClient<ias_projected_light::cp> ("change_pattern");

  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "adjust_exposure");
  ros::NodeHandle nh;

  AdjExposure a(nh);

  ros::spin();
}
