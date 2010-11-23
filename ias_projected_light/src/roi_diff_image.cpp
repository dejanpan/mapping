#include <opencv/cv.h>
#include <math.h>
#include <iostream>
#include <opencv/highgui.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <cv_bridge/CvBridge.h>
#include "ias_projected_light/cp.h"

using namespace std;

float percentage = 0.3; //Percentage of maximal difference that is used to filter out weak reflections
bool use_partitions = false;
bool recompute = false; //If false the roi is only computed once => the same roi is published every time

int threshold = 2; //Threshold for the equals function for partitioning

string input_topic;
string output_topic;

ros::Subscriber sub;
ros::Publisher pub;
ros::ServiceClient cl;
sensor_msgs::CvBridge bridge;
sensor_msgs::RegionOfInterest r;

IplImage* black;
IplImage* white;

int pattern = 0;

//Equality function for partitioning:
bool equals(CvPoint p1, CvPoint p2)
{
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y)) < threshold;
}

void getImageRoi(const sensor_msgs::ImageConstPtr& ms)
{
  IplImage* image = bridge.imgMsgToCv(ms);
  switch (pattern)
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

  case 1: //Save image without pattern and project white pattern:
  {
    black = cvCreateImage(cvSize(image->width, image->height), image->depth, 1);
    cvCopy(image, black);
    ias_projected_light::cp srv;
    srv.request.pattern = 1;
    srv.request.block_size = 1;
    while (ros::ok() && !cl.call(srv))
    {
      ROS_INFO("Failed to show white pattern!");

      cvWaitKey(50);
    }
    ROS_INFO("Switched to white image!");
    sleep(1);
  }
    break;

  default: //Compute the exposure time:
  {
    if (pattern == 2 || recompute)
    {
      white = cvCreateImage(cvSize(image->width, image->height), image->depth, 1);
      cvCopy(image, white);


    IplImage* diff = cvCreateImage(cvSize(white->width, white->height), IPL_DEPTH_8U, 1);

    //Get differences between image with black and white pattern:
    int max_diff = 0;
    for (int x = 0; x < white->width; x++)
    {
      for (int y = 0; y < white->height; y++)
      {
        float d = max(0.0, cvGet2D(white, y, x).val[0] - cvGet2D(black, y, x).val[0]);
        cvSet2D(diff, y, x, cvScalar(d));

        if (abs(d) > max_diff)
          max_diff = abs(d);
      }
    }

    //Find all points that have a difference bigger then a certain percentage of the maximal difference
    vector<CvPoint> pv;
    for (int x = 0; x < white->width; x++)
    {
      for (int y = 0; y < white->height; y++)
      {
        if (abs(cvGet2D(diff, y, x).val[0]) > max_diff * percentage)
        {
          CvPoint p;
          p.x = x;
          p.y = y;
          pv.push_back(p);
        }
      }
    }

    cvReleaseImage(&diff);

    vector<int> labels;
    int pmax = 0;

    if (use_partitions)
    {
      //Find all partitions:

      unsigned int partition_count = cv::partition(pv, labels, equals);

      //Find biggest partition:
      unsigned int number_of_points[partition_count];
      for (unsigned int i = 0; i < partition_count; i++)
      {
        number_of_points[i] = 0;
      }
      for (unsigned int i = 0; i < pv.size(); i++)
      {
        number_of_points[labels[i]]++;
      }

      for (unsigned int i = 1; i < partition_count; i++)
      {
        if (number_of_points[i] > number_of_points[pmax])
          pmax = i;
      }
    }

    CvMemStorage* stor = cvCreateMemStorage(0);
    CvSeq* seq = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint), stor);

    //Find bounding box:
    for (unsigned int i = 0; i < pv.size(); i++)
    {
      if (!use_partitions || (labels[i] == pmax))
      {
        cvSeqPush(seq, &pv[i]);
      }
    }
    CvRect roi = cvBoundingRect(seq);

    r.x_offset = roi.x;
    r.y_offset = roi.y;
    r.height = roi.height;
    r.width = roi.width;

    }

    pub.publish(r);
  }

  }
  if (pattern < 3)
    pattern++;
}

int main(int argc, char** argv)
{
  if (argc > 1)
  {
    percentage = atof(argv[1]);
  }
  if (argc > 2)
  {
    use_partitions = atoi(argv[2]);
  }
  if (argc > 3)
  {
    recompute = atoi(argv[3]);
  }

  ros::init(argc, argv, "find_roi_diff_image");
  ros::NodeHandle n;
  n.param("input_topic", input_topic, std::string("/stereo/left/image_rect"));
  n.param("output_topic", output_topic, std::string("/stereo/left/roi"));
  sub = n.subscribe(input_topic, 1, &getImageRoi);
  pub = n.advertise<sensor_msgs::RegionOfInterest> (output_topic, 1);
  cl = n.serviceClient<ias_projected_light::cp> ("change_pattern");

  ros::spin();
}

