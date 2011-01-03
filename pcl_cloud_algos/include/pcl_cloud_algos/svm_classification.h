
#ifndef CLOUD_ALGOS_SVM_CLASSIFICATION_H
#define CLOUD_ALGOS_SVM_CLASSIFICATION_H

// TODO: keep only needed ones
#include <float.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <string.h>
#include <algorithm>
#include <vector>
#include <set>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>

#include <libsvm/svm.h>
// #include <pcl/io/pcd_io.h>
#include <pcl_cloud_algos/cloud_algos.h>
#include <pcl_cloud_algos/pcl_cloud_algos_point_types.h>

namespace pcl_cloud_algos
{

class SVMClassification : public CloudAlgo
{
 public:

  // Input/output type
  // typedef sensor_msgs::PointCloud OutputType;
  // typedef sensor_msgs::PointCloud InputType;
    typedef pcl::PointCloud<GRSDSignature21> InputType;
    typedef pcl::PointCloud<GRSDSignature21> OutputType;

  // Options
  std::string model_file_name_; // filename where the model should be loaded from
  std::string scale_file_name_; // filename where the scale parameters should be loaded from
  bool scale_self_;             // should features be scaled with their maximum to 1 and minimum to -1 or not (also see: scale_file_)
  bool scale_file_;             // if scale_self_ disabled, sets if scale parameters from scale_file_name_ should be used or not

  // Default names
  static std::string default_input_topic ()
    {return std::string ("cloud_pcd");}
  static std::string default_output_topic ()
    {return std::string ("cloud_svm");};
  static std::string default_node_name () 
    {return std::string ("svm_classification_node");};

  // Algorithm methods
  void init (ros::NodeHandle& nh);
  void pre ();
  void post ();
  std::vector<std::string> requires  ();
  std::vector<std::string> provides ();
  std::string process (const boost::shared_ptr<const InputType>&);
  boost::shared_ptr<const OutputType> output ();

  // Constructor-Destructor
  SVMClassification () : CloudAlgo ()
  {
    model_file_name_ = std::string ("svm/fpfh.model");
    // scale_file_name_ = std::string ("svm/fpfh.scp");
    scale_file_name_ = std::string ("svm/teapot_smooth_fpfh.scp");
    scale_self_ = false;
    scale_file_ = true; // gets considered only if scale_self is false
  }

  static inline double
    scaleFeature (int index, double value, double **min_max_values, double lower, double upper)
  {
    // Skip single-valued attributes
    if(min_max_values[0][index] == min_max_values[1][index])
      return (0);

    // TODO: do the first two cases help?
    if (value <= min_max_values[0][index])
      value = lower;
    else if (value >= min_max_values[1][index])
      value = upper;
    else
      // Linear interpolation
      value = lower + (upper - lower) * (value - min_max_values[0][index]) / (min_max_values[1][index] - min_max_values[0][index]);

    return value;
  }

  // TODO make this a general function which gets a list of channels / all channels... point_cloud_mapping::statistics doesn't have it
  static inline double**
    computeScaleParameters (const boost::shared_ptr<const InputType>& cloud, int startIdx, int nr_values)
  {
    // Allocate result vector for min and max values
    double*  p   = new double[2*nr_values];
    for (int i = 0; i < nr_values; i++)
    {
      p[i] = DBL_MAX;
      p[nr_values + i] = -DBL_MAX;
    }
    double** res = new double*[2];
    for (int i = 0; i < 2; i++)
      res[i] = & ( p[i*nr_values] );

    for (int i = 0; i < nr_values; i++)
      std::cerr << i << ": " << res[0][i] << " " << res[1][i] << std::endl;

    // Go through all the channels and compute min&max
    for (size_t cp = 0; cp < cloud->points.size (); cp++)
    {
      // TODO: parallelize, maybe put this as outer for
      for (int i = 0; i < nr_values; i++)
      {
        if (res[0][i] > cloud->channels[startIdx+i].values[cp])
          res[0][i] = cloud->channels[startIdx+i].values[cp];
        else if (res[1][i] < cloud->channels[startIdx+i].values[cp])
          res[1][i] = cloud->channels[startIdx+i].values[cp];
      }
    }

    for (int i = 0; i < nr_values; i++)
      std::cerr << i << ": " << res[0][i] << " " << res[1][i] << std::endl;

    // Return result
    return res;
  }

  static inline double**
    parseScaleParameterFile (const char *fileName, double &lower, double &upper, int nr_values, bool verbose = true)
  {
    // Allocate result vector for min and max values
    double*  p   = new double[2*nr_values];
    for (int i = 0; i < 2*nr_values; i++) p[i] = 0.0;
    double** res = new double*[2];
    for (int i = 0; i < 2; i++)
      res[i] = & ( p[i*nr_values] );
    //ANNpointArray res = annAllocPts (2, nr_values, 0);

    // For reading from files
    std::ifstream fs;

    // Open file
    fs.open (fileName);
    if (!fs.is_open ())
    {
      if (verbose)
        ROS_ERROR ("Couldn't open %s for reading!", fileName);
      return NULL;
    }

    // Get the type
    std::string mystring;
    fs >> mystring;
    if (mystring.substr (0, 1) != "x")
    {
      if (verbose)
        ROS_WARN ("X scaling not found in %s or unknown!", fileName);
      return NULL;
    }

    // Get the bounds
    fs >> lower >> upper;

    // Get the min and max values
    while (!fs.eof ())
    {
      int idx;
      float fmin, fmax;
      fs >> idx >> fmin >> fmax;
      if (idx <= nr_values)
      {
        res[0][idx-1] = fmin;
        res[1][idx-1] = fmax;
      }
    }

    // Close file
    fs.close ();

    // Return result
    return res;
  }

  ros::Publisher createPublisher (ros::NodeHandle& nh)
  {
    ros::Publisher p = nh.advertise<OutputType> (default_output_topic (), 5);
    return p;
  }
 private:

  // ROS stuff
  ros::NodeHandle nh_;
  //ros::Publisher pub_;

  // ROS messages
  boost::shared_ptr<sensor_msgs::PointCloud> cloud_svm_;
};

}
#endif

