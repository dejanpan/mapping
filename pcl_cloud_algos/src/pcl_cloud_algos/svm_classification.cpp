
#include <pcl_cloud_algos/cloud_algos.h>
#include <pcl_cloud_algos/svm_classification.h>

using namespace cloud_algos;

void SVMClassification::init (ros::NodeHandle& nh)
{
  nh_ = nh;
}

void SVMClassification::pre ()
{
  //nh_.param ("/" + default_node_name() + "/model_file_name", model_file_name_, std::string("svm/fpfh.model"));
  //nh_.param ("/" + default_node_name() + "/scale_file_name", scale_file_name_, std::string("svm/teapot_smooth_fpfh.scp"));
  nh_.param("model_file_name", model_file_name_, model_file_name_);
  nh_.param("scale_file_name", scale_file_name_, scale_file_name_);
  nh_.param("scale_self", scale_self_, scale_self_);
  nh_.param("scale_file", scale_file_, scale_file_);
}

void SVMClassification::post ()
{

}

std::vector<std::string> SVMClassification::requires ()
{
  std::vector<std::string> requires;
  // requires at least a feature
  requires.push_back("f1");
  return requires;
}

std::vector<std::string> SVMClassification::provides ()
{
  std::vector<std::string> provides;
  provides.push_back("point_class");
  return provides;
}

std::string SVMClassification::process (const boost::shared_ptr<const SVMClassification::InputType>& cloud)
{
  // Check if features exist and how many of them
  int fIdx = -1;
  for (unsigned int d = 0; d < cloud->channels.size (); d++)
    if (cloud->channels[d].name == "f1")
    {
      fIdx = d;
      break;
    }
  if (fIdx == -1)
  {
    if (verbosity_level_ > -2) ROS_ERROR ("[SVMClassification] Provided point cloud does not have features computed. Use PFH or similar first!");
    output_valid_ = false;
    return std::string("missing features");
  }
  int nr_values = 1;
  for (unsigned int d = fIdx+1; d < cloud->channels.size (); d++)
  {
    char dim_name[16];
    sprintf (dim_name, "f%d", nr_values+1);
    if (cloud->channels[d].name == dim_name)
      nr_values++;
  }
  if (verbosity_level_ > 0) ROS_INFO ("[SVMClassification] Found %d feature values in input PCD.", nr_values);

  // Check if expected classification results are provided as well
  int plIdx = -1;
  for (unsigned int d = 0; d < cloud->channels.size (); d++)
    if (cloud->channels[d].name == "point_label")
    {
      plIdx = d;
      break;
    }
  if (plIdx == -1)
    if (verbosity_level_ > 0) ROS_INFO ("[SVMClassification] NOTE: Points are not labeled with the expected classification results. If you want to evaluate the results please add point_label channel.");

  /// Load the SVM model
  struct svm_node* node;
  struct svm_model* model;
  if ((model = svm_load_model (model_file_name_.c_str ())) == 0)
  {
    if (verbosity_level_ > -2) ROS_ERROR ("[SVMClassification] Couldn't load SVM model from %s", model_file_name_.c_str ());
    output_valid_ = false;
    return std::string("incorrect model file");
  }
  node = (struct svm_node*) malloc ((nr_values+1) * sizeof (struct svm_node));
  ROS_INFO ("[SVMClassification] SVM model type: %d with %d output classes (read from %s).", svm_get_svm_type (model), svm_get_nr_class (model), model_file_name_.c_str ());

  // If scale enabled....
  double lower, upper;
  double** value_ranges = NULL;
  if (scale_self_)
  {
    lower = -1;
    upper = +1;
    value_ranges = computeScaleParameters (cloud, fIdx, nr_values);
    if (verbosity_level_ > 0) ROS_INFO ("[SVMClassification] Scaling data to the interval (%g,%g) enabled.", lower, upper);
  }
  else if (scale_file_)
  {
    value_ranges = parseScaleParameterFile (scale_file_name_.c_str (), lower, upper, nr_values);
    if (value_ranges == NULL)
    {
      if (verbosity_level_ > -2) ROS_ERROR ("[SVMClassification] Scaling requested from file %s but it is not possible!", scale_file_name_.c_str ());
      output_valid_ = false;
      return std::string("incorrect scale parameter file");
    }
    else
    {
      if (verbosity_level_ > 0) ROS_INFO ("[SVMClassification] Scaling according to the limits from %s to the interval (%g,%g) enabled.", scale_file_name_.c_str (), lower, upper);
    }
  }

  // Timers
  ros::Time global_time = ros::Time::now ();
  //ros::Time ts;

  // Copy the original PCD
  cloud_svm_ = boost::shared_ptr<sensor_msgs::PointCloud> (new sensor_msgs::PointCloud());
  cloud_svm_->header   = cloud->header;
  cloud_svm_->points   = cloud->points;
  cloud_svm_->channels = cloud->channels;

  // Allocate the extra needed channels
  if (verbosity_level_ > 0) ROS_INFO ("[SVMClassification] Saving classification results to point_class channel.");
  int pcIdx = cloud_svm_->channels.size ();
  cloud_svm_->channels.resize (pcIdx + 1);
  cloud_svm_->channels[pcIdx].name = "point_class";
  cloud_svm_->channels[pcIdx].values.resize (cloud_svm_->points.size (), 0.0);
  if (verbosity_level_ > 0) ROS_INFO ("[SVMClassification] Added channel: %s", cloud_svm_->channels[pcIdx].name.c_str ());

  // Go through all the points and classify them
  int success = 0;
  for (size_t cp = 0; cp < cloud_svm_->points.size (); cp++)
  {
    // Construct node
    int i = 0;
    for (i = 0; i < nr_values; i++)
    {
      node[i].index = i+1;
      double feature_value = cloud_svm_->channels[fIdx + i].values[cp];
      if (value_ranges != NULL)
        node[i].value = scaleFeature (i, feature_value, value_ranges, lower, upper);
      else
        node[i].value = feature_value; //points[cp][fIdx + i];
    }
    node[i].index = -1;

    // Predict
    cloud_svm_->channels[pcIdx].values[cp] = svm_predict (model, node);

    // If labels were provided count the number of successful classifications
    if (plIdx != -1 && cloud_svm_->channels[pcIdx].values[cp] == cloud_svm_->channels[plIdx].values[cp])
      success++;
  }
  if (plIdx != -1)
    if (verbosity_level_ > 0) ROS_INFO ("[SVMClassification] Accuracy: %d/%d (%g%%).", success, (int)(cloud_svm_->points.size ()), success * 100.0 / cloud_svm_->points.size ());

  // Deallocate
  svm_destroy_model (model);
  free (node);

  // Finish
  if (verbosity_level_ > 0) ROS_INFO ("[SVMClassification] SVM classification done in %g seconds.", (ros::Time::now () - global_time).toSec ());
  output_valid_ = true;
  return std::string("ok");
}

boost::shared_ptr<const SVMClassification::OutputType> SVMClassification::output ()
  {return cloud_svm_;}

#ifdef CREATE_NODE
int main (int argc, char* argv[])
{
  return standalone_node <SVMClassification> (argc, argv);
}
#endif

