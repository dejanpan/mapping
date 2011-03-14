#include <pcl_cloud_algos/global_rsd.h>

using namespace std;
using namespace pcl_cloud_algos;

void GlobalRSD::init (ros::NodeHandle& nh)
{
  nh_ = nh;
  pub_cloud_vrsd_ = pcl_ros::Publisher<pcl::PointNormal> (nh_, "cloud_vrsd", 1);
  pub_cloud_centroids_ = pcl_ros::Publisher<pcl::PointNormal> (nh_, "cloud_centroids", 1);
}

void GlobalRSD::pre ()
{
  nh_.param("point_label", point_label_, point_label_);
  nh_.param("width", width_, width_);
  nh_.param("step", step_, step_);
  nh_.param("min_voxel_pts", min_voxel_pts_, min_voxel_pts_);
  nh_.param("publish_cloud_vrsd", publish_cloud_vrsd_, publish_cloud_vrsd_);
  nh_.param("publish_cloud_centroids", publish_cloud_centroids_, publish_cloud_centroids_);
  nh_.param("min_radius_plane", min_radius_plane_,  min_radius_plane_);
  nh_.param("min_radius_noise", min_radius_noise_,  min_radius_noise_);
  nh_.param("max_radius_noise", max_radius_noise_,  max_radius_noise_);
  nh_.param("max_min_radius_diff", max_min_radius_diff_,  max_min_radius_diff_);
  nh_.param("min_radius_edge", min_radius_edge_,  min_radius_edge_);
  nh_.param("publish_octree", publish_octree_,  publish_octree_);
  cloud_grsd_.points.resize(1);
}

void GlobalRSD::post ()
{

}

std::vector<std::string> GlobalRSD::requires ()
{
  std::vector<std::string> requires;
  // requires 3D coordinates
  requires.push_back("x");
  requires.push_back("y");
  requires.push_back("z");
  // requires normals
  requires.push_back("normal");
  return requires;
}

std::vector<std::string> GlobalRSD::provides ()
{
  std::vector<std::string> provides;

  // provides features (variable number)
  for (int i = 0; i < nr_bins_; i++)
  {
    char dim_name[16];
    sprintf (dim_name, "f%d", i+1);
    provides.push_back (dim_name);
  }

  // sets point label if required
  if (point_label_ != -1)
    provides.push_back ("point_label");

  return provides;
}

std::string GlobalRSD::process (const boost::shared_ptr<const GlobalRSD::InputType>& cloud)
 {
   int norm = pcl::getFieldIndex (*cloud, "normal_x");
   
   if (norm == -1)
   {
     ROS_ERROR ("[GlobalRSD] Provided point cloud does not have normals. Use the normal_estimation or mls_fit first!");
     output_valid_ = false;
     return std::string("missing normals");
   }

   // Timers
  ros::Time global_time = ros::Time::now ();
  ros::Time ts;
  pcl::fromROSMsg(*cloud, cloud_vrsd_);
  cloud_centroids_.points.resize(cloud_vrsd_.points.size());    /// @NOTE: will be overwritten and need to be resized afterwards
  // Create a fixed-size octree decomposition
  ts = ros::Time::now ();
  setOctree (cloud_vrsd_, width_, -1); //width_ = width of the octree cell
  if (verbosity_level_ > 0) 
    ROS_INFO ("[GlobalRSD] kdTree created in %g seconds.", (ros::Time::now () - ts).toSec ());

  // Maximum distance in the user-specified neighborhood
  double max_dist = (2*step_+1)*width_*sqrt(3);
  double radius = max_dist/2;

  // Make sure that we provide enough points for radius computation:
  KdTreePtr tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointNormal> > ();

  if (step_ == 0)
  {
    ts = ros::Time::now ();
    /// @NOTE: passing the pointer here is OK as kdTree and cloud_vrsd_ have the same scope (+ for KdTreeANN it doesn't matter)
    tree->setInputCloud(boost::make_shared <const pcl::PointCloud<pcl::PointNormal> > (cloud_vrsd_));
    if (verbosity_level_ > 0) 
      ROS_INFO ("[GlobalRSD] kdTree created in %g seconds.", (ros::Time::now () - ts).toSec ());
  }

  // Select only the occupied leaves
  std::list<octomap::OcTreeVolume> cells;
  octree_->getOccupied(cells, 0);

  // Set surface type for each cell in advance
  ts = ros::Time::now ();
  int cnt_centroids = 0;
  double x_min, y_min, z_min, x_max, y_max, z_max;
  octree_->getMetricMin(x_min, y_min, z_min);
  octree_->getMetricMax(x_max, y_max, z_max);
  for (std::list<octomap::OcTreeVolume>::iterator it_i = cells.begin (); it_i != cells.end (); ++it_i)
  {
    // Get a cell
    octomap::point3d centroid_i;
    centroid_i(0) = it_i->first.x();
    centroid_i(1) = it_i->first.y();
    centroid_i(2) = it_i->first.z();
    octomap::OcTreeNodePCL *node_i = octree_->search(centroid_i);

    // Get its contents
    vector<int> indices_i = node_i->get3DPointInliers ();
    if (indices_i.size () < min_voxel_pts_)
      continue;

    // Iterating through neighbors
    vector<int> neighbors;
    if (step_ == 0)
    {
      Eigen::Vector4f central_point;
      pcl::compute3DCentroid (cloud_vrsd_, indices_i, central_point);
      vector<float> sqr_distances;
      QueryPoint central_point_pcl(central_point[0], central_point[1], central_point[2]);
      tree->radiusSearch (central_point_pcl, radius, neighbors, sqr_distances); // max_nn_ is INT_MAX by default
    }
    else
    {
      neighbors = indices_i;
      for (int i = step_; i <= step_; i++)
      {
        for (int j = -step_; j <= step_; j++)
        {
          for (int k = -step_; k <= step_; k++)
          {
            // skip current point
            if (i==0 && j==0 && k==0)
              continue;
            // skip inexistent cells
            octomap::point3d centroid_neighbor;
            centroid_neighbor(0) = centroid_i(0) + i*width_;
            centroid_neighbor(1) = centroid_i(1) + j*width_;
            centroid_neighbor(2) = centroid_i(2) + k*width_;
            if (centroid_neighbor(0)<x_min || centroid_neighbor(1)<y_min || centroid_neighbor(2)<z_min || centroid_neighbor(0)>x_max || centroid_neighbor(1)>y_max || centroid_neighbor(2)>z_max)
              continue;
            // accumulate indices
            octomap::OcTreeNodePCL *node_neighbor = octree_->search(centroid_neighbor);
            if (node_neighbor == NULL) // TODO: why does the previous check fail?
              continue;
            vector<int> ni = node_neighbor->get3DPointInliers ();
            neighbors.insert (neighbors.end (), ni.begin (), ni.end ());
          } // i
        } // j
      } // k
    }

    // Mark all points with result
    int type = setSurfaceType (cloud_vrsd_, &indices_i, &neighbors, max_dist);

    // Mark the node label as well
    node_i->label = type;

    // Set up PCD with centroids
    cloud_centroids_.points[cnt_centroids].x = centroid_i(0);
    cloud_centroids_.points[cnt_centroids].y = centroid_i(1);
    cloud_centroids_.points[cnt_centroids].z = centroid_i(2);
    cloud_centroids_.points[cnt_centroids].curvature = (float) type;
    cnt_centroids++;
  }
  cloud_centroids_.points.resize (cnt_centroids);
  //   for (size_t d = 0; d < cloud_centroids_.channels.size (); d++)
  //     cloud_centroids_.channels[d].values.resize (cnt_centroids);
  
  if (verbosity_level_ > 0) 
    ROS_INFO ("[GlobalRSD] Cells annotated in %g seconds.", (ros::Time::now () - ts).toSec ());

  //GLOBAL part
  // Initialize transition matrix for counting
  vector<vector<int> > transitions (NR_CLASS+1);
  for (size_t i = 0; i < transitions.size (); i++)
    transitions[i].resize (NR_CLASS+1);

  /// Iterate over all cells
  ts = ros::Time::now ();
  float line_p1[3], line_p2[3], box_bounds[6];
  for (std::list<octomap::OcTreeVolume>::iterator it_i = cells.begin (); it_i != cells.end (); ++it_i)
  {
    // Get a cell
    octomap::point3d centroid_i;
    centroid_i(0) = it_i->first.x();
    centroid_i(1) = it_i->first.y();
    centroid_i(2) = it_i->first.z();
    octomap::OcTreeNodePCL *node_i = octree_->search(centroid_i);

    // Get its contents
    vector<int> indices_i = node_i->get3DPointInliers ();
    if (indices_i.size () < min_voxel_pts_)
      continue;

    // Connect current cell to all the remaining ones
    for (std::list<octomap::OcTreeVolume>::iterator it_j = it_i; it_j != cells.end (); ++it_j)
    {
      /// TODO: just start for from it_i+1
      if (it_i == it_j)
       continue;

      // Get a cell
      octomap::point3d centroid_j;
      centroid_j(0) = it_j->first.x();
      centroid_j(1) = it_j->first.y();
      centroid_j(2) = it_j->first.z();
      octomap::OcTreeNodePCL *node_j = octree_->search(centroid_j);

      // Get its contents
      vector<int> indices_j = node_j->get3DPointInliers ();
      if (indices_j.size () < min_voxel_pts_)
        continue;

      // Create a paired histogram vector which holds: a) the actual centroid value of the intersected voxel, b) the distance from start_voxel to voxel_i
      vector<pair<int, IntersectedLeaf> > histogram_values;

      // Get the leaves along the ray
      vector<octomap::point3d> ray;
      octree_->computeRay(centroid_i, centroid_j, ray);

      // Iterate over leaves
      for (vector<octomap::point3d>::iterator centroid_ray = ray.begin (); centroid_ray != ray.end (); centroid_ray++)
      {
        // Compute the distance to the start leaf
        pair<int, IntersectedLeaf> histogram_pair;
        histogram_pair.second.centroid = *centroid_ray;
        histogram_pair.second.sqr_distance = _sqr_dist (*centroid_ray, centroid_i);

        // Save the label of the cell
        octomap::OcTreeNodePCL *node_ray = octree_->search(*centroid_ray);
        if (node_ray == NULL) // @TODO: this should never happen, the octree_ should be fully expanded!
          histogram_pair.first = -1;
        else
          histogram_pair.first = node_ray->label;

        // Save data about cell
        histogram_values.push_back (histogram_pair);
      }

      // Add the first voxel
      pair<int, IntersectedLeaf> histogram_pair1;
      histogram_pair1.second.sqr_distance = _sqr_dist (centroid_i, centroid_i); // 0.0
      histogram_pair1.second.centroid = centroid_i;
      histogram_pair1.first = node_i->label;
      //histogram_pair1.first = (int)(cloud_vrsd.channels[regIdx].values[indices_i.at (0)]);
      histogram_values.push_back (histogram_pair1);

      // Add the last voxel
      pair<int, IntersectedLeaf> histogram_pair2;
      histogram_pair2.second.sqr_distance = _sqr_dist (centroid_j, centroid_i); // line length
      histogram_pair2.second.centroid = centroid_j;
      histogram_pair2.first = node_j->label;
      //histogram_pair2.first = (int)(cloud_vrsd.channels[regIdx].values[indices_j.at (0)]);
      histogram_values.push_back (histogram_pair2);

      // Sort the histogram according to the distance of the leaves to the start leaf
      sort (histogram_values.begin (), histogram_values.end (), histogramElementCompare);

      // Count transitions between the first and last voxel
      for (unsigned int hi = 1; hi < histogram_values.size (); hi++)
      {
        // transition matrix has to be symmetrical
        transitions[histogram_values[hi].first+1][histogram_values[hi-1].first+1]++;
        transitions[histogram_values[hi-1].first+1][histogram_values[hi].first+1]++;
        //cerr << "transition: " << histogram_values[hi].first-EMPTY_VALUE << " => " << histogram_values[hi-1].first-EMPTY_VALUE << endl;
      }
    }
  }
  if (verbosity_level_ > 0) 
    ROS_INFO ("[GlobalRSD] Transitions counted in %g seconds.", (ros::Time::now () - ts).toSec ());

  /// Outputting result in libSVM format
  if (verbosity_level_ > 1)
  {
    ROS_INFO ("[GlobalRSD] Transition matrix is:");
    for (int i=0; i<NR_CLASS+1; i++)
    {
      stringstream line;
      for (int j=0; j<NR_CLASS+1; j++)
        line << " " << transitions[i][j];
      ROS_INFO ("%s", line.str ().c_str ());
    }
  }
  int nrf = 0;
  for (int i=0; i<NR_CLASS+1; i++)
  {
    for (int j=i; j<NR_CLASS+1; j++)
    {
      cloud_grsd_.points[0].histogram[nrf++] = transitions[i][j]; //@TODO: resize point cloud
    }
  }
  //@TODO: Check with Zoli what are following 2 lines for:
  //if (point_label_ != -1)
  //cloud_grsd_.channels[nr_bins_].values[0] = point_label_;
  
  // Publish partial results for visualization
  if (publish_cloud_centroids_)
    pub_cloud_centroids_.publish (cloud_centroids_);
  if (publish_cloud_vrsd_)
    pub_cloud_vrsd_.publish (cloud_vrsd_);

  // Finish
  if (verbosity_level_ > 0) 
    ROS_INFO ("[GlobalRSD] Computed features in %g seconds.", (ros::Time::now () - global_time).toSec ());
  output_valid_ = true;
  return std::string("ok");
}

boost::shared_ptr<const GlobalRSD::OutputType> GlobalRSD::output ()
{
  //  sensor_msgs::PointCloud2 cloud_grsd_msg;
  sensor_msgs::PointCloud2 cloud_grsd_msg;
  pcl::toROSMsg (cloud_grsd_, cloud_grsd_msg);
  return boost::make_shared<sensor_msgs::PointCloud2> (cloud_grsd_msg);
}

#ifdef CREATE_NODE
int main (int argc, char* argv[])
{
  return standalone_node <GlobalRSD> (argc, argv);
}
#endif
