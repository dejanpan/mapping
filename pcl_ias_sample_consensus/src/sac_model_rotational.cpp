/* 
 * Copyright (c) 2010, Nico Blodow <blodow@cs.tum.edu>
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
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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

//#include <point_cloud_mapping/sample_consensus/sac_model_plane.h>
#include <ias_sample_consensus/sac_model_rotational.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/distances.h>
#include <point_cloud_mapping/geometry/nearest.h>
#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/geometry/transforms.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <algorithm>
#include <iterator>
#include <Eigen/Cholesky>
#include <Eigen/LU>
 
//#include <lm.h>
#include <cminpack.h>

template <class T>
void print_vector (T vec)
{
  copy (vec.begin(), vec.end(), std::ostream_iterator<double> (std::cerr, " "));
  std::cerr << std::endl;
}

int polynomial_order = 5;
int min_nr_samples = 10; // for axis estimation, then polynomial fit - actual number will be max of this and order+1

namespace ias_sample_consensus
{
  // Some helper functions
  inline void
    Transform3DTo2D (const geometry_msgs::Point32 &p, double &x, double &y, const geometry_msgs::Point32 &axis, const geometry_msgs::Point32 &point0, const geometry_msgs::Point32 &origin_point)
  {
    // the position (x-value?) of the projection of current point on the rot. axis
    x =  (cloud_geometry::dot (p, axis) 
        - cloud_geometry::dot (origin_point, axis))
        / sqrt(cloud_geometry::dot (axis, axis));

    y = cloud_geometry::distances::pointToLineDistance (p, point0, axis);
  }

  // Compute distance from point to the surface of revolution
  double 
    SACModelRotational::PointToRotationalDistance (const std::vector<double> &model_coefficients, const geometry_msgs::Point32 &p)
  { 
    /** @todo: test me */
    geometry_msgs::Point32 axis, point0, origin_point;
    axis.x = model_coefficients[3] - model_coefficients[0];
    axis.y = model_coefficients[4] - model_coefficients[1];
    axis.z = model_coefficients[5] - model_coefficients[2];
    point0.x = model_coefficients[0];
    point0.y = model_coefficients[1];
    point0.z = model_coefficients[2];
    origin_point.x = model_coefficients[6];
    origin_point.y = model_coefficients[7];
    origin_point.z = model_coefficients[8];
    
    // project 3D point onto 2D plane
    double k,y;
    Transform3DTo2D (p, k, y, axis, point0, origin_point);

    // evaluate polynomial at position k
    double r = 0.0; 
    for (int w = 0; w < polynomial_order + 1; w++)
      r += model_coefficients[9+w] * pow (k, (double)w);

    return fabs(y - fabs(r));
  }
  
  void
    SACModelRotational::getDistancesToModel (const std::vector<double> &model_coefficients, std::vector<double> &distances)
  {
    distances.resize (indices_.size ());

    // Iterate through the 3d points and calculate the distances from them to the cylinder
    for (unsigned int i = 0; i < indices_.size (); i++)
      distances[i] = fabs (PointToRotationalDistance (model_coefficients, cloud_->points.at (indices_[i])));
    return;
  }

  std::vector<double>
    LineToLineSegment (std::vector<double> line_a, std::vector<double> line_b, double eps)
  {
    std::vector<double> segment (6);
    geometry_msgs::Point32 u;
    geometry_msgs::Point32 v;
    geometry_msgs::Point32 w;

    // a = x2 - x1 = line_a[1] - line_a[0]
    u.x = line_a[3] - line_a[0];
    u.y = line_a[4] - line_a[1];
    u.z = line_a[5] - line_a[2];
    // b = x4 - x3 = line_b[1] - line_b[0]
    v.x = line_b[3] - line_b[0];
    v.y = line_b[4] - line_b[1];
    v.z = line_b[5] - line_b[2];
    // c = x2 - x3 = line_a[1] - line_b[0]
    w.x = line_a[3] - line_b[0];
    w.y = line_a[4] - line_b[1];
    w.z = line_a[5] - line_b[2];

    double a = cloud_geometry::dot (u, u);
    double b = cloud_geometry::dot (u, v);
    double c = cloud_geometry::dot (v, v);
    double d = cloud_geometry::dot (u, w);
    double e = cloud_geometry::dot (v, w);
    double denominator = a*c - b*b;
    double sc, tc;
    // Compute the line parameters of the two closest points
    if (denominator < eps)          // The lines are almost parallel
    {
      sc = 0.0;
      tc = (b > c ? d / b : e / c);  // Use the largest denominator
    }
    else
    {
      sc = (b*e - c*d) / denominator;
      tc = (a*e - b*d) / denominator;
    }
    // Get the closest points
    segment[0] = line_a[3] + (sc * u.x);
    segment[1] = line_a[4] + (sc * u.y);
    segment[2] = line_a[5] + (sc * u.z);

    segment[3] = line_b[0] + (tc * v.x);
    segment[4] = line_b[1] + (tc * v.y);
    segment[5] = line_b[2] + (tc * v.z);

    return segment;
  }

  double
    LineToLineDistance (std::vector<double> line_a, std::vector<double> line_b, double eps)
  {
    std::vector<double> segment = LineToLineSegment (line_a, line_b, eps);
    std::vector<double> dP(3);
    // Get the difference of the two closest points
    dP[0] = segment[3] - segment[0];
    dP[1] = segment[4] - segment[1];
    dP[2] = segment[5] - segment[2];
    // Get the closest distance
    double distance = sqrt (dP[0]*dP[0] + dP[1]*dP[1] + dP[2]*dP[2]);
    return distance;
  }

  int
    SACModelRotational::functionToOptimizeAxis (void *p, int m, int n, const double *x, double *fvec, int iflag)
  {
     // Compute the L2 norm of the residuals
    SACModelRotational *model = (SACModelRotational*)p;
   
    std::vector<double> rot_coeff (6);
    for (int d = 0; d < 6; d++)
      rot_coeff[d] = x[d];

    std::vector<double> point_normal_line (6);

    geometry_msgs::Polygon poly;
    geometry_msgs::Point32 p1, p2;
    p1.x = rot_coeff [0]; 
    p1.y = rot_coeff [1]; 
    p1.z = rot_coeff [2]; 
    p2.x = rot_coeff [3];
    p2.y = rot_coeff [4]; 
    p2.z = rot_coeff [5]; 
    poly.points.push_back (geometry_msgs::Point32(p1));
    poly.points.push_back (geometry_msgs::Point32(p2));
    p2.x = p2.x - p1.x;
    p2.y = p2.y - p1.y;
    p2.z = p2.z - p1.z;

//    model->pmap_.polygons.push_back (poly);
    double sum = 0.0; 
    for (int i = 0; i < m; i++)
    {
      point_normal_line[0] = model->cloud_->points[model->tmp_inliers_->at(i)].x;
      point_normal_line[1] = model->cloud_->points[model->tmp_inliers_->at(i)].y;
      point_normal_line[2] = model->cloud_->points[model->tmp_inliers_->at(i)].z;
      
      point_normal_line[3] = point_normal_line[0] + model->cloud_->channels[model->nx_idx_].values[model->tmp_inliers_->at(i)];
      point_normal_line[4] = point_normal_line[1] + model->cloud_->channels[model->ny_idx_].values[model->tmp_inliers_->at(i)];
      point_normal_line[5] = point_normal_line[2] + model->cloud_->channels[model->nz_idx_].values[model->tmp_inliers_->at(i)];
      
      double ll = LineToLineDistance (rot_coeff, point_normal_line, 1e-5); 
      //p1.x = model->cloud_->channels[model->nx_idx_].values[model->tmp_inliers_->at(i)];
      //p1.y = model->cloud_->channels[model->ny_idx_].values[model->tmp_inliers_->at(i)];
      //p1.z = model->cloud_->channels[model->nz_idx_].values[model->tmp_inliers_->at(i)];
      //double weight = (cos(cloud_geometry::angles::getAngle3D(p1,p2) * 2.0) + 3.0) * .25;
      fvec[i] = ll * ll;// * weight;
//       fvec[i] = pointToRotationalDistance (model->cloud_->points[model->tmp_inliers_->at (i)], rot_coeff, model->polynomial_order) - x[6];
      sum += ll;
    }
    
    return (0);
  }

  bool 
    SACModelRotational::MinimizeAxisDistancesToSamples (const std::vector<int> samples, std::vector<double> &model_coefficients, double &err)
  { 
    std::cerr << "REFIT AXIS COEFFS BEFOR: ";
    print_vector (model_coefficients);    

    ROS_INFO ("optimizing axis from %i points", (int)samples.size());
    
    if (samples.size () == 0)
    {
      ROS_ERROR ("[SACModelRotational::MinimizeAxisDistancesToSamples] Cannot re-fit 0 inliers!");
      return false;
    }
 
    if (model_coefficients.size () == 0)
    {
      ROS_WARN ("[SACModelRotational::MinimizeAxisDistancesToSamples] Initial model coefficients have not been estimated yet!");
      return false;
    }

    tmp_inliers_ = &samples;
    
    int m = samples.size ();

    double *fvec = new double[m];

    int n = 6;      // 6 unknowns
    int iwa[n];

    int lwa = m * n + 5 * n + m;
    double *wa = new double[lwa];

    // Set the initial solution
    double x[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    for (int d = 0; d < n; d++)
      x[d] = model_coefficients.at (d);

    // Set tol to the square root of the machine. Unless high solutions are required, these are the recommended settings.
    double tol = 0.00000000000001;//(dpmpar (2));

    // Optimize using forward-difference approximation LM
    int info = lmdif1 (&ias_sample_consensus::SACModelRotational::functionToOptimizeAxis, this, m, n, x, fvec, tol, iwa, wa, lwa);
    info = info; // to get rid of stoopid compiler warnings...
    // Compute the L2 norm of the residuals
    err = enorm (m, fvec);
    ROS_INFO ("LM solver finished with exit code %i, having a residual norm of %g. ",
               info, enorm (m, fvec));

    //model_coefficients.resize (n);
    for (int d = 0; d < n; d++)
      model_coefficients[d] = x[d];

    std::cerr << "REFIT AXIS COEFFS AFTER: ";
    print_vector (model_coefficients);    

    free (wa); free (fvec);
    
    return 0.0;
  }

  void 
    Collect3DPointsInRotatedPlane (const std::vector<int> samples, const sensor_msgs::PointCloud cloud,
                                   const std::vector<double> model_coefficients,
                                   std::vector<double> &vals_2d_x, std::vector<double> &vals_2d_y, double &min_k, double &max_k)
  {
    vals_2d_x.clear();
    vals_2d_y.clear();
    min_k = FLT_MAX;
    max_k = -FLT_MAX;
    geometry_msgs::Point32 axis, point0, origin_point;
    axis.x = model_coefficients[3] - model_coefficients[0];
    axis.y = model_coefficients[4] - model_coefficients[1];
    axis.z = model_coefficients[5] - model_coefficients[2];
    point0.x = model_coefficients[0];
    point0.y = model_coefficients[1];
    point0.z = model_coefficients[2];
    origin_point.x = model_coefficients[6];
    origin_point.y = model_coefficients[7];
    origin_point.z = model_coefficients[8];
    
    for (unsigned int i = 0; i < samples.size(); i++)
    {
      double k,y;
      Transform3DTo2D (cloud.points.at(samples[i]), k, y, axis, point0, origin_point);

      vals_2d_x.push_back (k);
      vals_2d_y.push_back (y);
      
      if (k < min_k)
        min_k = k;
      if (k > max_k)
        max_k = k;
    }
  }

  bool 
    SACModelRotational::EstimateContourFromSamples (const std::vector<int> samples, std::vector<double> &model_coefficients)
  { 
    std::vector<double> vals_2d_x;
    std::vector<double> vals_2d_y;
    double min_k, max_k;
    Collect3DPointsInRotatedPlane (samples, *cloud_, model_coefficients, vals_2d_x, vals_2d_y, min_k, max_k);
    // compute mean: double mv = accumulate (cont.begin(), cont.end(), 0) / distance(cont.begin(),cont.end());

    // refit polynomial
    Eigen::MatrixXd A = Eigen::MatrixXd (samples.size(), polynomial_order+1); // 4 == polynomial order + 1
    Eigen::VectorXd b = Eigen::VectorXd (samples.size());

    // fill in A and b from the 2d arrays (vals_2d_[x,y])
    for (unsigned int d1 = 0; d1 < vals_2d_x.size(); d1++)
    {
      b[d1] = vals_2d_y[d1];
      for (int d2 = 0; d2 < polynomial_order+1; d2++)
        A(d1,d2) = pow (vals_2d_x[d1], (double) d2);
    }

    // allocate and initialize the parts of the equation system
    Eigen::MatrixXd M (polynomial_order+1,polynomial_order+1);
    M.part<Eigen::SelfAdjoint>() = A.transpose() * A;
    Eigen::VectorXd x = A.transpose() * b;

    // solve
    //Eigen::VectorXd x1 = x;
    M.llt().solveInPlace(x);

    //Eigen::VectorXd x2 = M.inverse() * x;
    //std::cerr << x1.transpose() << std::endl << x2.transpose() << std::endl << (x1-x2).norm() << std::endl;

    // and dismount..
    for (int i = 0; i < polynomial_order+1; i++)
      model_coefficients[9+i] = x[i];

    //std::cerr << "[EstimateContourFromSamples] polynomial : " << better_b[0] << " + x*"<<better_b[1] << " + x*x*" << better_b[2] << " + x*x*x*" << better_b[3] << std::endl;
    //std::cerr << "[EstimateContourFromSamples] polynomial : " << xvec[0] << " + x*"<<xvec[1] << " + x*x*" << xvec[2] << " + x*x*x*" << xvec[3] << std::endl;
    //std::cerr << "[EstimateContourFromSamples] shape coefficients: ";
    //print_vector (model_coefficients);
    //std::cerr << "[EstimateContourFromSamples] range: " << min_k << ", " << max_k << std::endl;
    //std::cerr << "[EstimateContourFromSamples] x_values: ";
    //print_vector (vals_2d_x);
    //std::cerr << "[EstimateContourFromSamples] y_values: ";
    //print_vector (vals_2d_y);
    //for (unsigned int i = 0; i < vals_2d_x.size(); i++)
    //    std::cerr << vals_2d_x.at(i) << "\t" << vals_2d_y.at(i) << std::endl;
    model_coefficients_ = model_coefficients;
    return true; 
  }
  
  bool 
    SACModelRotational::RefitAxis (const std::vector<int> &inliers, std::vector<double> &refit_coefficients)
  { 
    /** @todo: test me */
    double err;
    MinimizeAxisDistancesToSamples (inliers, refit_coefficients, err);
    return true;
  }

  void 
    SACModelRotational::RefitContour (const std::vector<int> &inliers, std::vector<double> &refit_coefficients)
  { 
    /** @todo: test me */
    EstimateContourFromSamples (inliers, refit_coefficients);
  }
  
  bool 
    SACModelRotational::EstimateAxisFromSamples (const std::vector<int> samples, std::vector<double> &model_coefficients)
  {
    /** @todo: fix issues with noisy data sets */
    ROS_INFO ("estimating axis from %i points", (int)samples.size());
    // Save the nx/ny/nz channel indices the first time we run this
    if (nx_idx_ == -1)
    {
      nx_idx_ = cloud_geometry::getChannelIndex (*cloud_, "nx");
      if (nx_idx_ == -1)
      {
        ROS_ERROR ("[SACModelRotational::EstimateAxisFromSamples] I Need normals!");
        return (false);
      }
    }
    if (ny_idx_ == -1)
    {
      ny_idx_ = cloud_geometry::getChannelIndex (*cloud_, "ny");
      if (ny_idx_ == -1) return (false);
    }
    if (nz_idx_ == -1)
    {
      nz_idx_ = cloud_geometry::getChannelIndex (*cloud_, "nz");
      if (nz_idx_ == -1) return (false);
    }
    
    geometry_msgs::Point32 centroid;
    geometry_msgs::Polygon best_poly;
    geometry_msgs::Point32 axis;
    geometry_msgs::Point32 n1;
    geometry_msgs::Point32 n2;
    
    std::vector<double> temp_coefficients (model_coefficients.size(), 0.0);

    double min_err = FLT_MAX;

// @todo: fix me. this is legacy code from the EGI-visualized axis 
// estimation for noisy crap mixed with regular axis est.
    for (unsigned int i = 0; i < 2/*samples.size()*/; i++)
      for (unsigned int j = 0; j < i; j++)
      {
        cloud_geometry::nearest::computeCentroid (*cloud_, samples, centroid);
        temp_coefficients [0] = centroid.x; 
        temp_coefficients [1] = centroid.y; 
        temp_coefficients [2] = centroid.z; 
      
        n1.x = cloud_->channels[nx_idx_].values.at (samples[i]);
        n1.y = cloud_->channels[ny_idx_].values.at (samples[i]);
        n1.z = cloud_->channels[nz_idx_].values.at (samples[i]);
        
        n2.x = cloud_->channels[nx_idx_].values.at (samples[j]);
        n2.y = cloud_->channels[ny_idx_].values.at (samples[j]);
        n2.z = cloud_->channels[nz_idx_].values.at (samples[j]);
        
        axis = cloud_geometry::cross (n1, n2);
        
    double trand = 0.3 / (RAND_MAX + 1.0);
    // Get a random number between 1 and indices.size ()
        temp_coefficients [3] = centroid.x + (rand () * trand)-.15;//axis.x;
        temp_coefficients [4] = centroid.y + (rand () * trand)-0.15;//axis.y; 
        temp_coefficients [5] = centroid.z + 1;//axis.z; 
        // origin_point
        temp_coefficients [6] = centroid.x;
        temp_coefficients [7] = centroid.y; 
        temp_coefficients [8] = centroid.z; 

        std::vector<double> line_a(6), line_b(6);
        line_a[0] = cloud_->points.at (samples[i]).x;
        line_a[1] = cloud_->points.at (samples[i]).y;
        line_a[2] = cloud_->points.at (samples[i]).z;
        line_a[3] = cloud_->points.at (samples[i]).x + n1.x; 
        line_a[4] = cloud_->points.at (samples[i]).y + n1.y;
        line_a[5] = cloud_->points.at (samples[i]).z + n1.z;
        
        line_b[0] = cloud_->points.at (samples[j]).x;
        line_b[1] = cloud_->points.at (samples[j]).y;
        line_b[2] = cloud_->points.at (samples[j]).z;
        line_b[3] = cloud_->points.at (samples[j]).x + n2.x; 
        line_b[4] = cloud_->points.at (samples[j]).y + n2.y;
        line_b[5] = cloud_->points.at (samples[j]).z + n2.z;
        std::vector<double> segment = LineToLineSegment (line_a, line_b, 1e-5);
        //temp_coefficients = segment; 
        double err = 0;
        MinimizeAxisDistancesToSamples (samples, temp_coefficients, err);
        
    // compute minimal and maximal points of the axis (min_k, max_k)
    std::vector<double> vals_2d_x;
    std::vector<double> vals_2d_y;
    double min_k, max_k;
    Collect3DPointsInRotatedPlane (samples, *cloud_, temp_coefficients, vals_2d_x, vals_2d_y, min_k, max_k);
    
    Eigen::Vector3d point0 (temp_coefficients[0], temp_coefficients[1], temp_coefficients[2]);
    Eigen::Vector3d axis (temp_coefficients[3] - temp_coefficients[0],
                          temp_coefficients[4] - temp_coefficients[1],
                          temp_coefficients[5] - temp_coefficients[2] );
    Eigen::Vector3d axis_normalized = axis.normalized ();

    Eigen::Vector3d origin (temp_coefficients[6], temp_coefficients[7], temp_coefficients[8]);
    origin = point0 + (origin - point0).dot (axis_normalized) * axis_normalized;
   
    Eigen::Vector3d p1 = origin + min_k * axis_normalized; 
    Eigen::Vector3d p2 = origin + max_k * axis_normalized; 
    
        temp_coefficients[0] = p1[0];
        temp_coefficients[1] = p1[1];
        temp_coefficients[2] = p1[2];
        temp_coefficients[3] = p2[0];
        temp_coefficients[4] = p2[1];
        temp_coefficients[5] = p2[2];
        temp_coefficients[6] = centroid.x;
        temp_coefficients[7] = centroid.y; 
        temp_coefficients[8] = centroid.z; 

        geometry_msgs::Polygon p;
        geometry_msgs::Point32 ros_p1,ros_p2;
        ros_p1.x = temp_coefficients [0]; 
        ros_p1.y = temp_coefficients [1]; 
        ros_p1.z = temp_coefficients [2]; 
        ros_p2.x = temp_coefficients [3];
        ros_p2.y = temp_coefficients [4]; 
        ros_p2.z = temp_coefficients [5]; 
        p.points.push_back (ros_p1);
        p.points.push_back (ros_p2);

        if (err <= min_err)
        {
          min_err = err;
          best_poly = p;
//          pmap_.polygons.clear ();
//          pmap_.polygons.push_back (p);
          model_coefficients.swap (temp_coefficients);
        }
      }
    ROS_INFO ("created pmap with %i lines", (int)pmap_->polygons.size ());
    assert ((int)model_coefficients.size() == 9+polynomial_order+1);

    pmap_->polygons.push_back (best_poly);
    return true;
  }
  
  void
    SACModelRotational::getSamples (int &iterations, std::vector<int> &samples)
  {
    samples.resize (min_nr_samples > polynomial_order+1 ? 6 : polynomial_order+1);
    double trand = indices_.size () / (RAND_MAX + 1.0);

    // Get a random number between 1 and indices.size ()
    int idx = (int)(rand () * trand);
    // Get the index
    samples[0] = indices_.at (idx);

    // Get the rest of the sample points which are different from one another
    bool same;
    for (unsigned int i = 1; i < samples.size(); i++)
      do
      {
        same = false;
        idx = (int)(rand () * trand);
        samples[i] = indices_.at (idx);
        for (unsigned int j = 0; j < i; j++)
          if (samples[i] == samples[j])
            same = true;
      } while (same);
//    samples = std::vector<int> (indices_);
    return;
  }

  void
    SACModelRotational::selectWithinDistance (const std::vector<double> &model_coefficients, double threshold, std::vector<int> &inliers)
  {
    int nr_p = 0;
    inliers.resize (indices_.size ());

    // Go through all points and select those who are within the distance threshold to the rotational obj.
    for (unsigned int i = 0; i < indices_.size (); i++)
    {
      if (PointToRotationalDistance (model_coefficients, cloud_->points.at (indices_.at (i))) < threshold)
      {
        // Store indices of points whose distances are smaller than the threshold
        inliers[nr_p] = indices_[i];
        nr_p++;
      }
    }
    inliers.resize (nr_p);
    return;
  }

  bool
    SACModelRotational::computeModelCoefficients (const std::vector<int> &samples)
  {
    std::cerr << "... sample points:";
    print_vector (samples);    
    model_coefficients_.resize (9+polynomial_order+1);
    
    if (!EstimateAxisFromSamples (samples, model_coefficients_))
      return false;
     
    if (!EstimateContourFromSamples (samples, model_coefficients_))
    //if (!EstimateContourFromSamples (indices_, model_coefficients_))
      return false;

    return (true);
  }

  void
    SACModelRotational::refitModel (const std::vector<int> &inliers, std::vector<double> &refit_coefficients)
  {
    if (inliers.size () == 0)
    {
      ROS_ERROR ("[SACModelRotational::refitModel] Cannot re-fit 0 inliers!");
      refit_coefficients = model_coefficients_;
      return;
    }
    refit_coefficients = model_coefficients_;
    
    ROS_WARN ("[SACModelRotational::refitModel] About to refit");
    RefitAxis (inliers, refit_coefficients);
    RefitContour (inliers, refit_coefficients);
  }

  bool
    SACModelRotational::doSamplesVerifyModel (const std::set<int> &indices, double threshold)
  {
    for (std::set<int>::iterator it = indices.begin (); it != indices.end (); ++it)
      if (PointToRotationalDistance (model_coefficients_, cloud_->points.at (*it)) > threshold)
        return (false);

    return (true);
  }
  
  void
    SACModelRotational::projectPoints (const std::vector<int> &inliers, const std::vector<double> &model_coefficients,
                                  sensor_msgs::PointCloud &projected_points)
  {
    /** @todo: this is only a dummy implementation */
    // Allocate enough space
    projected_points.points.resize (inliers.size ());
    projected_points.channels.resize (cloud_->channels.size ());

    // Create the channels
    for (unsigned int d = 0; d < projected_points.channels.size (); d++)
    {
      projected_points.channels[d].name = cloud_->channels[d].name;
      projected_points.channels[d].values.resize (inliers.size ());
    }

    // Iterate through the 3d points and calculate the distances from them to the plane
    for (unsigned int i = 0; i < inliers.size (); i++)
    {
      // Copy the poitn coordinates
      projected_points.points[i].x = cloud_->points.at (inliers.at (i)).x;
      projected_points.points[i].y = cloud_->points.at (inliers.at (i)).y;
      projected_points.points[i].z = cloud_->points.at (inliers.at (i)).z;
      // Copy the other attributes
      for (unsigned int d = 0; d < projected_points.channels.size (); d++)
        projected_points.channels[d].values[i] = cloud_->channels[d].values[inliers.at (i)];
    }
  }

  void
    SACModelRotational::projectPointsInPlace (const std::vector<int> &inliers, const std::vector<double> &model_coefficients)
  {
    /** @todo: implement me */
  }
  
  void
    SACModelRotational::samplePointsOnRotational (const std::vector<double> modelCoefficients, std::vector<int> inliers, 
                                                  boost::shared_ptr<triangle_mesh_msgs::TriangleMesh> ret)
  {
    static int count = 0;
    count++;
    
    // compute minimal and maximal points of the axis (min_k, max_k)
    std::vector<double> vals_2d_x;
    std::vector<double> vals_2d_y;
    double min_k, max_k;
    std::vector<double> model_coefficients (modelCoefficients);
    Collect3DPointsInRotatedPlane (inliers, *cloud_, model_coefficients, vals_2d_x, vals_2d_y, min_k, max_k);
    //Collect3DPointsInRotatedPlane (indices_, *cloud_, model_coefficients, vals_2d_x, vals_2d_y, min_k, max_k);
   
    Eigen::Vector3d point0 (model_coefficients[0], model_coefficients[1], model_coefficients[2]);
    geometry_msgs::Point32 point0_point;
    point0_point.x = point0[0];
    point0_point.y = point0[1];
    point0_point.z = point0[2];

    Eigen::Vector3d axis (model_coefficients[3] - model_coefficients[0],
                          model_coefficients[4] - model_coefficients[1],
                          model_coefficients[5] - model_coefficients[2] );
    Eigen::Vector3d axis_normalized = axis.normalized ();
    geometry_msgs::Point32 axis_point;
    axis_point.x = axis_normalized[0];
    axis_point.y = axis_normalized[1];
    axis_point.z = axis_normalized[2];

    Eigen::Vector3d origin (model_coefficients[6], model_coefficients[7], model_coefficients[8]);
    geometry_msgs::Point32 origin_point;
    origin_point.x = origin[0];
    origin_point.y = origin[1];
    origin_point.z = origin[2];
    
    origin = point0 + (origin - point0).dot (axis_normalized) * axis_normalized;
   
    Eigen::Vector3d p1 = origin + min_k * axis_normalized; 
    Eigen::Vector3d p2 = origin + max_k * axis_normalized; 

    ROS_ERROR ("------------------------ min_k, max_k = %f <-> %f", min_k, max_k);
    
    double res_axial = 50.0;
    double res_radial = 30.0;
    int cp = 0;
    for (int i = 0; i < res_axial; i++)
    {
      double X = i / res_axial;
      
      geometry_msgs::Point32 X_3D;
      Eigen::Vector3d p_temp = p1 + (p2-p1) * X; 
      X_3D.x = p_temp[0];
      X_3D.y = p_temp[1];
      X_3D.z = p_temp[2];

      double y_dummy;
      // note: this function is overwriting X
      Transform3DTo2D (X_3D, X, y_dummy, axis_point, point0_point, origin_point);

      // evaluate polynomial at position X
      double Y = 0.0;
      for (int w = 0; w < polynomial_order + 1; w++)
        Y += model_coefficients[9+w] * pow(X,(double)w);

      geometry_msgs::Point32 p;
      for (int j = 0; j < res_radial; j++)
      {
        p.x = Y;
        p.y = 0.0;
        p.z = ((double)i/res_axial)*(p2-p1).norm();

        Eigen::Matrix3d rotation1, rotation2;
        geometry_msgs::Point32 z_axis_vec;
        z_axis_vec.x = 0;
        z_axis_vec.y = 0;
        z_axis_vec.z = 1;
        std::vector<double> z_axis(3);
        z_axis[0] = 0;
        z_axis[1] = 0;
        z_axis[2] = 1;
        cloud_geometry::transforms::convertAxisAngleToRotationMatrix (z_axis_vec, M_PI*2.0*((double)j)/res_radial, rotation2);
        Eigen::Matrix4d transformation;
        cloud_geometry::transforms::getPlaneToPlaneTransformation (z_axis, axis_point,
            p1[0],
            p1[1],
            p1[2], transformation);
        
        Eigen::Vector3d p_0 (p.x, p.y, p.z);
        p_0 = rotation2 * p_0;
        Eigen::Vector4d p_1 (p_0[0], p_0[1], p_0[2], 1.0);
        Eigen::Vector4d q_0 = transformation * p_1;

        p.x = q_0[0];
        p.y = q_0[1];
        p.z = q_0[2];
        
        ret->points.push_back (p);
        triangle_mesh_msgs::Triangle tr;
        if (i > 0) // not first line
        {
          if (j > 0)
          {
            tr.i = cp - res_radial - 1;
            tr.j = cp - res_radial;
            tr.k = cp;
            ret->triangles.push_back (tr);
            tr.i = cp - res_radial - 1;
            tr.j = cp;
            tr.k = cp - 1;
            ret->triangles.push_back (tr);
          }
          else
          {
            tr.i = cp - 1;
            tr.j = cp - res_radial;
            tr.k = cp;
            ret->triangles.push_back (tr);
            tr.i = cp - 1;
            tr.j = cp;
            tr.k = cp + res_radial - 1;
            ret->triangles.push_back (tr);
          }
        }
        cp ++;
      }
    }
  }
}

