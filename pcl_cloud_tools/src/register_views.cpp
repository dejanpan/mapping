/*
 *  Experiment with efficent view registration/idetification methods.
 *
 *  Copywrong (K) 2010 Z.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * $Id: RegisterViews.cc,v 1.0 2010/30/10 12:00:00 zoli Exp $
 */
#include "../common/CommonTerminalRoutines.h"
#include "../common/CommonVTKRoutines.h"
#include "../common/CommonIORoutines.h"
#include "../common/CommonANNRoutines.h"
#include "../common/ANN-VTK.h"

#include <Eigen/Array>
#include <Eigen/Geometry>
#include <Eigen/SVD>

// for srand init
#include <time.h>

#define BUCKET_SIZE 30

using namespace std;
using namespace Eigen;

ANNkd_tree*  kd_tree;
ANNidxArray  nnIdx;
ANNdistArray sqrDists;

struct PCD
{
  PCD_Header header;
  ANNpointArray points;
};

double maxZdiff = 0.03;
double minAdiff = DEG2RAD(30);
int nxIdx, model_nxIdx;

inline bool potentialMatch (const ANNpoint p0, const ANNpoint p1, const bool check_angle)
{
  // if hight is ok
  if ((p0[2] - p1[2]) < maxZdiff)
  {
    // if angle to Z has to be checked
    if (check_angle)
      return (acos (p1[model_nxIdx+2]) > minAdiff);
    else
      return true;
  }
  return false;
}

inline Matrix4f computeTransformation (const ANNpoint target, const ANNpoint source, const bool rotation)
{
  // compute transformation
  Matrix4f transform = Matrix4f::Identity ();
  transform (0,3) = target[0] - source[0];
  transform (1,3) = target[1] - source[1];
  double angle2D = 0;
  if (rotation)
    angle2D = atan2 (target[nxIdx+1], target[nxIdx+0]) - atan2 (source[model_nxIdx+1], source[model_nxIdx+0]);
  else
    angle2D = rand () / ((RAND_MAX + 1.0) / (2*M_PI)); // random rotation
  transform (0,0) = +cos (angle2D);
  transform (0,1) = -sin (angle2D);
  transform (1,0) = -transform (0,1);
  transform (1,1) = +transform (0,0);
  return transform;
}

ANNpointArray evaluateTransformation (Matrix4f transform, vector<PCD>::iterator model, double sqr_threshold, int step, int limit_match, int limit_inliers, vector<int> &match_idx, vector<int> &tmp_inliers)
{
  int nr_check = model->header.nr_points / step;
  
  // evaluate transformation through exhaustive search
  match_idx.clear (); match_idx.reserve (nr_check);
  tmp_inliers.clear (); tmp_inliers.reserve (nr_check);
  ANNpointArray transformed = annAllocPts (model->header.nr_points, 3);
  int pos_match = nr_check;
  int pos_inliers = nr_check;
  for (int cp = 0; cp < model->header.nr_points; cp += step)
  {
    Vector4f sp (model->points[cp][0], model->points[cp][1], model->points[cp][2], 1);
    //ANNpoint point = annAllocPt (3);
    Map<Vector3f> tp (&(transformed[cp][0]));
    tp = (transform * sp).start<3> ();
    /*kd_tree->annkSearch (point, 1, nnIdx, sqrDists, 0.0);
    if (sqrDists[0] < sqr_threshold)
    {
      match_idx.push_back (cp);
      tmp_inliers.push_back (nnIdx[0]);
      pos_match++;
      pos_inliers++;
    }*/
    int nr_nn = kd_tree->annkFRSearch (transformed[cp], sqr_threshold, 0, NULL, NULL, 0.0);
    if (nr_nn > 0)
    {
      match_idx.push_back (cp);
      //kd_tree->annkSearch (point, 1, nnIdx, sqrDists, 0.0);
      kd_tree->annkFRSearch (transformed[cp], sqr_threshold, 1, nnIdx, sqrDists, 0.0);
      tmp_inliers.push_back (nnIdx[0]);
      pos_match++;
      pos_inliers++;
    }
    //annDeallocPt (point);
    // skip checking if there is no chance of producing a better match
    if (pos_match - cp/step < limit_match ||
        pos_inliers - cp/step < limit_inliers) // this will not exit early enough as tmp_inliers may contains duplicates as well, but it helps
      break;
  }
  return transformed;
}

/* ---[ */
int
  main (int argc, char** argv)
{
  srand (time (NULL));
  
  /// Init
  cANN::tictoc tt, t_global;
  double threshold = 0.03; // 3 cm
  double p_success = 0.999;
  double p2success = 0.999;
  int maxIterations = 100;
  int maxInnerIterations = 50;
  int minInnerIterations = 25;
  double percent = 50.0;
  int max_nr_nn = 1;
  double center_x, center_y;
  double center_th = -1;
  
  /// Check and info
  if (argc < 3)
  {
    print_error (stderr, "Syntax is: %s <scan>.pcd <result>.pcd <model1-N>.pcd <options>\n", argv[0]);
    fprintf (stderr, "  where options are: -threshold X = specify the inlier threshold (default "); print_value (stderr, "%g", threshold); fprintf (stderr, ")\n");
    fprintf (stderr, "                     -maxZdiff X  = specify the maximum height difference between corespoinding points (default "); print_value (stderr, "%g", maxZdiff); fprintf (stderr, ")\n");
    fprintf (stderr, "                     -max_iter N  = maximum number of outer RANSAC iteration (default "); print_value (stderr, "%d", maxIterations); fprintf (stderr, ")\n");
    fprintf (stderr, "                     -max2iter N  = maximum number of inner RANSAC iteration (default "); print_value (stderr, "%d", maxInnerIterations); fprintf (stderr, ")\n");
    fprintf (stderr, "                     -min2iter N  = minimum number of inner RANSAC iteration (default "); print_value (stderr, "%d", minInnerIterations); fprintf (stderr, ")\n");
    fprintf (stderr, "                     -p_success X = expected probability for a successful RANSAC match (default "); print_value (stderr, "%g", p_success); fprintf (stderr, ")\n");
    fprintf (stderr, "                     -p2success X = expected probability for finding the best transformation - exhaustive search if 1 (default "); print_value (stderr, "%g", p2success); fprintf (stderr, ")\n");
    fprintf (stderr, "                     -percent N   = use only this percent of model points for a pre-check (default "); print_value (stderr, "%g", percent); fprintf (stderr, ")\n");
    fprintf (stderr, "                     -max_nr_nn N = use at most this many neighbors when getting the final number of inliers (default "); print_value (stderr, "%g", max_nr_nn); fprintf (stderr, ")\n");
    fprintf (stderr, "                     -center_th X = check deviation in 2D of model center to specified x,y and reject if less than X -- disabled if < 0 (default "); print_value (stderr, "%g", center_th); fprintf (stderr, ")\n");
    fprintf (stderr, "                     -center X,Y  = if center_th > 0, the model's center should be closer to these coordinates in 2D than the threshold\n");
    fprintf (stderr, "\n");
    fprintf (stderr, "                     -message S   = message string to be saved as comment (default "); print_value (stderr, "\"\" - none"); fprintf (stderr, ")\n");
    fprintf (stderr, "                     -params 0/1  = disable/enable saving of parameters as comment (default "); print_value (stderr, "disabled"); fprintf (stderr, ")\n");
    fprintf (stderr, "                     -precision N = number of digits after decimal point (default "); print_value (stderr, "5"); fprintf (stderr, ")\n");
    return (-1);
  }
  
  /// Parsing arguments
  ParseArgument (argc, argv, "-threshold", threshold); double sqr_threshold = threshold * threshold;
  ParseArgument (argc, argv, "-maxZdiff", maxZdiff);
  ParseArgument (argc, argv, "-p_success", p_success);
  ParseArgument (argc, argv, "-p2success", p2success);
  ParseArgument (argc, argv, "-max_iter", maxIterations);
  ParseArgument (argc, argv, "-max2iter", maxInnerIterations);
  ParseArgument (argc, argv, "-min2iter", minInnerIterations);
  ParseArgument (argc, argv, "-percent", percent);
  ParseArgument (argc, argv, "-center_th", center_th); double sqr_center_th = center_th * center_th;
  ParseRangeArguments (argc, argv, "-center", center_x, center_y);
  int precision = 5; ParseArgument (argc, argv, "-precision", precision); cout.precision (precision);
  bool binary_output = true; ParseArgument (argc, argv, "-binary", binary_output);
  string message; ParseArgument (argc, argv, "-message", message); 
  bool params = false; ParseArgument (argc, argv, "-params", params);
  ParseArgument (argc, argv, "-max_nr_nn", max_nr_nn);
  
  print_info (stderr, "Probability of failure: %g (global), %g (local)\n", 1-p_success, 1-p2success);

  /// @TODO: would actually creating a model kd-tree and searching target points in it be faster?
  if (max_nr_nn < 1)
    max_nr_nn = 1;
  nnIdx    = new ANNidx[max_nr_nn];
  sqrDists = new ANNdist[max_nr_nn];

  /// Take only the first two .pcd files into account
  std::vector<int> pPCDFileIndices = ParseFileExtensionArgument (argc, argv, ".pcd");
  if (pPCDFileIndices.size () < 3)
  {
    print_error (stderr, "Need a scan, a result and at elast one model .PCD file!\n");
    return (-1);
  }

  /// Load the points from file
  PCD_Header header;
  print_info (stderr, "Loading ");
  print_value (stderr, "%s... ", argv[pPCDFileIndices.at (0)]);
  ANNpointArray points = LoadPCDFile (argv[pPCDFileIndices.at (0)], header);
  if (points == NULL)
    return (-1);
  fprintf (stderr, "[done : "); print_value (stderr, "%d %d", header.nr_points, header.dimID.size ()); fprintf (stderr, "D points]\n");
  print_info (stderr, "Available dimensions: "); print_value (stderr, "%s\n", getAvailableDimensions (header).c_str ());
  if (header.comments.size () != 0)
  {
    print_value (stderr, "Header comments:\n");
    fprintf (stderr, "%s", header.comments.c_str ());
  }
  //unsigned nr_dims = header.dimID.size ();

  /// Checking dimensions
  int xIdx = getIndex (header, "x");
  if (xIdx != 0)
  {
    print_error (stderr, "XYZ coordinates not first dimensions!\n");
    return (-1);
  }
  nxIdx = getIndex (header, "nx");
  if (nxIdx == -1)
  {
    print_error (stderr, "Missing normal information!\n");
    return (-1);
  }

  /// Create a kD-tree
  kd_tree  = new ANNkd_tree (points, header.nr_points, 3, BUCKET_SIZE);
  //ANNkd_tree*  kd_tree  = new ANNkd_tree (points, header.nr_points, 3, BUCKET_SIZE);
  //ANNidxArray  nnIdx    = new ANNidx[1];
  //ANNdistArray sqrDists = new ANNdist[1];
  
  /// Load the models from the files
  vector<PCD> models (pPCDFileIndices.size () - 2);
  /**
  for (unsigned i = 2; i < pPCDFileIndices.size (); i++)
  {
    PCD_Header header2;
    print_info (stderr, "Loading model ");
    print_value (stderr, "%s... ", argv[pPCDFileIndices.at (i)]);
    ANNpointArray points2 = LoadPCDFile (argv[pPCDFileIndices.at (i)], header2);
    if (points == NULL)
      return (-1);
    fprintf (stderr, "[done : "); print_value (stderr, "%d %d", header2.nr_points, header2.dimID.size ()); fprintf (stderr, "D points]\n");
    print_info (stderr, "Available dimensions: "); print_value (stderr, "%s\n", getAvailableDimensions (header2).c_str ());
    //if (header2.comments.size () != 0)
    //{
    //  print_value (stderr, "Header comments:\n");
    //  fprintf (stderr, "%s", header2.comments.c_str ());
    //}
  
    /// Checking dimensions
    int xIdx = getIndex (header2, "x");
    if (xIdx != 0)
    {
      print_error (stderr, "XYZ coordinates not first dimensions!\n");
      return (-1);
    }
  
    /// Saving reference in list
    models[i-2].points = points2;
    models[i-2].header = header2;
    //cerr << models[i-2].points << endl;
    //fprintf (stderr, "Saved %s header: ", models[i-2].header.data_type?"ASCII":"BINARY"); print_value (stderr, "%d %d", models[i-2].header.nr_points, models[i-2].header.dimID.size ());
    //fprintf (stderr, "D "); print_value (stderr, "%s\n", getAvailableDimensions (models[i-2].header).c_str ());
    //if (models[i-2].header.comments.size () != 0)
    //  fprintf (stderr, "%s", models[i-2].header.comments.c_str ());
  }
  //*/
  
  /// Starting global timer  
  t_global.tic ();
  
  /// Registering each model to the scan and find the best
  vector<PCD>::iterator best_model;
  std::vector<int> best_model_inliers;
  ANNpointArray best_model_result = NULL;
  Matrix4f best_model_transform;
  int best_i = -1;
  int sum_pts = 0;
  ///for (vector<PCD>::iterator model = models.begin (); model != models.end (); model++)
  for (unsigned i = 2; i < pPCDFileIndices.size (); i++)
  {
    PCD_Header header2;
    print_info (stderr, "\nLoading model ");
    print_value (stderr, "%s... ", argv[pPCDFileIndices.at (i)]);
    ANNpointArray points2 = LoadPCDFile (argv[pPCDFileIndices.at (i)], header2);
    if (points == NULL)
      return (-1);
    fprintf (stderr, "[done : "); print_value (stderr, "%d %d", header2.nr_points, header2.dimID.size ()); fprintf (stderr, "D points]\n");
    print_info (stderr, "Available dimensions: "); print_value (stderr, "%s\n", getAvailableDimensions (header2).c_str ());
    //if (header2.comments.size () != 0)
    //{
    //  print_value (stderr, "Header comments:\n");
    //  fprintf (stderr, "%s", header2.comments.c_str ());
    //}
  
    /// Checking dimensions
    int xIdx = getIndex (header2, "x");
    if (xIdx != 0)
    {
      print_error (stderr, "XYZ coordinates not first dimensions!\n");
      return (-1);
    }
    model_nxIdx = getIndex (header2, "nx");
    if (nxIdx == -1)
    {
      print_error (stderr, "Missing normal information!\n");
      return (-1);
    }
  
    /// Saving reference in list
    models[i-2].points = points2;
    models[i-2].header = header2;
    vector<PCD>::iterator model = models.begin () + i - 2;
    sum_pts += model->header.nr_points;
    
    //cerr << model->points << endl;
    //fprintf (stderr, "Loaded %s header: ", model->header.data_type?"ASCII":"BINARY"); print_value (stderr, "%d %d", model->header.nr_points, model->header.dimID.size ());
    //fprintf (stderr, "D "); print_value (stderr, "%s\n", getAvailableDimensions (model->header).c_str ());
    //if (model->header.comments.size () != 0)
    //  fprintf (stderr, "%s", model->header.comments.c_str ());
    
    int nr_check = (int)(model->header.nr_points * percent/100.0); // can introduce numeric errors, so we'll re-set it
    //int max_check = 1500;
    //if (nr_check > max_check) nr_check = max_check;
    int step = model->header.nr_points / nr_check;
    nr_check = (model->header.nr_points-1) / step + 1; // more accurate than nr_points/step?
    print_info (stderr, "Using %g%%: ", percent); print_value (stderr, "%d/%d", nr_check, model->header.nr_points); fprintf (stderr, " points for pre-checking, with a step of "); print_value (stderr, "%d\n", step);
    
    int iterations = 0;
    double k = (double)maxIterations; /// <1> or <min_iter> instead of this?
  
    std::vector<int> best_inliers, inliers;
    ANNpointArray best_result = NULL;
    Matrix4f best_transform;
  
    /// Starting local timer  
    tt.tic ();
    
    /// Iterate
    int used_rot = 0;
    int trial = 0;
    long sum_iter = 0;
    while (iterations < k)
    {
      // Get a point from the scan
      int target = 0;
      while (header.nr_points <= (target = rand() / (RAND_MAX/header.nr_points)));

      // if normal not parallel to Z, use it to align model with the PCD
      bool use_rotation = false;
      if (acos (fabs (points[target][nxIdx+2])) > minAdiff)
      {
        used_rot++;
        use_rotation = true;
      }
  
      // Search for the best matching model point
      ANNpointArray result = NULL;
      Matrix4f match_transform;
      
      /// @TODO save these for outside the RANSAC loop as well?
      vector<int> best_match_idx;
      int best_source;
      
      /// Iterate
      bool found_correspondence = false;
      if (p2success < 1.0)
      {
        int iterations2 = 0;
        int trial2 = 0;
        double k2 = (double)maxInnerIterations; /// <1> or <min_iter> instead of this?
        while (iterations2 < k2 || iterations2 < minInnerIterations)
        {
          // get a suitable point from the model
          int source = 0;
          while (model->header.nr_points <= (source = rand() / (RAND_MAX/model->header.nr_points)));

          // skip points that have no correspondence, but give up if too many
          if (!potentialMatch (points[target], model->points[source], use_rotation))
          {
            trial2++;
            if (trial2 < maxInnerIterations)
              continue;
            else
              break;
          }
          trial2 = 0;
          found_correspondence = true;
          
          // get transform and evaluate
          Matrix4f transform = computeTransformation (points[target], model->points[source], use_rotation);
          vector<int> match_idx, tmp_inliers;
          ANNpointArray transformed = evaluateTransformation (transform, model, sqr_threshold, step, best_match_idx.size (), best_model_inliers.size (), match_idx, tmp_inliers);
          
//          // check centroid
//          if (SQR(transform(0,3)-center_x) + SQR(transform(1,3)-center_y) > sqr_center_th)
//          {
//            nr_skipped++;
//            annDeallocPts (transformed);
//            continue;
//          }

          // save best transformation
          if (best_match_idx.size () < match_idx.size ())
          {
            if (result != NULL) annDeallocPts (result);
            match_transform = transform;
            best_match_idx = match_idx;
            result = transformed;
            inliers = tmp_inliers;
            best_source = source;
            
            // compute the k2 parameter (k2=log(z)/log(1-w^n))
            double w = match_idx.size () / (double)nr_check;
            double pNoOutliers = 1 - w;
            pNoOutliers = max (std::numeric_limits<double>::epsilon (), pNoOutliers);       // Avoid division by -Inf
            pNoOutliers = min (1 - std::numeric_limits<double>::epsilon (), pNoOutliers);   // Avoid division by 0.
            k2 = log (1 - p2success) / log (pNoOutliers);
            
            //print_info (stderr, "[MODEL] Trial %d out of %g: best is: %d/%d so far.\n" , iterations2, ceil (k2), best_match_idx.size (), nr_check);
          }
          else
            annDeallocPts (transformed);
          
          iterations2++;
          //#if DEBUG
          //print_info (stderr, "[MODEL] Trial %d out of %g: %d inliers (best is: %d so far).\n" , iterations2, ceil (k2), match_idx.size (), best_match_idx.size ());
          //#endif
          if (iterations2 >= maxInnerIterations)
          {
            //print_warning (stderr, "[MODEL-RANSAC] reached the maximum number of trials: %d\n", maxInnerIterations);
            break;
          }
        }
        //print_info (stderr, "[MODEL-RANSAC] done %d iterations\n", iterations2);// in ", iterations); print_value (stderr, "%g", tt.toc ()); fprintf (stderr, " seconds\n");
        sum_iter += iterations2;
      }
      else
      {
        for (int source = 0; source < model->header.nr_points; source++)
        {
          // check only point correspondences that have a high chance of producing a good transform
          if (!potentialMatch (points[target], model->points[source], use_rotation))
            continue;
          found_correspondence = true;

          // get transform and evaluate
          Matrix4f transform = computeTransformation (points[target], model->points[source], use_rotation);
          vector<int> match_idx, tmp_inliers;
          ANNpointArray transformed = evaluateTransformation (transform, model, sqr_threshold, step, best_match_idx.size (), best_model_inliers.size (), match_idx, tmp_inliers);
          
//          // check centroid
//          if (SQR(transform(0,3)-center_x) + SQR(transform(1,3)-center_y) > sqr_center_th)
//          {
//            nr_skipped++;
//            annDeallocPts (transformed);
//            continue;
//          }

          // save best transformation
          if (best_match_idx.size () < match_idx.size ())
          {
            if (result != NULL) annDeallocPts (result);
            match_transform = transform;
            best_match_idx = match_idx;
            result = transformed;
            inliers = tmp_inliers;
            //print_info (stderr, "[MODEL] Trial %d out of %ld: best is: %d so far.\n" , source, model->header.nr_points, best_match_idx.size ());
            best_source = source;
          }
          else
            annDeallocPts (transformed);
        }
        sum_iter += model->header.nr_points;
      }
      
      // skip points that have no correspondence, but give up if too many
      if (!found_correspondence)
      {
        trial++;
        if (trial < maxIterations)
          continue;
        else
          break;
      }
      trial = 0;

      // get the final number of inliers
      if (best_match_idx.size () > 0)
      {
        //print_info (stderr, "[MODEL-%d] Best match (%d) has ", target, best_source);
        //fprintf (stderr, "%d/%g%%", best_match_idx.size (), 100*best_match_idx.size () / (double)nr_check);
        //fprintf (stderr, " inliers\n");
        
        /// @TODO: make this nicer
///        if (step != 1)
///        {
          //cerr << "before: " << inliers.size () << endl;
///
          inliers.clear ();
          inliers.reserve (model->header.nr_points);
          //int pos_match = model->header.nr_points;
          int pos_inliers = model->header.nr_points;
          for (int cp = 0; cp < model->header.nr_points; cp++)
          {
            /// @TODO: why is this not working if after the skip? shouldn't they be transformed in the function call already?
            Vector4f sp (model->points[cp][0], model->points[cp][1], model->points[cp][2], 1);
            Map<Vector3f> tp (&(result[cp][0]));
            tp = (match_transform * sp).start<3> ();
            // skip points that were already checked
///
///            if (cp % step == 0)
///              continue;
            //if (cp == 4)
            //{
            //  cerr << sp.transpose () << " - " << tp.transpose () << endl;
            //  cerr << match_transform << endl;
            //}
            int nr_nn = kd_tree->annkFRSearch (result[cp], sqr_threshold, 0, NULL, NULL, 0.0);
            //cerr << nr_nn << "/";
            if (nr_nn > 0)
            {
              //cerr << nnIdx[0] << " ";
              //match_idx.push_back (cp);
              if (nr_nn > max_nr_nn)
                nr_nn = max_nr_nn;
              kd_tree->annkFRSearch (result[cp], sqr_threshold, nr_nn, nnIdx, sqrDists, 0.0);
              for (int nn = 0; nn < nr_nn; nn++)
                inliers.push_back (nnIdx[nn]);
              //pos_match++;
              pos_inliers++;
            }
            // skip checking if there is no chance of producing a better match
            if (//pos_match - cp < best_match_idx.size () ||
                pos_inliers - cp < (int)best_model_inliers.size ()) // this will not exit early enough as tmp_inliers may contains duplicates as well, but it helps
              break;
          }
          //cerr << "after: " << inliers.size () << endl;
///        }
      
        // Remove doubles from inliers list
        sort (inliers.begin (), inliers.end ());
        inliers.erase (unique (inliers.begin (), inliers.end ()), inliers.end ());
        //cerr << "unique: " << inliers.size () << endl;
        
        // Better match?
        if (best_inliers.size () < inliers.size ())
        {
          if (best_result != NULL) annDeallocPts (best_result);
          best_inliers = inliers;
          best_result = result;
          best_transform = match_transform;
          //inliers.clear ();
    
          // compute the k parameter (k=log(z)/log(1-w^n))
          double w = inliers.size () / (double)header.nr_points;
          double pNoOutliers = 1 - w;
          pNoOutliers = max (std::numeric_limits<double>::epsilon (), pNoOutliers);       // Avoid division by -Inf
          pNoOutliers = min (1 - std::numeric_limits<double>::epsilon (), pNoOutliers);   // Avoid division by 0.
          k = log (1 - p_success) / log (pNoOutliers);
        }
        else
          annDeallocPts (result);
      }
      //else
      //  print_error (stderr, "[MODEL] Unable to find a solution!\n");
  
      iterations++;
      //#if DEBUG
      //print_info (stderr, "Trial %d out of %g: %d inliers (best is: %d so far).\n" , iterations, ceil (k), inliers.size (), best_inliers.size ());
      //#endif
      if (iterations >= maxIterations)
      {
        print_warning (stderr, "RANSAC reached the maximum number of trials\n");
        break;
      }
    }
    print_info (stderr, "[RANSAC] done %d iterations (%d with rotations, %ld internal loops) in ", iterations, used_rot, sum_iter); print_value (stderr, "%g", tt.toc ()); fprintf (stderr, " seconds using %s search\n", p2success<1?"sac":"exchaustive");
  
    if (best_inliers.size () > 0)
    {
      print_info (stderr, "[RANSAC] Best match has ");
      print_value (stderr, "%d/%g%%", best_inliers.size (), 100*best_inliers.size () / (double)header.nr_points);
      fprintf (stderr, " inliers\n");
      
      // check centroid
      print_info(stderr, "Distance to (%g,%g) is %g\n", center_x, center_y, sqrt (SQR(best_transform(0,3)-center_x) + SQR(best_transform(1,3)-center_y)));

      if (best_model_inliers.size () < best_inliers.size ())
      {
        if (best_model_result != NULL) annDeallocPts (best_model_result);
        best_model_transform = best_transform;
        best_model = model;
        best_model_inliers = best_inliers;
        best_model_result = best_result;
        best_i = i;
      }
      else
        annDeallocPts (best_result);
    }
    else
      print_error (stderr, "[RANSAC] Unable to find a solution!\n");
  }
  
  /// Save the best model
  if (best_model_inliers.size () > 0)
  {
    print_info (stderr, "\nBest model was loaded from model file ");
    print_value (stderr, "%d: %s\n", best_i-1, argv[pPCDFileIndices.at (best_i)]);
    print_info (stderr, "Best model out of ");
    print_value (stderr, "%d", pPCDFileIndices.size () - 2);
    fprintf (stderr, " has ");
    print_value (stderr, "%d/%g%%", best_model_inliers.size (), 100*best_model_inliers.size () / (double)header.nr_points);
    fprintf (stderr, " inliers\n");
    print_info (stderr, "Transformation between best model and scan:\n");
    cerr << best_model_transform << endl;
    
    // precaution in case not all points are saved
    //best_model->header.nr_points = best_model_result.size ();
    // (int)(model->header.nr_points * percent/100.0)
    best_model->header.dimID.resize (3);
    print_info (stderr, "Resulting dimensions: %s\n", getAvailableDimensions (best_model->header).c_str ());
    
    // Save
    if (message.size ())
      addCommentToHeader (best_model->header, "%s\n", message.c_str ());
    if (params)
    {
      string parameters (argv[0]);
      for (int i = 1; i < argc; i++)
      {
        parameters.append (" ");
        parameters.append (argv[i]);
      }
      addCommentToHeader (best_model->header, "%s\n", parameters.c_str ());
    }
    print_info (stderr, "Writing resulting "); print_value (stderr, "%d", best_model->header.nr_points); fprintf (stderr, " points to "); print_value (stderr, "%s ", argv[pPCDFileIndices.at (1)]);// fprintf (stderr, "... ");
    best_model->header.data_type = binary_output ? PCD_BINARY : PCD_ASCII;
    SavePCDFile (argv[pPCDFileIndices.at (1)], best_model_result, best_model->header, precision);
    fprintf (stderr, "[done]\n");
  }
  else
    print_error (stderr, "Did not find a suitable solution!\n");
  
  /// Done
  print_info (stderr, "[done searching through "); print_value (stderr, "%d", sum_pts); fprintf (stderr, " model points in "); print_value (stderr, "%g", t_global.toc ()); fprintf (stderr, " seconds]\n");
  return 0;
}
/* ]--- */

