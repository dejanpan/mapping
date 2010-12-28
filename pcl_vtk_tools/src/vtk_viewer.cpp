/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
  * \author Radu Bogdan Rusu
  * \author prelimiary port Dejan Pangercic
  *
  * @b vtk_viewer visualizes .vtk and .ply formats
  * Press w(wireframe), s(surface) or p(point) to switch between modes.
  */

#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <sstream>

//terminal_tools includes  
#include <terminal_tools/print.h>
#include <terminal_tools/parse.h>
#include <terminal_tools/time.h>

#include <vtk3DSImporter.h>
#include <vtkDataSet.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPolyDataReader.h>
#include <vtkDataSetMapper.h>
#include <vtkMaskPoints.h>
#include <vtkLODActor.h> 
#include <vtkFloatArray.h>
#include <vtkPointData.h> 
#include <vtkVectorText.h> 
#include <vtkPolyDataMapper.h> 
#include <vtkFollower.h> 
#include <vtkLightKit.h> 

#include <pcl_vtk_tools/misc.h>

using namespace std;
using terminal_tools::print_color;
using terminal_tools::print_error;
using terminal_tools::print_warn;
using terminal_tools::print_info;
using terminal_tools::print_debug;
using terminal_tools::print_value;
using terminal_tools::print_highlight;
using terminal_tools::TT_BRIGHT;
using terminal_tools::TT_RED;
using terminal_tools::TT_GREEN;
using terminal_tools::TT_BLUE;

#define SQR(x) ((x)*(x))
//  dataPruner->PointMergingOn ();

////////////////////////////////////////////////////////////////////////////////
void
GetRandomColors (double &r, double &g, double &b)
{
  r = (double)(rand () / (RAND_MAX + 1.0));
  g = (double)(rand () / (RAND_MAX + 1.0));
  b = (double)(rand () / (RAND_MAX + 1.0));
  print_warn (stderr, "Using the following random colors: [");
  print_value (stderr, "%g", r); print_warn (stderr, ",");
  print_value (stderr, "%g", g); print_warn (stderr, ",");
  print_value (stderr, "%g", b);
  print_warn (stderr, "] as foreground color.\n");
}

/* ---[ */
int
main (int argc, char** argv)
{
  terminal_tools::TicToc tictoc;
  srand ((unsigned)time (0));
  vector<int> pFileIndices;

  vector<int> psize;
  double scale = 0.0;
  bool lut_enable = false;
  bool lod_enable = false;
  bool cell_scalar = false;
  bool no_shadows = false;

  if (argc < 2)
  {
    print_error (stderr, "Syntax is: %s [fileName1..N] <options>\n", argv[0]);
    fprintf (stderr, "  where options are: -fc r,g,b = foreground color\n");
    fprintf (stderr, "                     -ps X     = point size\n");
    fprintf (stderr, "                     -lw X     = line width\n");
    fprintf (stderr, "                     -sc X     = add X to scale the color (useful when using remission/distance)\n");
    fprintf (stderr, "                     -lod 0/1  = use a LOD (Level Of Detail) actor instead (enabled by default)\n");
    fprintf (stderr, "                     -text X   = add point indices as text labels with size as X * BB_DIAM (disabled by default)\n");
    fprintf (stderr, "                     -minmaxIdx N,M  = use a minimum/maximum threshold for indices to be shown. OPTIONAL\n");
    fprintf (stderr, "                     -lut_enable 0/1 = add a color rainbow legend (disabled by default)\n");
    fprintf (stderr, "                     -save_cam 0/1   = save the last camera position to file on exit (default "); print_value (stderr, "enabled"); fprintf (stderr, ")\n");
    fprintf (stderr, "                     -movie 0/1      = {dis/en}able movie mode (default "); print_value (stderr, "disabled"); fprintf (stderr, ")"); print_error (stderr, " NOTE: resolution set at 640x480!\n");
    fprintf (stderr, "                     -cell 0/1       = put colors as scalars for cell data (1) or point data (0) (default : "); print_value (stderr, "point"); fprintf (stderr, ")\n");
    fprintf (stderr, "                     -no_shadows 0/1 = turns shadows off (default : "); print_value (stderr, "disabled"); fprintf (stderr, ")\n");
    fprintf (stderr, "\n");
    fprintf (stderr, "                     -play   0/1    = replays the events and motions specified in the given <logdata.log> file\n");
    //  _print_default_CommonVTK_options_ ();
    return (-1);
  }
  int minIdx = 0, maxIdx = INT_MAX;
  terminal_tools::parse_2x_arguments (argc, argv, "-minmaxIdx", minIdx, maxIdx);
  
  bool play = false;
  terminal_tools::parse_argument (argc, argv, "-play", play);
  std::vector<int> pLogFileIndices;
  if (play)
  {
    pLogFileIndices = terminal_tools::parse_file_extension_argument (argc, argv, ".log");
    if (pLogFileIndices.size () != 1)
    {
      print_error (stderr, "Need exactly 1 logfile!\n"); 
      return (-1);
    }
  }
  bool save_camera_position = true;
  terminal_tools::parse_argument (argc, argv, "-save_cam", save_camera_position);
  terminal_tools::parse_argument (argc, argv, "-cell", cell_scalar);
  terminal_tools::parse_argument (argc, argv, "-no_shadows", no_shadows);
  
  double text = 0;
  terminal_tools::parse_argument (argc, argv, "-text", text);
  int movieMode = 0;
  terminal_tools::parse_argument (argc, argv, "-movie", movieMode);
  const char* movieFile = "output.avi";

  double fcolor[3] = {0.9, 0.9, 0.9};
  std::vector<double> fcolorR, fcolorB, fcolorG;
  bool fcolorparam = terminal_tools::parse_multiple_3x_arguments (argc, argv, "-fc", fcolorR, fcolorG, fcolorB);
   
  double line_width = -1;
  terminal_tools::parse_argument (argc, argv, "-lw", line_width);
  terminal_tools::parse_multiple_arguments (argc, argv, "-ps", psize);
  terminal_tools::parse_argument (argc, argv, "-sc", scale);
  terminal_tools::parse_argument (argc, argv, "-lut", lut_enable);
  terminal_tools::parse_argument (argc, argv, "-lod", lod_enable);
  if (lod_enable)
    print_info (stdout, "LOD enabled.\n");

  // Parse the command line arguments for .vtk or .ply files
  pFileIndices = terminal_tools::parse_file_extension_argument (argc, argv, ".vtk");;
  //TODO: implement parser for ply too

  vtkSmartPointer<vtkPolyData> data;
  
  // Create Renderer
  vtkRenderer* ren = vtkRenderer::New ();
  //ren->AutomaticLightCreationOff ();
  
  // Fix the PSize bug
  if (psize.size () != pFileIndices.size ())
    for (unsigned int i = psize.size (); i < pFileIndices.size (); i++)
      psize.push_back (1);

  // Create the dataset files
  for (unsigned int i = 0; i < pFileIndices.size (); i++)
  {
    vtkActor *dataActor;
    double minmax[2];
    print_info (stderr, "Loading ");
    print_value (stderr, "%s ... ", argv[pFileIndices.at(i)]);
    tictoc.tic ();
    if (pFileIndices.size() != 0)
    {
      data = load_poly_data_as_data_set(argv[pFileIndices.at (i)]);
      //stringstream filename_stream;
      //filename_stream << argv[pFileIndices.at (0)];
      //filename = filename_stream.str();
      print_info ("Loading vtk model with %d vertices/points.", (int)data->GetNumberOfPoints ());
    }

    data->GetScalarRange (minmax);
    //  data->GetPointData ()->GetScalars ()->SetLookupTable (lut);
    data->Update ();
    fprintf (stderr, "[done, "); print_value (stderr, "%g", tictoc.toc ()); fprintf (stderr, " seconds : "); print_value (stderr, "%d", data->GetNumberOfPoints ()); fprintf (stderr, " 3D points]\n");

    // If no scalars, select the colors ourselves
    if ((minmax[0] == 0) && (minmax[1] == 1))
    {
      if (!fcolorparam)
        GetRandomColors (fcolor[0], fcolor[1], fcolor[2]);
      // Color parameters given
      else
	    {
	      // Do we have enough parameters for every point cloud ?
	      if (fcolorR.size () > i)
        {
          fcolor[0] = fcolorR[i];
          fcolor[1] = fcolorG[i];
          fcolor[2] = fcolorB[i];
        }
	      // No, select a random color for this one
	      else
          GetRandomColors (fcolor[0], fcolor[1], fcolor[2]);
	    }

      //dataActor = createActorFromDataSet (data[i], fcolor[0], fcolor[1], fcolor[2], psize[i], lod_enable);
      dataActor = create_actor_from_data_set(data, fcolor[0], fcolor[1], fcolor[2], psize[i], lod_enable);
    }
    else
    {
      print_info (stderr, "Scalar values found for %s: ", argv[pFileIndices.at (i)]);
      print_value (stderr, "%g", minmax[0]); fprintf (stderr, " -> ");
      print_value (stderr, "%g\n", minmax[1]);

      // Still, if the foreground color is given, overwrite the scalar values
      //      if (fcolorparam)
      //        dataActor = createActorFromDataSet (data[i], fcolor, psize[i], false);
      // Display the colors using a Lookup table for the scalar values
      //      else
      {
        vtkSmartPointer<vtkLookupTable> lut = create_LUT (minmax, argc, argv);

        if (lut_enable)
          ren->AddActor (create_scalar_bar_actor (lut, argc, argv));
        
        //Create the data actor 
        if (scale > 0)
        {
          vtkFloatArray *distances = (vtkFloatArray*)data->GetPointData ()->GetScalars ();
          for (long int j = 0; j < distances->GetNumberOfTuples (); j++)
          {
            double c[1];
            distances->GetTuple (j, c);
            c[0] += scale;
            distances->SetTuple (j, c);
          }
          
          data->GetPointData ()->SetScalars (distances);
        }
        dataActor = create_actor_from_data_set (data, psize[i], lut, minmax, lod_enable);
        if (cell_scalar)
          dataActor->GetMapper ()->SetScalarModeToUseCellData ();
        //dataActor = create_actor_from_data_set (data, psize, lod_enable);
      }
    }
    dataActor->GetProperty ()->SetRepresentationToPoints ();
    
    // Disable shadows if necessary
    if (no_shadows)
    {
      dataActor->GetProperty ()->ShadingOff (); // is this needed?
      dataActor->GetProperty ()->SetAmbient (1);
      dataActor->GetProperty ()->SetDiffuse (0);
    }

    if (line_width != -1)
      dataActor->GetProperty ()->SetLineWidth (line_width);
    
    ren->AddActor (dataActor);
    
    if ((text > 0) && (i == 0))
    {
      char idx[10];
      //vtkPolyData* polydata=reinterpret_cast<vtkPolyData*>(data);
      double bounds[6];
      data->GetBounds (bounds);
      double text_scale = text * sqrt ( SQR(bounds[0]-bounds[1]) + SQR(bounds[2]-bounds[3]) +SQR(bounds[4]-bounds[5]) );
      double point[3];
      int start = max (minIdx, 0);
      int stop = min (maxIdx, int(data->GetNumberOfPoints ())-1);
      print_info (stderr, "Labeling points with their indices in ");
      print_value (stderr, "[%d,%d]\n", start, stop);
      for (int j = start; j <= stop; j++)
      {
        data->GetPoint (j, point);
        sprintf (idx, "%d", j);
        vtkVectorText *atext = vtkVectorText::New ();
        atext->SetText (idx);
        vtkPolyDataMapper *textMapper = vtkPolyDataMapper::New ();
        textMapper->SetInputConnection (atext->GetOutputPort ());
        vtkFollower *textActor = vtkFollower::New ();
        textActor->SetMapper (textMapper);
        textActor->SetScale (text_scale);
        textActor->SetPosition (point[0], point[1], point[2]);
        //textActor->AddPosition (0, -0.1, 0);
        textActor->SetCamera (ren->GetActiveCamera ());
        ren->AddActor (textActor);
      }
    }
  }

  vtkLightKit* lightKit = vtkLightKit::New ();
  lightKit->AddLightsToRenderer (ren);

  // Create the VTK window
  char title[256];
  if (pFileIndices.size () == 1)
    sprintf (title, "3D PointCloud viewer - %s", argv[pFileIndices.at (0)]);
  else
    sprintf (title, "3D PointCloud viewer");
  vtkRenderWindowInteractor* iren;
  //   if (movieMode)
  //   {
  //     print_info (stderr, "Movie file is being written to: ");
  //     print_value (stderr, "%s\n", movieFile);
  // //    iren = CreatePerspectiveRenderWindowAndInteractorMovie (ren, title, movieFile, 44.382, argc, argv, 2);
  //     iren = CreateRenderWindowAndInteractorMovie (ren, title, movieFile, argc, argv, 2);
  //   }
  //   elses
  iren = CreateRenderWindowAndInteractor (ren, title, argc, argv);

  vtkSmartPointer<vtkInteractorStyleTUM> style = (vtkSmartPointer<vtkInteractorStyleTUM>)reinterpret_cast<vtkInteractorStyleTUM*>(iren->GetInteractorStyle ());
  style->setAdvancedMode (true);
  
  if (play)
  {
    vtkSmartPointer<vtkInteractorEventRecorder> recorder = vtkSmartPointer<vtkInteractorEventRecorder>::New ();
    recorder->SetInteractor (iren);
    recorder->SetFileName (argv[pLogFileIndices.at (0)]);
    recorder->SetEnabled (1);
    recorder->Play ();
  }
  
  iren->Start ();
  //   if (save_camera_position)
  //     SaveCameraPosition (ren->GetActiveCamera (), false);
}
/* ]--- */
