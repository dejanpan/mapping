/*
 *  Export a VTK file into a different format (using vtkExporter)
 *  Copywrong (K) 2007 R.
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
 * $Id: Viewer.cc,v 1.0 2006/10/04 12:00:00 radu Exp $
 */
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <sstream>

//#include "../common/CommonVTKRoutines.h"
//#include "../common/CommonTerminalRoutines.h"
//#include "../common/CommonANNRoutines.h"

//terminal_tools includes  
#include <terminal_tools/print.h>
#include <terminal_tools/parse.h>
#include <terminal_tools/time.h>

//VTK dependencies -- still needed??
#include <vtk3DSImporter.h>
#include <vtkDataSet.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPolyDataReader.h>
#include <vtkDataSetMapper.h>
#include <vtkMaskPoints.h>
#include <vtkLODActor.h> 

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

////////////////////////////////////////////////////////////////////////////////
// Loads a 3D point cloud from a given fileName.
// Returns: a vtkPolyData object containing the point cloud.
// vtkPolyData*
// load_poly_data_as_data_set (const char* fileName)
// {
//   vtkPolyDataReader* reader = vtkPolyDataReader::New ();
//   reader->SetFileName (fileName);
//   reader->Update ();
//   return reader->GetOutput ();
// }


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
   //ParseRangeArguments (argc, argv, "-minmaxIdx", minIdx, maxIdx);
   terminal_tools::parse_2x_arguments (argc, argv, "-minmaxIdx", minIdx, maxIdx);
  
//   // Parse the command line arguments for .log files
   bool play = false;
//   ParseArgument (argc, argv, "-play", play);
   terminal_tools::parse_argument (argc, argv, "-play", play);
   std::vector<int> pLogFileIndices;
   if (play)
   {
     //pLogFileIndices = ParseFileExtensionArgument (argc, argv, ".log");
     pLogFileIndices = terminal_tools::parse_file_extension_argument (argc, argv, ".log");
     if (pLogFileIndices.size () != 1)
     {
       print_error (stderr, "Need exactly 1 logfile!\n"); 
       return (-1);
     }
   }
   bool save_camera_position = true;
//   ParseArgument (argc, argv, "-save_cam", save_camera_position);
   terminal_tools::parse_argument (argc, argv, "-save_cam", save_camera_position);

//   ParseArgument (argc, argv, "-cell", cell_scalar);
   terminal_tools::parse_argument (argc, argv, "-cell", cell_scalar);
//   ParseArgument (argc, argv, "-no_shadows", no_shadows);
   terminal_tools::parse_argument (argc, argv, "-no_shadows", no_shadows);
  
   double text = 0;
//   ParseArgument (argc, argv, "-text", text);
   terminal_tools::parse_argument (argc, argv, "-text", text);
   int movieMode = 0;
//   ParseArgument (argc, argv, "-movie", movieMode);
   terminal_tools::parse_argument (argc, argv, "-movie", movieMode);
   const char* movieFile = "output.avi";

   double fcolor[3] = {0.9, 0.9, 0.9};
   //  bool fcolorparam = Parse3xArguments (argc, argv, "-fc", fcolor[0], fcolor[1], fcolor[2]);
   std::vector<double> fcolorR, fcolorB, fcolorG;
   //bool fcolorparam = ParseMultiple3xArguments (argc, argv, "-fc", fcolorR, fcolorG, fcolorB);
   bool fcolorparam = terminal_tools::parse_multiple_3x_arguments (argc, argv, "-fc", fcolorR, fcolorG, fcolorB);
   
   double line_width = -1;
//   ParseArgument (argc, argv, "-lw", line_width);
   terminal_tools::parse_argument (argc, argv, "-lw", line_width);
//   ParseMultipleArguments (argc, argv, "-ps", psize);
   terminal_tools::parse_multiple_arguments (argc, argv, "-ps", psize);
//   ParseArgument (argc, argv, "-sc", scale);
   terminal_tools::parse_argument (argc, argv, "-sc", scale);
//   ParseArgument (argc, argv, "-lut", lut_enable);
   terminal_tools::parse_argument (argc, argv, "-lut", lut_enable);
//   ParseArgument (argc, argv, "-lod", lod_enable);
   terminal_tools::parse_argument (argc, argv, "-lod", lod_enable);
   if (lod_enable)
     print_info (stdout, "LOD enabled.\n");

   // Parse the command line arguments for .vtk or .ply files
   //pFileIndices = ParseFileNamesArgument (argc, argv);
   pFileIndices = terminal_tools::parse_file_extension_argument (argc, argv, ".vtk");;
   //TODO: implement parser for ply too

   //vtkDataSet *data[pFileIndices.size ()];
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

        // if (lut_enable)
	   //           ren->AddActor (createScalarBarActor (lut, argc, argv));

//         // Create the data actor 
//         if (scale > 0)
//         {
//           vtkFloatArray *distances = (vtkFloatArray*)data[i]->GetPointData ()->GetScalars ();
//           for (long int j = 0; j < distances->GetNumberOfTuples (); j++)
//           {
//             double c[1];
//             distances->GetTuple (j, c);
//             c[0] += scale;
//             distances->SetTuple (j, c);
//           }

//           data[i]->GetPointData ()->SetScalars (distances);
//         }
//         dataActor = createActorFromDataSet (data[i], psize[i], lut, minmax, lod_enable);
//         if (cell_scalar)
//           dataActor->GetMapper ()->SetScalarModeToUseCellData ();
// //        dataActor = createActorFromDataSet (data[i], psize, lod_enable);
//       }
//     }
//     dataActor->GetProperty ()->SetRepresentationToPoints ();
    
//     // Disable shadows if necessary
//     if (no_shadows)
//     {
//       dataActor->GetProperty ()->ShadingOff (); // is this needed?
//       dataActor->GetProperty ()->SetAmbient (1);
//       dataActor->GetProperty ()->SetDiffuse (0);
//     }

//     if (line_width != -1)
//       dataActor->GetProperty ()->SetLineWidth (line_width);

//     ren->AddActor (dataActor);

//     if ((text > 0) && (i == 0))
//     {
//       char idx[10];
//       vtkPolyData* polydata=reinterpret_cast<vtkPolyData*>(data[i]);
//       double bounds[6];
//       polydata->GetBounds (bounds);
//       double text_scale = text * sqrt ( SQR(bounds[0]-bounds[1]) + SQR(bounds[2]-bounds[3]) +SQR(bounds[4]-bounds[5]) );
//       double point[3];
//       int start = max (minIdx, 0);
//       int stop = min (maxIdx, int(data[i]->GetNumberOfPoints ())-1);
//       print_info (stderr, "Labeling points with their indices in ");
//       print_value (stderr, "[%d,%d]\n", start, stop);
//       for (int j = start; j <= stop; j++)
//       {
//         data[i]->GetPoint (j, point);
//         sprintf (idx, "%d", j);
//         vtkVectorText *atext = vtkVectorText::New ();
//         atext->SetText (idx);
//         vtkPolyDataMapper *textMapper = vtkPolyDataMapper::New ();
//         textMapper->SetInputConnection (atext->GetOutputPort ());
//         vtkFollower *textActor = vtkFollower::New ();
//         textActor->SetMapper (textMapper);
//         textActor->SetScale (text_scale);
//         textActor->SetPosition (point[0], point[1], point[2]);
//         //textActor->AddPosition (0, -0.1, 0);
//         textActor->SetCamera (ren->GetActiveCamera ());
//         ren->AddActor (textActor);
       }
     }
  }

  //vtkLightKit* lightKit = vtkLightKit::New ();
//  lightKit->AddLightsToRenderer (ren);

  // Create the VTK window
 //  char title[256];
//   if (pFileIndices.size () == 1)
//     sprintf (title, "3D PointCloud viewer - %s", argv[pFileIndices.at (0)]);
//   else
//     sprintf (title, "3D PointCloud viewer");
//   vtkRenderWindowInteractor* iren;
//   if (movieMode)
//   {
//     print_info (stderr, "Movie file is being written to: ");
//     print_value (stderr, "%s\n", movieFile);
// //    iren = CreatePerspectiveRenderWindowAndInteractorMovie (ren, title, movieFile, 44.382, argc, argv, 2);
//     iren = CreateRenderWindowAndInteractorMovie (ren, title, movieFile, argc, argv, 2);
//   }
//   else
//     iren = CreateRenderWindowAndInteractor (ren, title, argc, argv);

//   vtkSmartPointer<vtkInteractorStyleTUM> style = (vtkSmartPointer<vtkInteractorStyleTUM>)reinterpret_cast<vtkInteractorStyleTUM*>(iren->GetInteractorStyle ());
//   style->setAdvancedMode (true);
    
//   if (play)
//   {
//     vtkSmartPointer<vtkInteractorEventRecorder> recorder = vtkSmartPointer<vtkInteractorEventRecorder>::New ();
//     recorder->SetInteractor (iren);
//     recorder->SetFileName (argv[pLogFileIndices.at (0)]);
//     recorder->SetEnabled (1);
//     recorder->Play ();
//   }

//   iren->Start ();
//   if (save_camera_position)
//     SaveCameraPosition (ren->GetActiveCamera (), false);
}
/* ]--- */
