/*
 *  Load and export a VTK 3D point cloud
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
 * $Id: VTKExporter.cc,v 1.0 2010/02/23 12:00:00 zoli Exp $
 */
#include <iostream>
#include <string>
#include <vector>
#include <math.h>

//#include "../common/CommonVTKRoutines.h"
//#include "../common/CommonTerminalRoutines.h"
//#include "../common/CommonANNRoutines.h"

#include <vtkIVExporter.h>
//#include <vtkGL2PSExporter.h>
#include <vtkOOGLExporter.h>
#include <vtkRIBExporter.h>
//#include <vtkPOVExporter.h>
#include <vtkIVExporter.h>
#include <vtkOBJExporter.h>
#include <vtkX3DExporter.h>

#include <vtkVRMLExporter.h>

//pcl_visualizer includes
#include <pcl_visualization/pcl_visualizer.h>
#include <terminal_tools/print.h>
#include <terminal_tools/parse.h>

#include <pcl_cloud_tools/misc.h>

using namespace std;
using terminal_tools::print_color;
using terminal_tools::print_error;
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


/* ---[ */
int
  main (int argc, char** argv)
{
  if (argc < 2)
  {
    print_error (stderr, "Syntax is: %s input.vtk [output.extension1...N]\n", argv[0]);
    fprintf (stderr, "  where the VTK file will be saved in files with the specified extensions\n");
    //fprintf (stderr, "  supported extensions are: OBJ*, VRML, X3D, IV, POV, RIB*, OOGL, GL2PS*\n");
    fprintf (stderr, "  supported extensions are: OBJ*, VRML, X3D, IV, RIB*, OOGL\n");
    fprintf (stderr, "  * multiple files are produced, please see corresponding documantation\n");
    fprintf (stderr, "  (OBJ is easy to check and works with blender - you can use that to convert it)\n");
    return (-1);
  }
  
  // Parse the command line arguments for .vtk or .ply files
//   vector<int> pFileIndicesVTK = ParseFileNamesArgument (argc, argv);
//   vector<int> pFileIndicesVRML = ParseFileExtensionArgument (argc, argv, ".wrl");
//   vector<int> pFileIndicesX3D = ParseFileExtensionArgument (argc, argv, ".x3d");
//   vector<int> pFileIndicesOBJ = ParseFileExtensionArgument (argc, argv, ".obj");
//   vector<int> pFileIndicesIV = ParseFileExtensionArgument (argc, argv, ".iv");
//   //vector<int> pFileIndicesPOV = ParseFileExtensionArgument (argc, argv, ".pov");
//   vector<int> pFileIndicesRIB = ParseFileExtensionArgument (argc, argv, ".rib");
//   vector<int> pFileIndicesOOGL = ParseFileExtensionArgument (argc, argv, ".oogl");
//   //vector<int> pFileIndicesGL2PS = ParseFileExtensionArgument (argc, argv, ".gl2ps");

  vector<int> p_file_indices_vtk = terminal_tools::parse_file_extension_argument (argc, argv, ".vtk");
  
  
  // Loading VTK file
  vtkPolyData* data = reinterpret_cast<vtkPolyData*>(load_poly_data_as_data_set(argv[p_file_indices_vtk.at (0)]));
  data->Update ();

  // Print info
  print_info (stderr, "Loaded "); print_value (stderr, "%s", argv [p_file_indices_vtk.at (0)]);
  fprintf (stderr, " with "); print_value (stderr, "%d", data->GetNumberOfPoints ()); fprintf (stderr, " points and ");
  print_value (stderr, "%d", data->GetNumberOfPoints ()); fprintf (stderr, " polygons.\n");

  // Create Renderer
  vtkRenderer* ren = vtkRenderer::New ();
  //ren->AutomaticLightCreationOff ();

  // Create an actor for the data and add it to the renderer
//   ren->AddActor (createActorFromDataSet (data, 0.0, 0.0, 0.0));
  
//   // Create the VTK window
//   char title[256];
//   sprintf (title, "VTK exporter");
//   vtkRenderWindow *renWin = vtkRenderWindow::New ();
//   renWin->SetWindowName (title);
//   renWin->AddRenderer (ren);
//   renWin->SetSize (RENWIN_WIDTH, RENWIN_HEIGHT);
//   /*vtkRenderWindowInteractor* iren;
//   iren = CreateRenderWindowAndInteractor (ren, title, argc, argv);
//   iren->Start ();*/

//   // Create exporter
//   vector<vtkExporter*> exporters;
//   vector<char*> filenames;

//   if (pFileIndicesVRML.size () > 0)
//   {
//     vtkVRMLExporter* exporter = vtkVRMLExporter::New ();
//     exporter->SetFileName (argv [pFileIndicesVRML.at (0)]);
//     filenames.push_back (argv [pFileIndicesVRML.at (0)]);
//     exporters.push_back (exporter);
//   }
//   if (pFileIndicesX3D.size () > 0)
//   {
//     vtkX3DExporter* exporter = vtkX3DExporter::New ();
//     exporter->SetFileName (argv [pFileIndicesX3D.at (0)]);
//     filenames.push_back (argv [pFileIndicesX3D.at (0)]);
//     exporters.push_back (exporter);
//   }
//   if (pFileIndicesOBJ.size () > 0)
//   {
//     vtkOBJExporter* exporter = vtkOBJExporter::New ();
//     exporter->SetFilePrefix (argv [pFileIndicesOBJ.at (0)]);
//     filenames.push_back (argv [pFileIndicesOBJ.at (0)]);
//     exporters.push_back (exporter);
//   }
//   if (pFileIndicesIV.size () > 0)
//   {
//     vtkIVExporter* exporter = vtkIVExporter::New ();
//     exporter->SetFileName (argv [pFileIndicesIV.at (0)]);
//     filenames.push_back (argv [pFileIndicesIV.at (0)]);
//     exporters.push_back (exporter);
//   }
//   /*if (pFileIndicesPOV.size () > 0)
//   {
//     vtkPOVExporter* exporter = vtkPOVExporter::New ();
//     exporter->SetFileName (argv [pFileIndicesPOV.at (0)]);
//     filenames.push_back (argv [pFileIndicesPOV.at (0)]);
//     exporters.push_back (exporter);
//   }*/
//   if (pFileIndicesRIB.size () > 0)
//   {
//     vtkRIBExporter* exporter = vtkRIBExporter::New ();
//     exporter->SetFilePrefix (argv [pFileIndicesRIB.at (0)]);
//     filenames.push_back (argv [pFileIndicesRIB.at (0)]);
//     exporters.push_back (exporter);
//   }
//   if (pFileIndicesOOGL.size () > 0)
//   {
//     vtkOOGLExporter* exporter = vtkOOGLExporter::New ();
//     exporter->SetFileName (argv [pFileIndicesOOGL.at (0)]);
//     filenames.push_back (argv [pFileIndicesOOGL.at (0)]);
//     exporters.push_back (exporter);
//   }
//   /*if (pFileIndicesGL2PS.size () > 0)
//   {
//     vtkGL2PSExporter* exporter = vtkGL2PSExporter::New ();
//     exporter->SetFilePrefix (argv [pFileIndicesGL2PS.at (0)]);
//     filenames.push_back (argv [pFileIndicesGL2PS.at (0)]);
//     exporters.push_back (exporter);
//   }*/

//   // Saving
//   for (unsigned i = 0; i < exporters.size (); i++)
//   {
//     print_info (stderr, "Writing to "); print_value (stderr, "%s\n", filenames[i]);
//     //exporter->SetInput (iren->getRenderWindow ());
//     exporters[i]->SetInput (renWin);
//     exporters[i]->Write ();
//   }
//   fprintf (stderr, "[done]\n");
}
/* ]--- */
