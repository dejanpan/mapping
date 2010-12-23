#ifndef PCL_CLOUD_TOOLS_MISC_H_
#define PCL_CLOUD_TOOLS_MISC_H_

#include <vtkLODActor.h> 
#include <vtkProperty.h> 
#include <vtkPolyDataReader.h>
#include <vtkMaskPoints.h>
#include <vtkSmartPointer.h> 
#include <vtkDataSetMapper.h>
/* #include <vector> */
#include <vtkCellArray.h>
#include <vtkLookupTable.h>

#define NR_COLOR 65536
#define S_COLOR 100

////////////////////////////////////////////////////////////////////////////////
// Loads a 3D point cloud from a given fileName.
// Returns: a vtkPolyData object containing the point cloud.
vtkPolyData*
load_poly_data_as_data_set (const char* fileName)
{
  vtkPolyDataReader* reader = vtkPolyDataReader::New ();
  reader->SetFileName (fileName);
  reader->Update ();
  return reader->GetOutput ();
}

////////////////////////////////////////////////////////////////////////////////
// ---[ Create a vtkActor from vtkDataSet
vtkActor* create_actor_from_data_set (vtkDataSet *data, double c1, double c2, double c3, bool lod_enable = false)
{
  vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New ();
  //  vtkSmartPointer<vtkPainterPolyDataMapper> mapper = vtkSmartPointer<vtkPainterPolyDataMapper>::New ();
  //vtkSmartPointer<vtkOpenGLPolyDataMapper> mapper = vtkSmartPointer<vtkOpenGLPolyDataMapper>::New ();
  mapper->SetInput ((vtkPolyData*)data);
  mapper->ScalarVisibilityOff ();
  
  vtkActor *actor;
  if (lod_enable)
    {
      vtkSmartPointer<vtkMaskPoints> low_mask = vtkSmartPointer<vtkMaskPoints>::New ();
      low_mask->SetMaximumNumberOfPoints (data->GetNumberOfPoints () / 16);
      actor = vtkLODActor::New ();
      reinterpret_cast<vtkLODActor*>(actor)->SetNumberOfCloudPoints (data->GetNumberOfPoints () / 4);
      reinterpret_cast<vtkLODActor*>(actor)->SetLowResFilter (low_mask);
      actor->GetProperty ()->SetInterpolationToFlat ();
    }
  else
    {
      actor = vtkActor::New ();
      actor->GetProperty ()->SetInterpolationToPhong ();
      //    actor->GetProperty ()->SetInterpolationToFlat ();
    }
  actor->GetProperty ()->SetColor         (c1, c2, c3);
  actor->GetProperty ()->SetAmbientColor  (c1, c2, c3);
  actor->GetProperty ()->SetSpecularColor (c1, c2, c3);
  actor->GetProperty ()->SetAmbient (0.8);
  actor->SetMapper (mapper);
  //  actor->GetProperty ()->SetPointSize (0);
  //  actor->GetProperty ()->SetRepresentationToPoints ();
  //  actor->GetProperty ()->SetRepresentationToSurface();
  return actor;
}

vtkActor*
create_actor_from_data_set (vtkDataSet *data, double c1, double c2, double c3, double psize, bool lod_enable)
{
  vtkActor *actor = create_actor_from_data_set (data, c1, c2, c3, lod_enable); 
  actor->GetProperty ()->SetPointSize (psize);
  return actor;
}


//////////////////////////////////////////////////////////////////////////////
// ---[ Implementation of srand/rand for RGB [+ A] values between [0,1]     //
//////////////////////////////////////////////////////////////////////////////

static unsigned stepRGBA = 100;
inline void
srandRGBA (unsigned seed, unsigned step)
{
  srand (seed);
  stepRGBA = step;
}

inline double*
randRGBA1 (double min=0.2, double max=2.8)
{
  double* rgba = new double[4];
  double sum;
  do
    {
      sum = 0;
      rgba[0] = (rand ()%stepRGBA) / (double)stepRGBA;
      while ((rgba[1] = (rand ()%stepRGBA) / (double)stepRGBA) == rgba[0]);
      while (((rgba[2] = (rand ()%stepRGBA) / (double)stepRGBA) == rgba[0]) && (rgba[2] == rgba[1]));
      sum = rgba[0] + rgba[1] + rgba[2];
    } while (sum <= min || sum >= max);
  rgba[3] = 1.0;
  //cerr << "r=" << rgba[0] << " g=" << rgba[1] << " b=" << rgba[2] << " a=" << rgba[3] << endl;
  return rgba;
}

////////////////////////////////////////////////////////////////////////////////
// ---[ Creates & builds a lookup table with random colors initialized by param 
vtkSmartPointer<vtkLookupTable>
create_LUT (double minmax[2], int log, int seed_rand, int nr_colors, unsigned step_rand, bool debug)
{
/*   if (debug) */
/*     { */
/*       print_info  (stderr, "Initialization of random color generator with: "); */
/*       print_value (stderr, "%d\n", seed_rand); */
/*       print_info  (stderr, "Generating color LUT: "); */
/*       print_value (stderr, (log?"logarithmic":"linear")); */
/*       fprintf     (stderr, " representation with a "); */
/*       print_value (stderr, "1/%u", step_rand); */
/*       fprintf     (stderr, " distanced subset of "); */
/*       print_value (stderr, "%d", nr_colors); */
/*       fprintf     (stderr, " colors\n"); */
/*     } */

  vtkSmartPointer<vtkLookupTable> lut = vtkSmartPointer<vtkLookupTable>::New ();
  lut->SetScaleToLinear ();
  lut->SetRampToLinear ();
  //lut->SetNumberOfColors (nr_colors);
  lut->SetNumberOfTableValues (nr_colors);
  if (seed_rand >= 0)
    {
      srandRGBA ((unsigned)seed_rand, step_rand);
      for (vtkIdType i=0; i<nr_colors; i++)
	lut->SetTableValue (i, randRGBA1 ());
    }
  if (log != 0)
    lut->SetScaleToLog10 ();
  //lut->SetHueRange (0.0, 0.66667);
  lut->SetRange (minmax);
  lut->Build ();
  return lut;
}

////////////////////////////////////////////////////////////////////////////////
// ---[ Creates & builds a lookup table with random colors initialized by cmdln
vtkSmartPointer<vtkLookupTable>
create_LUT (double minmax[2], int argc, char** argv)
{
  int log = 0; terminal_tools::parse_argument (argc, argv, "-log", log); 
  int nr_colors = NR_COLOR; terminal_tools::parse_argument (argc, argv, "-nrRGBcolors", nr_colors);
  int step_rand = S_COLOR;  terminal_tools::parse_argument (argc, argv, "-randRGBstep", step_rand); 
  int seed_rand = -1;  terminal_tools::parse_argument (argc, argv, "-randRGBseed", seed_rand); 
  /// Impose limits on the scalar data
  terminal_tools::parse_2x_arguments (argc, argv, "-lut_limits", minmax[0], minmax[1]);
  //  return CreateLUT (minmax, log, seed_rand, nr_colors, (unsigned)step_rand, false);
}

#endif
