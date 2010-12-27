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
#include <vtkScalarBarActor.h>
#include <vtkTextProperty.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkRendererCollection.h>
#include <vtkPNGWriter.h>
#include <vtkWindowToImageFilter.h>
#include <vtkDataArrayCollection.h>
#include <vtkLegendScaleActor.h>
#include <vtkTextActor.h>


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

////////////////////////////////////////////////////////////////////////////////
// ---[ Create a vtkActor from vtkDataSet with a vtkLookupTable
vtkActor*
create_actor_from_data_set (vtkDataSet *data, double psize, vtkLookupTable* lut, double minmax[2], bool lod_enable)
{
  vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New ();
  mapper->SetInput (data);
  mapper->SetLookupTable (lut);
  mapper->SetScalarRange (minmax);
  mapper->SetScalarModeToUsePointData ();
  mapper->ScalarVisibilityOn ();
  
  vtkActor *actor;
  if (lod_enable)
  {
    actor = vtkLODActor::New ();
    reinterpret_cast<vtkLODActor*>(actor)->SetNumberOfCloudPoints (data->GetNumberOfPoints () / 10);
    actor->GetProperty ()->SetInterpolationToFlat ();
  }
  else
  {
    actor = vtkActor::New ();
//    actor->GetProperty ()->SetInterpolationToFlat ();
    actor->GetProperty ()->SetInterpolationToPhong ();
  }
  actor->GetProperty()->SetPointSize(psize);
  actor->SetMapper (mapper);
//  actor->GetProperty ()->SetAmbient (0.8);
  return actor;
}

////////////////////////////////////////////////////////////////////////////////
// ---[ Create a vtkActor from vtkDataSet
vtkActor*
create_actor_from_data_set (vtkDataSet *data, bool lod_enable)
{
  vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New ();
  mapper->SetInput (data);
  mapper->SetScalarModeToUsePointData ();
//  mapper->SetScalarModeToUseCellData ();
  mapper->ScalarVisibilityOn ();
  
  vtkActor *actor;
  if (lod_enable)
  {
    actor = vtkLODActor::New ();
    reinterpret_cast<vtkLODActor*>(actor)->SetNumberOfCloudPoints (data->GetNumberOfPoints () / 10);
    actor->GetProperty ()->SetInterpolationToFlat ();
  }
  else
  {
    actor = vtkActor::New ();
//    actor->GetProperty ()->SetInterpolationToFlat ();
    actor->GetProperty ()->SetInterpolationToPhong ();
  }
  actor->SetMapper (mapper);
//  actor->GetProperty ()->SetAmbient (0.8);
  return actor;
}


vtkActor*
create_actor_from_data_set (vtkDataSet *data, double psize, bool lod_enable)
{
  vtkActor *actor = create_actor_from_data_set (data, lod_enable);
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
  return create_LUT (minmax, log, seed_rand, nr_colors, (unsigned)step_rand, false);
}

////////////////////////////////////////////////////////////////////////////////
// Loads a 3D point cloud from a given fileName containing Unstructured Grid data
// Returns: a vtkDataSet object containing the point cloud.
vtkScalarBarActor*
create_scalar_bar_actor (vtkLookupTable *lut, int argc, char** argv)
{
  // Put in a color bar (called a scalar bar in vtk)
  vtkScalarBarActor *barActor = vtkScalarBarActor::New ();
  barActor->SetLookupTable (lut);
  barActor->SetTitle ("");
  barActor->SetOrientationToHorizontal ();
  barActor->SetPosition (0.05, 0.01);
  
  int nr_labels = barActor->GetNumberOfLabels ();
  terminal_tools::parse_argument (argc, argv, "-nr_bar_labels", nr_labels); 
  barActor->SetNumberOfLabels (nr_labels);
  
  barActor->SetWidth (0.9); barActor->SetHeight (0.1);
  vtkTextProperty *prop = barActor->GetLabelTextProperty ();
  prop->SetFontSize (10);
  barActor->SetLabelTextProperty (prop);
  barActor->SetTitleTextProperty (prop);
//    barActor->GetLabelTextProperty ()->PrintSelf (std::cerr, 0);
  return barActor;
}

struct ScalarsContainer
{
  vtkDataArrayCollection* scalars;
  std::vector<double> minScalar;
  std::vector<double> maxScalar;
  std::vector<std::string> dimensions;
  
  vtkActor* actor;
  
  // Hold the N-d (default 16d) features histogram (where available)
  vtkDoubleArray* histogram;
};


////////////////////////////////////////////////////////////////////////////////
//------------------------------------------------------------------------------
////////////////////////////////////////////////////////////////////////////////
class vtkInteractorStyleTUM : public vtkInteractorStyleTrackballCamera //vtkInteractorStyle
{
  public:
    static vtkInteractorStyleTUM *New ();
    void OnChar ();
    void OnTimer ();
    void setRenderer (vtkRenderer* ren);
    vtkRenderer* getRenderer () { return this->renderer; }
    void setRendererCollection (vtkRendererCollection* rencol);
    void setWriter (vtkPNGWriter* w) { this->writer = w; }
    void setWindowFilter (vtkWindowToImageFilter *f) { this->filter = f; }
    vtkWindowToImageFilter* getWindowFilter () { return this->filter; }

    void ZoomIn ();
    void ZoomOut ();

    void setAdvancedMode (bool mode);
    std::vector<ScalarsContainer> allScalars;       // contains all the scalars associated with all currently loaded datasets

    void setHistogramNrBins (unsigned int nr_bins)
    { this->histNrBins = nr_bins; terminal_tools::print_info (stderr, "Number of histogram bins used: "); terminal_tools::print_value (stderr, "%d\n", this->histNrBins); };

    void setTextsList (std::vector<std::string> texts) { this->textList = texts; };
    std::vector<std::string> getTextsList () { return this->textList; };

    void setTextsProp (double x, double y, double s) { this->tAx = x; this->tAy = y; this->tS = s; };
    void setTextsColor (double r, double g, double b) { this->tR = r; this->tG = g; this->tB = b; };

  protected:
    vtkRenderer *renderer;
    vtkRendererCollection *rendererCollection;
    bool one_renderer;
    int pointsize;
  public:
    vtkPNGWriter* writer;
    vtkWindowToImageFilter *filter;
  protected:
    char fileName[80];
    char camFN[80];

    bool advanced;
    vtkLegendScaleActor* gridActor;
    vtkScalarBarActor*   lutActor;
    bool grid_enabled, lut_enabled;

  public:
    unsigned int histNrBins;

  private:
    // vtkText on screen renderings
    std::vector<std::string> textList;
    vtkTextActor*  textActor;
    float tAx, tAy, tS;
    float tR, tG, tB;
    bool texts_enabled;
    int cur_text;

  public:
    int argc;
    char** argv;
};


#endif
