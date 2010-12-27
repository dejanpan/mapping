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
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkCommand.h>
#include <vtkProperty.h>
#include <vtkProperty2D.h>
#include <vtkPointPicker.h>
#include <vtkObjectFactory.h>
#include <vtkAssemblyPath.h>

//from CommonVTKRoutines.h
#include <vtkGraphicsFactory.h>
#include <vtkGeneralTransform.h>
#include <vtkGeometryFilter.h>
#include <vtkImagingFactory.h>
#include <vtkAbstractTransform.h>
#include <vtkActor.h>
#include <vtkAlgorithmOutput.h>
#include <vtkAppendPolyData.h>
#include <vtkArrowSource.h>
#include <vtkAssemblyPath.h>
#include <vtkAssembly.h>
#include <vtkCubeSource.h>
#include <vtkConeSource.h>
#include <vtkCylindricalTransform.h>
#include <vtkTransformTextureCoords.h>
#include <vtkBMPReader.h>
#include <vtkTexture.h>
#include <vtkAxes.h>
#include <vtkAxisActor2D.h>
#include <vtkButterflySubdivisionFilter.h>
#include <vtkBoxWidget.h>
#include <vtkCamera.h>
#include <vtkCameraInterpolator.h>
#include <vtkCommand.h>
#include <vtkCellData.h>
#include <vtkCellArray.h>
#include <vtkCleanPolyData.h>
#include <vtkClipPolyData.h>
#include <vtkContourFilter.h>
#include <vtkCylinderSource.h>
#include <vtkCutter.h>
#include <vtkDataSetMapper.h>
#include <vtkDataObjectCollection.h>
#include <vtkDiskSource.h>
#include <vtkDataSetTriangleFilter.h>
#include <vtkDataSetWriter.h>
#include <vtkDataSetCollection.h>
#include <vtkDataArrayCollection.h>
#include <vtkDebugLeaks.h>
#include <vtkDelaunay2D.h>
#include <vtkDelaunay3D.h>
#include <vtkDoubleArray.h>
#include <vtkExtractEdges.h>
#include <vtkInteractorEventRecorder.h>
#include <vtkFeatureEdges.h>
#include <vtkFloatArray.h>
#include <vtkFFMPEGWriter.h>
#include <vtkFollower.h>
#include <vtkGenericPointIterator.h>
#include <vtkGlyph3D.h>
#include <vtkGreedyTerrainDecimation.h>
#include <vtkHedgeHog.h>
#include <vtkHull.h>
#include <vtkHexahedron.h>
#include <vtkInterpolatingSubdivisionFilter.h>
#include <vtkImageBlend.h>
#include <vtkImageMapper.h>
#include <vtkImplicitModeller.h>
#include <vtkImplicitPlaneWidget.h>
#include <vtkIntArray.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkJPEGWriter.h>
#include <vtkKdTree.h>
#include <vtkLightKit.h>
#include <vtkLinearSubdivisionFilter.h>
#include <vtkLegendScaleActor.h>
#include <vtkLeaderActor2D.h>
#include <vtkLineSource.h>
#include <vtkLODActor.h>
#include <vtkLandmarkTransform.h>
#include <vtkLookupTable.h>
#include <vtkLogLookupTable.h>
#include <vtkLoopSubdivisionFilter.h>
#include <vtkMarchingCubes.h>
#include <vtkMaskPoints.h>
#include <vtkMath.h>
#include <vtkMatrix4x4.h>
#include <vtkMatrixToLinearTransform.h>
#include <vtkMassProperties.h>
#include <vtkOBBTree.h>
#include <vtkObjectFactory.h>
#include <vtkOBJReader.h>
#include <vtkOBJExporter.h>
#include <vtkOrderedTriangulator.h>
#include <vtkPerspectiveTransform.h>
#include <vtkPlane.h>
#include <vtkPlanes.h>
#include <vtkPolygon.h>
#include <vtkPlatonicSolidSource.h>
#include <vtkPLYReader.h>
#include <vtkPNGReader.h>
#include <vtkPNGWriter.h>
#include <vtkPlaneSource.h>
#include <vtkSuperquadricSource.h>
#include <vtkSphericalTransform.h>
#include <vtkSubdivideTetra.h>
#include <vtkPointData.h>
#include <vtkPointPicker.h>
#include <vtkPointDataToCellData.h>
#include <vtkPointLocator.h>
#include <vtkPoints.h>
#include <vtkPointSet.h>
#include <vtkPolyData.h>
#include <vtkPolyLine.h>
#include <vtkPolyDataAlgorithm.h>
#include <vtkPolyDataMapper.h>
#include <vtkOpenGLPolyDataMapper.h>
#include <vtkPainterPolyDataMapper.h>
#include <vtkPolyDataNormals.h>
#include <vtkPolyDataReader.h>
#include <vtkPolyDataWriter.h>
#include <vtkPolyVertex.h>
#include <vtkProperty.h>
#include <vtkProperty2D.h>
#include <vtkQuad.h>
#include <vtkRecursiveDividingCubes.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkScalarBarActor.h>
#include <vtkSmartPointer.h>
#include <vtkSpatialRepresentationFilter.h>
#include <vtkSphere.h>
#include <vtkSphereSource.h>
#include <vtkStripper.h>
#include <vtkStructuredPointsReader.h>
#include <vtkSurfaceReconstructionFilter.h>
#include <vtkSystemIncludes.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkTimerLog.h>
#include <vtkTransform.h>
#include <vtkTransformFilter.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkTriangleFilter.h>
#include <vtkTubeFilter.h>
#include <vtkUnstructuredGrid.h>
#include <vtkUnstructuredGridReader.h>
#include <vtkUnsignedIntArray.h>
#include <vtkVectorText.h>
#include <vtkVoxel.h>
#include <vtkVoxelContoursToSurfaceFilter.h>
#include <vtkVRMLImporter.h>
#include <vtkWindowedSincPolyDataFilter.h>
#include <vtkWindowToImageFilter.h>
#include <vtkXRenderWindowInteractor.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLPPolyDataReader.h>
#include <vtkXYPlotActor.h>
#include <vtkXYPlotWidget.h>
#include <vtkGraphToPolyData.h>
#include <vtkGraphLayout.h>
#include <vtkGraphReader.h>
#include <vtkGraph.h>
#include <vtkDirectedGraph.h>
#include <vtkUndirectedGraph.h>
#include <vtkMutableUndirectedGraph.h>
#include <vtkPolyDataCollection.h>

#define NR_COLOR 65536
#define S_COLOR 100
#define RENWIN_WIDTH 1200
#define RENWIN_HEIGHT 800
#define _sqr(x) ((x)*(x))
#define _sqr_dist(x,y) ( _sqr((x[0])-(y[0])) + _sqr((x[1])-(y[1])) + _sqr((x[2])-(y[2])) )
#define SPACE_KEY 32
#define KEY_PLUS  43
#define KEY_MINUS 45

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


class vtkCameraCallback : public vtkCommand
{
  public:
    static vtkCameraCallback *New () { return new vtkCameraCallback;}
    void SetTextActor (vtkTextActor *txt);
    virtual void Execute (vtkObject *caller, unsigned long, void*);
  protected:
    vtkTextActor *camTextActor;
};

////////////////////////////////////////////////////////////////////////////////
//------------------------------------------------------------------------------
////////////////////////////////////////////////////////////////////////////////
void
  vtkCameraCallback::SetTextActor (vtkTextActor *txt)
{
  this->camTextActor = txt;
}
void
  vtkCameraCallback::Execute (vtkObject *caller, unsigned long, void*)
{
  vtkRenderer *ren = reinterpret_cast<vtkRenderer *> (caller);
  vtkCamera *cam = ren->GetActiveCamera ();
  double eye[3], up[3], viewray[3];
  cam->GetPosition (eye);
  cam->GetViewUp (up);
  cam->GetDirectionOfProjection (viewray);

  char camTextBuff[256];
  snprintf (camTextBuff, 255, "Position: %.3g,%.3g,%.3g ; ViewUp: %.3g,%.3g,%.3g ; Direction Of Projection %.3g,%.3g,%.3g", eye[0], eye[1], eye[2], up[0], up[1], up[2], viewray[0], viewray[1], viewray[2]);
//  this->camTextActor->SetInput (camTextBuff);
}

////////////////////////////////////////////////////////////////////////////////
//------------------------------------------------------------------------------
////////////////////////////////////////////////////////////////////////////////
class vtkMouseCallback : public vtkCommand
{
  public:
    static vtkMouseCallback *New () { return new vtkMouseCallback; }
    
    virtual void
      Execute (vtkObject *caller, unsigned long eventid, void*)
    {
      vtkInteractorStyleTUM *style = reinterpret_cast<vtkInteractorStyleTUM*>(caller);
      vtkRenderWindowInteractor* iren = style->GetInteractor ();
      vtkRenderer* ren_main = iren->GetRenderWindow ()->GetRenderers ()->GetFirstRenderer ();
      
      if ((eventid == vtkCommand::LeftButtonPressEvent) && (iren->GetShiftKey () == 1))
      {
        int idx = PerformPick (iren);
        if (idx < 0)
          return;
        // CTRL+SHIFT+Pick => Draw Histogram (if available)
        if ((iren->GetControlKey () == 1) && (style->allScalars.size () > 0)) // If scalars are available
        {
          // Add a text actor to the main renderer
          char text[10];
          sprintf (text, "%d", idx);
          vtkVectorText *atext = vtkVectorText::New ();
          atext->SetText (text);

          // Use the same color as for the title
          double tr = 1.0, tg = 1.0 , tb = 1.0;
          terminal_tools::parse_3x_arguments (style->argc, style->argv, "-texts_color", tr, tg, tb);
          double index_scale = 0.001;
          terminal_tools::parse_argument (style->argc, style->argv, "-index_scale", index_scale);
          vtkPolyDataMapper *textMapper = vtkPolyDataMapper::New ();
          textMapper->SetInputConnection (atext->GetOutputPort ());
          vtkFollower *textActor = vtkFollower::New ();
          textActor->SetMapper (textMapper);
          textActor->SetScale (index_scale);
          textActor->SetPosition (pt[0], pt[1], pt[2]);
          textActor->GetProperty ()->SetColor (tr, tg, tb);
          textActor->SetCamera (ren_main->GetActiveCamera ());
          ren_main->AddActor (textActor);
          ren_main->Render ();
          
          // Get the first scalar
          ScalarsContainer s = style->allScalars[0];
          
          cerr << "  -> Dimensions: ";
          vtkDataArrayCollection* scalars = s.scalars;
          for (unsigned int d = 0; d < s.dimensions.size (); d++)
          {
            vtkDataArray *scalar = scalars->GetItem (d);
            double value[1];
            scalar->GetTuple (idx, value);
            cerr << s.dimensions.at (d).c_str () << " = " << value[0] << "   ";
          }
          cerr << endl;

          // If feature histograms are available
          if ((s.histogram != NULL) && (style->histNrBins > 0))
          {
            // Get the histogram corresponding to point IDX
            double hist[style->histNrBins ];
            s.histogram->GetTuple (idx, hist);

            cerr << "  -> Feature Histogram: ";
            for (unsigned int d = 0; d < style->histNrBins; d++)
            {
              cerr << hist[d] << "   ";
            }
            cerr << endl;
            vtkSmartPointer<vtkXYPlotActor> xyplot = vtkSmartPointer<vtkXYPlotActor>::New ();
            xyplot->SetDataObjectPlotModeToColumns ();
            xyplot->SetXValuesToValue ();

            vtkSmartPointer<vtkDoubleArray> XYarray = vtkSmartPointer<vtkDoubleArray>::New ();
            XYarray->SetNumberOfComponents (2);
            XYarray->SetNumberOfTuples (style->histNrBins);

            // Copy its content into a vtkFieldValue
            for (unsigned int d = 0; d < style->histNrBins; d++)
            {
              double xy[2];
              xy[0] = d;
              xy[1] = hist[d];
              if (xy[1] > 99.99)
                xy[1] = 99.99;
              XYarray->SetTuple (d, xy);
            }
            // Create the data object
            vtkSmartPointer<vtkFieldData> fieldValues = vtkSmartPointer<vtkFieldData>::New ();
            fieldValues->AddArray (XYarray);
            vtkSmartPointer<vtkDataObject> data = vtkSmartPointer<vtkDataObject>::New ();
            data->SetFieldData (fieldValues);

            xyplot->AddDataObjectInput (data);

            xyplot->SetPlotColor (0, 1.0, 0.0, 0.0);

            xyplot->SetDataObjectXComponent (0, 0); xyplot->SetDataObjectYComponent (0, 1);
            xyplot->PlotPointsOn ();
            //xyplot->PlotCurvePointsOn ();
            //xyplot->PlotLinesOn ();
            xyplot->PlotCurveLinesOn ();

            char title[128];
            sprintf (title, "Feature Histogram for point %d", idx);
            xyplot->SetYTitle (""); xyplot->SetXTitle ("");
            xyplot->SetYRange (0, 100); xyplot->SetXRange (0, style->histNrBins - 1);
//            xyplot->SetTitle (title);
            xyplot->GetProperty ()->SetColor (0, 0, 0);
            vtkSmartPointer<vtkTextProperty> tprop = xyplot->GetTitleTextProperty ();
            xyplot->AdjustTitlePositionOn ();
            tprop->SetFontSize (8);
            tprop->ShadowOff (); tprop->ItalicOff ();
            tprop->SetColor (xyplot->GetProperty ()->GetColor ());

            xyplot->SetAxisLabelTextProperty (tprop);
            xyplot->SetAxisTitleTextProperty (tprop);
            xyplot->SetNumberOfXLabels (8);
            xyplot->GetProperty ()->SetPointSize (10);
            xyplot->GetProperty ()->SetLineWidth (4);

            xyplot->SetPosition (0, 0);
            xyplot->SetWidth (1); xyplot->SetHeight (1);

            vtkSmartPointer<vtkRenderer> ren = vtkSmartPointer<vtkRenderer>::New ();
            ren->AddActor2D (xyplot);
            ren->SetBackground (1, 1, 1);

            vtkSmartPointer<vtkRenderWindow> renWin = vtkSmartPointer<vtkRenderWindow>::New ();
            renWin->SetWindowName (title);
            renWin->AddRenderer (ren);
            renWin->SetSize (640, 240);
            renWin->SetBorders (1);

            //vtkSmartPointer<vtkRenderWindowInteractor> iren = renWin->MakeRenderWindowInteractor ();
            vtkRenderWindowInteractor *iren = renWin->MakeRenderWindowInteractor ();
            iren->Start ();
          }
        }
      }
      else if ((eventid == vtkCommand::LeftButtonPressEvent) && (iren->GetAltKey () == 1))
      {
        pick_first = !pick_first;
        if (pick_first)
          idx_1 = PerformPick (iren, point_1);
        else
          idx_2 = PerformPick (iren, point_2);
        
        if ((idx_1 > 0)  && (idx_2 > 0))
        {
          double dist = sqrt (_sqr_dist (point_1, point_2));
          cerr << "Distance between " << idx_1 << ": [" << point_1[0] << ", " << point_1[1] << ", " << point_1[2] << "] and " <<
                                         idx_2 << ": [" << point_2[0] << ", " << point_2[1] << ", " << point_2[2] << "] is: " << dist << endl;

          vtkLeaderActor2D *leader = vtkLeaderActor2D::New ();
          leader->GetPositionCoordinate ()->SetCoordinateSystemToWorld ();
          leader->GetPositionCoordinate ()->SetValue (point_1[0], point_1[1], point_1[2]);
          leader->GetPosition2Coordinate ()->SetCoordinateSystemToWorld ();
          leader->GetPosition2Coordinate ()->SetValue (point_2[0], point_2[1], point_2[2]);
          leader->SetArrowStyleToFilled ();
          leader->AutoLabelOn ();
          
          vtkRenderWindow* renWin = iren->GetRenderWindow ();
          vtkRendererCollection *coll = renWin->GetRenderers ();
          coll->GetFirstRenderer ()->AddActor (leader);
          renWin->Render ();
        }
      }
      else
      {
        reinterpret_cast<vtkInteractorStyle*>(caller)->OnLeftButtonDown ();
      }
    }
    
  public:
    // idx_1 and idx_2 are used with a vtkLeaderActor2D for computing and displaying Euclidean distances between points
    int idx_1, idx_2;
    // Decide which index should be filled next
    bool pick_first;

  private:
    
    int
      PerformPick (vtkRenderWindowInteractor *iren)
      {
        int mouse_x, mouse_y;
        
        cerr.precision (10);
        
        // Get picker
        vtkPointPicker *picker = (vtkPointPicker*) iren->GetPicker ();
        iren->GetMousePosition (&mouse_x, &mouse_y);
        
        vtkRenderWindow* renWin = iren->GetRenderWindow ();
        vtkRendererCollection *coll = renWin->GetRenderers ();
        
        iren->StartPickCallback ();
        
        picker->Pick (mouse_x, mouse_y, 0.0, coll->GetFirstRenderer ());
        idx = picker->GetPointId ();
        cerr << "Picked Point with Index: " << idx;
        
        if (idx < 0)
        {
          cerr << endl;
          return idx;
        }
        
        if (picker->GetDataSet () != NULL)
        {
          picker->GetDataSet ()->GetPoint (idx, pt);
          cerr << " [" << pt[0] << ", " << pt[1] << ", " << pt[2] << "]" << endl;
        }
        else
          cerr << endl;
        iren->EndPickCallback ();
        return idx;
      }

    int
      PerformPick (vtkRenderWindowInteractor *iren, double point[3])
      {
        int mouse_x, mouse_y;
        
        // Get picker
        vtkPointPicker *picker = (vtkPointPicker*) iren->GetPicker ();
        iren->GetMousePosition (&mouse_x, &mouse_y);
        
        vtkRenderWindow* renWin = iren->GetRenderWindow ();
        vtkRendererCollection *coll = renWin->GetRenderers ();
        
        iren->StartPickCallback ();
        
        picker->Pick (mouse_x, mouse_y, 0.0, coll->GetFirstRenderer ());
        idx = picker->GetPointId ();
        cerr << "Picked Point with Index: " << idx;
        
        if (idx < 0)
        {
          cerr << endl;
          return idx;
        }
        
        if (picker->GetDataSet () != NULL)
        {
          picker->GetDataSet ()->GetPoint (idx, point);
          cerr << " [" << point[0] << ", " << point[1] << ", " << point[2] << "]" << endl;
        }
        else
          cerr << endl;
        iren->EndPickCallback ();
        return idx;
      }

    int idx;
    double pt[3], point_1[3], point_2[3];
};

vtkStandardNewMacro (vtkInteractorStyleTUM);
void
  vtkInteractorStyleTUM::setAdvancedMode (bool mode)
{
  this->advanced = mode;
  this->gridActor = vtkLegendScaleActor::New ();
  this->grid_enabled = false;
  this->lut_enabled = false;
  this->lutActor = vtkScalarBarActor::New ();
  this->lutActor->SetTitle ("");
  this->lutActor->SetOrientationToHorizontal ();
  this->lutActor->SetPosition (0.05, 0.01);
  this->lutActor->SetWidth (0.9); 
  this->lutActor->SetHeight (0.1);
  this->lutActor->SetNumberOfLabels (this->lutActor->GetNumberOfLabels () * 2);
  vtkSmartPointer<vtkTextProperty> prop = this->lutActor->GetLabelTextProperty ();
  prop->SetFontSize (10);
  this->lutActor->SetLabelTextProperty (prop);
  this->lutActor->SetTitleTextProperty (prop);

//  vtkSmartPointer<vtkTextActor> camTxt = vtkSmartPointer<vtkTextActor>::New ();
  vtkTextActor* camTxt = vtkTextActor::New ();
  vtkSmartPointer<vtkCameraCallback> camUpdateInfo = vtkSmartPointer<vtkCameraCallback>::New ();
  camUpdateInfo->SetTextActor (camTxt);
  double *camTxtPos = camTxt->GetPosition ();
  camTxt->SetPosition (camTxtPos[0], camTxtPos[0]+20);
  camTxt->GetProperty ()->SetColor (0.0, 1.0, 0.0);
  
  if (one_renderer)
  {
    this->renderer->AddObserver (vtkCommand::EndEvent, camUpdateInfo);
    this->renderer->AddActor (camTxt);
  }

  vtkMouseCallback *mc = vtkMouseCallback::New ();
//  mc->NR_BINS = this->histNrBins;     // Set the number of feature histogram bins to use
  mc->idx_1 = mc->idx_2 = -1;         // Set both indices to -1
  mc->pick_first = true;              // Fill first index first
  
  this->AddObserver (vtkCommand::LeftButtonPressEvent, mc);
//  this->AddObserver (vtkCommand::LeftButtonReleaseEvent, mc);
//  this->AddObserver (vtkCommand::KeyPressEvent, mc);
//  this->AddObserver (vtkCommand::MouseMoveEvent, mc);
  mc->Delete ();
  cerr << "Advanced mode enabled." << endl;
}

void
  vtkInteractorStyleTUM::setRenderer (vtkRenderer *ren)
{
  this->advanced  = false;
  this->renderer  = ren;
  pointsize       = 1;
  one_renderer    = true;
  
  this->texts_enabled = false;
  this->textActor     = NULL;
}

void
  vtkInteractorStyleTUM::setRendererCollection (vtkRendererCollection *rencol)
{
  this->advanced  = false;
  this->rendererCollection  = rencol;
  pointsize                 = 1;
  one_renderer              = false;

  this->texts_enabled = false;
  this->textActor     = NULL;
}


void
  vtkInteractorStyleTUM::OnTimer ()
{
//  cerr << "now " << endl;
  this->filter->Modified ();
  if (one_renderer)
    this->renderer->Render ();
  else
  {
    this->rendererCollection->Render ();
  }
}

void
  vtkInteractorStyleTUM::ZoomIn ()
{
  vtkRenderWindowInteractor *rwi = this->Interactor;
  this->FindPokedRenderer(rwi->GetEventPosition()[0],
                          rwi->GetEventPosition()[1]);
  // Zoom in
  this->StartDolly ();
  double factor = 10.0 * 0.2 * .5;
  this->Dolly (pow (1.1, factor));
  this->EndDolly ();
}

void
  vtkInteractorStyleTUM::ZoomOut ()
{
  vtkRenderWindowInteractor *rwi = this->Interactor;
  this->FindPokedRenderer(rwi->GetEventPosition()[0],
                          rwi->GetEventPosition()[1]);
  // Zoom out
  this->StartDolly ();
  double factor = 10.0 * -0.2 * .5;
  this->Dolly (pow (1.1, factor));
  this->EndDolly ();
}

////////////////////////////////////////////////////////////////////////////////
// Save a vtkDataSet object data to a VTK given fileName.
void
  SaveDataSetAsVTK (vtkDataSet *data, const char *fileName)
{
  vtkDataSetWriter *writer = vtkDataSetWriter::New ();
//  vtkPolyDataWriter *writer = vtkPolyDataWriter::New ();
  writer->SetInput (data);
  writer->SetFileName (fileName);
  writer->SetFileTypeToASCII ();
//  writer->SetFileTypeToBinary ();
  writer->Write ();
}


////////////////////////////////////////////////////////////////////////////////
// Save a vtkDataSet object data to a given fileName. The file type is given 
// based on the extension.
void
  SaveDataSet (vtkDataSet *data, const char *fileName)
{
  std::string fname = std::string (fileName);
  // if VTK, save with PolyDataWriter
  if (fname.compare (fname.size () - 3, 3, "vtk") == 0)
    SaveDataSetAsVTK (data, fileName);
  // else if (fname.compare (fname.size () - 4, 4, "cxyz") == 0)
  //   SaveDataSetAsCXYZPoints (data, fileName);
  // else if (fname.compare (fname.size () - 3, 3, "obj") == 0)
  //   SaveDataSetAsOBJ ((vtkPolyData*)data, fileName);
  // else if (fname.compare (fname.size () - 4, 4, "xyzr") == 0)
  //   SaveDataSetAsXYZ000RPoints (data, fileName);
  // else if (fname.compare (fname.size () - 3, 3, "xyz") == 0)
  //   SaveDataSetAsXYZPoints (data, fileName);
}

////////////////////////////////////////////////////////////////////////////////
void
  changeActorScalarAppearance (ScalarsContainer s, int dim, int argc, char** argv)
{
  double minmax[2];
  vtkSmartPointer<vtkLookupTable> lut;
  vtkPolyData *data = static_cast<vtkPolyData*>(s.actor->GetMapper ()->GetInput ());

  if (dim != -1)
  {
    minmax[0] = s.minScalar[dim]; minmax[1] = s.maxScalar[dim];
    lut = create_LUT (minmax, argc, argv);

    s.scalars->InitTraversal ();
    for (int i = 0; i < dim; i++) s.scalars->GetNextItem ();
    data->GetPointData ()->SetScalars (s.scalars->GetNextItem ());
  }

  data->Update ();

  vtkPolyDataMapper* mapper = static_cast<vtkPolyDataMapper*>(s.actor->GetMapper ());
    ///vtkPolyDataMapper::New ();//static_cast<vtkPolyDataMapper*>(s.actor->GetMapper ());
  mapper->SetInput (data);
  if (dim != -1)
  {
    mapper->SetLookupTable (lut);
    mapper->SetScalarRange (minmax);
    mapper->SetScalarModeToUsePointData ();
    mapper->ScalarVisibilityOn ();
  }
  else
  {
    mapper->ScalarVisibilityOff ();
  }

  s.actor->SetMapper (mapper);
  s.actor->Modified ();
}

void
  vtkInteractorStyleTUM::OnChar ()
{
  if (pointsize < 1)
    pointsize = 1;

  bool shift = this->Interactor->GetShiftKey   ();
  bool ctrl  = this->Interactor->GetControlKey ();
  bool alt   = this->Interactor->GetAltKey     ();
  
  if (shift);
  
  bool potentialPCDSave = false;
  
  vtkRenderWindowInteractor *rwi = this->Interactor;
  if (ctrl && alt)
    potentialPCDSave = true;
  
  // ---[ Check key symbols together with Control+Meta(ALT)
//  fprintf (stderr, "Key sym: %s\n", this->Interactor->GetKeySym ());
  std::string key (this->Interactor->GetKeySym ());
  //printf("Key hit: %s\n", key.c_str ());
  if (key.find ("XF86ZoomIn") != std::string::npos)
    this->ZoomIn ();
  else if (key.find ("XF86ZoomOut") != std::string::npos)
    this->ZoomOut ();
  else if (key.find ("space") != std::string::npos)
  {
    vtkAppendPolyData *allData = vtkAppendPolyData::New ();
    vtkActorCollection *ac;
    vtkActor *anActor, *aPart;
    vtkAssemblyPath *path;
    this->FindPokedRenderer(rwi->GetEventPosition()[0],
                            rwi->GetEventPosition()[1]);
    ac = this->CurrentRenderer->GetActors();
    vtkCollectionSimpleIterator ait;
    for (ac->InitTraversal(ait); (anActor = ac->GetNextActor(ait)); )
    {
      for (anActor->InitPathTraversal(); (path=anActor->GetNextPath()); )
      {
        aPart=(vtkActor *)path->GetLastNode()->GetViewProp();
        vtkProperty *prop = aPart->GetProperty ();
        double prop_rgb[3];
        prop->GetDiffuseColor (prop_rgb);
        prop_rgb[0] *= 255; prop_rgb[1] *= 255; prop_rgb[2] *= 255;
        
        vtkPolyData *aData = (vtkPolyData*)(aPart->GetMapper ()->GetInput ());
        aData->Update ();

        // Get the point and cell scalars associated with this dataset
        /// @note: enable at will
/*        vtkDataArray *pscalars = aData->GetPointData ()->GetScalars ();
        vtkDataArray *cscalars = aData->GetPointData ()->GetScalars ();
        if ( (!pscalars) && (!cscalars) )
        {
          vtkUnsignedCharArray* colors = vtkUnsignedCharArray::New ();
          colors->SetNumberOfComponents (3);
          for (int cp = 0; cp < aData->GetNumberOfCells (); cp++)
          {
            unsigned char char_rgb[3];
            char_rgb[0] = (unsigned char)(prop_rgb[0]);
            char_rgb[1] = (unsigned char)(prop_rgb[1]);
            char_rgb[2] = (unsigned char)(prop_rgb[2]);
            colors->InsertNextTupleValue (char_rgb);
          }
          aData->GetCellData ()->SetScalars (colors);
          aData->Update ();
        }*/
        allData->AddInput (aData);
      }
    }
    
    allData->Update ();
    cerr << "Total number of points on screen: " << allData->GetOutput ()->GetNumberOfPoints () << "... Dumping...";
    if (potentialPCDSave)
    {
      // Assemble everything here so we don't have to include CommonIORoutines too
      std::ofstream fs;
      fs.precision (5);
      fs.open ("dump-screen.pcd");
      fs << "COLUMNS x y z" << std::endl;
      vtkSmartPointer<vtkCleanPolyData> cleaner = vtkSmartPointer<vtkCleanPolyData>::New ();
      cleaner->SetTolerance (0.0);
      cleaner->SetInput (allData->GetOutput ());
      cleaner->ConvertLinesToPointsOff ();
      cleaner->ConvertPolysToLinesOff ();
      cleaner->ConvertStripsToPolysOff ();
      cleaner->PointMergingOn ();
      cleaner->Update ();
      fs << "POINTS " << cleaner->GetOutput ()->GetNumberOfPoints () << std::endl;

      double p[3];
      for (int i = 0; i < cleaner->GetOutput ()->GetNumberOfPoints (); i++)
      {
        cleaner->GetOutput ()->GetPoint (i, p);
        for (unsigned int j = 0; j < 3; j++)
          fs << p[j] << " ";
        fs << std::endl;
      }
      fs.close ();
      
      cerr << "...pruned " << 
        allData->GetOutput ()->GetNumberOfPoints () - cleaner->GetOutput ()->GetNumberOfPoints () << 
        ", left " << cleaner->GetOutput ()->GetNumberOfPoints () << " (PCD)";
    }
    else
    {
      cerr << "(VTK)";
      SaveDataSet (allData->GetOutput (), "dump-screen.vtk");
    }
    cerr << "[done]" << endl;
  }
  
  this->FindPokedRenderer (rwi->GetEventPosition ()[0], rwi->GetEventPosition ()[1]);

  // ---[ Check the rest of the key codes
  switch (this->Interactor->GetKeyCode ())
  {
    case 'h':
    case 'H':
    {
      cerr << "History/Legend:" << endl;
      cerr << "          p, P   : switch to a point-based representation" << endl;
      cerr << "          w, W   : switch to a wireframe-based representation" << endl;
      cerr << "          s, S   : switch to a surface-based representation" << endl;
      cerr << endl;
      cerr << "          i, I   : display information about each dataset (using printself)" << endl;
      cerr << endl;
      cerr << "          j, J   : take a .PNG snapshot of the current window view" << endl;
      cerr << "         <space> : save all data to file in VTK format" << endl;
      cerr << "CTRL+ALT+<space> : perform a vtkCleanPolyData and save all data to file in PCD format" << endl;
      cerr << endl;
      cerr << "          c, C   : display current camera settings/parameters" << endl;
      cerr << "           +     : increment overall point size" << endl;
      cerr << "           -     : decrement overall point size" << endl;
      cerr << endl;
      cerr << "          t, T   : enable/disable the display of texts (given as command line parameters) on screen" << endl;
      cerr << "          <, >   : display 'previous', 'next' text in the list" << endl;
      cerr << endl;
      cerr << "          g, G   : display scale grid (on/off)" << endl;
      cerr << "         0 - 9   : change the scalars (see L)" << endl;
      cerr << "          l, L   : display scalar legend at the console" << endl;
      cerr << "          u, U   : display the LUT (LookUp Table) actor (on/off)" << endl;
      cerr << "          o, O   : switch the LUT (LookUp Table) between logarithmic and linear" << endl;
      break;
    }

    case 't':
    case 'T':
    {
      // If no texts given on the command line, just exit
      if (textList.size () == 0)
        break;
      
      // First time we press 't' this should pop up
      if (this->textActor == NULL)
      {
        this->cur_text = 0;
        
        this->textActor = vtkTextActor::New ();
        this->textActor->SetInput (this->textList.at (this->cur_text).c_str ());
        this->textActor->GetTextProperty ()->SetFontSize (this->tS);
        this->textActor->GetTextProperty ()->SetFontFamilyToArial ();
        this->textActor->GetTextProperty ()->BoldOn ();
        this->textActor->GetTextProperty ()->SetJustificationToRight ();
        this->textActor->GetTextProperty ()->SetVerticalJustificationToTop ();
        this->textActor->GetTextProperty ()->SetColor (this->tR, this->tG, this->tB);
        this->textActor->GetProperty ()->SetOpacity (0.75);
        
        this->textActor->SetDisplayPosition (this->tAx, this->tAy);
        this->textActor->VisibilityOn ();
        
        if (one_renderer)
          this->renderer->AddActor (textActor);
        else
        {
          this->rendererCollection->InitTraversal ();
          vtkRenderer *ren = this->rendererCollection->GetFirstRenderer ();
          do
          {
            ren->AddActor (textActor);
          }
          while ((ren = this->rendererCollection->GetNextItem ()) != NULL);
        }
        
        this->texts_enabled = true;
        
        break;
      }

      if (!texts_enabled)
      {
        this->textActor->VisibilityOn ();
        this->texts_enabled = true;
      }
      else
      {
        this->textActor->VisibilityOff ();
        this->texts_enabled = false;
      }
      
      break;
    }
      
    case '<':
    {
      if (this->textActor == NULL)
        break;

      this->cur_text--;

      // Circular loop
      if (this->cur_text < 0) 
        this->cur_text = this->textList.size () - 1;
      
      // Update objects
      this->textActor->SetInput (this->textList.at (this->cur_text).c_str ());
      this->textActor->Modified ();
      break;
    }
        
    case '>':
    {
      if (this->textActor == NULL)
        break;
      
      this->cur_text++;
       
      // Circular loop
      if (this->cur_text > (int)this->textList.size () - 1)
        this->cur_text = 0;

      // Update objects
      this->textActor->SetInput (this->textList.at (this->cur_text).c_str ());
      this->textActor->Modified ();
      break;
    }

    case 'i':
    case 'I':
    {
      vtkActorCollection *ac;
      vtkActor *anActor, *aPart;
      vtkAssemblyPath *path;
      ac = this->CurrentRenderer->GetActors ();
      vtkCollectionSimpleIterator ait;
      for (ac->InitTraversal (ait); (anActor = ac->GetNextActor (ait)); )
      {
        for (anActor->InitPathTraversal (); (path = anActor->GetNextPath ()); )
        {
          aPart=(vtkActor *)path->GetLastNode ()->GetViewProp ();
//          aPart->GetMapper ()->GetInputAsDataSet ()->PrintSelf (std::cerr, 0);
        }
      }
      break;
    }
    case 'p':
    case 'P':
    {
      vtkActorCollection *ac;
      vtkActor *anActor, *aPart;
      vtkAssemblyPath *path;
      ac = this->CurrentRenderer->GetActors ();
      vtkCollectionSimpleIterator ait;
      for (ac->InitTraversal (ait); (anActor = ac->GetNextActor (ait)); )
      {
        for (anActor->InitPathTraversal (); (path = anActor->GetNextPath ()); )
        {
          aPart=(vtkActor *)path->GetLastNode ()->GetViewProp ();
          aPart->GetProperty ()->SetRepresentationToPoints ();
        }
      }
      break;
    }
/*    case 'f':
    case 'F':
    {
      int *scrSize = rwi->GetRenderWindow ()->GetScreenSize ();
      int *winSize = rwi->GetRenderWindow ()->GetSize ();
      if ((scrSize[0] == winSize[0]) && (scrSize[1] == winSize[1]))
      {
        rwi->GetRenderWindow ()->SetSize (600, 600);
        rwi->GetRenderWindow ()->SetBorders (1);
      }
      else
      {
        rwi->GetRenderWindow ()->SetSize (scrSize[0], scrSize[1]);
        rwi->GetRenderWindow ()->SetBorders (0);
      }
      break;
    }*/
    case 'J':
    case 'j':
    {
      unsigned t = time (0);
      sprintf (camFN, "screenshot-%d.cam", t);
      sprintf (fileName, "screenshot-%d.png" , t);
      this->writer->SetFileName (fileName);
      this->writer->Write ();
      cerr << "> Wrote " << fileName << "..." << endl;
      
      ofstream ofs_cam;
      ofs_cam.open (camFN);
      
      vtkCamera* cam = rwi->GetRenderWindow ()->GetRenderers ()->GetFirstRenderer ()->GetActiveCamera ();
      double clip[2], focal[3], pos[3], view[3];
      cam->GetClippingRange (clip);
      cam->GetFocalPoint (focal);
      cam->GetPosition (pos);
      cam->GetViewUp (view);
      ofs_cam << clip[0]  << "," << clip[1]  << "/" <<
                 focal[0] << "," << focal[1] << "," << focal[2] << "/" <<
                 pos[0]   << "," << pos[1]   << "," << pos[2]   << "/" <<
                 view[0]  << "," << view[1]  << "," << view[2]  << endl;
      ofs_cam.close ();
      cerr << "> Wrote camera information to file (" << camFN << ") : -[ Clipping Range / Focal Point / Position / ViewUp ]-" << endl;
      cerr << clip[0]  << "," << clip[1]  << "/" <<
              focal[0] << "," << focal[1] << "," << focal[2] << "/" <<
              pos[0]   << "," << pos[1]   << "," << pos[2]   << "/" <<
              view[0]  << "," << view[1]  << "," << view[2]  << endl;
      break;
    }
    case 'C':
    case 'c':
    {
      vtkCamera* cam = rwi->GetRenderWindow ()->GetRenderers ()->GetFirstRenderer ()->GetActiveCamera ();
//      cam->PrintSelf (cerr, 0);
      double clip[2], focal[3], pos[3], view[3];
      cam->GetClippingRange (clip);
      cam->GetFocalPoint (focal);
      cam->GetPosition (pos);
      cam->GetViewUp (view);
      cerr << "-[ Clipping Range / Focal Point / Position / ViewUp ]-" << endl;
      cerr << clip[0]  << "," << clip[1]  << "/" <<
              focal[0] << "," << focal[1] << "," << focal[2] << "/" <<
              pos[0]   << "," << pos[1]   << "," << pos[2]   << "/" <<
              view[0]  << "," << view[1]  << "," << view[2]  << endl;
      break;
    }
    case KEY_PLUS:
    {
      cerr << "Current point size: " << pointsize << endl;
      vtkActorCollection *ac;
      vtkActor *anActor, *aPart;
      vtkAssemblyPath *path;
      ac = this->CurrentRenderer->GetActors ();
      vtkCollectionSimpleIterator ait;
      for (ac->InitTraversal (ait); (anActor = ac->GetNextActor (ait)); )
      {
        for (anActor->InitPathTraversal (); (path=anActor->GetNextPath ()); )
        {
          aPart=(vtkActor *)path->GetLastNode ()->GetViewProp ();
          aPart->GetProperty ()->SetPointSize (aPart->GetProperty ()->GetPointSize () + 1);
        }
      }
      break;
    }
    case KEY_MINUS:
    {
      vtkActorCollection *ac;
      vtkActor *anActor, *aPart;
      vtkAssemblyPath *path;
      ac = this->CurrentRenderer->GetActors ();
      vtkCollectionSimpleIterator ait;
      for (ac->InitTraversal (ait); (anActor = ac->GetNextActor (ait)); )
      {
        for (anActor->InitPathTraversal (); (path=anActor->GetNextPath ()); )
        {
          aPart=(vtkActor *)path->GetLastNode ()->GetViewProp ();
          aPart->GetProperty ()->SetPointSize (aPart->GetProperty ()->GetPointSize () - 1);
        }
      }
      break;
    }
    
    // Display a grid/scale over the screen
    case 'G':
    case 'g':
    {
      if (!grid_enabled)
      {
        gridActor->TopAxisVisibilityOn ();
        this->CurrentRenderer->AddViewProp (gridActor);
        
        this->grid_enabled = true;
      }
      else
      {
        this->CurrentRenderer->RemoveViewProp (gridActor);
        this->grid_enabled = false;
      }
      this->CurrentRenderer->RemoveActor (lutActor);
      this->lut_enabled = false;
      
      break;
    }
    
    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    // ---[ Entries regarding the legend and color auto-switching possibilities
    case '0':                 // Return to grayscale or the user given color
    {
      if (!advanced) break;

      for (unsigned int cp = 0; cp < this->allScalars.size (); cp++)
      {
        ScalarsContainer s = this->allScalars[cp];
        changeActorScalarAppearance (s, -1, argc, argv);
        lutActor->SetVisibility (false);
      }
      
      // Change the text actor (if enabled)
      if (this->textActor == NULL)
        break;

      if (this->textList.size () > 0)
        this->cur_text = 0;

      // Update objects
      this->textActor->SetInput (this->textList.at (this->cur_text).c_str ());
      this->textActor->Modified ();
      break;
    }
    ///////////////////////////////////////////////////////////////////////////
    
    case '1':
    {
      if (!advanced) break;

      if (this->Interactor->GetControlKey ());
///        cerr << "Ctrl + 1" << endl;
      else
      {
        for (unsigned int cp = 0; cp < this->allScalars.size (); cp++)
        {
          ScalarsContainer s = this->allScalars[cp];
          if (s.dimensions.size () == 0) continue;
          changeActorScalarAppearance (s, 0, argc, argv);

          vtkLookupTable* lut = static_cast<vtkLookupTable*>(s.actor->GetMapper ()->GetLookupTable ());
          lutActor->SetLookupTable (lut);
          lutActor->Modified ();
        }
      }

      // Change the text actor (if enabled)
      if (this->textActor == NULL)
        break;

      if (this->textList.size () > 1)
        this->cur_text = 1;

      // Update objects
      lutActor->SetTitle (this->textList.at (this->cur_text).c_str ());
      lutActor->Modified ();
      this->textActor->SetInput (this->textList.at (this->cur_text).c_str ());
      this->textActor->Modified ();
      break;
    }
    ///////////////////////////////////////////////////////////////////////////
    case '2':
    {
      if (!advanced) break;
      
      if (this->Interactor->GetControlKey ());
///        cerr << "Ctrl + 2" << endl;
      else
        for (unsigned int cp = 0; cp < this->allScalars.size (); cp++)
        {
          ScalarsContainer s = this->allScalars[cp];
          if (s.dimensions.size () < 2) continue;
          changeActorScalarAppearance (s, 1, argc, argv);
          
          vtkLookupTable* lut = static_cast<vtkLookupTable*>(s.actor->GetMapper ()->GetLookupTable ());
          lutActor->SetLookupTable (lut);
          lutActor->Modified ();
        }

      // Change the text actor (if enabled)
      if (this->textActor == NULL)
        break;

      if (this->textList.size () > 2)
        this->cur_text = 2;

      // Update objects
      lutActor->SetTitle (this->textList.at (this->cur_text).c_str ());
      lutActor->Modified ();
      this->textActor->SetInput (this->textList.at (this->cur_text).c_str ());
      this->textActor->Modified ();
      break;
    }
    ///////////////////////////////////////////////////////////////////////////
    case '3':
    {
      if (!advanced) break;

      if (this->Interactor->GetControlKey ());
///        cerr << "Ctrl + 3" << endl;
      else
        for (unsigned int cp = 0; cp < this->allScalars.size (); cp++)
        {
          ScalarsContainer s = this->allScalars[cp];
          if (s.dimensions.size () < 3) continue;
          changeActorScalarAppearance (s, 2, argc, argv);
          
          vtkLookupTable* lut = static_cast<vtkLookupTable*>(s.actor->GetMapper ()->GetLookupTable ());
          lutActor->SetLookupTable (lut);
          lutActor->Modified ();
        }

      // Change the text actor (if enabled)
      if (this->textActor == NULL)
        break;

      if (this->textList.size () > 3)
        this->cur_text = 3;

      // Update objects
      lutActor->SetTitle (this->textList.at (this->cur_text).c_str ());
      lutActor->Modified ();
      this->textActor->SetInput (this->textList.at (this->cur_text).c_str ());
      this->textActor->Modified ();
      break;
    }
    ///////////////////////////////////////////////////////////////////////////
    case '4':
    {
      if (!advanced) break;

      if (this->Interactor->GetControlKey ());
///        cerr << "Ctrl + 4" << endl;
      else
        for (unsigned int cp = 0; cp < this->allScalars.size (); cp++)
        {
          ScalarsContainer s = this->allScalars[cp];
          if (s.dimensions.size () < 4) break;
          changeActorScalarAppearance (s, 3, argc, argv);
          
          vtkLookupTable* lut = static_cast<vtkLookupTable*>(s.actor->GetMapper ()->GetLookupTable ());
          lutActor->SetLookupTable (lut);
          lutActor->Modified ();
        }

      // Change the text actor (if enabled)
      if (this->textActor == NULL)
        break;

      if (this->textList.size () > 4)
        this->cur_text = 4;

      // Update objects
      lutActor->SetTitle (this->textList.at (this->cur_text).c_str ());
      lutActor->Modified ();
      this->textActor->SetInput (this->textList.at (this->cur_text).c_str ());
      this->textActor->Modified ();
      break;
    }
    ///////////////////////////////////////////////////////////////////////////
    case '5':
    {
      if (!advanced) break;

      if (this->Interactor->GetControlKey ());
///        cerr << "Ctrl + 5" << endl;
      else
        for (unsigned int cp = 0; cp < this->allScalars.size (); cp++)
        {
          ScalarsContainer s = this->allScalars[cp];
          if (s.dimensions.size () < 5) continue;
          changeActorScalarAppearance (s, 4, argc, argv);
          
          vtkLookupTable* lut = static_cast<vtkLookupTable*>(s.actor->GetMapper ()->GetLookupTable ());
          lutActor->SetLookupTable (lut);
          lutActor->Modified ();
        }

      // Change the text actor (if enabled)
      if (this->textActor == NULL)
        break;

      if (this->textList.size () > 5)
        this->cur_text = 5;

      // Update objects
      lutActor->SetTitle (this->textList.at (this->cur_text).c_str ());
      lutActor->Modified ();
      this->textActor->SetInput (this->textList.at (this->cur_text).c_str ());
      this->textActor->Modified ();
      break;
    }
    ///////////////////////////////////////////////////////////////////////////
    case '6':
    {
      if (!advanced) break;

      if (this->Interactor->GetControlKey ());
///        cerr << "Ctrl + 6" << endl;
      else
        for (unsigned int cp = 0; cp < this->allScalars.size (); cp++)
        {
          ScalarsContainer s = this->allScalars[cp];
          if (s.dimensions.size () < 6) continue;
          changeActorScalarAppearance (s, 5, argc, argv);
          
          vtkLookupTable* lut = static_cast<vtkLookupTable*>(s.actor->GetMapper ()->GetLookupTable ());
          lutActor->SetLookupTable (lut);
          lutActor->Modified ();
        }

      // Change the text actor (if enabled)
      if (this->textActor == NULL)
        break;

      if (this->textList.size () > 6)
        this->cur_text = 6;

      // Update objects
      lutActor->SetTitle (this->textList.at (this->cur_text).c_str ());
      lutActor->Modified ();
      this->textActor->SetInput (this->textList.at (this->cur_text).c_str ());
      this->textActor->Modified ();
      break;
    }
    ///////////////////////////////////////////////////////////////////////////
    case '7':
    {
      if (!advanced) break;

      if (this->Interactor->GetControlKey ());
///        cerr << "Ctrl + 7" << endl;
      else
        for (unsigned int cp = 0; cp < this->allScalars.size (); cp++)
        {
          ScalarsContainer s = this->allScalars[cp];
          if (s.dimensions.size () < 7) continue;
          changeActorScalarAppearance (s, 6, argc, argv);
          
          vtkLookupTable* lut = static_cast<vtkLookupTable*>(s.actor->GetMapper ()->GetLookupTable ());
          lutActor->SetLookupTable (lut);
          lutActor->Modified ();
        }

      // Change the text actor (if enabled)
      if (this->textActor == NULL)
        break;

      if (this->textList.size () > 7)
        this->cur_text = 7;

      // Update objects
      lutActor->SetTitle (this->textList.at (this->cur_text).c_str ());
      lutActor->Modified ();
      this->textActor->SetInput (this->textList.at (this->cur_text).c_str ());
      this->textActor->Modified ();
      break;
    }
    ///////////////////////////////////////////////////////////////////////////
    case '8':
    {
      if (!advanced) break;
      
      if (this->Interactor->GetControlKey ());
///        cerr << "Ctrl + 8" << endl;
      else
        for (unsigned int cp = 0; cp < this->allScalars.size (); cp++)
        {
          ScalarsContainer s = this->allScalars[cp];
          if (s.dimensions.size () < 8) continue;
          changeActorScalarAppearance (s, 7, argc, argv);
          
          vtkLookupTable* lut = static_cast<vtkLookupTable*>(s.actor->GetMapper ()->GetLookupTable ());
          lutActor->SetLookupTable (lut);
          lutActor->Modified ();
        }

      // Change the text actor (if enabled)
      if (this->textActor == NULL)
        break;

      if (this->textList.size () > 8)
        this->cur_text = 8;

      // Update objects
      lutActor->SetTitle (this->textList.at (this->cur_text).c_str ());
      lutActor->Modified ();
      this->textActor->SetInput (this->textList.at (this->cur_text).c_str ());
      this->textActor->Modified ();
      break;
    }
    ///////////////////////////////////////////////////////////////////////////
    case '9':
    {
      if (!advanced) break;
      
      if (this->Interactor->GetControlKey ());
///        cerr << "Ctrl + 9" << endl;
      else
        for (unsigned int cp = 0; cp < this->allScalars.size (); cp++)
        {
          ScalarsContainer s = this->allScalars[cp];
          if (s.dimensions.size () < 9) continue;
          changeActorScalarAppearance (s, 8, argc, argv);
          
          vtkLookupTable* lut = static_cast<vtkLookupTable*>(s.actor->GetMapper ()->GetLookupTable ());
          lutActor->SetLookupTable (lut);
          lutActor->Modified ();
        }

      // Change the text actor (if enabled)
      if (this->textActor == NULL)
        break;

      if (this->textList.size () > 9)
        this->cur_text = 9;

      // Update objects
      lutActor->SetTitle (this->textList.at (this->cur_text).c_str ());
      lutActor->Modified ();
      this->textActor->SetInput (this->textList.at (this->cur_text).c_str ());
      this->textActor->Modified ();
      break;
    }
    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    
    
    
    // Display LUT 
    case 'U':
    case 'u':
    {
      if (!advanced) break;
      
      this->FindPokedRenderer (rwi->GetEventPosition ()[0],
                               rwi->GetEventPosition ()[1]);
      
      for (unsigned int cp = 0; cp < this->allScalars.size (); cp++)
      {
        ScalarsContainer s = this->allScalars[cp];
        if (s.dimensions.size () == 0) break;
        vtkLookupTable* lut = static_cast<vtkLookupTable*>(s.actor->GetMapper ()->GetLookupTable ());
        lutActor->SetLookupTable (lut);
        lutActor->Modified ();
        
        if (!lut_enabled)
        {
          this->CurrentRenderer->AddActor (lutActor);
          lutActor->SetVisibility (true);
          this->lut_enabled = true;
        }
        else
        {
          this->CurrentRenderer->RemoveActor (lutActor);
          this->lut_enabled = false;
        }
      }
      
      rwi->Render();
      break;
    }
    
    // Switch LUT between logarithmic and normal
    case 'O':
    case 'o':
    {
      if (!advanced) break;
      
      this->FindPokedRenderer (rwi->GetEventPosition ()[0],
                               rwi->GetEventPosition ()[1]);
      
      for (unsigned int cp = 0; cp < this->allScalars.size (); cp++)
      {
        ScalarsContainer s = this->allScalars[cp];
        if (s.dimensions.size () < 1) break;
        vtkLookupTable* lut = static_cast<vtkLookupTable*>(s.actor->GetMapper ()->GetLookupTable ());
///        cerr << lut->GetClassName () << endl;
        lut->SetScale (!lut->GetScale ());
        
        lutActor->SetLookupTable (lut);
        lutActor->Modified ();
        
        s.actor->GetMapper ()->SetLookupTable (lut);
        s.actor->Modified ();
      }
      
      rwi->Render();
      break;
    }
    
    case 'L':
    case 'l':
    {
      if (!advanced) break;
      
      for (unsigned int cp = 0; cp < this->allScalars.size (); cp++)
      {
        ScalarsContainer s = this->allScalars[cp];
        if (s.dimensions.size () == 0) break;
        
        cerr << "Available dimensions: default(0) ";
        for (unsigned int dim = 0; dim < s.dimensions.size (); dim++)
          cerr << s.dimensions[dim] << "(" << dim+1 << ") ";
        cerr << endl;
      }
      if (this->Interactor->GetControlKey ())
        cerr << "control" << endl;
      
      
      break;
    }
    default:
    {
//      printf("Key hit: %d\n", this->Interactor->GetKeyCode());
      this->Superclass::OnChar ();
      break;
    }
  }
  
  
  rwi->Render();
}


class vtkFPSCallback : public vtkCommand
{
public:
  static vtkFPSCallback *New () { return new vtkFPSCallback;}
  void SetTextActor (vtkTextActor *txt);
  virtual void Execute (vtkObject *caller, unsigned long, void*);
protected:
  vtkTextActor *TextActor;
  char TextBuff[128];
};

////////////////////////////////////////////////////////////////////////////////
//------------------------------------------------------------------------------
////////////////////////////////////////////////////////////////////////////////
void
vtkFPSCallback::SetTextActor (vtkTextActor *txt)
{
  this->TextActor = txt;
}

void
vtkFPSCallback::Execute (vtkObject *caller, unsigned long, void*)
{
  vtkRenderer *ren = reinterpret_cast<vtkRenderer *> (caller);
  
  long int nr_points = 0;
  vtkActorCollection *ac = ren->GetActors ();
  vtkActor *anActor, *aPart;
  vtkCollectionSimpleIterator ait;
  vtkAssemblyPath *path;
  for (ac->InitTraversal (ait); (anActor = ac->GetNextActor (ait)); )
  {
    for (anActor->InitPathTraversal(); (path=anActor->GetNextPath ()); )
    {
      aPart=(vtkActor *)path->GetLastNode ()->GetViewProp ();
      nr_points += aPart->GetMapper ()->GetInputAsDataSet ()->GetNumberOfPoints ();
    }
  }
  
  float fps = 1.0/ren->GetLastRenderTimeInSeconds ();
  snprintf (this->TextBuff, 127, "%.1f FPS, %ld points", fps, nr_points);
  this->TextActor->SetInput (this->TextBuff);
}


////////////////////////////////////////////////////////////////////////////////
// ---[ Construct the perspective RenderWindow and the RenderWindowInteractor
vtkRenderWindowInteractor*
  create_render_window_and_interactor (vtkRenderer *ren, const char* title)
{
//  vtkSmartPointer<vtkLightKit> lightKit = vtkLightKit::New ();
//  lightKit->AddLightsToRenderer (ren);

  vtkSmartPointer<vtkTextActor> txt = vtkSmartPointer<vtkTextActor>::New ();
  vtkSmartPointer<vtkFPSCallback> UpdateFPS = vtkSmartPointer<vtkFPSCallback>::New ();
  UpdateFPS->SetTextActor (txt);
  ren->AddObserver (vtkCommand::EndEvent, UpdateFPS);
  txt->GetProperty ()->SetColor (0.0, 0.0, 1.0);
  ren->AddActor (txt);

  vtkRenderWindow *renWin = vtkRenderWindow::New ();
  renWin->SetWindowName (title);
  renWin->AddRenderer (ren);
  renWin->SetSize (RENWIN_WIDTH, RENWIN_HEIGHT);

  vtkSmartPointer<vtkWindowToImageFilter> wif = vtkSmartPointer<vtkWindowToImageFilter>::New ();
  wif->SetInput (renWin);

  // Write screenshot to file
  vtkPNGWriter* w = vtkPNGWriter::New ();
//  w->SetQuality (100);
  w->SetInputConnection (wif->GetOutputPort ());

  vtkRenderWindowInteractor *iren = renWin->MakeRenderWindowInteractor ();
  vtkSmartPointer<vtkInteractorStyleTUM> style = vtkSmartPointer<vtkInteractorStyleTUM>::New ();
  style->setRenderer (ren);
  style->UseTimersOn ();
  style->setWriter (w);
  style->setWindowFilter (wif);

  vtkPointPicker *pp = vtkPointPicker::New ();
  iren->SetPicker (pp);

  iren->SetInteractorStyle (style);
  iren->Initialize ();
  //iren->CreateRepeatingTimer (1000L);
  iren->CreateRepeatingTimer (5000L);
  return iren;
}
#endif
