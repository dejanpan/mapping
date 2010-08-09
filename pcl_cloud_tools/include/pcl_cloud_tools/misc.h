#ifndef PCL_CLOUD_TOOLS_MISC_H_
#define PCL_CLOUD_TOOLS_MISC_H_

#include <vtkPolyDataReader.h>
#include <vtkMaskPoints.h>

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
vtkActor* 
create_actor_from_data_set (vtkDataSet *data, double c1, double c2, double c3, bool lod_enable = false)
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

#endif
