

#include <iostream>
#include <pcl17/point_types.h>
#include <pcl17/point_cloud.h>
#include <pcl17/io/vtk_lib_io.h>
#include <pcl17/io/pcd_io.h>
#include <pcl17/common/transforms.h>
#include <pcl17/visualization/pcl_visualizer.h>
#include <pcl17/console/print.h>


int main(int argc, char ** argv) {
  vtkSmartPointer<vtkPLYReader> readerQuery = vtkSmartPointer<vtkPLYReader>::New();
  readerQuery->SetFileName (argv[1]);
  vtkSmartPointer<vtkPolyData> polydata = readerQuery->GetOutput();
  polydata->Update();

  pcl17::visualization::PCLVisualizer vis("Visualizer");

  vis.addModelFromPolyData (polydata, "mesh1", 0);

  vis.camera_.window_size[0] = 480;
  vis.camera_.window_size[1] = 480;
  vis.camera_.pos[0] = 0;
  vis.camera_.pos[1] = 5;
  vis.camera_.pos[2] = 5;
  vis.camera_.focal[0] = 0;
  vis.camera_.focal[1] = 0;
  vis.camera_.focal[2] = 0;
  vis.camera_.view[0] = 0;
  vis.camera_.view[1] = 1;
  vis.camera_.view[2] = 0;

  vis.updateCamera();
  vis.resetCamera();

  vis.setRepresentationToSurfaceForAllActors();
  vis.spin();

  //call render in the visualizer to obtain a point cloud of the scene
  pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud_out (new pcl17::PointCloud<pcl17::PointXYZ> ());
  vis.renderView(256,256, cloud_out);

  pcl17::io::savePCDFileBinary("scene.pcd",*cloud_out);

  return 0;
}
