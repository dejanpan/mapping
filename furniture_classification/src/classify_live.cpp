#include <pcl17/console/print.h>
#include <pcl17/io/openni_grabber.h>
#include <pcl17/visualization/cloud_viewer.h>
#include <pcl17/classification/PHVObjectClassifier.h>
#include <pcl17/features/sgfall.h>
#include <pcl17/console/parse.h>
#include <pcl17/features/vfh.h>
#include <pcl17/features/esf.h>

template<class FeatureType, class FeatureEstimatorType>
  class SimpleOpenNIViewer
  {
  public:
    SimpleOpenNIViewer(std::string database) :
      viewer("PCL OpenNI Viewer"), classify(false), classificaton_running(false), interface(new pcl17::OpenNIGrabber())
    {
      std::string database_dir = database;
      std::string debug_folder = "debug_classification/";

      typename pcl17::Feature<pcl17::PointNormal, FeatureType>::Ptr feature_estimator(new FeatureEstimatorType);
      oc.setFeatureEstimator(feature_estimator);

      oc.setDatabaseDir(database_dir);
      oc.loadFromFile();

      oc.setDebugFolder(debug_folder);
      oc.setDebug(true);

    }

    void cloud_cb_(const pcl17::PointCloud<pcl17::PointXYZRGBA>::ConstPtr &cloud)
    {
      if (viewer.wasStopped())
        return;

      if (!classify && !classificaton_running)
      {
        viewer.showCloud(convert(cloud, 30));
      }
      else if (classify && !classificaton_running)
      {
        classificaton_running = true;
        interface->stop();
        //viewer.showCloud(cloud);

        pcl17::PointCloud<pcl17::PointXYZRGBA>::Ptr cloud_transformed = convert(cloud, 30);

        pcl17::PointCloud<pcl17::PointXYZ>::Ptr scene(new pcl17::PointCloud<pcl17::PointXYZ>);

        pcl17::copyPointCloud(*cloud_transformed, *scene);

        scene->sensor_origin_ = cloud_transformed->sensor_origin_;
        scene->sensor_orientation_ = cloud_transformed->sensor_orientation_;

        std::cerr << "Added scene" << std::endl;
        oc.setScene(scene);
        std::cerr << "Classifying" << std::endl;
        oc.classify();

        map<string, vector<pcl17::PointCloud<pcl17::PointNormal>::Ptr> > objects = oc.getFoundObjects();

        typedef typename map<string, vector<pcl17::PointCloud<pcl17::PointNormal>::Ptr> >::value_type vt;

        BOOST_FOREACH(vt &v, objects)
{        for(size_t i=0; i<v.second.size(); i++)
        {
          pcl17::PointCloud<pcl17::PointXYZRGBA>::Ptr model(new pcl17::PointCloud<pcl17::PointXYZRGBA>);
          pcl17::copyPointCloud(*v.second[i], *model);

          for(size_t i=0; i<model->points.size(); i++)
          {
            model->points[i].r = 255;
            model->points[i].g = 0;
            model->points[i].b = 0;
          }

          *cloud_transformed += *model;

        }
      }

      viewer.showCloud(cloud_transformed);

    }
  }

  void keyboard_cb(const pcl17::visualization::KeyboardEvent &event, void * tmp)
  {
    if (event.getKeySym() == "c" && event.keyDown())
    {
      classify = true;
    }
  }

  void run()
  {

    boost::function<void(const pcl17::PointCloud<pcl17::PointXYZRGBA>::ConstPtr&)> f =
    boost::bind(&SimpleOpenNIViewer::cloud_cb_, this, _1);

    //boost::function<void (const pcl17::visualization::KeyboardEvent&)> k =
    //        boost::bind(&SimpleOpenNIViewer::keyboard_cb, this, _1);

    interface->registerCallback(f);
    viewer.registerKeyboardCallback(&SimpleOpenNIViewer::keyboard_cb, *this, NULL);

    interface->start();

    while (!viewer.wasStopped())
    {
      boost::this_thread::sleep(boost::posix_time::seconds(1));
    }

    interface->stop();
  }

  pcl17::PointCloud<pcl17::PointXYZRGBA>::Ptr convert(pcl17::PointCloud<pcl17::PointXYZRGBA>::ConstPtr scene, int tilt)
  {

    pcl17::PointCloud<pcl17::PointXYZRGBA>::Ptr cloud_transformed(new pcl17::PointCloud<pcl17::PointXYZRGBA>), cloud_aligned(new pcl17::PointCloud<pcl17::PointXYZRGBA>);

    Eigen::Affine3f view_transform;
    view_transform.matrix() << 0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1;

    Eigen::AngleAxis<float> rot(tilt * M_PI / 180, Eigen::Vector3f(0, 1, 0));

    view_transform.prerotate(rot);

    pcl17::transformPointCloud(*scene, *cloud_transformed, view_transform);

    pcl17::ModelCoefficients::Ptr coefficients(new pcl17::ModelCoefficients);
    pcl17::PointIndices::Ptr inliers(new pcl17::PointIndices);

    pcl17::SACSegmentation<pcl17::PointXYZRGBA> seg;

    seg.setOptimizeCoefficients(true);

    seg.setModelType(pcl17::SACMODEL_PLANE);
    seg.setMethodType(pcl17::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);

    seg.setInputCloud(cloud_transformed);
    seg.segment(*inliers, *coefficients);

    std::cout << "Z vector: " << coefficients->values[0] << " " << coefficients->values[1] << " "
    << coefficients->values[2] << " " << coefficients->values[3] << std::endl;

    Eigen::Vector3f z_current(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    Eigen::Vector3f y(0, 1, 0);

    Eigen::Affine3f rotation;
    rotation = pcl17::getTransFromUnitVectorsZY(z_current, y);
    rotation.translate(Eigen::Vector3f(0, 0, coefficients->values[3]));

    pcl17::transformPointCloud(*cloud_transformed, *cloud_aligned, rotation);

    Eigen::Affine3f res = (rotation * view_transform);

    cloud_aligned->sensor_origin_ = res * Eigen::Vector4f(0, 0, 0, 1);
    cloud_aligned->sensor_orientation_ = res.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f(0, 0, 1))).rotation();

    seg.setInputCloud(cloud_aligned);
    seg.segment(*inliers, *coefficients);

    std::cout << "Z vector: " << coefficients->values[0] << " " << coefficients->values[1] << " "
    << coefficients->values[2] << " " << coefficients->values[3] << std::endl;

    return cloud_aligned;

  }

  pcl17::visualization::CloudViewer viewer;

  bool classify;
  bool classificaton_running;
  pcl17::PHVObjectClassifier<pcl17::PointXYZ, pcl17::PointNormal, FeatureType> oc;
  pcl17::Grabber* interface;

};

int main(int argc, char **argv)
{

  if (argc < 5)
  {
    PCL17_INFO ("Usage %s -database /path/to/database -features sgf | esf | vfh \n", argv[0]);
    return -1;
  }

  std::string database;
  std::string features = "sgf";

  pcl17::console::parse_argument(argc, argv, "-database", database);
  pcl17::console::parse_argument(argc, argv, "-features", features);

  if (features == "sgf")
  {
    SimpleOpenNIViewer<pcl17::Histogram<pcl17::SGFALL_SIZE>, pcl17::SGFALLEstimation<pcl17::PointNormal, pcl17::Histogram<
        pcl17::SGFALL_SIZE> > > v(database);
    v.run();
  }
  else if (features == "esf")
  {
    SimpleOpenNIViewer<pcl17::ESFSignature640, pcl17::ESFEstimation<pcl17::PointNormal, pcl17::ESFSignature640> > v(database);
    v.run();
  }
  else if (features == "vfh")
  {
    SimpleOpenNIViewer<pcl17::VFHSignature308, pcl17::VFHEstimation<pcl17::PointNormal, pcl17::PointNormal,
        pcl17::VFHSignature308> > v(database);
    v.run();
  }
  else
  {
    std::cerr << "Unknown feature type " << features << " specified" << std::endl;
  }

  return 0;
}
