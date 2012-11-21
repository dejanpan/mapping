#include <ros/ros.h>
#include <pcl17_ros/point_cloud.h>
#include <pcl17/console/print.h>
#include <pcl17/classification/PHVObjectClassifier.h>
#include <pcl17/features/sgfall.h>
#include <pcl17/console/parse.h>
#include <pcl17/features/vfh.h>
#include <pcl17/features/esf.h>

template<class FeatureType, class FeatureEstimatorType>
  class FitModelsNode
  {
  public:
    FitModelsNode(std::string database, int n)
    {
      std::string database_dir = database;
      std::string debug_folder = "debug_classification/";
      std::string n_str = boost::lexical_cast<std::string>(n);

      typename pcl17::Feature<pcl17::PointNormal, FeatureType>::Ptr feature_estimator(new FeatureEstimatorType);
      oc.setFeatureEstimator(feature_estimator);

      oc.setDatabaseDir(database_dir);
      oc.loadFromFile();

      //oc.setDebugFolder(debug_folder);
      oc.setDebug(false);

      ros::NodeHandle nh;
      pub = nh.advertise<pcl17::PointCloud<pcl17::PointNormal> > ("/fitted_models" + n_str, 1);
      sub = nh.subscribe<pcl17::PointCloud<pcl17::PointXYZ> > ("/cloud_pcd", 1, &FitModelsNode::cloud_cb,
                                                               this);

      sub_hp = nh.subscribe<furniture_classification::Hypothesis> ("/furniture_hypothesis" + n_str , 1, &FitModelsNode::cloud_hp,
                                                               this);

    }

    void cloud_cb(const typename pcl17::PointCloud<pcl17::PointXYZ>::ConstPtr& msg)
    {
      oc.setScene(msg);
    }

    void cloud_hp(const typename furniture_classification::Hypothesis::ConstPtr & msg)
    {
      if(oc.getScene() == NULL) return;
      std::cerr << "Started fitting" << std::endl;
      pcl17::PointCloud<pcl17::PointNormal>::Ptr res = oc.fit_objects(msg);
      res->header.frame_id = "/base_link";
      std::cerr << "Finished fitting with "<< res->points.size() << " points " << std::endl;
      pub.publish(res);

    }

    pcl17::PHVObjectClassifier<pcl17::PointXYZ, pcl17::PointNormal, FeatureType> oc;
    ros::Publisher pub;
    ros::Subscriber sub;
    ros::Subscriber sub_hp;

  };

int main(int argc, char **argv)
{

  if (argc < 5)
  {
    PCL17_INFO("Usage %s -database /path/to/database -features sgf | esf | vfh \n", argv[0]);
    return -1;
  }



  std::string database;
  std::string features = "sgf";
  int n=0;

  pcl17::console::parse_argument(argc, argv, "-database", database);
  pcl17::console::parse_argument(argc, argv, "-features", features);
  pcl17::console::parse_argument(argc, argv, "-n", n);

  ros::init(argc, argv, "model_fitter" + boost::lexical_cast<std::string>(n));


  if (features == "sgf")
  {
    FitModelsNode<pcl17::Histogram<pcl17::SGFALL_SIZE>, pcl17::SGFALLEstimation<pcl17::PointNormal, pcl17::Histogram<
        pcl17::SGFALL_SIZE> > > v(database, n);
    ros::spin();
  }
  else if (features == "esf")
  {
    FitModelsNode<pcl17::ESFSignature640, pcl17::ESFEstimation<pcl17::PointNormal, pcl17::ESFSignature640> >
                                                                                                             v(database,n);
    ros::spin();
  }
  else if (features == "vfh")
  {
    FitModelsNode<pcl17::VFHSignature308, pcl17::VFHEstimation<pcl17::PointNormal, pcl17::PointNormal,
        pcl17::VFHSignature308> > v(database,n);
    ros::spin();
  }
  else
  {
    std::cerr << "Unknown feature type " << features << " specified" << std::endl;
  }

  return 0;
}
