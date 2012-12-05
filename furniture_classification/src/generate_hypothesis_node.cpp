#include <ros/ros.h>
#include <pcl17_ros/point_cloud.h>
#include <pcl17/console/print.h>
#include <pcl17/classification/PHVObjectClassifier.h>
#include <pcl17/features/sgfall.h>
#include <pcl17/console/parse.h>
#include <pcl17/features/vfh.h>
#include <pcl17/features/esf.h>

template<class FeatureType, class FeatureEstimatorType>
  class HypothesisGeneratorNode
  {
  public:
    HypothesisGeneratorNode(std::string database)
    {
      std::string database_dir = database;
      std::string debug_folder = "debug_classification/";

      typename pcl17::Feature<pcl17::PointNormal, FeatureType>::Ptr feature_estimator(new FeatureEstimatorType);
      oc.setFeatureEstimator(feature_estimator);

      oc.setDatabaseDir(database_dir);
      oc.loadFromFile();

      //oc.setDebugFolder(debug_folder);
      oc.setDebug(false);

      ros::NodeHandle nh;
      pub = nh.advertise<furniture_classification::Hypothesis> ("/furniture_hypothesis", 1);
      sub = nh.subscribe<pcl17::PointCloud<pcl17::PointXYZ> > ("/cloud_pcd", 1, &HypothesisGeneratorNode::cloud_cb,
                                                               this);

      std::map<std::string, vector<pcl17::PointCloud<pcl17::PointNormal>::Ptr> >::iterator iter;
      for (iter = oc.class_name_to_full_models_map_.begin(); iter != oc.class_name_to_full_models_map_.end(); iter++)
      {
        votes_publisher[iter->first] = nh.advertise<pcl17::PointCloud<pcl17::PointXYZ> > ("/" + iter->first + "_votes",
                                                                                          1);
      }

    }

    void cloud_cb(const typename pcl17::PointCloud<pcl17::PointXYZ>::ConstPtr& msg)
    {
      oc.setScene(msg);
      //std::cout << "Recieved pointcloud" << std::endl;
      std::map<std::string, pcl17::PointCloud<pcl17::PointXYZ>::Ptr> votes_map;
      pub.publish(oc.generate_hypothesis(votes_map));

      std::map<std::string, pcl17::PointCloud<pcl17::PointXYZ>::Ptr>::iterator iter;
      for (iter = votes_map.begin(); iter != votes_map.end(); iter++)
      {
    	  iter->second->header.frame_id = msg->header.frame_id;
        votes_publisher[iter->first].publish(iter->second);
      }

    }

    pcl17::PHVObjectClassifier<pcl17::PointXYZ, pcl17::PointNormal, FeatureType> oc;
    ros::Publisher pub;
    ros::Subscriber sub;
    std::map<std::string, ros::Publisher> votes_publisher;

  };

int main(int argc, char **argv)
{

  if (argc < 5)
  {
    PCL17_INFO("Usage %s -database /path/to/database -features sgf | esf | vfh \n", argv[0]);
    return -1;
  }

  ros::init(argc, argv, "hypothesis_generator");

  std::string database;
  std::string features = "sgf";

  pcl17::console::parse_argument(argc, argv, "-database", database);
  pcl17::console::parse_argument(argc, argv, "-features", features);

  if (features == "sgf")
  {
    HypothesisGeneratorNode<pcl17::Histogram<pcl17::SGFALL_SIZE>, pcl17::SGFALLEstimation<pcl17::PointNormal,
        pcl17::Histogram<pcl17::SGFALL_SIZE> > > v(database);
    ros::spin();
  }
  else if (features == "esf")
  {
    HypothesisGeneratorNode<pcl17::ESFSignature640, pcl17::ESFEstimation<pcl17::PointNormal, pcl17::ESFSignature640> >
                                                                                                                       v(
                                                                                                                         database);
    ros::spin();
  }
  else if (features == "vfh")
  {
    HypothesisGeneratorNode<pcl17::VFHSignature308, pcl17::VFHEstimation<pcl17::PointNormal, pcl17::PointNormal,
        pcl17::VFHSignature308> > v(database);
    ros::spin();
  }
  else
  {
    std::cerr << "Unknown feature type " << features << " specified" << std::endl;
  }

  return 0;
}
