/*
 * parameter_server.cpp
 *
 */
#include "RGBD_registration/parameter_server.h"

using namespace std;

ParameterServer* ParameterServer::_instance = NULL;

ParameterServer::ParameterServer() {
  std::stringstream node_pre;
  node_pre << "/" << ros::this_node::getName() << "/config/";
  pre = node_pre.str();

    defaultConfig();
    getValues();
}

ParameterServer* ParameterServer::instance() {
    if (_instance == NULL) {
        _instance = new ParameterServer();
    }
    //_instance->getValues();
    return _instance;
}

//        Default config for Frame Alignment
// #####Please see launch/RGBD_registration.launch for a description of these parameters ######
// it is recommended to change the parameters in the launchfile rather than the source!
void ParameterServer::defaultConfig() {

    config["source_cloud_filename"]           = std::string("../pcds/node_0.pcd");
    config["target_cloud_filename"]           = std::string("../pcds/node_4.pcd");

    // ----------RGB feature extraction------------
    config["feature_extractor"]               = std::string("SURF");
    config["feature_descriptor"]              = std::string("SURF");
    config["descriptor_matcher"]              = std::string("FLANN");
	  config["minimum_inliers"]                 = static_cast<int> (50);
    config["max_dist_for_inliers"]            = static_cast<double> (0.03);
    config["ransac_iterations"]               = static_cast<int> (1000);
    config["save_features_image"]             = static_cast<bool> (false);
    config["show_feature_matching"]           = static_cast<bool> (false);
    config["save_all_pointclouds"]            = static_cast<bool> (true);

    // --------------ICP settings------------------
    config["use_openmp_normal_calculation"]   = static_cast<bool> (true);
    config["alpha"]                           = static_cast<double> (0.5);
    config["max_correspondence_dist"]         = static_cast<double> (0.05);
    config["max_iterations"]                  = static_cast<int> (75);
    config["transformation_epsilon"]          = static_cast<double> (1e-8);
    config["euclidean_fitness_epsilon"]       = static_cast<double> (0);
    config["use_ransac_to_initialize_icp"]    = static_cast<bool> (false);
    config["enable_pcl_debug_verbosity"]      = static_cast<bool> (true);
}

void ParameterServer::getValues() {
    map<string, boost::any>::const_iterator itr;
    for (itr = config.begin(); itr != config.end(); ++itr) {
        string name = itr->first;
        if (itr->second.type() == typeid(string)) {
            config[name] = getFromParameterServer<string> (pre + name,
                    boost::any_cast<string>(itr->second));
            ROS_DEBUG_STREAM("Value for " << name << ":             " << boost::any_cast<string>(itr->second));
        } else if (itr->second.type() == typeid(int)) {
            config[name] = getFromParameterServer<int> (pre + name,
                    boost::any_cast<int>(itr->second));
            ROS_DEBUG_STREAM("Value for " << name << ":             " << boost::any_cast<int>(itr->second));
        } else if (itr->second.type() == typeid(double)) {
            config[name] = getFromParameterServer<double> (pre + name,
                    boost::any_cast<double>(itr->second));
            ROS_DEBUG_STREAM("Value for " << name << ":             " << boost::any_cast<double>(itr->second));
        } else if (itr->second.type() == typeid(bool)) {
            config[name] = getFromParameterServer<bool> (pre + name,
                    boost::any_cast<bool>(itr->second));
            ROS_DEBUG_STREAM("Value for " << name << ":             " << boost::any_cast<bool>(itr->second));
        }
    }
}
