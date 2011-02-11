/*
 * Based on the original code by Kai M. Wurm and Armin Hornung
 * (http://octomap.sourceforge.net)
 * Author: Hozefa Indorewala
 */

#ifndef OCTREE_SERVER_PCL_H_
#define OCTREE_SERVER_PCL_H_

#include <octomap/octomap.h>
#include "OcTreePCL.h"
#include <octomap_ros/OctomapBinary.h>
#include <octomap_ros/GetOctomap.h>
#include <iostream>


namespace octomap_server{

	
        /*
        **
	 * Converts an octree structure to a ROS octree msg as binary data
	 *
	 * @param octomap input OcTree
	 * @param mapMsg output msg
	 */
	static inline void octomapMapToMsg(const octomap::OcTreePCL& octree, octomap_ros::OctomapBinary& mapMsg){
		// conversion via stringstream

		// TODO: read directly into buffer? see
		// http://stackoverflow.com/questions/132358/how-to-read-file-content-into-istringstream
		std::stringstream datastream;
		octree.writeBinaryConst(datastream);
		std::string datastring = datastream.str();
		mapMsg.header.stamp = ros::Time::now();
		mapMsg.data = std::vector<int8_t>(datastring.begin(), datastring.end());
	}
        
	/**
	 * Converts a ROS octree msg (binary data) to an octree structure
	 *
	 * @param mapMsg
	 * @param octomap
	 */
	static inline void octomapMsgToMap(const octomap_ros::OctomapBinary& mapMsg, octomap::OcTreePCL& octree){
		std::stringstream datastream;
		assert(mapMsg.data.size() > 0);
		datastream.write((const char*) &mapMsg.data[0], mapMsg.data.size());
		octree.readBinary(datastream);
	}
}

#endif /* OCTREE_SERVER_PCL_H_ */
