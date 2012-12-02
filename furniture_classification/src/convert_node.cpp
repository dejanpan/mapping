

#include <ros/ros.h>
#include <pcl17/ModelCoefficients.h>
#include <pcl17/io/pcd_io.h>
#include <pcl17/point_types.h>
#include <pcl17/sample_consensus/method_types.h>
#include <pcl17/sample_consensus/model_types.h>
#include <pcl17/segmentation/sac_segmentation.h>
#include <pcl17/filters/extract_indices.h>
#include <pcl17/common/transforms.h>
#include <pcl17_ros/point_cloud.h>

#include <pcl17/console/parse.h>

class ConvertNode {
public:
	ConvertNode(int tilt) {

		this->tilt = tilt;
		ros::NodeHandle nh;

		pub = nh.advertise<pcl17::PointCloud<pcl17::PointXYZRGB> >("/cloud_pcd",
				1);
		sub = nh.subscribe<pcl17::PointCloud<pcl17::PointXYZRGB> >(
				"/camera/depth_registered/points", 1, &ConvertNode::cloud_cb, this);

	}

	void cloud_cb(
			const typename pcl17::PointCloud<pcl17::PointXYZRGB>::ConstPtr& cloud) {

		pcl17::PointCloud<pcl17::PointXYZRGB> cloud_transformed,
				cloud_aligned, cloud_filtered;

		Eigen::Affine3f view_transform;
		view_transform.matrix() << 0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1;

		Eigen::AngleAxis<float> rot(tilt * M_PI / 180,
				Eigen::Vector3f(0, 1, 0));

		view_transform.prerotate(rot);

		pcl17::transformPointCloud(*cloud, cloud_transformed, view_transform);

		pcl17::ModelCoefficients::Ptr coefficients(
				new pcl17::ModelCoefficients);
		pcl17::PointIndices::Ptr inliers(new pcl17::PointIndices);

		pcl17::SACSegmentation<pcl17::PointXYZRGB> seg;

		seg.setOptimizeCoefficients(true);

		seg.setModelType(pcl17::SACMODEL_PLANE);
		seg.setMethodType(pcl17::SAC_RANSAC);
		seg.setDistanceThreshold(0.05);
		seg.setProbability(0.99);

		seg.setInputCloud(cloud_transformed.makeShared());
		seg.segment(*inliers, *coefficients);

		pcl17::ExtractIndices<pcl17::PointXYZRGB> extract;

		extract.setInputCloud(cloud_transformed.makeShared());
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(cloud_transformed);

		std::cout << "Z vector: " << coefficients->values[0] << " "
				<< coefficients->values[1] << " " << coefficients->values[2]
				<< " " << coefficients->values[3] << std::endl;

		Eigen::Vector3f z_current(coefficients->values[0],
				coefficients->values[1], coefficients->values[2]);
		Eigen::Vector3f y(0, 1, 0);

		Eigen::Affine3f rotation;
		rotation = pcl17::getTransFromUnitVectorsZY(z_current, y);
		rotation.translate(Eigen::Vector3f(0, 0, coefficients->values[3]));

		pcl17::transformPointCloud(cloud_transformed, cloud_aligned, rotation);

		Eigen::Affine3f res = (rotation * view_transform);

		cloud_aligned.sensor_origin_ = res * Eigen::Vector4f(0, 0, 0, 1);
		cloud_aligned.sensor_orientation_ = res.rotate(
				Eigen::AngleAxisf(M_PI, Eigen::Vector3f(0, 0, 1))).rotation();

		seg.setInputCloud(cloud_aligned.makeShared());
		seg.segment(*inliers, *coefficients);

		std::cout << "Z vector: " << coefficients->values[0] << " "
				<< coefficients->values[1] << " " << coefficients->values[2]
				<< " " << coefficients->values[3] << std::endl;

		pub.publish(cloud_aligned);

	}

	ros::Publisher pub;
	ros::Subscriber sub;
	int tilt;

};

int main(int argc, char **argv) {

	int tilt = 30;

	pcl17::console::parse_argument(argc, argv, "-tilt", tilt);

	ros::init(argc, argv, "convert_node");
	ConvertNode cn(tilt);
	ros::spin();

	return 0;
}
