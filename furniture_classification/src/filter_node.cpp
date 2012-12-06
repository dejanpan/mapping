#include <ros/ros.h>
#include <pcl17/point_types.h>
#include <pcl17_ros/point_cloud.h>
#include <pcl17/console/print.h>
#include <pcl17/console/parse.h>
#include <furniture_classification/FittedModels.h>
#include <vector>

using namespace std;

class FilterNode {
public:
	FilterNode(int n) {

		this->n = n;

		ros::NodeHandle nh;
		pub = nh.advertise<pcl17::PointCloud<pcl17::PointNormal> >(
				"/fitted_models", 1);

		for (int i = 0; i < n; i++) {
			std::string i_str = boost::lexical_cast<std::string>(i);
			sub.push_back(
					nh.subscribe<furniture_classification::FittedModels>(
							"/fitted_models" + i_str, 1, &FilterNode::models_cb,
							this));
		}
	}

	void models_cb(const furniture_classification::FittedModelsConstPtr & msg) {
		fitted.push_back(msg);

		if (fitted.size() == n) {

			pcl17::PointCloud<pcl17::PointNormal>::Ptr res(new pcl17::PointCloud<pcl17::PointNormal>);
			res->header.frame_id = msg->frame_id;

			std::map<std::string,
					std::vector<pcl17::PointCloud<pcl17::PointNormal>::Ptr> > pmap;
			std::map<std::string, std::vector<float> > smap;

			for (int i = 0; i < n; i++) {
				for (int j = 0; j < fitted[i]->classes.size(); j++) {
					pcl17::PointCloud<pcl17::PointNormal>::Ptr p(
							new pcl17::PointCloud<pcl17::PointNormal>);
					pcl17::fromROSMsg(fitted[i]->models[j], *p);
					pmap[fitted[i]->classes[j]].push_back(p);
					smap[fitted[i]->classes[j]].push_back(fitted[i]->scores[j]);
				}
			}

			for (std::map<std::string,
					std::vector<pcl17::PointCloud<pcl17::PointNormal>::Ptr> >::iterator it =
					pmap.begin(); it != pmap.end(); it++) {

				vector<pcl17::PointCloud<pcl17::PointNormal>::Ptr> result_vector;

				result_vector = removeIntersecting(it->second, smap[it->first]);

				for (int i = 0; i < result_vector.size(); i++) {
					*res += *result_vector[i];
				}

			}

			pub.publish(res);
			fitted.clear();

		}

	}

	bool intersectXY(const pcl17::PointCloud<pcl17::PointNormal> & cloud1,
			const pcl17::PointCloud<pcl17::PointNormal> & cloud2) {

		pcl17::PointNormal min1, max1, min2, max2;
		pcl17::getMinMax3D<pcl17::PointNormal>(cloud1, min1, max1);
		pcl17::getMinMax3D<pcl17::PointNormal>(cloud2, min2, max2);

		bool intersectX, intersectY;
		if (min1.x < min2.x)
			intersectX = max1.x > min2.x;
		else if (min1.x > min2.x)
			intersectX = max2.x > min1.x;
		else
			// min1.x == min2.x
			intersectX = true;

		if (min1.y < min2.y)
			intersectY = max1.y > min2.y;
		else if (min1.y > min2.y)
			intersectY = max2.y > min1.y;
		else
			// min1.y == min2.y
			intersectY = true;

		return intersectX && intersectY;

	}

	vector<typename pcl17::PointCloud<pcl17::PointNormal>::Ptr> removeIntersecting(
			vector<typename pcl17::PointCloud<pcl17::PointNormal>::Ptr> & result_,
			vector<float> & scores_, vector<float> * selected_scores = NULL) {

		vector<pcl17::PointCloud<pcl17::PointNormal>::Ptr> no_intersect_result;

		if (result_.size() == 0)
			return no_intersect_result;

		for (size_t i = 0; i < result_.size(); i++) {
			bool best = true;
			for (size_t j = 0; j < result_.size(); j++) {
				if (intersectXY(*result_[i], *result_[j])) {
					if (scores_[i] > scores_[j])
						best = false;
				}

			}
			if (best) {
				if (selected_scores) {
					selected_scores->push_back(scores_[i]);
				}
				no_intersect_result.push_back(result_[i]);
			}
		}

		return no_intersect_result;

	}

	int n;
	ros::Publisher pub;
	std::vector<ros::Subscriber> sub;
	std::vector<furniture_classification::FittedModelsConstPtr> fitted;

};

int main(int argc, char **argv) {

	int n = 0;

	pcl17::console::parse_argument(argc, argv, "-n", n);

	ros::init(argc, argv, "filter_node");
	FilterNode fn(n);
	ros::spin();

	return 0;
}
