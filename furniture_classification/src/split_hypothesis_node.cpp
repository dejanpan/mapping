#include <ros/ros.h>
#include <pcl17/console/print.h>
#include <pcl17/console/parse.h>
#include <furniture_classification/Hypothesis.h>

class SplitHypothesis {
public:
	SplitHypothesis(int num_splits) {
		ros::NodeHandle nh;

		for(int i=0; i<num_splits; i++){
			pubs.push_back(nh.advertise<furniture_classification::Hypothesis>("/furniture_hypothesis" + boost::lexical_cast<std::string>(i), 1));
		}

		sub_hp = nh.subscribe < furniture_classification::Hypothesis
				> ("/furniture_hypothesis", 1, &SplitHypothesis::cloud_hp, this);

	}

	void cloud_hp(
			const typename furniture_classification::Hypothesis::ConstPtr & msg) {


		int num_points_in_message = msg->classes.size()/pubs.size();

		for(int i=0; i<pubs.size()-1; i++){
			furniture_classification::Hypothesis::Ptr h(new furniture_classification::Hypothesis);

			h->classes.insert(h->classes.begin(), msg->classes.begin()+i*num_points_in_message, msg->classes.begin()+(i+1)*num_points_in_message);
			h->poses.insert(h->poses.begin(), msg->poses.begin()+i*num_points_in_message, msg->poses.begin()+(i+1)*num_points_in_message);
			pubs[i].publish(h);
		}

		furniture_classification::Hypothesis::Ptr h(new furniture_classification::Hypothesis);

		h->classes.insert(h->classes.begin(), msg->classes.end()-num_points_in_message, msg->classes.end());
		h->poses.insert(h->poses.begin(), msg->poses.end()-num_points_in_message, msg->poses.end());
		pubs[pubs.size()-1].publish(h);

	}

	std::vector<ros::Publisher> pubs;
	ros::Subscriber sub_hp;

};

int main(int argc, char **argv) {

	if (argc < 3) {
		PCL17_INFO(
				"Usage %s -num_splits N \n", argv[0]);
		return -1;
	}

	ros::init(argc, argv, "split_hypothesis");

	int num_splits = 2;
	pcl17::console::parse_argument(argc, argv, "-num_splits", num_splits);

	SplitHypothesis sh(num_splits);
	ros::spin();

	return 0;
}
