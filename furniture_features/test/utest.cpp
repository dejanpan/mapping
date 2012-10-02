/*
 * utest.cpp
 *
 *  Created on: May 5, 2012
 *      Author: vsu
 */

/*
 * test.cpp
 *
 *  Created on: May 5, 2012
 *      Author: vsu
 */

#include <pcl17/features/sgfall.h>
#include <gtest/gtest.h>

TEST(Feature1NumberOfBoundaryPoints, SquareTest)
{

	int square_size = 5;
	pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud(
			new pcl17::PointCloud<pcl17::PointXYZ>);

	for (int i = 0; i < square_size; i++) {
		for (int j = 0; j < square_size; j++) {
			pcl17::PointXYZ point;
			point.x = i;
			point.y = j;
			point.z = 1.0f;
			cloud->points.push_back(point);
		}
	}

	cloud->width = cloud->points.size();
	cloud->height = 1;
	cloud->is_dense = true;

	boost::shared_ptr<std::vector<int> > indicesptr(new std::vector<int>());

	for (size_t i = 0; i < cloud->points.size(); i++) {
		(*indicesptr).push_back(i);
	}

	pcl17::search::KdTree<pcl17::PointXYZ>::Ptr tree(new pcl17::search::KdTree<
			pcl17::PointXYZ>());
	pcl17::PointCloud<pcl17::Histogram<pcl17::SGF1_SIZE> >::Ptr sgf1s(
			new pcl17::PointCloud<pcl17::Histogram<pcl17::SGF1_SIZE> >());
	pcl17::SGF1Estimation<pcl17::PointXYZ, pcl17::Histogram<pcl17::SGF1_SIZE> > sgf1;
	sgf1.setInputCloud(cloud);
	sgf1.setIndices(indicesptr);
	sgf1.setSearchMethod(tree);
	sgf1.setKSearch(10);
	sgf1.compute(*sgf1s);

	EXPECT_FLOAT_EQ(16.0/25.0, sgf1s->points[0].histogram[0]);
}

int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
