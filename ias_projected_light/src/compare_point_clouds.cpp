/* 
 * Copyright (c) 2010, Florian Zacherl <Florian.Zacherl1860@mytum.de>, Dejan Pangercic <dejan.pangercic@cs.tum.edu>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Intelligent Autonomous Systems Group/
 *       Technische Universitaet Muenchen nor the names of its contributors 
 *       may be used to endorse or promote products derived from this software 
 *       without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv/cv.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <dirent.h>
#include <sys/stat.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

//#define OUTPUT_RAW; //If defined, just the pure values without any description are returned

using namespace std;

class PCLCompare
{
protected:
  pcl::PointCloud<pcl::PointXYZ> model;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  const static float tol = 0.008;

  int output;

  int number;

public:
  int compare(int compare_mode)
  {
    switch (compare_mode)
    {
      //Just returns the size of the point cloud
      case 0:
      {
        int r = cloud.points.size();
        return r;
      }
        break;

        //Returns the number of points that are equal with a given tolerance
        //Therefore the point clouds have to be in the exactly same coordinate frame!!
      case 1:
      {
        int point_count = 0;
        for (unsigned int i = 0; i < cloud.points.size(); i++)
        {
          for (unsigned int j = 0; j < model.points.size(); j++)
          {
            if (abs(model.points[j].x - cloud.points[i].x) < tol && abs(model.points[j].y - cloud.points[i].y) < tol
                && abs(model.points[j].z - cloud.points[i].z) < tol)
            {
              point_count++;
              break;
            }
          }
        }
        return point_count;
      }
        break;

        //Returns the ICP-fitness score:
      case 2:
      {
        pcl::IterativeClosestPoint < pcl::PointXYZ, pcl::PointXYZ > icp;
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud));
        icp.setInputTarget(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(model));
        pcl::PointCloud < pcl::PointXYZ > res;
        icp.align(res);
        return 10e6 * icp.getFitnessScore(numeric_limits<double>::max());
      }
        break;

      default:
        ROS_INFO("Invalid compare mode!");
	return -1;
    }
  }

  void getFiles(string path)
  {
    vector<int> val[3];
    DIR *pDIR;
    struct dirent *entry;
    struct stat s;

    if ((pDIR = opendir(path.c_str()))) //Given folder was found
    {
      while ((entry = readdir(pDIR))) //Read all entries in the folder
      {
        stringstream stst;
        stst << path << "/" << entry->d_name;
        stat(stst.str().c_str(), &s);
        if ((strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0)) //Ignore . and ..
        {
          if (S_ISDIR(s.st_mode)) //If entry is a folder:
          {
            cout << stst.str() << ":" << endl; //Return folder name
            getFiles(stst.str()); //Recursive call
          }
          else //If entry is no folder
          {
            string file(entry->d_name);
            if (strcmp(file.substr(file.size() - 3, 3).c_str(), "pcd") == 0) //Use just .pcd files
            {
              //Read point cloud:
              sensor_msgs::PointCloud2 cloud_mess1;
              if (pcl::io::loadPCDFile(stst.str(), cloud_mess1) == -1)
              {
                ROS_ERROR("Couldn't read file!");
                return;
              }
              pcl::fromROSMsg(cloud_mess1, cloud);
              number++;

              //Save values for all three compare modes:
              for (int i = 0; i < 3; i++)
              {
                val[i].push_back(compare(i));
              }
            }
          }
        }
      }

      //Compute average value and standard deviation:
      if (val[0].size() > 0)
      {
        int avg[3] = {0, 0, 0};
        int sd[3] = {0, 0, 0};
        for (int i = 0; i < 3; i++)
        {
          int sum = 0;
          for (unsigned int j = 0; j < val[0].size(); j++)
          {
            sum += val[i][j];
          }
          avg[i] = sum / val[0].size();

          for (unsigned int j = 0; j < val[0].size(); j++)
          {
            sd[i] += (avg[i] - val[i][j]) * (avg[i] - val[i][j]);
          }
          sd[i] = sqrt(sd[i] / val[0].size());
        }

#ifdef OUTPUT_RAW
        if (output == 0 || output == 3)
        cout << avg[0] << endl;
        if (output == 1 || output == 3)
        cout << avg[1] << endl;
        if (output == 2 || output == 3)
        cout << avg[2] << endl;
#else
        //Output:
        if (output == 0 || output == 3)
          cout << "Average number of points: " << avg[0] << ", Standard deviation: " << sd[0] << endl;
        if (output == 1 || output == 3)
          cout << "Average matching points: " << avg[1] << "(" << (int)((float)avg[1] / avg[0] * 100) << "%)"
              << ", Standard deviation: " << sd[1] << endl;
        if (output == 2 || output == 3)
          cout << "Average ICP-Value: " << avg[2] << ", Standard deviation: " << sd[2] << "\n" << endl;
#endif
      }
    }
    else
    {
      ROS_ERROR("Couldn't find folder %s!", path.c_str());
      return;
    }
  }

  PCLCompare(pcl::PointCloud<pcl::PointXYZ>& m, int o) :
    model(m), output(o), number(0)
  {

  }
};

int main(int argc, char** argv)
{
  string model_path;
  if (argc < 4)
  {
    cout << "Usage " << argv[0] << " [Path to model] [Point cloud folder] [Compare mode]" << endl;
    return -1;
  }

  sensor_msgs::PointCloud2 cloud_mess2;
  pcl::PointCloud < pcl::PointXYZ > cloud2;

  if (pcl::io::loadPCDFile(argv[1], cloud_mess2) == -1)
  {
    ROS_ERROR("Couldn't read file %s!", argv[1]);

    return (-1);
  }

  pcl::fromROSMsg(cloud_mess2, cloud2);
  PCLCompare t(cloud2, atoi(argv[3]));

  t.getFiles(argv[2]);

}

