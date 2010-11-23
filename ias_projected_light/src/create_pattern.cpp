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

#include <opencv/cv.h>
#include "opencv/highgui.h"
#include <iostream>
#include <fstream>
#include <math.h>
#include <ros/ros.h>
#include <dirent.h>
#include "ias_projected_light/cp.h"
//#include <gdk/gdk.h>


using namespace std;

class CreatePattern
{

private:

  int pwidth;
  int pheight;
  int awidth;
  int aheight;
  IplImage *pattern;
  int block_size; //The size of the smallest block in the pattern
  const static int num_patterns = 5; //number of patterns that are not created from files

  int pattern_index, pattern_count;
  vector<string> filepaths;
  bool service;

  ros::NodeHandle n;
  ros::ServiceServer serv;

  void setBlock(int posx, int posy, int col)
  {
    for (int x = 0; x < block_size; x++)
    {
      for (int y = 0; y < block_size; y++)
      {
        int ypix = posy * block_size + y;
        int xpix = posx * block_size + x;
        if (xpix < pwidth && ypix < pheight)
        {
          cvSet2D(pattern, ypix, xpix, cvScalar(col));
        }
      }
    }
  }


  void drawPattern()
  {
    switch (pattern_index)
    {
      case 0: //No (black) pattern
        for (int x = 0; x < ceil((float)pwidth / block_size); x++)
        {
          for (int y = 0; y < ceil((float)pheight / block_size); y++)
          {
            setBlock(x, y, 0);
          }
        }
        break;

      case 1: //White image
        for (int x = 0; x < ceil((float)pwidth / block_size); x++)
        {
          for (int y = 0; y < ceil((float)pheight / block_size); y++)
          {
            setBlock(x, y, 255);
          }
        }
        break;

      case 2: // Block pattern
        for (int x = 0; x < ceil((float)pwidth / block_size); x++)
        {
          for (int y = 0; y < ceil((float)pheight / block_size); y++)
          {
            if (x % 2 == y % 2)
            {
              setBlock(x, y, 255);
            }
            else
            {
              setBlock(x, y, 0);
            }
          }
        }
        break;

      case 3: //Stripe pattern
        for (int x = 0; x < ceil((float)pwidth / block_size); x++)
        {
          for (int y = 0; y < ceil((float)pheight / block_size); y++)
          {
            if (y % 2)
            {
              setBlock(x, y, 255);
            }
            else
            {
              setBlock(x, y, 0);
            }
          }
        }
        break;

      case 4: // Random pattern
        for (int x = 0; x < ceil((float)pwidth / block_size); x++)
        {
          for (int y = 0; y < ceil((float)pheight / block_size); y++)
          {
            if (rand() < RAND_MAX / 2)
            {
              setBlock(x, y, 0);
            }
            else
            {
              setBlock(x, y, 255);
            }

          }
        }
        break;

      default: //Pattern from textfile

        ifstream i(filepaths[pattern_index - num_patterns].c_str());
        if (!i)
        {
          ROS_INFO("Cannot open file.\n");
          return;
        }

        string line;

        int row = 0;

        while (!i.eof())
        {
          getline(i, line);
          if (line[0] == '0' || line[0] == '1') //Pattern data
          {
            for (int i = 0; i < aheight; i++)
            {
              for (int k = 0; k < ceil((float)pheight / aheight / block_size); k++)
              {
                for (int g = 0; g < ceil((float)pwidth / awidth / block_size); g++)
                {

                  int posx = row + g * awidth;
                  int posy = k * aheight + i;
                  setBlock(posx, posy, (line[i] == '0' ? 0 : 255));
                }
              }
            }
            row++;
          }
          else //Pattern header (get size of pattern, ignore the rest of the header)
          {
            if (line.substr(0, 11) == "Array size:")
            {
              unsigned int pos = 12;
              string n = "";
              while (line[pos] != ' ')
              {
                n += line[pos];
                pos++;
              }
              awidth = atoi(n.c_str());
              pos += 3;
              n = "";
              while (pos < line.size())
              {
                n += line[pos];
                pos++;
              }
              aheight = atoi(n.c_str());
              //cout << "Array size: " << awidth << " x " << aheight << endl;
            }
          }
        }

    }

    cvShowImage("Window", pattern);

  }

  string pattern_name(int index)
  {
    switch (pattern_index)
    {
      case 0:
        return "black";
        break;
      case 1:
        return "white";
        break;
      case 2:
        return "block";
        break;
      case 3:
        return "stripe";
        break;
      case 4:
        return "random";
        break;
      default:
        return filepaths[pattern_index - num_patterns].c_str();
        break;
    }
  }

public:

  bool change_pattern(ias_projected_light::cp::Request &req, ias_projected_light::cp::Response &res)
  {
    if (req.pattern < pattern_count)
    {
      pattern_index = req.pattern;
      block_size = req.block_size;
      res.name = pattern_name(pattern_index);

      drawPattern();
      cvWaitKey(100);
    }
    else
    {
      ROS_WARN("Wrong pattern number!");
      return false;
    }
    return true;
  }

  CreatePattern(ros::NodeHandle& n, int psize, char* p, bool s) :
    n(n), block_size(psize), pattern_index(0), service(s)
  {
    string path(p);

    pwidth = 1024;
    pheight = 768;
    //block_size = psize;

    DIR *pDIR;
    struct dirent *entry;
    if ((pDIR = opendir(path.c_str())))
    {
      while ((entry = readdir(pDIR)))
      {
        string file(entry->d_name);
        if (file.size() > 3 && strcmp(file.substr(file.size() - 3, 3).c_str(), "txt") == 0)
        {
          stringstream ss;
          ss << path << "/" << file;
          filepaths.push_back(ss.str());
          ROS_INFO("%s",ss.str().c_str());
        }
      }
    }
    else
    {
      ROS_ERROR("Couldn't find folder %s!", path.c_str());
      return;
    }
    pattern = cvCreateImage(cvSize(pwidth, pheight), IPL_DEPTH_8U, 1);

    pattern_count = num_patterns + filepaths.size();

    cvNamedWindow("Window");
    /*GdkWindow *w = gtk_widget_get_parent_window(cvGetWindowHandle("Window"));
    gdk_window_fullscreen(w);*/


    drawPattern();

    if (service)
    {
      serv = n.advertiseService("change_pattern", &CreatePattern::change_pattern, this);
      cvWaitKey(100);
    }
    else
    {
      while (true)
      {
        char c = cvWaitKey(0);
        switch (c)
        {
          case '+':
            block_size++;
            ROS_INFO("Block size: %d", block_size);
            break;
          case '-':
            if (block_size > 1)
              block_size--;
            ROS_INFO("Block size: %d", block_size);
            break;
          case 'p':
            pattern_index = (pattern_index + 1) % pattern_count;
            ROS_INFO("Switch to %s pattern", pattern_name(pattern_index).c_str());
            break;

            break;
          case 'q':
            return;
            break;

          case 'h':
            ROS_INFO("\nPress \"p\" to switch through patterns and \"+\" or \"-\" to increment or decrement the block size by one.");

        }
        drawPattern();
      }
    }

  }
};

int main(int argc, char** argv)
{
  if (argc < 3)
  {
    ROS_INFO("Usage: %s <int: initial block size> <string: text file patterns folder> <bool: call pattern creator as ros service or change patterns with keybord>?", argv[0]);
    return 0;
  }

  ros::init(argc, argv, "create_pattern");
  ros::NodeHandle nh;
  bool s = false;
  if (argc > 3)
  {
    s = atoi(argv[3]);
  }
  CreatePattern cp(nh, atoi(argv[1]), argv[2], s);

  if(s) ros::spin();
}
