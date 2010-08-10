/*
 * Copyright (C) 2009 by Ulrich Friedrich Klank <klank@in.tum.de>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

volatile bool g_stopall = false;
 
#ifndef DXFWRTIER_H
#define DXFWRTIER_H
#include "File.h"
#define _USE_MATH_DEFINES
#include "math.h"

//for Mesh_t
#include "pcl_cloud_tools/misc.h"

#ifdef WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT
#endif
class dxfwriter :
public linfile::File
{
 public:
  static bool WriteMesh(Mesh_t mesh,  std::string FileName);
  EXPORT dxfwriter(std::string FileName);
  EXPORT ~dxfwriter(void);
  EXPORT void Write3dLINE(double startx, double starty, double startz, double endx, double endy, double endz);
  EXPORT void WriteBox(double offsetx, double offsety, double offsetz, double sizex, double sizey, double sizez);
  EXPORT void WriteSolid(double dx0, double dy0, double dz0,
			 double dx1, double dy1, double dz1,
			 double dx2, double dy2, double dz2,
			 double dx3, double dy3, double dz3);
  EXPORT void WriteSolid(double dx0, double dy0, double dz0,
			 double dx1, double dy1, double dz1,
			 double dx2, double dy2, double dz2);
  void WriteCylinder(double centerx0, double centery0, double centerz0,
		     double radius, double height, double angleX);
  EXPORT void WriteBoxFromPoints(double dx0, double dy0, double dz0,
				 double dx1, double dy1, double dz1,
				 double dx2, double dy2, double dz2,
				 double dx3, double dy3, double dz3,
				 double dx4, double dy4, double dz4,
				 double dx5, double dy5, double dz5,
				 double dx6, double dy6, double dz6,
				 double dx7, double dy7, double dz7);
 private:
  void WriteConstHeader();
  void WriteConstTail();
  void WriteGroup(std::string st, int value);
  void WriteGroup(std::string st, double value);
  void WriteGroup(int st, int value);
  void WriteGroup(int st, double value);
  void WriteGroupHex(int st, double value);
 private:
  int m_handle;
};
#endif//DXFWRTIER_H
