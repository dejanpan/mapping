/*
 * Based on the original code by Kai M. Wurm and Armin Hornung
 * (http://octomap.sourceforge.net)
 * Author: Hozefa Indorewala
 */

#include <bitset>
#include <cassert>

#include "pcl_to_octree/octree/OcTreeNodePCL.h"

namespace octomap 
{


OcTreeNodePCL::OcTreeNodePCL()
  : OcTreeNode() 
{
  setLabel("empty");
}

OcTreeNodePCL::~OcTreeNodePCL()
{
  // children are deleted in parent destructor
}


// ============================================================
// =  children          =======================================
// ============================================================

bool OcTreeNodePCL::createChild(unsigned int i) 
{
  if (itsChildren == NULL) 
  {
    allocChildren();
  }
  itsChildren[i] = new OcTreeNodePCL();
  return true;
}


OcTreeNodePCL* OcTreeNodePCL::getChild(unsigned int i) 
{
  assert((i < 8) && (itsChildren != NULL));
  assert(itsChildren[i] != NULL);
  return (OcTreeNodePCL*) itsChildren[i];
}

const OcTreeNodePCL* OcTreeNodePCL::getChild(unsigned int i) const
{
  assert((i < 8) && (itsChildren != NULL));
  assert(itsChildren[i] != NULL);
  return (const OcTreeNodePCL*) itsChildren[i];
}


// ============================================================
// =  data              =======================================
// ============================================================

void OcTreeNodePCL::setLabel(std::string l) 
{
  label = l;
}

std::string OcTreeNodePCL::getLabel() const
{
  return (label);
}

void OcTreeNodePCL::setCentroid(point3d c) 
{
  centroid = c;
}

point3d OcTreeNodePCL::getCentroid() const
{
  return (centroid);
}

void OcTreeNodePCL::setResolution(double res) 
{
  resolution = res;
}

double OcTreeNodePCL::getResolution() const
{
  return (resolution);
}

void OcTreeNodePCL::set3DPointInliers(int inlier_index) 
{
  inliers.push_back(inlier_index);
}

std::vector<int> OcTreeNodePCL::get3DPointInliers()
{
  return (inliers);
}

} // end namespace
