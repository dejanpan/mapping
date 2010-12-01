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
  setLabel(-1);
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

void OcTreeNodePCL::setLabel(int l) 
{
  label = l;
}

int OcTreeNodePCL::getLabel() const
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

// ============================================================
  // =  file I/O          =======================================
  // ============================================================

  std::istream& OcTreeNodePCL::readBinary(std::istream &s) {

    char child1to4_char;
    char child5to8_char;
    s.read((char*)&child1to4_char, sizeof(char));
    s.read((char*)&child5to8_char, sizeof(char));

    std::bitset<8> child1to4 ((unsigned long) child1to4_char);
    std::bitset<8> child5to8 ((unsigned long) child5to8_char);

    //     std::cout << "read:  "
    // 	      << child1to4.to_string<char,std::char_traits<char>,std::allocator<char> >() << " "
    // 	      << child5to8.to_string<char,std::char_traits<char>,std::allocator<char> >() << std::endl;


    // inner nodes default to occupied
    this->setLogOdds(CLAMPING_THRES_MAX);

    for (unsigned int i=0; i<4; i++) {
      if ((child1to4[i*2] == 1) && (child1to4[i*2+1] == 0)) {
        // child is free leaf
        createChild(i);
        getChild(i)->setLogOdds(CLAMPING_THRES_MIN);
      }
      else if ((child1to4[i*2] == 0) && (child1to4[i*2+1] == 1)) {
        // child is occupied leaf
        createChild(i);
        getChild(i)->setLogOdds(CLAMPING_THRES_MAX);
      }
      else if ((child1to4[i*2] == 1) && (child1to4[i*2+1] == 1)) {
        // child has children
        createChild(i);
        getChild(i)->setLogOdds(-200.); // child is unkown, we leave it uninitialized
      }
    }
    for (unsigned int i=0; i<4; i++) {
      if ((child5to8[i*2] == 1) && (child5to8[i*2+1] == 0)) {
        // child is free leaf
        createChild(i+4);
        getChild(i+4)->setLogOdds(CLAMPING_THRES_MIN);
      }
      else if ((child5to8[i*2] == 0) && (child5to8[i*2+1] == 1)) {
        // child is occupied leaf
        createChild(i+4);
        getChild(i+4)->setLogOdds(CLAMPING_THRES_MAX);
      }
      else if ((child5to8[i*2] == 1) && (child5to8[i*2+1] == 1)) {
        // child has children
        createChild(i+4);
        getChild(i+4)->setLogOdds(-200.); // set occupancy when all children have been read
      }
      // child is unkown, we leave it uninitialized
    }

    // read children's children and set the label
    for (unsigned int i=0; i<8; i++) {
      if (this->childExists(i)) {
        OcTreeNodePCL* child = this->getChild(i);
        if (fabs(child->getLogOdds()+200.)<1e-3) {
          child->readBinary(s);
          child->setLogOdds(child->getMaxChildLogOdds());
        }
      } // end if child exists
    } // end for children

    return s;
  }

  std::ostream& OcTreeNodePCL::writeBinary(std::ostream &s) const{

    // 2 bits for each children, 8 children per node -> 16 bits
    std::bitset<8> child1to4;
    std::bitset<8> child5to8;

    // 10 : child is free node
    // 01 : child is occupied node
    // 00 : child is unkown node
    // 11 : child has children


    // speedup: only set bits to 1, rest is init with 0 anyway,
    //          can be one logic expression per bit

    for (unsigned int i=0; i<4; i++) {
      if (childExists(i)) {
        const OcTreeNodePCL* child = this->getChild(i);
        if      (child->hasChildren())  { child1to4[i*2] = 1; child1to4[i*2+1] = 1; }
        else if (child->isOccupied())   { child1to4[i*2] = 0; child1to4[i*2+1] = 1; }
        else                            { child1to4[i*2] = 1; child1to4[i*2+1] = 0; }
      }
      else {
        child1to4[i*2] = 0; child1to4[i*2+1] = 0;
      }
    }

    for (unsigned int i=0; i<4; i++) {
      if (childExists(i+4)) {
        const OcTreeNodePCL* child = this->getChild(i+4);
        if      (child->hasChildren())  { child5to8[i*2] = 1; child5to8[i*2+1] = 1; }
        else if (child->isOccupied())   { child5to8[i*2] = 0; child5to8[i*2+1] = 1; }
        else                            { child5to8[i*2] = 1; child5to8[i*2+1] = 0; }
      }
      else {
        child5to8[i*2] = 0; child5to8[i*2+1] = 0;
      }
    }
    //     std::cout << "wrote: "
    // 	      << child1to4.to_string<char,std::char_traits<char>,std::allocator<char> >() << " "
    // 	      << child5to8.to_string<char,std::char_traits<char>,std::allocator<char> >() << std::endl;

    char child1to4_char = (char) child1to4.to_ulong();
    char child5to8_char = (char) child5to8.to_ulong();

    s.write((char*)&child1to4_char, sizeof(char));
    s.write((char*)&child5to8_char, sizeof(char));

    // write children's children
    for (unsigned int i=0; i<8; i++) {
      if (childExists(i)) {
        const OcTreeNodePCL* child = this->getChild(i);
        if (child->hasChildren()) {
          child->writeBinary(s);
        }
      }
    }

    return s;
  }


} // end namespace
