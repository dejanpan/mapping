#ifndef OCTOMAP_OCTREE_NODE_PCL_H
#define OCTOMAP_OCTREE_NODE_PCL_H

/*
* Based on orignal code writtien by K. M. Wurm, A. Hornung, University of Freiburg, Copyright (C) 2009.
* @author: Hozefa Indorewala 
*/

#include "octomap/OcTreeNode.h"

namespace octomap {


  /**
   *   Node class storing a label as additional information
   */
  class OcTreeNodePCL : public OcTreeNode {

  public:

    std::string label;
    point3d centroid;

  public:

    OcTreeNodePCL();
    virtual ~OcTreeNodePCL();


    // -- children  ----------------------------------

    virtual bool createChild(unsigned int i);
    virtual OcTreeNodePCL* getChild(unsigned int i);
    virtual const OcTreeNodePCL* getChild(unsigned int i) const;
 
    
    // data  -----------------------------------------

    /**
     * set a label
     */
    void setLabel(std::string l);
    /**
     * @return Label of node
     */
    std::string getLabel() const;


    /**
     * set a centroid
     */
    void setCentroid(point3d c);
    /**
     * @return centroid of node
     */
    point3d getCentroid() const;

  };



} // end namespace



#endif
