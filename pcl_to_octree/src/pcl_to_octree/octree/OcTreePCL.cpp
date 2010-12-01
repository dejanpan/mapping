/*
 * Based on the original code by Kai M. Wurm and Armin Hornung
 * (http://octomap.sourceforge.net)
 * Author: Hozefa Indorewala
 */

#include <cassert>
#include <fstream>
#include <stdlib.h>

#include "pcl_to_octree/octree/OcTreePCL.h"
#include "octomap/CountingOcTree.h"


// switch to true to disable uniform sampling of scans (will "eat" the floor)
#define NO_UNIFORM_SAMPLING false


namespace octomap {

OcTreePCL::OcTreePCL(double _resolution)
  : OccupancyOcTreeBase<OcTreeNodePCL> (_resolution)  
{
  itsRoot = new OcTreeNodePCL();
  tree_size++;
}


void OcTreePCL::insertScan(const ScanNode& scan, double maxrange, bool pruning) {
  if (scan.scan->size()< 1)
    return;

  if (NO_UNIFORM_SAMPLING){
    std::cerr << "Warning: Uniform sampling of scan is disabled!\n";

    pose6d scan_pose (scan.pose);

    // integrate beams
    octomap::point3d origin (scan_pose.x(), scan_pose.y(), scan_pose.z());
    octomap::point3d p;

    for (octomap::Pointcloud::iterator point_it = scan.scan->begin(); 
         point_it != scan.scan->end(); point_it++) {
      p = scan_pose.transform(**point_it);
      this->insertRay(origin, p, maxrange);
    } // end for all points
  } 
  else {
    this->insertScanUniform(scan, maxrange);
  }

  if (pruning)
    this->prune();
}

void OcTreePCL::toMaxLikelihood() {

  // convert bottom up
  for (unsigned int depth=tree_depth; depth>0; depth--) {
    toMaxLikelihoodRecurs(this->itsRoot, 0, depth);
  }

  // convert root
  itsRoot->toMaxLikelihood();
}
  
  
// -- Information  ---------------------------------  

unsigned int OcTreePCL::memoryUsage() const{
  unsigned int node_size = sizeof(OcTreeNodePCL);
  std::list<OcTreeVolume> leafs;
  this->getLeafNodes(leafs);
  unsigned int inner_nodes = tree_size - leafs.size();
  return node_size * tree_size + inner_nodes * sizeof(OcTreeNodePCL*[8]);
}

  
void OcTreePCL::calcNumThresholdedNodes(unsigned int& num_thresholded, 
                                        unsigned int& num_other) const {
  num_thresholded = 0;
  num_other = 0;
  calcNumThresholdedNodesRecurs(itsRoot, num_thresholded, num_other);
}


void OcTreePCL::readBinary(const std::string& filename){
  std::ifstream binary_infile( filename.c_str(), std::ios_base::binary);
  if (!binary_infile.is_open()){
    std::cerr << "ERROR: Filestream to "<< filename << " not open, nothing read.\n";
    return;
  } else {
    readBinary(binary_infile);
    binary_infile.close();
  }
}


// -- I/O  -----------------------------------------

std::istream& OcTreePCL::readBinary(std::istream &s) {
    
  if (!s.good()){
    std::cerr << "Warning: Input filestream not \"good\" in OcTree::readBinary\n";
  }

  int tree_type = -1;
  s.read((char*)&tree_type, sizeof(tree_type));
  if (tree_type == OcTreePCL::TREETYPE){

    this->tree_size = 0;
    sizeChanged = true;

    // clear tree if there are nodes
    if (itsRoot->hasChildren()) {
      delete itsRoot;
      itsRoot = new OcTreeNodePCL();
    }

    double tree_resolution;
    s.read((char*)&tree_resolution, sizeof(tree_resolution));

    this->setResolution(tree_resolution);

    unsigned int tree_read_size = 0;
    s.read((char*)&tree_read_size, sizeof(tree_read_size));
    std::cout << "Reading "
              << tree_read_size
              << " nodes from bonsai tree file..." <<std::flush;

    itsRoot->readBinary(s);

    tree_size = calcNumNodes();  // compute number of nodes

    std::cout << " done.\n";
  } else if (tree_type == OcTreePCL::TREETYPE+1){
    this->read(s);
  } else{
    std::cerr << "Binary file does not contain an OcTree!\n";
  }

  return s;
}


void OcTreePCL::writeBinary(const std::string& filename){
  std::ofstream binary_outfile( filename.c_str(), std::ios_base::binary);

  if (!binary_outfile.is_open()){
    std::cerr << "ERROR: Filestream to "<< filename << " not open, nothing written.\n";
    return;
  } else {
    writeBinary(binary_outfile);
    binary_outfile.close();
  }
}

void OcTreePCL::writeBinaryConst(const std::string& filename) const{
  std::ofstream binary_outfile( filename.c_str(), std::ios_base::binary);

  if (!binary_outfile.is_open()){
    std::cerr << "ERROR: Filestream to "<< filename << " not open, nothing written.\n";
    return;
  } 
  else {
    writeBinaryConst(binary_outfile);
    binary_outfile.close();
  }
}

std::ostream& OcTreePCL::writeBinary(std::ostream &s){

  // format:    treetype | resolution | num nodes | [binary nodes]

  this->toMaxLikelihood();
  this->prune();

  return writeBinaryConst(s);
}

std::ostream& OcTreePCL::writeBinaryConst(std::ostream &s) const{

  // format:    treetype | resolution | num nodes | [binary nodes]

  unsigned int tree_type = OcTreePCL::TREETYPE;
  s.write((char*)&tree_type, sizeof(tree_type));

  double tree_resolution = resolution;
  s.write((char*)&tree_resolution, sizeof(tree_resolution));

  unsigned int tree_write_size = this->size(); 
  fprintf(stderr, "writing %d nodes to output stream...", tree_write_size); fflush(stderr);
  s.write((char*)&tree_write_size, sizeof(tree_write_size));

  itsRoot->writeBinary(s);

  fprintf(stderr, " done.\n");

  return s;
}


// --  protected  --------------------------------------------

void OcTreePCL::insertScanUniform(const ScanNode& scan, double maxrange) {
    
  octomap::pose6d  scan_pose (scan.pose);
  octomap::point3d origin (scan_pose.x(), scan_pose.y(), scan_pose.z());


  // preprocess data  --------------------------

  octomap::point3d p;

  CountingOcTree free_tree    (this->getResolution());
  CountingOcTree occupied_tree(this->getResolution());
  std::vector<point3d> ray;

  for (octomap::Pointcloud::iterator point_it = scan.scan->begin(); point_it != scan.scan->end(); point_it++) {

    p = scan_pose.transform(**point_it);

    bool is_maxrange = false;
    if ( (maxrange > 0.0) && ((p - origin).norm2() > maxrange) ) is_maxrange = true;

    if (!is_maxrange) {
      // free cells
      if (this->computeRay(origin, p, ray)){
        for(std::vector<point3d>::iterator it=ray.begin(); it != ray.end(); it++) {
          free_tree.updateNode(*it);
        }
      }
      // occupied cells
      occupied_tree.updateNode(p);
    } // end if NOT maxrange

    else {
      point3d direction = (p - origin).unit();
      point3d new_end = origin + direction * maxrange;
      if (this->computeRay(origin, new_end, ray)){
        for(std::vector<point3d>::iterator it=ray.begin(); it != ray.end(); it++) {
          free_tree.updateNode(*it);
        }
      }
    } // end if maxrange


  } // end for all points

  std::list<OcTreeVolume> free_cells;
  free_tree.getLeafNodes(free_cells);

  std::list<OcTreeVolume> occupied_cells;
  occupied_tree.getLeafNodes(occupied_cells);


  // delete free cells if cell is also measured occupied
  for (std::list<OcTreeVolume>::iterator cellit = free_cells.begin(); cellit != free_cells.end();){
    if ( occupied_tree.search(cellit->first) ) {
      cellit = free_cells.erase(cellit);
    }
    else {
      cellit++;
    }
  } // end for


    // insert data into tree  -----------------------
  for (std::list<OcTreeVolume>::iterator it = free_cells.begin(); it != free_cells.end(); it++) {
    updateNode(it->first, false);
  }
  for (std::list<OcTreeVolume>::iterator it = occupied_cells.begin(); it != occupied_cells.end(); it++) {
    updateNode(it->first, true);
  }

  //    unsigned int num_thres = 0;
  //    unsigned int num_other = 0;
  //    calcNumThresholdedNodes(num_thres, num_other);
  //    std::cout << "Inserted scan, total num of thresholded nodes: "<< num_thres << ", num of other nodes: "<< num_other << std::endl;

}
  
void OcTreePCL::toMaxLikelihoodRecurs(OcTreeNodePCL* node, unsigned int depth,
                                      unsigned int max_depth) {

  if (depth < max_depth) {
    for (unsigned int i=0; i<8; i++) {
      if (node->childExists(i)) {
        toMaxLikelihoodRecurs(node->getChild(i), depth+1, max_depth);
      }
    }
  }

  else { // max level reached
    node->toMaxLikelihood();
  }
}
  
void OcTreePCL::calcNumThresholdedNodesRecurs (OcTreeNodePCL* node,
                                               unsigned int& num_thresholded, 
                                               unsigned int& num_other) const { 
  assert(node != NULL);

  for (unsigned int i=0; i<8; i++) {
    if (node->childExists(i)) {
      OcTreeNodePCL* child_node = node->getChild(i);
      if (child_node->atThreshold()) num_thresholded++;
      else num_other++;
      calcNumThresholdedNodesRecurs(child_node, num_thresholded, num_other);
    } // end if child
  } // end for children
}


void OcTreePCL::insertOcTreeNodePCL(octomap::OcTreeNodePCL * ocn)
{
  this->octree_node_list.push_back(ocn);
}

octomap::OcTreeNodePCL * OcTreePCL::getOcTreeNodePCL(point3d c) const
{
  
  for (int i = 0; i < this->octree_node_list.size(); i++)
  {
//     std::cerr << "cx: " << this->octree_node_list[i]->getCentroid().x() << " " << c.x() << std::endl; 
//     std::cerr << "cy: " << this->octree_node_list[i]->getCentroid().y() << " " << c.y() << std::endl; 
//     std::cerr << "cz: " << this->octree_node_list[i]->getCentroid().z() << " " << c.z() << std::endl; 
    if (this->octree_node_list[i]->getCentroid().x() == c.x() &&  
        this->octree_node_list[i]->getCentroid().y() == c.y() &&
        this->octree_node_list[i]->getCentroid().z() == c.z())
      return this->octree_node_list[i];
  }
  return NULL;
}

} // namespace
