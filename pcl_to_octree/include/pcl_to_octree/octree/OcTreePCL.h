#ifndef OCTOMAP_PCL_H
#define OCTOMAP_PCL_H
#include "octomap/OccupancyOcTreeBase.h"
#include "OcTreeNodePCL.h"
#include "octomap/ScanGraph.h"

namespace octomap {

  /**
   * octomap main map data structure, stores 3D occupancy grid map in an OcTree.
   * Basic functionality is implemented in OcTreeBase.
   *
   */
  class OcTreePCL : public OccupancyOcTreeBase <OcTreeNodePCL> {

  public:
    static const int TREETYPE=3;

  public:

    /**
     * Creates a new (empty) OcTree of a given resolution
     * @param _resolution
     */
    OcTreePCL(double _resolution);

   /**
     * Insert a 3d scan (given as a ScanNode) into the tree.
     * By default, the tree is pruned after insertion
     * (small run-time overhead, but decreases size)
     *
     * @param scan
     * @param maxrange maximum range for how long individual beams are inserted (default -1: complete beam)
     * @param pruning whether the tree is (losslessly) pruned after insertion (default: true)
     */
    void insertScan(const ScanNode& scan, double maxrange=-1., bool pruning = true);
    // -- Information  ---------------------------------

    /// \return Memory usage of the OcTree in bytes.
    unsigned int memoryUsage() const;

    void calcNumThresholdedNodes(unsigned int& num_thresholded, unsigned int& num_other) const; 


    
    /// Creates the maximum likelihood map by calling toMaxLikelihood on all
    /// tree nodes, setting their occupancy to the corresponding occupancy thresholds.
    void toMaxLikelihood();
    
     // -- I/O  -----------------------------------------

    /// binary file format: treetype | resolution | num nodes | [binary nodes]

    /// Reads an OcTree from an input stream. Existing nodes are deleted.
    std::istream& readBinary(std::istream &s);

    /// Writes OcTree to a binary stream.
    /// The OcTree is first converted to the maximum likelihood estimate and pruned
    /// for maximum compression.
    std::ostream& writeBinary(std::ostream &s);

    /// Writes the maximum likelihood OcTree to a binary stream (const variant).
    /// Files will be smaller when the tree is pruned first.
    std::ostream& writeBinaryConst(std::ostream &s) const;


    /// Reads OcTree from a binary file. Existing nodes are deleted.
    void readBinary(const std::string& filename);

    /// Writes OcTree to a binary file using writeBinary().
    /// The OcTree is first converted to the maximum likelihood estimate and pruned.
    void writeBinary(const std::string& filename);

    /// Writes OcTree to a binary file using writeBinaryConst().
    /// The OcTree is not changed, in particular not pruned first.
    void writeBinaryConst(const std::string& filename) const;


  protected:

    /// Helper for insertScan (internal use)
    void insertScanUniform(const ScanNode& scan, double maxrange=-1.);

    ///recursive call of toMaxLikelihood()
    void toMaxLikelihoodRecurs(OcTreeNodePCL* node, unsigned int depth, unsigned int max_depth);

    void calcNumThresholdedNodesRecurs (OcTreeNodePCL* node,
                                        unsigned int& num_thresholded, 
                                        unsigned int& num_other) const;
 
  };


} // end namespace

#endif
