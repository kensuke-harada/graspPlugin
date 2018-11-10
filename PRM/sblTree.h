#ifndef SBL_TREE_H
#define SBL_TREE_H

#include <vector>
#include <queue>

#include "mpkConfig.h"
#include "sblMilestone.h"
#include "sblBin.h"
#include "sblEdge.h"

#include "exportdef.h"


#define GRID_DIVS 10  // subdivision resolution of grid for binning

/**@memo Defines the tree structure for the sblPlanner roadmaps.  This
 class is responsible of maintain the record of all nodes added so far
 as well as to keep a representation of the sampled C-space in order
 to choose a node to expand according to the density of nodes.
*/
class sblTree {

public:
  
  ///
  sblTree();

  ~sblTree();

  /**@doc Sets the Grid size to which the tree nodes are to hashed*/
  void setGridSize(int a=10, int b=10);

  /**@doc Sets the 2 DOF's for sblBin Construction */
  void setDOFS(int dof0=0, int dof1=1)
    {DOF0=dof0; DOF1=dof1;};

  /**@doc Probabilistically chooses a node to expand from the Tree */
  int getNodeToExpand();

  /**@doc Adds a tree node to a Tree sblBin */
  void addTosblBins(const sblMilestone& m);

  /**@doc Delete a tree node from the tree sblBins */
  void delFromsblBins(const sblMilestone& m);

  /**@doc Picks a sblBin */
  int choosesblBin();

  /**@doc Checks if a node is in a particular sblBin or not */
  bool hasNodesIn(int bin);

  /**@doc Picks a Node from a bin randomly */
  int getNodeFromsblBin(int bin);

  /**@doc Returns the ratio of number of sblBins to the number of nodes in tree */
  double getRatio();

  /**@doc Finds the closest node in a sblBin to a sblMilestone */
  int getClosestNode(const sblMilestone& m, const vector<sblMilestone>& x);

  /**@doc Gets the sblBin id for a sblMilestone to which it would be hashed*/
  int getsblBinIndex(const sblMilestone& m);

  /**@doc Returns Tree size*/ 
  int getSize(){return nodes_on_tree;}
  
  /**@doc Prints the sizes of all the sblBins */
  void printsblBins();

  /**@doc Returns DELTA: grid precision */
  double getDelta(){return DELTA;}

  /**@doc Returns DIVS: number of grid divisions */
  int getDIVS(){return DIVS;}

  /**@doc Rebuilds a tree as per new grid parameters */
  void rebuildTree(sblTree& t, const vector<sblMilestone>& theNodes);

protected:

  /* Array for the bins */
  sblBin binList[(GRID_DIVS+1)*(GRID_DIVS+1)];

  /* For the valid bins */
  int filled_bins[GRID_DIVS*(GRID_DIVS+1)];     

  /* Tree size */
  int nodes_on_tree;

  /* Number of sblBins */
  int bins_ctr;

  /* Number of divisions to construct the bin*/
  int DIVS; 
  
  /* Number of bins*/
  int NR_BINS; 

  /* Size of sblBins */
  double DELTA; 

  /* DOFs with which the sblBins are constructed */ 
  int DOF0;
  int DOF1;
};


#endif
