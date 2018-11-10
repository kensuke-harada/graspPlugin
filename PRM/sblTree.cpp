// ***************************************************************
// File sblTree.C
// Implements a data structure to store a tree
//
// Gildardo Sanchez-Ante
// mpkRobotics Laboratory, Stanford University
// ante@robotics.stanford.edu
// ***************************************************************

#include <stdlib.h>
#include <vector>

#include "mpk_rand.h"
#include "sblEdge.h"
#include "sblTree.h"

// ------------------------------
// Constructor & Destructor
// ------------------------------
sblTree::sblTree() {

  bins_ctr=0;
  nodes_on_tree = 0;

  setGridSize(GRID_DIVS,GRID_DIVS);
}

sblTree::~sblTree()
{
}

void
sblTree::setGridSize(int a, int b)
{
  assert(a <= GRID_DIVS);
  DIVS = a; // define the size of the grid
  DELTA = double(1.000001/DIVS);
}

// ----------------------------------------------------------
// getOnesblBin() : returns the index of one of the bins that
//               has nodes inside
// ----------------------------------------------------------
int
sblTree::choosesblBin()
{
  long rInteg = mpk_lrand(); // Get a random number
  int mysblBin = (rInteg % bins_ctr); // bounded to the number of filled bins

  return(filled_bins[mysblBin]); // return one of the bins that has nodes
}

// ---------------------------------------------------------
// hasData_in: Function that checks if the are nodes inside
//             of one of its bins
// ---------------------------------------------------------
bool
sblTree::hasNodesIn(int i)
{
  if(binList[i].getSize()== 0)
    return false;
  else
    return true;
}

double
sblTree::getRatio()
{
  return(bins_ctr/(double)nodes_on_tree);
}
// --------------------------------------------------------
// getNodeFrom: The purpose of this funcion is to get at random
//              one milestone from bin i
// --------------------------------------------------------
int
sblTree::getNodeFromsblBin(int i)
{
  int num = binList[i].getSize();
  if(num == 0)
    return -1;
  if(num == 1)
    return binList[i].data[0];

  // Get a random number between 0 and the number of nodes in the bin
  long rInteg = mpk_lrand();
  int idx = (rInteg % num);

  // get the index of the node that is on such position
  int node = binList[i].data[idx];
  return(node);
}

// ----------------------------------------------------------------------
// addTosblBins: Function used to approximately represent the distribution
//            of configurations over the entire space of the robot.
//            Only the two first coordinates are taken into account.
// ----------------------------------------------------------------------
void
sblTree::addTosblBins(const sblMilestone& m)
{
  const mpkConfig& q = m.getConfig();
  nodes_on_tree++; //one node more on the tree

  // bin_index goes from 0 to the number of bins(NR_BINS)
  long bin_index = long(floor(q[DOF0]/DELTA)+floor(q[DOF1]/DELTA)*DIVS);
  
  //Search for the bin on the list
  if  (binList[bin_index].getSize() > 0){
    binList[bin_index].add_node(m); // if it already exists, add the node
  }  
  else   // otherwise,
    {
      sblBin new_bin(m); // create a new bin
      binList[bin_index]= new_bin; // add it to the list of bins
      filled_bins[bins_ctr]= bin_index; //update the list of filled bins
      bins_ctr++; //update the counter of bins
    }
}

void
sblTree::delFromsblBins(const sblMilestone& m)
{
  const mpkConfig& q = m.getConfig();

  // the bin in which the node was counted
  long bin_index = long(floor(q[DOF0]/DELTA)+floor(q[DOF1]/DELTA)*DIVS);
  binList[bin_index].del_node(m);
  nodes_on_tree--;

  if (binList[bin_index].getSize()==0){ // the bin is empty now...
    for(int i =0; i<bins_ctr-1; i++){
      if(filled_bins[i] == bin_index){
	filled_bins[i] = filled_bins[bins_ctr-1]; //move the last one
	break;
      }
    }
    bins_ctr--;
  }
}

// ---------------------------------------------------------------
// chooseNode: select the node to be expanded
// ---------------------------------------------------------------
int
sblTree::getNodeToExpand()
{
  if(bins_ctr == 1){
    
    // If there is only one bin, choose it with prob. 1
    // bin_idx: index between 0 and NUM_BINS, used to access directly a bin
    int bin_idx = filled_bins[0]; //get the index of the bin
    sblBin binToPick = binList[bin_idx];
    long rndInt = mpk_lrand();

    // get a number between 0 and the number of nodes inside of the bin
    int take_node = (rndInt % (binToPick.getSize()));

    // Take node given by (take_node) on bin (bin_idx)
    int chosenNode = binToPick.data[take_node];
    return (chosenNode);
  }
  else{
    // Compute the probability of choosing the node
    double sum=0.0;
    double rndDoub;
    long intRan;
    int normalizer;

    normalizer = (bins_ctr-1)*(nodes_on_tree);
    rndDoub = mpk_drand();
    for(int i=0;i<bins_ctr;i++){
		int bin_idx = filled_bins[i];
		sblBin binToPick = binList[bin_idx];
		sum += double((nodes_on_tree - binToPick.getSize())/double(normalizer));
		if( rndDoub <= sum ){
			intRan=mpk_lrand();

			// Get a number (0<node_index<nodes on the bin)
			int take_node = (intRan % (binToPick.getSize()));

			// Get the milestone of the bin
			int chosenNode = binToPick.data[take_node];

			return (chosenNode);
		} 
	}     
	int bin_idx = filled_bins[bins_ctr-1];
	sblBin binToPick = binList[bin_idx];
	intRan = mpk_lrand();
	int take_node = (intRan % (binToPick.getSize()));
	// Get the milestone of the bin
	int chosenNode = binToPick.data[take_node];
	return (chosenNode);
  
    fprintf(stderr,"could not select bin!!!! %f %f \n", rndDoub, sum);
  }
  return 0;
}

int sblTree::getClosestNode(const sblMilestone& m, const vector<sblMilestone>& theNodes)
{
  double min_dist = DBL_MAX;
  int closest_node = -1;

  int bin_idx = getsblBinIndex(m);
  int this_node;

  sblBin this_bin = binList[bin_idx];
  if (this_bin.getSize()==0) return -1;

  for(int j =0;j<this_bin.getSize();j++){
    this_node = this_bin.data[j];
    sblMilestone node = theNodes[this_node];
    double d = m.distance(node);
    
    if(d<min_dist){
      min_dist = d;
      closest_node = this_node;
    }
  }
  return closest_node;
}

int sblTree::
getsblBinIndex(const sblMilestone& m)
{
  const mpkConfig& q = m.getConfig();
  return (int(floor(q[DOF0]/DELTA)+floor(q[DOF1]/DELTA)*DIVS));
}

void sblTree::printsblBins()
{
  for(int i=0;i<DIVS*DIVS;i++)
    printf(" %i ",binList[i].getSize());
}

void sblTree::rebuildTree(sblTree& T, const vector<sblMilestone>& theNodes)
{
  for(int i=0;i<bins_ctr;i++){ // for all the filled bins on the tree

    int binIdx = filled_bins[i]; // pick a bin with nodes inside

    for(int j=0;j<binList[binIdx].getSize();j++){ // for all nodes inside it
      int nodeIdx = binList[binIdx].getData(j); // get the node
      //cerr<<"Transferring nodes";
      T.addTosblBins(theNodes[nodeIdx]); // add it to the new tree
    }
  }
}
