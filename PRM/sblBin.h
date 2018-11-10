#ifndef SBL_BIN_H
#define SBL_BIN_H

#include <iostream>
#include <vector>

#include "sblMilestone.h"

#include "exportdef.h"


/**@memo Class sblBin is used to keep record on the distribution of the
   nodes on C-space.  Each bin containts an identifier, given by i, a
   counter where the number of nodes in the bin is kept (count), and a
   space to store a certain number of nodes per bin. There are also
   some auxiliary functions.
*/

class sblBin {
  
 public:
  
  ///
  sblBin() {}

  ~sblBin() {}

  ///
  sblBin(const sblMilestone& m)
    {
      // Add the node index at the first position
      data.push_back(m.getIndex());
   }
  
  /**@doc For adding a milestone in the bin */
  void add_node(const sblMilestone& node)
    { 
      data.push_back(node.getIndex());
    }
  
  /**@doc For deleting a node from the bin */
  void del_node(const sblMilestone& node)
    { 
      vector<int>::iterator I;
      for(I=data.begin(); I!=data.end(); ++I)
	if((*I)== node.getIndex())
	  break;  
      data.erase(I);
    }
  
  /**@doc Returns the size of the bin */
  int getSize()
    {
      return data.size();
    }

  /**@doc Retrieves an entry */
  int getData(int i)
    { 
      return data[i];
    }

  /**@doc Prints the bin list */
  void print()
    {
      vector<int>::iterator I;
      for(I=data.begin(); I!=data.end(); ++I)
	cerr << (*I) << ", ";
      cerr << endl;
    }
 
  ///@memo To store the indexes of the nodes on the bin
  vector<int> data;
};
#endif











