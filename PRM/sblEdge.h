#ifndef SBL_EDGE_H
#define SBL_EDGE_H

using namespace std;

#include <math.h>

#ifdef ADAPT_COLLCHECKER
#include "mpkAdaptSegmentChecker.h"
#else
#include "mpkSimpleSegmentChecker.h"
#endif

#include "exportdef.h"

/**@memo Defines and implements the connection between two milestones
   on the tree. It is composed by information on both nodes (their
   indexes), the distance between them and the status on collision
   checking for the straight-line segment that joins both milestones
   on the C-space.  
*/

class sblEdge{
  
public:

  ///
  sblEdge(){ 
    from = 0;
    to = 0;
    lambda = 1000;
    seg_checker = 0;
  };
  
  /**@doc Constructor that recieves 2 ends of the sblEdge */
  sblEdge(int f, int t){
    from = f;
    to = t;
    lambda = 1000;
    seg_checker = 0;
  };

  /**@doc Constructor that recieves 2 ends of the sblEdge and extent of
     edge partition */
  sblEdge(int f, int t, double d){
    from = f;
    to = t;
    lambda = d;
    seg_checker = 0;
  };

  ~sblEdge() {
    if (seg_checker) delete seg_checker;
  };

  ///
  sblEdge(const sblEdge& e) {
    //this->dist_seg_checker = e.dist_seg_checker;
    this->from = e.from;
    this->to = e.to;
    this->lambda = e.lambda;
    seg_checker = 0;  
  }

  ///  
  sblEdge& operator=(const sblEdge& e) {
    if (&e!=this) {
      //this->dist_seg_checker = e.dist_seg_checker;
      this->from = e.from;
      this->to = e.to;
      this->lambda = e.lambda;
    }
    return *this;
  }
  
  /**@doc To set the start of the edge*/
  void setFrom(int f) {
    from =f;
  }

  /**@doc To set the end of the edge */
  void setTo(int t){
    to= t;
  }

  /**@doc To set the edge length based on partition*/
  void setDist(double d){
    lambda = d;
  }

  /**@doc To mark that the edge is collision free */
  void markSafe(){
    cerr << "marksafe not implemented! " << endl;
  }

  /**@doc To mark that the edge is in collision */
  void markBad(){
    cerr << "markbad not implemented! " << endl;
  }

  /**@doc To set a edge as per given parameters*/
  void setsblEdge(int f, int t, double d=0){
    from = f;  
    to = t;  
    lambda = d; 
  }

  /**@doc To retrieve the start of edge */
  int getFrom(){
    return from;
  };

  /**@doc To retrieve the end of edge */
  int getTo(){
    return to;
  };

  /**@doc To retrieve the edge-length based on its partioning */
  double getDist(){
    return lambda;
  };


  /**@doc To check if the segement is collision free or not */
  bool isSafe(){
    cerr << "issafe not implemented! " << endl;
    return true;
  };

  /**@doc To print the edge parameters */
  void print(){
    cerr<<"Link from "<<from<<" to  "<<to<<
      " distance: "<<lambda<< '\n';
  };

  friend class sblPrioritizeEdges;

  ///
#ifdef ADAPT_COLLCHECKER
  mpkAdaptSegmentChecker *seg_checker;
#else 
  mpkSimpleSegmentChecker *seg_checker;
#endif  

 protected:

  /** Reference of the node where the link starts */
  int        from; 
  // @doc Reference of the node where the link ends */
  int        to;
  /** distance between the two milestones */
  double     lambda; 

};

/**@memo Friend class of {@link sblEdge sblEdge} with operator for
    comparison of Objects. This class is used to compare two edges by
    their lengths.
*/

class sblPrioritizeEdges {
public :
  ///
  int operator()( const sblEdge *x, const sblEdge *y )
  {      
    return x->seg_checker->prio() < y->seg_checker->prio();
  }
};

#endif
