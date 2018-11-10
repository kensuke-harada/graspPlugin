/* **********************************************************
   File sblMilestone.h
   It defines and implements the basic unit of the PRM, a
   sblMilestone. In this case it is composed by the configuration of
   the robot, the link to its father and their childs, as well as
   a unique identifier for each of them

   Gildardo Sanchez-Ante
   mpkRobotics Lab., Stanford University
*************************************************************/
#ifndef SBL_MILESTONE_H
#define SBL_MILESTONE_H

#include <iostream>
#include <stdio.h>
#include <vector>

#include "mpkConfig.h"
#include "sblEdge.h"

#ifdef ADAPT_COLLCHECKER
#include "mpkConfigChecker.h"
#endif

#include "exportdef.h"


/**@memo A sblMilestone represents the data structure for storing a free
   sample. The SBL sblPlanner grows two trees (one from the start and one
   form the goal) consisting of these collision-free 'sblMilestones' as
   nodes. Each sblMilestone stores in itself links to its parent node and
   children nodes.
*/

class sblMilestone {

public:

  ///
  sblMilestone();

  /**@doc Function setRoot on the planner is the one that uses this */
  sblMilestone(int inx, const mpkConfig& qq, double tim=0);

  sblMilestone(const sblMilestone& m);
  sblMilestone& operator=(const sblMilestone& m);

  ~sblMilestone() {
# ifdef ADAPT_COLLCHECKER
    if(!adapt_point_checker) delete adapt_point_checker;
#endif
    delete q;
  }


  /**@doc To set the serial number of this sblMilestone */
  void setIndex(const int& index){i=index;}

  /**@doc To set the Configuration of the sblMilestone */
  void setConfig(mpkConfig& q_n) {*q = q_n;};
  void setTime(const double& time) {t=time;}

  /**@doc This function is used to set that the current milestone is the
      father of the milestone child. */
  void isFatherOf(sblMilestone& child);

  /**@doc Links to the parent node in the tree */
  void setFather(int from, int to) { edgeFromFather.setsblEdge(from, to, 0); }

  /**@doc To store the index of the Children Nodes */
  void addChild(const sblMilestone& m) { children.push_back(m.getIndex()); }

  /**@doc Returns the Configuration of the sblMilestone */
  mpkConfig& getConfig() { return *q; }
  const mpkConfig& getConfig() const { return *q; }

  /**@doc To set the serial number of this sblMilestone */
  int getIndex()const {return i;}
  double getTime() const { return t;}

  /**@doc Returns the Father of this node in the tree */
  int getFather() {return edgeFromFather.getFrom();}

  /**@doc Returns the edge from its father */
  sblEdge& getsblEdge(){return edgeFromFather;}
  const sblEdge& getsblEdge() const {return edgeFromFather;}

  /**@doc Returns the number of its children in the Tree */
  int getNrSons()const{return children.size();}

  /**@doc Returns the 'kth' children */
  int getChild(int k){return children[k];}

  /**@doc Marks that the edge to its father is collision free */
  void markSafe(){edgeFromFather.markSafe();}

  void removeChild(const sblMilestone& m);

  /**@doc To check if the two sblMilestones are equal or not */
  bool isEqual(const sblMilestone& m) {return (*q)==(*m.q);};

  /**@doc Prints the Configuration and the parent information */
  void print();

  /**@doc Prints the list of Children */
  void printChilds();

  /**@doc Returns Euclidean Distance */
  double distance(const sblMilestone& m) const {return q->dist(*m.q);}

  /**@doc Returns L-infinity distance */
  double L_inf(const sblMilestone& m) const {return q->Linf_dist(*m.q);}

  /**@memo link to the father of the node */
  sblEdge      edgeFromFather;

  /**@memo parameters used in Visual PRM planner identifying each milestone is Guard or Connector*/
  bool isGuard, isConnector, visited;

#ifdef ADAPT_COLLCHECKER
  mpkConfigChecker *adapt_point_checker;
#endif

 protected:

  /**@memo an index to identify the nodes */
  int       i;

  /**@memo Configuration */
  mpkConfig* q;

  /**@memo time associated to the configuration */
  double    t;

  /**@memo to store who are the children of the node */
  vector<int> children;

};


/* Default Constructor */
inline
sblMilestone::
sblMilestone(){
  i = 0; t = 0;
  edgeFromFather.setsblEdge(0,0,0);
  q = new mpkConfig(0);
  isGuard=false; isConnector=false;

#ifdef ADAPT_COLLCHECKER
  adapt_point_checker = 0;
#endif

}

/* Function setRoot on the planner is the one that uses this */
inline
sblMilestone::
sblMilestone(int inx, const mpkConfig& qq, double tim) {
  i = inx; t=tim;
  edgeFromFather.setsblEdge(-999, inx);
  q = new mpkConfig(qq);
  isGuard=false; isConnector=false;

#ifdef ADAPT_COLLCHECKER
  adapt_point_checker = 0;
#endif

}


inline
sblMilestone::
sblMilestone(const sblMilestone& m) {
  this->edgeFromFather = m.edgeFromFather;
  this->i = m.i;
  this->q = new mpkConfig(*m.q);
  this->t = m.t;
  this->children = m.children;
  this->isGuard = m.isGuard;
  this->isConnector = m.isConnector;
#ifdef ADAPT_COLLCHECKER
  this->adapt_point_checker = m.adapt_point_checker;
#endif

}

inline
sblMilestone&
sblMilestone::
operator=(const sblMilestone& m) {
  if (&m!=this) {
    this->edgeFromFather = m.edgeFromFather;
    this->i = m.i;
    *q = *m.q;
    this->t = m.t;
    this->children = m.children;
    this->isGuard = m.isGuard;
    this->isConnector = m.isConnector;
#ifdef ADAPT_COLLCHECKER
    this->adapt_point_checker = m.adapt_point_checker;
#endif

  }
  return *this;
}

inline
void
sblMilestone::
isFatherOf(sblMilestone& child) {
  child.edgeFromFather.setsblEdge(getIndex(),child.getIndex(),distance(child));
  addChild(child);
}

inline
void
sblMilestone::
removeChild(const sblMilestone& m){
  vector<int>::iterator I;
  for(I= children.begin();I!=children.end();++I)
    if((*I)== m.getIndex()){
      children.erase(I);
      break;
    }
}


/* Prints the Configuration and the parent information */
inline
void
sblMilestone::
print() {
  printf("%i   ",i);
  q->print();
  cerr<<"This Node has "<<edgeFromFather.getFrom()<<" as father"<<endl;
  edgeFromFather.print();cerr<<"\n\n";
}

/* Prints the list of Children */
inline
void
sblMilestone::
printChilds(){
  vector<int>::iterator I;
  cerr<<"Sons of: "<<getIndex()<<"  ";
  for(I=children.begin();I!=children.end();++I)
    cerr<<(*I)<<", ";
    cerr<<endl;
}


#endif









