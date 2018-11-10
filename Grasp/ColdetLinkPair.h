/*!
  Ported to Grasp @1.4-1.5
*/


#include <vector>

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)

#include <cnoid/ColdetLinkPair>
typedef std::vector<cnoid::ColdetLinkPairPtr> ColdetLinkPairVector;

#else

#ifndef CNOID_BODY_COLDET_LINK_PAIR_H_INCLUDED
#define CNOID_BODY_COLDET_LINK_PAIR_H_INCLUDED
#include <cnoid/Body>
#include <cnoid/Link>

#include "ColdetConverter.h"
#include "ColdetModelGetter.h"
#include "../../../src/AISTCollisionDetector/ColdetModelPair.h"

#include "exportdef.h"

namespace grasp{

class ColdetLinkPair : public cnoid::ColdetModelPair
{
public:
   ColdetLinkPair(const cnoid::BodyPtr& body1, cnoid::Link* link1, const cnoid::BodyPtr& body2, cnoid::Link* link2)
		: ColdetModelPair(ColdetModelGetter::get(link1), ColdetModelGetter::get(link2))
   {
       bodies[0] = body1;
       bodies[1] = body2;
       links[0] = link1;
       links[1] = link2;
			 model(0)->setName(link1->name());
			 model(1)->setName(link2->name());
   }
        
   ColdetLinkPair(const ColdetLinkPair& org)
    : ColdetModelPair(org)
   {
       bodies[0] = org.bodies[0];
       bodies[1] = org.bodies[1];
       links[0] = org.links[0];
       links[1] = org.links[1];
			 model(0)->setName(links[0]->name());
			 model(1)->setName(links[1]->name());
   }
       
   virtual ~ColdetLinkPair()
   {
   }
   
   void updatePositions()
   {
       model(0)->setPosition(links[0]->T());
       model(1)->setPosition(links[1]->T());
   }

   const cnoid::BodyPtr& body(int index) const { return bodies[index]; }
   cnoid::Link* link(int index) const { return links[index]; }
    
 protected:
   cnoid::BodyPtr bodies[2];
   cnoid::Link* links[2];
};

typedef boost::shared_ptr<ColdetLinkPair> ColdetLinkPairPtr;

}

namespace cnoid {
	typedef grasp::ColdetLinkPair ColdetLinkPair;
	typedef grasp::ColdetLinkPairPtr ColdetLinkPairPtr;
}

typedef std::vector<grasp::ColdetLinkPairPtr> ColdetLinkPairVector;

#endif
#endif
