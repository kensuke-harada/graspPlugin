/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#ifndef _FINGER_H
#define _FINGER_H

#include <stdlib.h>
#include <time.h>
#include <cnoid/BodyItem>
#include <cnoid/JointPath>

#include "ColdetLinkPair.h"
#include "exportdef.h"

namespace grasp{


class Finger;
typedef Finger* FingerPtr;

class Finger{
    public:
        Finger(cnoid::BodyPtr body, cnoid::Link *palm, cnoid::Link *tip);
        virtual ~Finger();
        int number;
        std::vector<bool>contact;
        std::vector <int>compLink;
        std::vector <double>close;
        cnoid::JointPathPtr fing_path;
        cnoid::Link *tip;
        cnoid::BodyPtr body;
        int nJoints;
        double offset;

        std::vector<double> fingerOpenPose, fingerGraspPose;
        std::vector<double> fingerOpenPoseOffset;
        std::vector<double> fingerCloseOffset;

        void coldetLinkPair(cnoid::BodyItemPtr bo);

        //virtual bool IK_fingContact_Envelope(int lk, int& iter, const cnoid::Vector3 &p);

        virtual bool closeFinger(int lk, int iter, cnoid::Vector3 &oPos, cnoid::Vector3 &objN, cnoid::Vector3& fingerN);
        virtual bool contactSearch(int&cnt, int iter, cnoid::Vector3* oPos, cnoid::Vector3* objN, cnoid::Vector3* fingerN=NULL);
        virtual bool fingtipGrasp(void);
        virtual bool checkJointLimit(cnoid::Link* joint);
        virtual bool checkFingLimit();
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
        cnoid::ColdetLinkPairPtr *linkObjPair;
#else
        grasp::ColdetLinkPairPtr *linkObjPair;
#endif


        cnoid::Link* joint(int i){
            return fing_path->joint(i);
        }
//		std::ostream& os;
 private:
				// Disallow copying to prevent double free.
				// If copying is required, implement copy constructor and assignment operator that do deep copy.
				Finger(const Finger& fing) {;}
				Finger& operator= (const Finger& fing) {;}
};

}

#endif
