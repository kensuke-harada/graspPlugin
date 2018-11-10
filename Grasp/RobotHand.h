#ifndef _GRASP_ROBOTHAND_H_
#define _GRASP_ROBOTHAND_H_

#include <vector>
#include <string>

#include <cnoid/BodyItem>
#include <cnoid/Link>
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <cnoid/YamlReader>
#else
#include <cnoid/YAMLReader>
#endif
#include <cnoid/ItemList>

/* #include <boost/filesystem.hpp> */

#include "Finger.h"
#include "Arm.h"
#include "InterLink.h"
#include "PrehensionParameter.h"
#include "ObjectBase.h"
#include "exportdef.h"

namespace grasp {
	class ObjectManager;

	class EXCADE_API RobotHand :
		public ObjectBase {
	public:
		friend class ObjectManager;

		virtual ~RobotHand();
				
		HandID id;

		cnoid::BodyItemPtr bodyItem_;

		std::string dataFilePath;

		int nFing;
		int nHandLink;
		cnoid::Link* palm;
		FingerPtr *fingers;
		cnoid::LinkTraverse *handJoint;

		std::multimap<std::string, std::string> contactLinks;

		double mu;
		double fmax;
		double hmax;

		std::string handName;

		std::vector<InterLink> interLinkList;

		std::vector<std::string> prehensionFilePathList;
		std::vector<PrehensionPtr> prehensionList;

		Arm* const arm() const;
		void setArm(Arm* arm);

		void initialCollisionSelf();

	private:
		RobotHand();
		RobotHand(cnoid::BodyItemPtr bodyitem);

		ObjectBasePtr clone(ObjectManager* owner) const;

		void makeHand(cnoid::BodyItemPtr bodyitem);
		void removePrehension();

		Arm* connected_arm_;
	};
}

#endif
