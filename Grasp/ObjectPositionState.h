#ifndef _GRASP_OBJECTPOSITIONSTATE_H_
#define _GRASP_OBJECTPOSITIONSTATE_H_

#include <vector>
#include <map>
#include <iostream>

#include <cnoid/EigenTypes>

#include <boost/variant.hpp>

#include "Connection.h"

#include "exportdef.h"

namespace grasp {
	typedef size_t ObjectDescriptor;
	class ObjectBase;
	class ObjectManager;

	class EXCADE_API ObjectPositionState {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		ObjectPositionState();
		virtual ~ObjectPositionState();

		static void storeState(const ObjectBase* object, ObjectPositionState& os_in);
		static void restoreState(ObjectBase* object, const ObjectPositionState& os_out);

		cnoid::VectorXd& jointSeq();
		const cnoid::VectorXd& jointSeq() const;

		double& q(int id);
	  double q(int id) const;

		cnoid::Position& T();
		const cnoid::Position& T() const;

		cnoid::Position::TranslationPart p();
		cnoid::Position::ConstTranslationPart p() const;

		cnoid::Position::LinearPart R();
		cnoid::Position::ConstLinearPart R() const;

	protected:
		cnoid::VectorXd joint_seq_;
		cnoid::Position root_p_;
	};

	class EXCADE_API ObjectsState {
		friend class ObjectManager;
		friend std::ostream& operator<<(std::ostream& os, const grasp::ObjectsState& oss);
	public:
		typedef boost::variant<int, double, std::vector<int>, std::vector<double> > ExtraData;

		struct ObjectState {
			ObjectPositionState obj_pos_state;
			std::vector<Connection> descendant_connections;
			std::vector<ExtraData> extra_state_data;
		};

		ObjectsState();
		virtual ~ObjectsState();

		ObjectState& objectState(ObjectDescriptor des);
		bool hasStateData(ObjectDescriptor des) const;

	protected:
		std::map<ObjectDescriptor, ObjectState> states_;
	};

	EXCADE_API std::ostream& operator<<(std::ostream& os, const grasp::ObjectPositionState& ops);
	EXCADE_API std::ostream& operator<<(std::ostream& os, const grasp::ObjectsState::ObjectState& oss);
	//EXCADE_API std::ostream& operator<<(std::ostream& os, const grasp::ObjectsState& oss);
}

#endif /* _GRASP_OBJECTPOSITIONSTATE_H_ */
