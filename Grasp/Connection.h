#ifndef _GRASP_CONNECTION_H_
#define _GRASP_CONNECTION_H_

#include <iostream>

#include <cnoid/BodyItem>
#include <cnoid/EigenTypes>

#include "exportdef.h"

namespace grasp {
	class ObjectBase;

	class EXCADE_API Connection {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		Connection();
		virtual ~Connection();

		enum ConnectionState {STATE_UNCONNECTED, STATE_UNDERCONNECT, STATE_CONNECTED};

		ObjectBase* const slaveObject() const;
		void setSlaveObject(ObjectBase* slave);

		cnoid::Link* const masterLink() const;
		void setMasterLink(cnoid::Link* link);

		void setRelativeT(const cnoid::Position& T);
		cnoid::Position& relativeT();
		const cnoid::Position& relativeT() const;
		cnoid::Position::LinearPart relativeRot();
		cnoid::Position::ConstLinearPart relativeRot() const;
		cnoid::Position::TranslationPart relativePos();
		cnoid::Position::ConstTranslationPart relativePos() const;

		bool isConnected() const;
		bool isUnconnected() const;
		bool isUnderconnect() const;

		void setConnected();
		void setUnconnected();
		void setUnderconnect();

		void setState(ConnectionState state);
		ConnectionState state() const;

	protected:
		ObjectBase* slave_object_;
		cnoid::Link* master_link_;
		cnoid::Position relative_pos_;
		ConnectionState state_;
	};

	std::ostream& operator<<(std::ostream& os, const grasp::Connection& con);
}

#endif /* _GRASP_CONNECTION_H_ */
