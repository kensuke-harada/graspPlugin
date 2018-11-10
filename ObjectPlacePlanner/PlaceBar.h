/**
   c) Kensuke Harada (AIST)
*/

#ifndef PLACEBAR_H
#define PLACEBAR_H

#include <cnoid/BodyItem>
#include <cnoid/ToolBar>

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <cnoid/SignalProxy>
#endif

#include "../Grasp/PlanBase.h"
#include "exportdef.h"
#include "PlacePlanner.h"
#include "ZmapInterface.h"

namespace cnoid {
    class MessageView;
}

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
class EXCADE_API PlaceBar : public cnoid::ToolBar, public boost::signals::trackable
#else
class EXCADE_API PlaceBar : public cnoid::ToolBar
#endif
{
public:

	static PlaceBar* instance();

	virtual ~PlaceBar();

protected:

	virtual bool storeState(cnoid::Archive& archive);
	virtual bool restoreState(const cnoid::Archive& archive);

private:

	PlaceBar();

	cnoid::Vector3 Po;
	cnoid::Matrix3 Ro;
	bool placePos;

	cnoid::MessageView& mes;
	std::ostream& os;

	void onStartButtonClicked();
};

#endif
