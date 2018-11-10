// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/**
   c) Kensuke Harada (AIST)
*/

#include "PlaceBar.h"
#include <cnoid/ItemTreeView>
#include <cnoid/MessageView>
#include <cnoid/Archive>
#include <boost/bind.hpp>
#include <boost/format.hpp>

#ifdef WIN32
#include <windows.h>
void usleep(int t){
	Sleep(t/1000);
}
#endif

//#define USE_ZMAP

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;
using namespace grasp::PickAndPlacePlanner;


PlaceBar* PlaceBar::instance()
{
	static PlaceBar* instance = new PlaceBar();
	return instance;
}

PlaceBar::PlaceBar()
	: ToolBar("PlaceBar"),
	  mes(*MessageView::mainInstance()),
	  os (MessageView::mainInstance()->cout() )
{

	addSeparator();

	addLabel(("=Place="));

	addButton(("Start"), ("Place Planning"))->
		sigClicked().connect(bind(&PlaceBar::onStartButtonClicked, this));

	addSeparator();

	placePos = true;
}

PlaceBar::~PlaceBar()
{
}

void PlaceBar::onStartButtonClicked()
{
#ifdef USE_ZMAP
	ZmapInterface::instance()->doReadFromFile();
#else
	PlanBase* tc = PlanBase::instance();
	if(placePos){
			BodyItemPtr envItem;
			for(list<BodyItemPtr>::iterator it = tc->bodyItemEnv.begin(); it != tc->bodyItemEnv.end(); ++it)
					if(tc->objPressName == (*it)->body()->name())
							envItem = *it;

            Vector3 pressPos = envItem->body()->link(0)->p() + envItem->body()->link(0)->attitude()*(tc->objPressPos);
			Matrix3 pressOri;
			vector<Vector3> Po_put;
			vector<Matrix3> Ro_put;
			PlacePlanner::instance()->calcPutPos(pressPos, pressOri, Po_put, Ro_put);

            Po = tc->targetObject->bodyItemObject->body()->link(0)->p();
			Ro = tc->targetObject->bodyItemObject->body()->link(0)->attitude();

			for(size_t i=0; i<Po_put.size(); i++){
                    tc->targetObject->bodyItemObject->body()->link(0)->p() = Po_put[i];
					tc->targetObject->bodyItemObject->body()->link(0)->setAttitude(Ro_put[i]);
					tc->targetObject->bodyItemObject->notifyKinematicStateChange();
					tc->flush();
					usleep(1000000);
			}
			placePos = false;
	}
	else{
            tc->targetObject->bodyItemObject->body()->link(0)->p() = Po;
			tc->targetObject->bodyItemObject->body()->link(0)->setAttitude(Ro);
			tc->targetObject->bodyItemObject->notifyKinematicStateChange();
			tc->flush();
			placePos = true;
	}

#endif

}

bool PlaceBar::storeState(Archive& archive)
{
	return true;
}

bool PlaceBar::restoreState(const Archive& archive)
{
	return true;
}
