// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include <iostream>
#include "SoftFingerStabilityBar.h"
#include <cnoid/ItemTreeView>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/MessageView>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/Archive>	/* modified by qtconv.rb 0th rule*/  
#include <boost/bind.hpp>
#include <boost/format.hpp> 

#include "SoftFingerStabilityHandler.h"

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

SoftFingerStabilityBar* SoftFingerStabilityBar::instance()
{
	static SoftFingerStabilityBar* instance = new SoftFingerStabilityBar();
	return instance;
}

SoftFingerStabilityBar::SoftFingerStabilityBar()
	: ToolBar("SoftFingerStability"),
	  mes(*MessageView::mainInstance()),
   	os (MessageView::mainInstance()->cout() )
{
	
	/*addSeparator();
	
	addLabel(("=SoftFingerStability="));


	addSeparator(); */
}

SoftFingerStabilityBar::~SoftFingerStabilityBar()
{
	// do nothing
}

bool SoftFingerStabilityBar::storeState(Archive& archive)
{
	return true;
}


bool SoftFingerStabilityBar::restoreState(const Archive& archive)
{
	return true;
}
