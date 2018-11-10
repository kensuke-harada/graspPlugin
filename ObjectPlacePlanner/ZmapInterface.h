#ifndef _ZMAPINTERFACE_H
#define _ZMAPINTERFACE_H

#include <stdio.h>
#include <fstream>
#include <iostream>
#ifndef WIN32
#include <unistd.h>
#endif

#include <cnoid/BodyItem>
#include <cnoid/ItemTreeView>
#include <cnoid/RootItem>
#include <cnoid/MessageView>
#include <cnoid/EigenUtil>

#include "../Grasp/PlanBase.h"

namespace grasp{

class ZmapInterface {

	public :

	ZmapInterface();
	virtual ~ZmapInterface(){};

	static ZmapInterface* instance( ZmapInterface* ri=NULL )
	{
		static ZmapInterface* instance = (ri) ? ri : new ZmapInterface();
		if(ri) instance = ri;
		return instance;
	};
	virtual void doReadFromFile();

	protected :
	private :
};



}


#endif
