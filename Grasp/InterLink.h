

#ifndef _INTERLINK_H
#define _INTERLINK_H

namespace grasp{

class InterLink{
public:
	cnoid::Link *slave;
	cnoid::Link *master;
	int slaveId;
	int masterId;
	double ratio;
};

}

#endif
