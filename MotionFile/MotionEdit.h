#ifndef GRASP_MOTIONEDIT_H_INCLUDED
#define GRASP_MOTIONEDIT_H_INCLUDED

#include <cnoid/View>

namespace grasp {

    class MotionEdit : public cnoid::View
    {
    public:
#if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)
	static void initializeClass(cnoid::ExtensionManager* ext);
#endif
	MotionEdit();
	~MotionEdit();
    };
}

#endif
