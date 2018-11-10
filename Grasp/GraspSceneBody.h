/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#ifndef EXCADE_GRASP_SCENE_BODY_H_INCLUDED
#define EXCADE_GRASP_SCENE_BODY_H_INCLUDED

//#include <cnoid/SceneBody>	/* modified by qtconv.rb 0th rule*/  
#ifdef CNOID_ENABLE_OSG
#include <cnoid/OSGSceneBody>
#endif

#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/  

//#include <cnoid/SceneBodyManager>	/* modified by qtconv.rb 0th rule*/  
#ifdef CNOID_ENABLE_OSG
#include <cnoid/OSGSceneBodyManager>
#endif

#include "exportdef.h"


namespace grasp{

    class GraspSceneBody : public cnoid::OSGSceneBody
	{
	  public:
			cnoid::BodyItemPtr bodyItem;
			cnoid::BodyPtr body;
//			GraspController *gc;


		GraspSceneBody(cnoid::BodyItemPtr _bodyItem) : OSGSceneBody(_bodyItem) {
				bodyItem = _bodyItem;
				body =  bodyItem->body();
		}
        static cnoid::OSGSceneBody* create(cnoid::BodyItem* item);


			
	  protected:

		virtual ~GraspSceneBody(){ };

//	    virtual void onAttachedToScene();
//	    virtual void onDetachedFromScene();

//		virtual bool onKeyPressEvent(const cnoid::SceneViewEvent& event);
//	    virtual bool onKeyReleaseEvent(const SceneViewEvent& event);
        virtual bool onButtonPressEvent(const cnoid::OSGSceneViewEvent& event);
//	    virtual bool onButtonReleaseEvent(const SceneViewEvent& event);
//	    virtual bool onDoubleClickEvent(const SceneViewEvent& event);
//	    virtual bool onPointerMoveEvent(const SceneViewEvent& event);
//	    virtual void onPointerLeaveEvent(const SceneViewEvent& event);
//	    virtual void onContextMenuRequest(const SceneViewEvent& event, MenuManager& menuManager);
//	    virtual void onSceneModeChanged();
//	    virtual bool onUndoRequest();
//	    virtual bool onRedoRequest();
		
	  private:

	};
		
	typedef osg::ref_ptr<GraspSceneBody> GraspSceneBodyPtr;
	
	
}

	
#endif
