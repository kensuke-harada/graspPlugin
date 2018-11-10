#ifndef __GraspBodyItem_H__
#define __GraspBodyItem_H__

#include <boost/dynamic_bitset.hpp>

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <src/BodyPlugin/SceneBody.h>
#else
#include "../../../src/BodyPlugin/EditableSceneBody.h"
#include <boost/function.hpp>
#endif

#include <cnoid/BodyItem>

#include "exportdef.h"


class EXCADE_API PlanBase;

namespace grasp{

class GraspBodyItem;
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
typedef boost::intrusive_ptr<GraspBodyItem> GraspBodyItemPtr;
#else
typedef cnoid::ref_ptr<GraspBodyItem> GraspBodyItemPtr;
#endif

class ExtSceneBody;

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
class EXCADE_API ExtSceneBody : public cnoid::SceneBody
#else
class EXCADE_API ExtSceneBody : public cnoid::EditableSceneBody
#endif
{
	public:
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		ExtSceneBody(cnoid::BodyItemPtr bodyItem) : SceneBody(bodyItem){ }
		static SceneBody* create(cnoid::BodyItemPtr bodyItem){
			return new ExtSceneBody(bodyItem) ;
		}
#else
		ExtSceneBody(cnoid::BodyItemPtr bodyItem) : EditableSceneBody(bodyItem){ }
		static EditableSceneBody* create(cnoid::BodyItemPtr bodyItem){
			return new ExtSceneBody(bodyItem) ;
		}
#endif
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		virtual bool onKeyPressEvent(const cnoid::SceneViewEvent& event){
			cnoid::MessageView::mainInstance()->cout() << "ExtSceneBody::test" << std::endl;
			return cnoid::SceneBody::onKeyPressEvent(event);
		}
#else
		virtual bool onKeyPressEvent(const cnoid::SceneWidgetEvent& event){
			cnoid::MessageView::mainInstance()->cout() << "ExtSceneBody::test" << std::endl;
			return cnoid::EditableSceneBody::onKeyPressEvent(event);
		}
#endif
/*		virtual bool onKeyReleaseEvent(const cnoid::SceneViewEvent& event){
		}
		virtual bool onButtonPressEvent(const cnoid::SceneViewEvent& event){
		}
		virtual bool onDoubleClickEvent(const cnoid::SceneViewEvent& event){
		}
		virtual bool onButtonReleaseEvent(const cnoid::SceneViewEvent& event){
		}
		virtual bool onPointerMoveEvent(const cnoid::SceneViewEvent& event){
		}
		virtual void onPointerLeaveEvent(const cnoid::SceneViewEvent& event){
		}
		virtual bool onScrollEvent(const cnoid::SceneViewEvent& event){
		}
*/
};
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
typedef boost::intrusive_ptr<ExtSceneBody> ExtSceneBodyPtr;
#else
typedef cnoid::ref_ptr<ExtSceneBody> ExtSceneBodyPtr;
#endif


class EXCADE_API TestSceneBody : public ExtSceneBody
{
	public:
		TestSceneBody(cnoid::BodyItemPtr bodyItem) : ExtSceneBody(bodyItem){ }
		
		static SceneBody* create(cnoid::BodyItemPtr bodyItem){
			return new TestSceneBody(bodyItem) ;
		}
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		virtual bool onKeyPressEvent(const cnoid::SceneViewEvent& event){
			cnoid::MessageView::mainInstance()->cout() << "TestSceneBody::test" << std::endl;
			return ExtSceneBody::onKeyPressEvent(event);
		}
#else
		virtual bool onKeyPressEvent(const cnoid::SceneWidgetEvent& event){
			cnoid::MessageView::mainInstance()->cout() << "TestSceneBody::test" << std::endl;
			return ExtSceneBody::onKeyPressEvent(event);
		}
#endif
/*		virtual bool onKeyReleaseEvent(const cnoid::SceneViewEvent& event){
		}
		virtual bool onButtonPressEvent(const cnoid::SceneViewEvent& event){
		}
		virtual bool onDoubleClickEvent(const cnoid::SceneViewEvent& event){
		}
		virtual bool onButtonReleaseEvent(const cnoid::SceneViewEvent& event){
		}
		virtual bool onPointerMoveEvent(const cnoid::SceneViewEvent& event){
		}
		virtual void onPointerLeaveEvent(const cnoid::SceneViewEvent& event){
		}
		virtual bool onScrollEvent(const cnoid::SceneViewEvent& event){
		}
*/
};


class EXCADE_API GraspBodyItem : public cnoid::BodyItem
{
	public:
		GraspBodyItem(const BodyItem& org) : BodyItem(org){
			scene_ = NULL;
		}
		virtual cnoid::SgNode* scene(){
			if(!scene_){
				scene_ = (ExtSceneBody*)(factory(NULL))(this);
			}
			return scene_.get();
		}
		static boost::function<cnoid::SceneBody*(cnoid::BodyItem*)> factory(boost::function<cnoid::SceneBody*(cnoid::BodyItem*)> ext=NULL){
			static boost::function<cnoid::SceneBody*(cnoid::BodyItem*)> factory_ = ExtSceneBody::create;
			if(ext) factory_ = ext;
			return factory_;
		}
		ExtSceneBodyPtr scene_;
};

}

#endif
