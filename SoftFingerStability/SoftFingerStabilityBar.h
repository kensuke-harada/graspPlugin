/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#ifndef _SOFTFINGERSTABILITY_SOFTFINGERSTABILITYBAR_H_
#define _SOFTFINGERSTABILITY_SOFTFINGERSTABILITYBAR_H_

#ifndef  CNOID_10_11_12_13
#include <cnoid/ItemList>
#endif
#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/ToolBar>	/* modified by qtconv.rb 0th rule*/ 
 
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <cnoid/SignalProxy>
#else
#include <cnoid/Signal>
#endif

#include "exportdef.h"

namespace cnoid {
	class MessageView;
}

namespace grasp {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	class EXCADE_API SoftFingerStabilityBar : public cnoid::ToolBar, public boost::signals::trackable
#else
	class EXCADE_API SoftFingerStabilityBar : public cnoid::ToolBar
#endif
	{
	public:

		static SoftFingerStabilityBar* instance();
		virtual ~SoftFingerStabilityBar();
	
	protected:

		virtual bool storeState(cnoid::Archive& archive);
		virtual bool restoreState(const cnoid::Archive& archive);

	private:
		
		SoftFingerStabilityBar();

		cnoid::MessageView& mes;
		std::ostream& os;

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		boost::signal<void(const cnoid::ItemList<cnoid::BodyItem>& selectedBodyItems)> sigBodyItemSelectionChanged_;
		boost::signal<void(cnoid::BodyItem* currentBodyItem)> sigCurrentBodyItemChanged_;
#else
		cnoid::Signal<void(const cnoid::ItemList<cnoid::BodyItem>& selectedBodyItems)> sigBodyItemSelectionChanged_;
		cnoid::Signal<void(cnoid::BodyItem* currentBodyItem)> sigCurrentBodyItemChanged_;
#endif    
	};
}

#endif /* _SOFTFINGERSTABILITY_SOFTFINGERSTABILITYBAR_H_ */
