#ifndef _GRASP_PREHENSIONPARAMETER_H_
#define _GRASP_PREHENSIONPARAMETER_H_

#include <vector>
#include <string>

#include <cnoid/EigenTypes>
#include <cnoid/Item>
#include <cnoid/ItemManager>
#include <cnoid/ValueTree>
#include <cnoid/YAMLReader>

#include "exportdef.h"

namespace grasp {
	class EXCADE_API PrehensionParameter {
		public:
			PrehensionParameter() {;}

			struct FingerParameter {
				std::vector<bool> contact;
				std::vector<int> complink;
				std::vector<double> close;
			};
			std::string name;
			cnoid::Vector3 GRCmin_pos, GRCmin_rpy, GRCmin_edge;
			cnoid::Vector3 GRCdes_pos, GRCdes_rpy, GRCdes_edge;
			cnoid::Vector3 GRCmax_pos, GRCmax_rpy, GRCmax_edge;
			double load_min;
			double load_max;
			bool use_palmclose;
			cnoid::Vector3 palmclose_dir;
			std::vector<std::string> ref_motion;
			std::vector<FingerParameter> fingers;
	};

	class EXCADE_API Prehension : public cnoid::Item {
	public:

		static void initializeClass(cnoid::ExtensionManager* ext);
		
		cnoid::MappingPtr info;

		void setNameDefault();

		void convertToPrehensionParameter(PrehensionParameter* param) const;
		void setInfo(const PrehensionParameter& param);

	protected:
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		virtual cnoid::ItemPtr doDuplicate() const;
#endif
		virtual void doPutProperties(cnoid::PutPropertyFunction& putProperty);
		virtual bool store(cnoid::Archive& archive);
		virtual bool restore(const cnoid::Archive& archive);
	};

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	typedef boost::intrusive_ptr<Prehension> PrehensionPtr;
#else
	typedef cnoid::ref_ptr<Prehension> PrehensionPtr;
#endif

	class PrehensionLoader {
	public:
		static void load(const std::string& filename, std::vector<PrehensionPtr>& prehension_list);
	private:
		PrehensionLoader() {;}
	};

}

#endif /* _GRASP_PREHENSIONPARAMETER_H_ */
