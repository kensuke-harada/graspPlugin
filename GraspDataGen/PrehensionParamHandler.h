#ifndef _GRASPDATAGEN_PREHENSIONPARAMHANDLER_H_
#define _GRASPDATAGEN_PREHENSIONPARAMHANDLER_H_

#include <vector>
#include <string>
#include <fstream>

#include <cnoid/MessageView>
#include <cnoid/EigenTypes>
#include <cnoid/ValueTree>

#include "../Grasp/PrehensionParameter.h"

namespace grasp {
	class PrehensionParameterExt
	: public PrehensionParameter {
	public:
		PrehensionParameterExt() {;}

		std::vector<std::vector<double> > angles;
		double coeff_GRCdes;
		double coeff_GRCmin;
		int id;

		static PrehensionParameterExt createPrehensionPrameterExt(const PrehensionPtr prehen, int id = -1);

		int fing_size() const {return fingers.size();}
		int joint_size(int i) const {if (i > fing_size()) return 0; return angles[i].size();}
	};

	
	class PrehensionParamHandler {
	public:
		PrehensionParamHandler() : os(cnoid::MessageView::mainInstance()->cout()) {;}
		virtual ~PrehensionParamHandler() {;}

		bool loadParams();
		int getNumParams() const {return params.size();}
		PrehensionParameterExt getParam(int i) const {return params[i];}
		void updateParam(int i, const PrehensionParameterExt& param) {params[i] = param;}
		void addParam(const PrehensionParameterExt& param) {params.push_back(param);}
		void updateYamlFile();

		void display(const PrehensionParameterExt& params);
		void grasp(const PrehensionParameterExt& params);
	protected:
		void setFingerAngle(const PrehensionParameterExt& params);
		void displayGRC(const PrehensionParameterExt& params);
		void parsePosfiles();
		void writePosfiles();

		std::ostream& os;
	private:
		std::vector<PrehensionParameterExt> params;
	};
}

#endif
