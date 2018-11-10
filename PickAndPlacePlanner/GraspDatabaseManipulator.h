#ifndef _PICKANDPLACEPLANNER_GRASPDATABASEMANIPULATOR_H_
#define _PICKANDPLACEPLANNER_GRASPDATABASEMANIPULATOR_H_

#include <vector>
#include <fstream>
#include <algorithm>

#include <cnoid/EigenTypes>

#include "exportdef.h"

namespace grasp{
	class GraspDatabaseManipulator {
	public:
		struct GraspPosture {
			int id;
			double q;
			cnoid::Vector3 p;
			cnoid::Matrix3 R;
			cnoid::VectorXd finger_q;

			static bool sortData(const GraspPosture& l,const GraspPosture& r) {return (l.q>r.q);}
			bool operator==(const GraspPosture& r) {return ((p == r.p) && (R == r.R) && (finger_q == r.finger_q));}
			bool operator!=(const GraspPosture& r) {return ((p != r.p) || (R != r.R) || (finger_q != r.finger_q));}
		};

		typedef std::vector<GraspPosture> GraspPoses;

		GraspDatabaseManipulator(){;}
		virtual ~GraspDatabaseManipulator(){;}

		bool readFile(const std::string& filepath);
		bool writeFile(const std::string& filepath); 
		GraspPoses getRecords() const {return records;}
		void setRecords(GraspPoses _records) {records = _records;}
		void addRecord(GraspPosture record) {records.push_back(record);}
		void appendRecords(GraspPoses _records) {records.insert(records.end(), _records.begin(), _records.end()); sort();}
		void unique() {records.erase(std::unique(records.begin(), records.end()), records.end());}
		void sort() {std::sort(records.begin(), records.end(), GraspPosture::sortData);}
	protected:
		GraspPoses records;
	};
}

#endif /* _PICKANDPLACEPLANNER_GRASPDATABASEMANIPULATOR_H_ */
