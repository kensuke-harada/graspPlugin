#ifndef _GRASP_INIFILEPARAMETERREADER_H_
#define _GRASP_INIFILEPARAMETERREADER_H_

#include <iostream>

#include <QSettings>
#include <QFile>
#include <QFileInfo>
#include <QStringList>

#include <cnoid/NullOut>
#include <cnoid/EigenTypes>

#include "exportdef.h"

namespace grasp {
	class EXCADE_API IniFileParameterReader {
	public:
		IniFileParameterReader();
		IniFileParameterReader(std::ostream& os);
		virtual ~IniFileParameterReader();

		virtual bool loadFile(const std::string& ini_file_path);
		void setCurrentDir(const std::string& current_dir);

		void beginGroup(const std::string& group_name) const;
		void endGroup() const;

		bool hasKey(const std::string& key) const;

		bool read(const std::string& key, int& out_value) const;
		bool read(const std::string& key, int& out_value, int default_value) const;
		bool read(const std::string& key, double& out_value) const;
		bool read(const std::string& key, double& out_value, double default_value) const;
		bool read(const std::string& key, std::string& out_value) const;
		bool read(const std::string& key, std::string& out_value, const std::string& default_value) const;
		bool read(const std::string& key, cnoid::Vector3& out_value) const;
		bool read(const std::string& key, cnoid::Vector3& out_value, const cnoid::Vector3& default_value) const;
		bool readPath(const std::string& key, std::string& out_value) const;
		bool readPath(const std::string& key, std::string& out_value, const std::string& default_value) const;

	protected:
		std::ostream& os_;
		QString current_dir_path_;

		QSettings* settings_;
	};
}

#endif /* _GRASP_INIFILEPARAMETERREADER_H_ */
