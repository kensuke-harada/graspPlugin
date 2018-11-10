#include "IniFileParameterReader.h"

using namespace grasp;

IniFileParameterReader::IniFileParameterReader() :
	os_(cnoid::nullout()),
	settings_(NULL)
{
}

IniFileParameterReader::IniFileParameterReader(std::ostream& os) :
	os_(os),
	settings_(NULL)
{
}

IniFileParameterReader::~IniFileParameterReader() {
	if (settings_ != NULL) {
		delete settings_;
	}
}

bool IniFileParameterReader::loadFile(const std::string& ini_file_path) {
	if (settings_ != NULL) delete settings_;

	QString file_path = QString::fromStdString(ini_file_path);
	if (!QFile::exists(file_path)) {
		os_ << "[ERROR] : Cannot read ini file \"" << ini_file_path << "\"" << std::endl;
		return false;
	}
	settings_ = new QSettings(file_path, QSettings::IniFormat);
	return true;
}

void IniFileParameterReader::setCurrentDir(const std::string& current_dir) {
	current_dir_path_ = QString::fromStdString(current_dir);
}

void IniFileParameterReader::beginGroup(const std::string& group_name) const {
	settings_->beginGroup(QString::fromStdString(group_name));
}

void IniFileParameterReader::endGroup() const {
	settings_->endGroup();
}

bool IniFileParameterReader::hasKey(const std::string& key) const {
	return settings_->contains(QString::fromStdString(key));
}

bool IniFileParameterReader::read(const std::string& key, int& out_value) const {
	QVariant ret = settings_->value(QString::fromStdString(key));
	if (ret.isNull()) {
		os_ << "[Error] : " << key << " is not found in INI file." << std::endl;
		return false;
	}
	out_value = ret.toInt();
	return true;
}

bool IniFileParameterReader::read(const std::string& key, int& out_value, int default_value) const {
	out_value = settings_->value(QString::fromStdString(key), default_value).toInt();
	return true;
}

bool IniFileParameterReader::read(const std::string& key, double& out_value) const {
	QVariant ret = settings_->value(QString::fromStdString(key));
	if (ret.isNull()) {
		os_ << "[Error] : " << key << " is not found in INI file." << std::endl;
		return false;
	}
	out_value = ret.toDouble();
	return true;
}

bool IniFileParameterReader::read(const std::string& key, double& out_value, double default_value) const {
	out_value = settings_->value(QString::fromStdString(key), default_value).toDouble();
	return true;
}

bool IniFileParameterReader::read(const std::string& key, std::string& out_value) const {
	QVariant ret = settings_->value(QString::fromStdString(key));
	if (ret.isNull()) {
		os_ << "[Error] : " << key << " is not found in INI file." << std::endl;
		return false;
	}
	out_value = ret.toString().toStdString();
	return true;
}

bool IniFileParameterReader::read(const std::string& key, std::string& out_value, const std::string& default_value) const {
	out_value = settings_->value(QString::fromStdString(key), QString::fromStdString(default_value)).toString().toStdString();
	return true;
}

bool IniFileParameterReader::read(const std::string& key, cnoid::Vector3& out_value) const {
	QVariant ret = settings_->value(QString::fromStdString(key));
	if (ret.isNull()) {
		os_ << "[Error] : " << key << " is not found in INI file." << std::endl;
		return false;
	}
	QStringList strlist = ret.toString().split(" ");
	if (strlist.size() != 3) {
		os_ << "[Error] : " << key << "'s value is not invalid." << std::endl;
		return false;
	}
	for (int i = 0; i < 3; i++) {
		out_value[i] = strlist.at(i).toDouble();
	}
	return true;
}

bool IniFileParameterReader::read(const std::string& key, cnoid::Vector3& out_value, const cnoid::Vector3& default_value) const {
	if (hasKey(key)) {
		return read(key, out_value);
	}
	out_value = default_value;
	return false;
}

bool IniFileParameterReader::readPath(const std::string& key, std::string& out_value) const {
	std::string rel_str;
	if (!read(key, rel_str)) {
		return false;
	}
	out_value = QFileInfo(current_dir_path_ + "/" + QString::fromStdString(rel_str)).absoluteFilePath().toStdString();
	return true;
}

bool IniFileParameterReader::readPath(const std::string& key, std::string& out_value, const std::string& default_value) const {
	if (hasKey(key)) {
		return readPath(key, out_value);
	}
	out_value = default_value;
	return true;
}
