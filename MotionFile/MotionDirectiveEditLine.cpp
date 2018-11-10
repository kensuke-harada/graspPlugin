#include <iostream>

#include <QLabel>
#include <QLayout>
#include <QSpacerItem>
#include <QMessageBox>

#include <Eigen/Dense>

#include "MotionDirectiveInfo.h"
#include "MotionDirectiveEditLine.h"
#include "MotionDirectiveValueEditLine.h"
#include "MotionUtil.h"

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace Eigen;
using namespace motionedit;

/**
 * @brief コンストラクタの実装です。
 */
MotionDirectiveEditLine::MotionDirectiveEditLine(QWidget* parent, int id) :QFrame(parent)
{
	// Id を設定します。
	m_Id = id;
	
	Initialize(parent);
	
}

MotionDirectiveEditLine::MotionDirectiveEditLine(QWidget* parent, int id, const std::map<std::string, std::vector<double> >& motionAndValuesMap):QFrame(parent)
{
	m_Id = id;

	m_motionAndValuesMap = motionAndValuesMap;
	
	Initialize(parent);
}

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
void MotionDirectiveEditLine::InitializeByRobot(intrusive_ptr<IRobotInMotionEdit> robot)
#else
void MotionDirectiveEditLine::InitializeByRobot(ref_ptr<IRobotInMotionEdit> robot)
#endif
{
	m_motionAndValuesMap = robot->GetMotionAndDefaultValuesMap();
	
	{
		for(int i = 0; i < m_motionDirectiveNameBox->count(); i++)
		{
			m_motionDirectiveNameBox->removeItem(0);
		}

		vector<string> motionNameList = util::GetKeyList(m_motionAndValuesMap);
		for(vector<string>::iterator iter = motionNameList.begin(); iter != motionNameList.end(); iter++)
		{
			m_motionDirectiveNameBox->addItem(util::ConvertToQString(*iter));
		}
	}

	m_currentMotionDirectiveTypeName = m_motionDirectiveNameBox->currentText();

	on_motionDirectiveType_Changed(m_motionDirectiveNameBox->currentIndex());
}

void MotionDirectiveEditLine::Initialize(QWidget* parent)
{
	// 動作の開始時間のラベルを作成します。
	QLabel* startTimeLabel = new QLabel(tr("StartTime:"));

	// 動作の開始時間を設定するボックスを作成します。
	m_startTimeEdit = new QDoubleSpinBox;
	{
		m_startTimeEdit->setSizePolicy(
			QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed)
			);
	}

	// 動作の終了時間のをラベルを作成します。
	QLabel* endTimeLabel = new QLabel(tr("EndTime:"));

	// 動作の終了時間を設定するボックスを作成します。
	m_endTimeEdit = new QDoubleSpinBox;
	{
		m_endTimeEdit->setSizePolicy(
			QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed)
			);
	}

	// 動作指示の種類を設定する部分のラベルを作成します。
	QLabel* motionLabel = new QLabel(tr("Motion:"));

	// 動作指示の種類を設定するボックスを作成します。
	m_motionDirectiveNameBox = new ComboBox;
	{
		m_motionDirectiveNameBox->setSizePolicy(
			QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed)
			);
		if(m_motionAndValuesMap.empty())
		{
			m_motionDirectiveNameBox->addItem("NONE");
		}
		else
		{
			vector<string> motionNameList = util::GetKeyList(m_motionAndValuesMap);

			for(vector<string>::iterator iter = motionNameList.begin(); iter != motionNameList.end(); iter++)
			{
				m_motionDirectiveNameBox->addItem(util::ConvertToQString(*iter));
			}
		}
	}

	// 動作指示の座標を設定する部分のラベルを作成します。
	QLabel* coordinateLabel = new QLabel("Coordinate:");

	// 動作指示の座標を設定するボックスを作成します。
	m_coordinateSystemNameBox = new ComboBox;
	{
		m_coordinateSystemNameBox->setSizePolicy(
			QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed)
			);
		m_coordinateSystemNameBox->addItem("NONE");
		m_coordinateSystemNameBox->addItem("ABS");
		m_coordinateSystemNameBox->addItem("REL");
	}

	// 動作指示の数値列を作成します。
	m_motionDirectiveValueEditLine = new MotionDirectiveValueEditLine(parent);
	
	if(!m_motionAndValuesMap.empty())
	{
		 m_motionDirectiveValueEditLine->MakeValueBoxes(m_motionAndValuesMap[util::ConvertToString(m_motionDirectiveNameBox->currentText())]);
	}

	// 自身のレイアウトに作成したウィジェットを追加します。
	{
		QGridLayout* layout = new QGridLayout();
		layout->addItem(new QSpacerItem(40, 20), 1, 0);
		layout->addWidget(startTimeLabel, 0, 1);
		layout->addWidget(m_startTimeEdit, 1, 1);
		layout->addItem(new QSpacerItem(40, 20, QSizePolicy::Fixed), 1, 2);
		layout->addWidget(endTimeLabel, 0, 3);
		layout->addWidget(m_endTimeEdit, 1, 3);
		layout->addItem(new QSpacerItem(40, 20, QSizePolicy::Fixed), 1, 4);
		layout->addWidget(motionLabel, 0, 5);
		layout->addWidget(m_motionDirectiveNameBox, 1, 5);
		layout->addItem(new QSpacerItem(40, 20, QSizePolicy::Fixed), 1, 6);
		layout->addWidget(coordinateLabel, 0, 7);
		layout->addWidget(m_coordinateSystemNameBox, 1, 7);
		layout->addItem(new QSpacerItem(40, 20, QSizePolicy::Fixed), 1, 8);
	        layout->addLayout(m_motionDirectiveValueEditLine, 1, 9);
		layout->addItem(new QSpacerItem(40, 20, QSizePolicy::Expanding), 1, 10);
		this->setLayout(layout);
	}
	
	// 自身のスロットと作成したウィジェットのシグナルを結合します。
	{
		m_motionDirectiveNameBox->sigCurrentIndexChanged().connect(bind(&MotionDirectiveEditLine::on_motionDirectiveType_Changed, this, _1));	
	}
	
	m_currentMotionDirectiveTypeName = m_motionDirectiveNameBox->currentText();

	return;
}

/**
 * @brief 自身の Id を得るメンバ変数の実装です。
 */
int MotionDirectiveEditLine::GetId()
{
	return m_Id;
}

/**
 * @brief 動作開始時間を得るメンバ関数の実装です。
 */
double MotionDirectiveEditLine::GetStartTime()
{
	return m_startTimeEdit->value();
}

/**
 * @brief 動作終了時間を得るメンバ関数の実装です。
 */
double MotionDirectiveEditLine::GetEndTime()
{
	return m_endTimeEdit->value();
}

string MotionDirectiveEditLine::GetMotionDirectiveTypeName()
{
	return util::ConvertToString(m_motionDirectiveNameBox->currentText());
}

string MotionDirectiveEditLine::GetCoordinateTypeName()
{
 	return util::ConvertToString(m_coordinateSystemNameBox->currentText());
}

vector<double> MotionDirectiveEditLine::GetMotionDirectiveVaueList()
{

	int valueEditBoxCount =
		this->m_motionDirectiveValueEditLine->GetMotionDirectiveValueCount();
	
	vector<double> valueList;

	for (int i = 0; i < valueEditBoxCount; i++)
	{
		valueList.push_back(m_motionDirectiveValueEditLine->GetValue(i));
	}

	return valueList;
	
}

int MotionDirectiveEditLine::GetMotionDirectiveValueCount()
{
	return m_motionDirectiveValueEditLine->GetMotionDirectiveValueCount(); 
}

/**
 * @brief 動作指示の種類の名前を設定するメンバ関数です。
 */
void MotionDirectiveEditLine::SetMotionDirectiveTypeName(const QString& motionDirectiveTypeName)
{
	int index = m_motionDirectiveNameBox->findText(motionDirectiveTypeName);

	m_motionDirectiveNameBox->setCurrentIndex((index >= 0) ? index : 0 );
}

/**
 * @brief 座標系の種類の名前を設定するメンバ関数です。
 */
void MotionDirectiveEditLine::SetCoordinateTypeName(const QString& coordinateTypeName)
{
	int index = m_coordinateSystemNameBox->findText(coordinateTypeName);
		
	m_coordinateSystemNameBox->setCurrentIndex((index >= 0) ? index : 0);
}

/**
 * @brief 動作開始時間を設定するメンバ関数の実装です。
 */
void MotionDirectiveEditLine::SetStartTime(double startTime)
{
	m_startTimeEdit->setValue(startTime);
}

/**
* @brief 動作終了時間を設定するメンバ関数の実装です。
*/
void MotionDirectiveEditLine::SetEndTime(double endTime)
{
	m_endTimeEdit->setValue(endTime);
}

/**
 * @brief 与えられた動作指示情報をもとに自身を更新するメンバ変数の実装です。
 */
void MotionDirectiveEditLine::Update(const MotionDirectiveInfo& motionDirectiveInfo)
{
	// 動作の開始時間と終了時間を更新します。
	this->m_startTimeEdit->setValue(motionDirectiveInfo.GetStartTime());
	this->m_endTimeEdit->setValue(motionDirectiveInfo.GetEndTime());

	// 動作指示の設定数値リストを得ます 
	vector<double> valueList = util::Convert(motionDirectiveInfo.GetDirectiveValueVector());
	
	// 設定数値リストをもとに自身の数値編集ボックスを更新します。
	this->m_motionDirectiveValueEditLine->MakeValueBoxes(valueList);
}

/**
 * @brief 与えられた動作指示情報をもとに
 * 動作指示情報を作成するメンバ関数の実装です。
 */
MotionDirectiveInfo MotionDirectiveEditLine::MakeMotionDirectiveInfo()
{
	MotionDirectiveInfo motionDirectiveInfo;

	motionDirectiveInfo.SetStartTime(this->m_startTimeEdit->value());
	motionDirectiveInfo.SetEndTime(this->m_endTimeEdit->value());
	
	motionDirectiveInfo.SetMotionDirectiveType(MotionType_None);

	motionDirectiveInfo.SetCoordinateType(CoordinateType_None);

	int valueEditBoxCount =
		this->m_motionDirectiveValueEditLine->GetMotionDirectiveValueCount();

	{
		VectorXd valueList(valueEditBoxCount);

		for (int i = 0; i < valueEditBoxCount; i++)
		{
			valueList[i] = m_motionDirectiveValueEditLine->GetValue(i);
		}
	}

	return motionDirectiveInfo;

}

void MotionDirectiveEditLine::Update(
	double startTime,
	double endTime,
	std::string motionDirectiveTypeName,
	std::string coordinateTypeName,
	const std::vector<double>& motionDirectiveValueList
	)
{

	this->m_startTimeEdit->setValue(startTime);

	this->m_endTimeEdit->setValue(endTime);
	
	QString qMotionDirectiveTypeName = util::ConvertToQString(motionDirectiveTypeName);

	this->SetMotionDirectiveTypeName(qMotionDirectiveTypeName);

	QString qCoordinateTypeName = util::ConvertToQString(coordinateTypeName);

	this->SetCoordinateTypeName(qCoordinateTypeName);

	this->m_motionDirectiveValueEditLine->MakeValueBoxes(motionDirectiveValueList);
}

vector<double> MotionDirectiveEditLine::GetValueList()
{
	vector<double> valueList;
	{
		for(int i = 0; i < m_motionDirectiveValueEditLine->GetMotionDirectiveValueCount();i++)
		{
			valueList.push_back(m_motionDirectiveValueEditLine->GetValue(i));
		}
	}
	return valueList;
}

/**
　* @brief 動作指示の種類が変更された場合に発生させるシグナルです。
 */
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
SignalProxy<signal< void(int, const QString&)> > MotionDirectiveEditLine::MotionDirectiveTypeChanged()
#else
SignalProxy<void(int, const QString&)> MotionDirectiveEditLine::MotionDirectiveTypeChanged()
#endif
{
	return m_motionDirectiveTypeChanged;
}

/**
 * @brief 動作指示の座標系が変更された場合に発生させるシグナルです。
 */
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
SignalProxy<signal< void(int, const QString&)> > MotionDirectiveEditLine::CoordinateTypeChanged()
#else
SignalProxy< void(int, const QString&)> MotionDirectiveEditLine::CoordinateTypeChanged()
#endif
{
	return m_coordinateTypeChanged;
}

/**
 * @brief 動作指示の種類が変更された場合に発生するシグナルを
 * 受け取るスロットの実装です。
 */
void MotionDirectiveEditLine::on_motionDirectiveTypeChanged(const QString& motionDiectiveTypeName)
{
	// 自身の ID 情報を追加してシグナルを再発行します。	
	//QQQ##        
	//emit MotionDirectiveTypeChanged(m_Id, motionDiectiveTypeName);

	m_motionDirectiveTypeChanged(m_Id, motionDiectiveTypeName);
}

/**
 * @brief 座標系が変更された場合に発生するシグナルを
 * 受け取るスロットの実装です。
 */
void MotionDirectiveEditLine::on_coordinateTypeChanged(const QString& coordinateTypeName)
{
	// 自身の ID 情報を追加してシグナルを再発行します。
	m_coordinateTypeChanged(m_Id, coordinateTypeName);
}

/**
 * @brief 動作指示の種類が変更された場合に発生するシグナルを
 * 受け取るスロットの実装です。
 */
void MotionDirectiveEditLine::on_motionDirectiveType_Changed(int index)
{
	QString motionDirectiveTypeName = m_motionDirectiveNameBox->currentText();

	m_motionAndValuesMap[util::ConvertToString(m_currentMotionDirectiveTypeName)] = GetValueList();

	string stdMotionDirectiveTypeName = util::ConvertToString(motionDirectiveTypeName);
	
	if(m_motionAndValuesMap.find(stdMotionDirectiveTypeName) == m_motionAndValuesMap.end())
	{
		return ;
	}
		
	m_motionDirectiveValueEditLine->MakeValueBoxes(m_motionAndValuesMap[stdMotionDirectiveTypeName]);

	return;
}



