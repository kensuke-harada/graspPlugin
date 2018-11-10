#include "MotionDirectiveEditPart.h"

#include <vector>
#include <QMessageBox>
#include <iostream>

#include "MotionDirectiveEditLine.h"
#include "RobotInMotionEdit.h"
#include "MotionUtil.h"

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace motionedit;

/**
* @brief MotionDirectiveEditPart のコンストラクタの実装です。
*/
MotionDirectiveEditPart::MotionDirectiveEditPart(QWidget* parent) :QVBoxLayout(parent)
{}

/**
* @brief 動作指示編集ラインの数を得るメンバ関数の実装です。
*/
int MotionDirectiveEditPart::GetLineCount()
{

	return m_motionDirectiveEditLineList.size();
}

/**
* @brief 与えられた動作指示情報をもとに
* 全ての動作指示編集ラインを更新するメンバ関数の実装です。
*/
void MotionDirectiveEditPart::Update(const std::vector<MotionDirectiveInfo>& motionDirectiveInfoList)
{
	// 自身のもつ動作指示ラインを全て削除します。
	this->Clear();

	// 与えられた動作指示情報のリストの要素数だけ動作指示編集ラインを新たに追加していきます。
	for (vector<MotionDirectiveInfo>::const_iterator iter = motionDirectiveInfoList.begin();
		iter != motionDirectiveInfoList.end(); iter++)
	{
		// 自身に動作指示編集ラインを 1 行追加します。
		this->AddLine();

		// 追加した動作指示編集ラインを
		// 対応する動作指示情報をもとに更新します。
		this->m_motionDirectiveEditLineList.back()->Update(*iter);
	}

	return;
}

/**
* @brief 動作指定編集列を 1 行追加するメンバ関数です。
* 
*/
void MotionDirectiveEditPart::AddLine()
{
	// 作成する動作指示編集ラインに与える ID を作成します。
	// ID は動作指示編集ラインが上から何番目に位置するかを表します。
	int id = GetLineCount();

	// 新たな動作指示列を作成します。
	MotionDirectiveEditLine* motionDirectiveEditLine =
		//new motionedit::MotionDirectiveEditLine(0, id);
		new motionedit::MotionDirectiveEditLine(0, id, m_motionAndValuesMap);
	// 追加する開始時間と終了時間の調整を行います。
	{
		const double diffFromPrevEndTimeToCurStartTime = 1.0;

		const double diffFromStartTimeToEndTime = 1.0;

		double startTime = 0;

		if (id != 0)
		{
			startTime = 
				m_motionDirectiveEditLineList.back()->GetEndTime() + diffFromPrevEndTimeToCurStartTime;
		}

		motionDirectiveEditLine->SetStartTime(startTime);

		motionDirectiveEditLine->SetEndTime(startTime + diffFromStartTimeToEndTime);
	}

	// 自身のメンバ変数、
	// 及びレイアウトに作成した動作指示列を追加します。
	m_motionDirectiveEditLineList.push_back(motionDirectiveEditLine);

	// 作成した動作指示編集ラインをレイアウトに追加します。
	addWidget(motionDirectiveEditLine);

	// 自身のシグナルと作成した動作指示列のシグナルを連結します。
	{
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		signals::connection connectionOfMotionDirectiveTypeChangedSignal = 
			motionDirectiveEditLine->MotionDirectiveTypeChanged().connect(bind(&MotionDirectiveEditPart::on_motionDirectiveTypeChanged, this, _1, _2));
		signals::connection connectionOfCoordinateTypeChangedSignal = 
			motionDirectiveEditLine->CoordinateTypeChanged().connect(bind(&MotionDirectiveEditPart::on_coordinateTypeChanged, this, _1, _2));
#else
		Connection connectionOfMotionDirectiveTypeChangedSignal = 
			motionDirectiveEditLine->MotionDirectiveTypeChanged().connect(bind(&MotionDirectiveEditPart::on_motionDirectiveTypeChanged, this, _1, _2));
		Connection connectionOfCoordinateTypeChangedSignal = 
			motionDirectiveEditLine->CoordinateTypeChanged().connect(bind(&MotionDirectiveEditPart::on_coordinateTypeChanged, this, _1, _2));
#endif
		m_connectionStackOfMotionDirectiveTypeChangedSignal.push(connectionOfMotionDirectiveTypeChangedSignal);
		
		m_connectionStackOfCoordinateTypeChangedSignal.push(connectionOfCoordinateTypeChangedSignal);
	}

	return;
}


/**
* @brief 全ての動作指示編集ラインを削除するメンバ関数の実装です。
*/
void MotionDirectiveEditPart::Clear()
{
	// 自身の持つ動作指示編集ラインの数を取得し、
	// 1 行ずつ削除します。
	int count = GetLineCount();

	for (int i = 0; i <= count; i++)
	{
		DeleteLine();
	}

	util::ClearLayout(this);

	return;
}

/**
* @brief 全ての動作指示編集ラインを 1 番下から
* 削除していくメンバ関数です。
*/
void MotionDirectiveEditPart::DeleteLine()
{
	// 自身が動作指示編集ラインを 1 つも持たない場合は何もしません。
	if (GetLineCount() == 0)
	{
		return;
	}

	MotionDirectiveEditLine* deletingLine = 
		m_motionDirectiveEditLineList[m_motionDirectiveEditLineList.size() - 1];

	// 自身のシグナルと削除する動作指示ラインのシグナルの連結を削除します。
	{
		m_connectionStackOfMotionDirectiveTypeChangedSignal.top().disconnect();

		m_connectionStackOfCoordinateTypeChangedSignal.top().disconnect();

		m_connectionStackOfMotionDirectiveTypeChangedSignal.pop();

		m_connectionStackOfCoordinateTypeChangedSignal.pop();
	}

	m_motionDirectiveEditLineList.erase(--m_motionDirectiveEditLineList.end());
}


/**
 * @biref 動作指示編集列をもとに動作指示情報のリストを作成するメンバ関数の実装です。
 */
vector<MotionDirectiveInfo> MotionDirectiveEditPart::MakeMotionDirectiveInfoList()
{
	vector<MotionDirectiveInfo> motionDirectiveInfoList;

	int lineCount = m_motionDirectiveEditLineList.size();

	for (vector<MotionDirectiveEditLine*>::iterator iter = m_motionDirectiveEditLineList.begin();
		iter != m_motionDirectiveEditLineList.end();
		iter++)
	{
		motionDirectiveInfoList.push_back((*iter)->MakeMotionDirectiveInfo());
	}

	return motionDirectiveInfoList;
}

/**
* @biref 与えられたロボットの情報を元に操作指示編集列を初期化するメンバ関数の実装です。
*/
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
void  MotionDirectiveEditPart::InitializeByRobot(intrusive_ptr<IRobotInMotionEdit> robot)
#else
void  MotionDirectiveEditPart::InitializeByRobot(ref_ptr<IRobotInMotionEdit> robot)
#endif
{
	int lineCount = m_motionDirectiveEditLineList.size();

	for (vector<MotionDirectiveEditLine*>::iterator iter = m_motionDirectiveEditLineList.begin();
		iter != m_motionDirectiveEditLineList.end();
		iter++)
	{
		(*iter)->InitializeByRobot(robot);
	}

        m_motionAndValuesMap = robot->GetMotionAndDefaultValuesMap();

	return;
}

/**
* @biref 引数で与えられた動作指示の情報を元にして、自身が含む指定されたインデックスの動作指示列の内容を更新するメンバ関数の実装です。
*/
void MotionDirectiveEditPart::Update(
	int lineIndex,
	double startTime,
	double endTime,
	std::string motionDirectiveTypeName,
	std::string coordinateTypeName,
	const std::vector<double>& motionDirectiveValueList
	)
{
	this->m_motionDirectiveEditLineList[lineIndex]->Update(
		startTime,
		endTime,
		motionDirectiveTypeName,
		coordinateTypeName,
		motionDirectiveValueList
		);
}

/**
* @biref　自身が含む動作指定編集列のうち、指定したインデックスの列の動作開始時間を得るメンバ関数の実装です。
*/
int MotionDirectiveEditPart::GetStartTime(int lineIndex)
{
	return m_motionDirectiveEditLineList[lineIndex]->GetStartTime();
}

/**
* @biref　自身が含む動作指定編集列のうち、指定したインデックスの列の動作終了時間を得るメンバ関数の実装です。
*/
int MotionDirectiveEditPart::GetEndTime(int lineIndex)
{
	return m_motionDirectiveEditLineList[lineIndex]->GetEndTime();
}

/**
* @biref　自身が含む動作指定編集列のうち、指定したインデックスの列の動作指示の種類の名前を得るメンバ関数の実装です。
*/
string MotionDirectiveEditPart::GetMotionDirectiveTypeName(int lineIndex)
{
	return m_motionDirectiveEditLineList[lineIndex]->GetMotionDirectiveTypeName();
}

/**
* @biref　自身が含む動作指定編集列のうち、指定したインデックスの列の座標系の種類の名前を得るメンバ関数の実装です。
*/
string MotionDirectiveEditPart::GetCoordinateTypeName(int lineIndex)
{
	return m_motionDirectiveEditLineList[lineIndex]->GetCoordinateTypeName();
}

/**
* @biref　自身が含む動作指定編集列のうち、指定したインデックスの列の数値列を得るメンバ関数の実装です。
*/
vector<double> MotionDirectiveEditPart::GetMotionDirectiveValueList(int lineIndex)
{
	return  m_motionDirectiveEditLineList[lineIndex]->GetMotionDirectiveVaueList();
}

/**
* @biref　自身が含む動作指定編集列のうち、指定したインデックスの列の数値の数を得るメンバ関数です。
*/
int  MotionDirectiveEditPart::GetMotionDirectiveValueCount(int lineIndex)
{
	m_motionDirectiveEditLineList[lineIndex]->GetMotionDirectiveValueCount();
} 

/**
* @brief いずれかの行で動作指示の種類が変更された場合に発生させるシグナルを得るメンバ関数です。
*/
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
SignalProxy<signal<void(int, const QString&)> > MotionDirectiveEditPart::MotionDirectiveTypeChanged()
#else
SignalProxy<void(int, const QString&)> MotionDirectiveEditPart::MotionDirectiveTypeChanged()
#endif
{
	return m_motionDirectiveTypeChanged;
}

/**
 * @brief いずれかの行で動作指示の座標系が変更された場合に発生させるシグナルを得るメンバ関数です。
 */
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
SignalProxy<signal<void(int, const QString&)> > MotionDirectiveEditPart::CoordinateTypeChanged()
#else
SignalProxy<void(int, const QString&)> MotionDirectiveEditPart::CoordinateTypeChanged()
#endif
{
	return m_coordinateTypeChanged;
}

/**
 * @brief 動作指示の種類が変更された場合に発生するシグナルを
 * 受け取るスロットです。
 */
void MotionDirectiveEditPart::on_motionDirectiveTypeChanged(int lineCount, const QString& type)
{
	m_motionDirectiveTypeChanged(lineCount, type);
}

/**
 * @brief 座標系の種類が変更された場合に発生するシグナルを
 * 受け取るスロットです。
 */
void MotionDirectiveEditPart::on_coordinateTypeChanged(int lineCount, const QString& type)
{
	m_coordinateTypeChanged(lineCount, type);
}



