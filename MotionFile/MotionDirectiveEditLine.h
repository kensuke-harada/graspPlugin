#ifndef MOTIONDIRECTIVE_EDIT_LINE_H
#define MOTIONDIRECTIVE_EDIT_LINE_H

#include <QWidget>
#include <QFrame>
#include <QLayout>
#include <QComboBox>
#include <QSpinBox>

#include <cnoid/ComboBox>
#include <boost/bind.hpp>

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <cnoid/SignalProxy>
#include <boost/signal.hpp>
#include <boost/signals.hpp>
#else
#include <cnoid/Signal>
#endif

#include "RobotInMotionEdit.h"

namespace motionedit
{
	class MotionDirectiveInfo;

	class MotionDirectiveValueEditLine;

	/**
	 * @brief 動作指示列を表すクラスです。
	 */
	class MotionDirectiveEditLine : public QFrame
	{

	public:
		
		/**
		 * @brief コンストラクタです。
		 */
		MotionDirectiveEditLine(QWidget*, int);

		MotionDirectiveEditLine(QWidget*, int, const std::map<std::string, std::vector<double> >& );

		/**
		 * @brief 自身の行番号を得るメンバ関数です。
		 */
		int GetId();

		/**
		 * @brief 動作開始時間を得るメンバ関数です。
		 */
		double GetStartTime();

		/**
		 * @brief 動作終了時間を得るメンバ関数です。
		 */
		double GetEndTime();

		std::string GetMotionDirectiveTypeName();

                std::string GetCoordinateTypeName();   

	        std::vector<double> GetMotionDirectiveVaueList();	

		int GetMotionDirectiveValueCount();

		/**
		 * @brief 動作開始時間を設定するメンバ関数です。
		 */
		void SetStartTime(double);

		/**
		 * @brief 動作指示の種類の名前を設定するメンバ関数です。
		 */
		void SetMotionDirectiveTypeName(const QString&);

		/**
		 * @brief 座標系の種類の名前を設定するメンバ関数です。
		 */
                void SetCoordinateTypeName(const QString&);

		/**
		 * @brief 動作終了時間を設定するメンバ関数です。

		 */
		void SetEndTime(double);

		/**
		 * @brief 与えられた動作指示情報をもとに
		 * 自身を更新するメンバ関数です。
		 */
		void Update(const MotionDirectiveInfo&);

		/**
		 * @brief 与えられた動作指示情報をもとに
		 * 動作指示情報を作成するメンバ関数です。
		 */
		MotionDirectiveInfo MakeMotionDirectiveInfo();

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		void InitializeByRobot(boost::intrusive_ptr<IRobotInMotionEdit>);
#else
		void InitializeByRobot(cnoid::ref_ptr<IRobotInMotionEdit>);
#endif

                std::vector<double> GetValueList();

		void Update(
			double startTime,
			double endTime,
			std::string motionDirectiveTypeName,
			std::string coordinateTypeName,
			const std::vector<double>& motionDirectiveValueList
			);
	//signals:
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		/**
		 * @brief 動作指示の種類が変更された場合に発生させるシグナルです。
		 */
		cnoid::SignalProxy<boost::signal<void(int, const QString&)> > MotionDirectiveTypeChanged();

		/**
		 * @brief 動作指示の座標系が変更された場合に発生させるシグナルです。
		 */
		cnoid::SignalProxy<boost::signal<void(int, const QString&)> > CoordinateTypeChanged();
#else
		/**
		 * @brief 動作指示の種類が変更された場合に発生させるシグナルです。
		 */
		cnoid::SignalProxy<void(int, const QString&)> MotionDirectiveTypeChanged();

		/**
		 * @brief 動作指示の座標系が変更された場合に発生させるシグナルです。
		 */
		cnoid::SignalProxy<void(int, const QString&)> CoordinateTypeChanged();
#endif
		
		void Initialize(QWidget*);

	//public slots:

		/**
		 * @brief 動作指示の種類が変更された場合に発生するシグナルを
		 * 受け取るスロットです。
		 */
		void on_motionDirectiveTypeChanged(const QString&);

		/**
		 * @brief 座標系の種類が変更された場合に発生するシグナルを
		 * 受け取るスロットです。
		 */
		void on_coordinateTypeChanged(const QString&);
	    
	private:

		/**
		 * @brief 自身の行番号です。
		 * 行番号は自身が動作指示編集ライン全体の上から何番目に位置するかを表します。
		 */
		int m_Id;

		/**
		 * @brief 動作指示の開始時間を設定するためのボックスです。
		 */
		QDoubleSpinBox* m_startTimeEdit;

		/**
		 * @brief 動作指示の終了時間を設定するためのボックスです。
	  	 */
		QDoubleSpinBox* m_endTimeEdit;
		
		/**
		 * @brief 動作指示の種類を設定するためのボックスです。
		 */
		cnoid::ComboBox* m_motionDirectiveNameBox;
		
		/**
		 * @brief 動作指示の座標系を設定するためのボックスです。
		 */
		cnoid::ComboBox* m_coordinateSystemNameBox;
		
                QString m_currentMotionDirectiveTypeName;
             
		/**
	  	 * @brief 動作指示の数値列です。
		 */
		MotionDirectiveValueEditLine* m_motionDirectiveValueEditLine;

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		boost::signal<void(int, const QString&)> m_motionDirectiveTypeChanged;
		boost::signal<void(int, const QString&)> m_coordinateTypeChanged;
#else
		cnoid::Signal<void(int, const QString&)> m_motionDirectiveTypeChanged;
		cnoid::Signal<void(int, const QString&)> m_coordinateTypeChanged;
#endif

		std::map<std::string, std::vector<double> > m_motionAndValuesMap;

		void on_motionDirectiveType_Changed(int);

	};
}
#endif
