#ifndef MOTIONDIRECTIVE_EDIT_PART_H
#define MOTIONDIRECTIVE_EDIT_PART_H

#include <QWidget>
#include <QFrame>
#include <QLayout>
#include <QComboBox>
#include <QSpinBox>

#include <string>
#include <vector>
#include <stack>

#include "MotionDirectiveInfo.h"

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <boost/signal.hpp>
#include <boost/signals.hpp>
#include <cnoid/SignalProxy>
#else
#include <cnoid/Signal>
#endif

namespace motionedit
{
	class MotionDirectiveEditLine;
	
	class IRobotInMotionEdit;

	 /**
	 * @brief 動作指示列編集部を表すクラスです。 
         * 複数の動作指示編集列を持ちます。
	 */
	class MotionDirectiveEditPart :public QVBoxLayout
	{

	public:
		 /**
		 * @brief コンストラクタです。
		 */
		MotionDirectiveEditPart(QWidget*);

 		/**
	        * @brief 動作指示編集ラインの数を得るメンバ関数です。
		*/
		int GetLineCount();
		 
		/**	 	
	        * @biref 自身が含む動作指定編集列のうち、
	        * 指定したインデックスの列の動作開始時間を得るメンバ関数です。
	        */
                int GetStartTime(int);
     
               /**
               * @biref 自身が含む動作指定編集列のうち、
	       * 指定したインデックスの列の動作終了時間を得るメンバ関数です。
	       */
               int GetEndTime(int);

	       /**
               * @biref 自身が含む動作指定編集列のうち、
	       * 指定したインデックスの列の動作指定の種類の名前を得るメンバ関数です。
	       */
               std::string GetMotionDirectiveTypeName(int);

	       /**
               * @biref 自身が含む動作指定編集列のうち、
	       * 指定したインデックスの列の座標系の種類の名前を得るメンバ関数です。
	       */
               std::string GetCoordinateTypeName(int);   

	       /**
               * @biref 自身が含む動作指定編集列のうち、
	       * 指定したインデックスの列の座標系の種類の名前を得るメンバ関数です。
	       */
	       std::vector<double> GetMotionDirectiveValueList(int);
 
		/**
               * @biref 自身が含む動作指定編集列のうち、
	       * 指定したインデックスの列の数値列の数を得るメンバ関数です。
	       */
	       int GetMotionDirectiveValueCount(int);

 		 /**
	        * @brief 動作指示行を行末に 1 行追加するメンバ関数です。
		*/
		void AddLine();

		/**
	        * @brief 行末の動作指示行を 1 行削除するメンバ関数です。
	        */
		void DeleteLine();
		
		/**
		* @brief 与えられたロボットの情報を元にして動作指示編集部を初期化するメンバ関数です。
		*/
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		void  InitializeByRobot(boost::intrusive_ptr<IRobotInMotionEdit>);
#else
		void  InitializeByRobot(cnoid::ref_ptr<IRobotInMotionEdit>);
#endif
		/**
		* @brief 与えられた動作指示情報をもとに
		* 全ての動作指示編集ラインを更新するメンバ関数です。
		*/
		void Update(const std::vector<MotionDirectiveInfo>&);

 		/**
		* @biref ●
		*/
		void Update(int, double, double, std::string, std::string, const std::vector<double>&);	

		/**
		* @brief 全ての動作指示編集ラインを削除するメンバ関数です。
		*/
		void Clear();

		 /**
		* @biref 動作指示編集列の内容をもとに動作指示情報のリストを作成するメンバ関数です。
		*/
		std::vector<MotionDirectiveInfo> MakeMotionDirectiveInfoList();

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		/**
		* @brief いずれかの行で動作指示の種類が変更された場合に発生させるシグナルです。
		*/
		cnoid::SignalProxy<boost::signal<void(int, const QString&)> > MotionDirectiveTypeChanged();

		/**
		* @brief いずれかの行で動作指示の座標系が変更された場合に発生させるシグナルです。

		*/
		cnoid::SignalProxy<boost::signal<void(int, const QString&)> > CoordinateTypeChanged();
#else
		/**
		* @brief いずれかの行で動作指示の種類が変更された場合に発生させるシグナルです。
		*/
		cnoid::SignalProxy<void(int, const QString&)> MotionDirectiveTypeChanged();

		/**
		* @brief いずれかの行で動作指示の座標系が変更された場合に発生させるシグナルです。
		*/
		cnoid::SignalProxy<void(int, const QString&)> CoordinateTypeChanged();
#endif
	      
	private:

	        /**
		* @brief 動作指示編集列のリストです。
		*/
		std::vector<MotionDirectiveEditLine*> m_motionDirectiveEditLineList;

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		/**
		* @brief 自身が含む動作指示編集列のいずれかで動作指示内容が変更された場合に発生するシグナルです。
		*/
		boost::signal<void(int, const QString&)> m_motionDirectiveTypeChanged;

		/**
		* @brief 自身が含む動作指示編集列のいずれかで座標系の内容が変更された場合に発生するシグナルです。
		*/
		boost::signal<void(int, const QString&)> m_coordinateTypeChanged;
		
		/**
		* @brief 動作指示の内容が変更された場合の各列のシグナルとスロットの結合を格納するリストです。
		*/
		std::stack<boost::signals::connection> m_connectionStackOfMotionDirectiveTypeChangedSignal;
		
		/**
		* @brief 座標系の内容が変更された場合の各列のシグナルとスロットの結合を格納するリストです。
		*/
		std::stack<boost::signals::connection> m_connectionStackOfCoordinateTypeChangedSignal;
#else
		/**
		* @brief 自身が含む動作指示編集列のいずれかで動作指示内容が変更された場合に発生するシグナルです。
		*/
		cnoid::Signal<void(int, const QString&)> m_motionDirectiveTypeChanged;

		/**
		* @brief 自身が含む動作指示編集列のいずれかで座標系の内容が変更された場合に発生するシグナルです。
		*/
		cnoid::Signal<void(int, const QString&)> m_coordinateTypeChanged;
		
		/**
		* @brief 動作指示の内容が変更された場合の各列のシグナルとスロットの結合を格納するリストです。
		*/
		std::stack<cnoid::Connection> m_connectionStackOfMotionDirectiveTypeChangedSignal;
		
		/**
		* @brief 座標系の内容が変更された場合の各列のシグナルとスロットの結合を格納するリストです。
		*/
		std::stack<cnoid::Connection> m_connectionStackOfCoordinateTypeChangedSignal;
#endif
	        /**
		* @brief 座標系の名前と対応する数値列のマップです。
		*/
		std::map<std::string, std::vector<double> > m_motionAndValuesMap;

		/**
		* @brief 動作指示の種類が変更された場合に発生するシグナルを
		* 受け取るスロットです。
		*/
		void on_motionDirectiveTypeChanged(int, const QString&);

		/**
		* @brief 座標系の種類が変更された場合に発生するシグナルを
		* 受け取るスロットです。
		*/
		void on_coordinateTypeChanged(int, const QString&);
	};

}
#endif
