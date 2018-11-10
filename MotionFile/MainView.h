#ifndef MOTION_EDIT_MAIN_VIEW_H
#define MOTION_EDIT_MAIN_VIEW_H

#include <string>
#include <vector>

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <boost/intrusive_ptr.hpp>
#else
#include <cnoid/Referenced>
#endif
#include <boost/bind.hpp>

namespace cnoid
{
	class BodyItem;
}

class QWidget;
class QFrame;
class QScrollArea;

namespace motionedit
{
	class FileIOAndExecutingPart;

	class MotionDirectiveEditPart;

	class LineEditPart;

	class MotionDirectiveInfo;

	class IMotionDirectiveTypeConverter;

	class IUsingHandTypeConverter;

	class ICoordinateTypeConverter;

	class IRobotInMotionEdit;

	class IMotionFileReader;

	class IMotionDirectiveWordAnalyzer;

	class IMotionDirectiveInfoWriter;

         /**
 	 * @brief 動作指示編集タブのメインとなる部分のクラスです。 
	 * 内部に ファイル読み込み・書き込み・動作実行部、
	 * 動作指示編集部、
         * 動作指示行編集部を持ちます。
	 */
	class MainView : public QFrame
	{

	public:

                /**
                * @brief コンストラクタです。
		*/
		MainView(QWidget*);       

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		/**
	        * @biref ファイル読み込み用オブジェクトを設定するメンバ関数です。
		*/
		void SetReader(boost::intrusive_ptr<IMotionFileReader>);

                /**
		* @biref ファイル書き込み用オブジェクトを設定するメンバ関数です。
		*/
		void SetWriter(boost::intrusive_ptr<IMotionDirectiveInfoWriter>);

		/**
		* @biref 動作編修対象のロボットを設定するメンバ関数です。
		*/
		void SetRobot(boost::intrusive_ptr<IRobotInMotionEdit>);
#else
		/**
	        * @biref ファイル読み込み用オブジェクトを設定するメンバ関数です。
		*/
		void SetReader(cnoid::ref_ptr<IMotionFileReader>);

                /**
		* @biref ファイル書き込み用オブジェクトを設定するメンバ関数です。
		*/
		void SetWriter(cnoid::ref_ptr<IMotionDirectiveInfoWriter>);

		/**
		* @biref 動作編修対象のロボットを設定するメンバ関数です。
		*/
		void SetRobot(cnoid::ref_ptr<IRobotInMotionEdit>);
#endif

		/**
		* @brief 動作指示列のラインを追加するメンバ関数です。
		*/
		void InitializeByRobot();

	        /**
		* @brief 動作指示列のラインを追加するメンバ関数です。
		*/
		void AddLine(int);

                /**
		* @biref動作指示編集列をもとに動作指示情報のリストを作成するメンバ関数です。
		*/
		std::vector<MotionDirectiveInfo> MakeMotionDirectiveInfoList();
		
	private:

		/**
		* @brief 動作指示編修タブのうち
   		* ファイルの読み込み書き出し、動作の実行を行うボタンを持つ部分です。
		*/
		FileIOAndExecutingPart* m_FileIOAndExecutingPart;

		/**
		* @brief 動作指示編修タブのうち
		* 動作指示の編修を行う部分です。
		*/
		MotionDirectiveEditPart* m_motionDirectiveEditPart;

		/**
		* @brief 動作指示編集部を内部に格納するスクロールエリアです。
		*/
		QScrollArea* m_motionDirectiveEditPartScrollArea;

		/**
		* @brief 動作指示編修タブのうち
		*動作指示行の追加などの操作を行う部分です。
		*/
		LineEditPart* m_LineEditPart;
		
		/**
		* @ brief 読み込んだファイル名です。
		*/
		std::string m_loadFileName;

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		/**
		 * @biref 動作指示列の書かれたファイルを読み込むために使用するメンバです。
		 */
		boost::intrusive_ptr<IMotionFileReader> m_motionFileReader;

		/**
		* @biref 動作指示の種類とそれを表す文字列を相互に変換するためのメンバです。
		*/
		boost::intrusive_ptr<IMotionDirectiveTypeConverter> m_motionDirectiveTypeConverter;

		/**
		* @biref 使用する腕の種類とそれを表す文字列を相互に変換するためのメンバです。

		*/
		boost::intrusive_ptr<IUsingHandTypeConverter> m_usingHandTypeConverter;

		/**
		* @biref 座標系の種類を種類とそれを表す文字列を相互に変換するためのメンバです。

		*/
		boost::intrusive_ptr<ICoordinateTypeConverter> m_coordinateTypeConverter;

		/**

		* @biref 文字列を解析して腕の種類と動作指示の種類の文字列を得るためのメンバです。  
		*/
		boost::intrusive_ptr<IMotionDirectiveWordAnalyzer> m_motionDirectiveWordAnalyzer;

		/**
 		* @biref 動作指示の編集内容のファイル書き込みを行うメンバです。
		*/
		boost::intrusive_ptr<IMotionDirectiveInfoWriter> m_motionDirectiveInfoWriter;
#else
		/**
		 * @biref 動作指示列の書かれたファイルを読み込むために使用するメンバです。
		 */
		cnoid::ref_ptr<IMotionFileReader> m_motionFileReader;

		/**
		* @biref 動作指示の種類とそれを表す文字列を相互に変換するためのメンバです。
		*/
		cnoid::ref_ptr<IMotionDirectiveTypeConverter> m_motionDirectiveTypeConverter;

		/**
		* @biref 使用する腕の種類とそれを表す文字列を相互に変換するためのメンバです。

		*/
		cnoid::ref_ptr<IUsingHandTypeConverter> m_usingHandTypeConverter;

		/**
		* @biref 座標系の種類を種類とそれを表す文字列を相互に変換するためのメンバです。

		*/
		cnoid::ref_ptr<ICoordinateTypeConverter> m_coordinateTypeConverter;

		/**

		* @biref 文字列を解析して腕の種類と動作指示の種類の文字列を得るためのメンバです。  
		*/
		cnoid::ref_ptr<IMotionDirectiveWordAnalyzer> m_motionDirectiveWordAnalyzer;

		/**
 		* @biref 動作指示の編集内容のファイル書き込みを行うメンバです。
		*/
		cnoid::ref_ptr<IMotionDirectiveInfoWriter> m_motionDirectiveInfoWriter;
#endif

		/**
	  	* @ brief 与えられた動作指示情報リストをもとに
		* 自身の全ての動作指示列を更新するメンバ変数です。
		*/
		void Update(const std::vector<MotionDirectiveInfo>&);
		
		/**
		* @ 動作指示編集の対象とするロボットメンバ変数です。
		*/
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		boost::intrusive_ptr<IRobotInMotionEdit> m_robot;	
#else
		cnoid::ref_ptr<IRobotInMotionEdit> m_robot;	
#endif

	        /*******************************************
		 * スロット宣言部
                 ******************************************/		
		/**
		* @brief ロードボタンがクリックされた場合実行するスロットです。
		*/
		void on_loadButton_Clicked();

		/**
	 	* @brief セーブボタンがクリックされた場合に実行するスロットです。
		*/
		void on_saveButton_Clicked();

		/**
		* @brief プレイボタンがクリックされた場合に実行するスロットです。
		*/
		void on_playButton_Clicked();

		/**
		* @brief ライン追加ボタンがクリックされた場合に実行するスロットです。
		*/
		void on_appendButton_Clicked();

		/**
		* @brief いずれかの行で動作指示の種類が変更された場合に実行するスロットです。
		*/
		void on_motionDirectiveType_Changed(int, const QString&);

		/**
	 	* @brief いずれかの行で座標系の種類が変更された場合に実行するスロットです。
		*/
		void on_coordinateType_Changed(int, const QString&);
		
		/**
	 	* @brief ロボットアイテムバーで選択されているロボットが変更された場合に実行するスロットです。
		*/
		void on_bodyItem_Changed(cnoid::BodyItem*);
	};
}

#endif
