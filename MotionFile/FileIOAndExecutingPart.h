#ifndef FILE_IO_AND_EXECUTING_PART_H
#define FILE_IO_AND_EXECUTING_PART_H

#include <cnoid/Button>

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <cnoid/SignalProxy>
#else
#include <cnoid/Signal>
#endif

#include <QLayout>


namespace cnoid
{
	class Button;
}

class QLabel;

namespace motionedit
{
	/**
	* @brief 動作指示編集タブのうち
	* ファイルの読み込み・書き込み・動作の実行の機能を持った部分を表すクラスです。
	*/
	class FileIOAndExecutingPart : public QGridLayout
	{

	public:

		/**
		* @brief コンストラクタです。
		*/
		FileIOAndExecutingPart(QWidget*);

		/**
		* @brief 読み込んだファイル名をファイル名表示部に設定するメンバ関数です、
		*/
		void SetLoadFileName(std::string);

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		/**
		* @brief ロードボタンが押された場合に発生するシグナルです。
		*/
		cnoid::SignalProxy<boost::signal<void(bool)> > LoadButtonClicked();
		
		/**
		* @brief セーブボタンが押された場合に発生するシグナルです。
		*/
		cnoid::SignalProxy<boost::signal<void(bool)> > SaveButtonClicked();

		/**
		* @brief プレイボタンが押された場合に発生するシグナルです。
		*/
		cnoid::SignalProxy<boost::signal<void(bool)> > PlayButtonClicked();
#elif defined(CNOID_15)
		/**
		* @brief ロードボタンが押された場合に発生するシグナルです。
		*/
		cnoid::SignalProxy<void(bool)> LoadButtonClicked();
		
		/**
		* @brief セーブボタンが押された場合に発生するシグナルです。
		*/
		cnoid::SignalProxy<void(bool)> SaveButtonClicked();

		/**
		* @brief プレイボタンが押された場合に発生するシグナルです。
		*/
		cnoid::SignalProxy<void(bool)> PlayButtonClicked();
#else
		/**
		* @brief ロードボタンが押された場合に発生するシグナルです。
		*/
		cnoid::SignalProxy<void()> LoadButtonClicked();
		
		/**
		* @brief セーブボタンが押された場合に発生するシグナルです。
		*/
		cnoid::SignalProxy<void()> SaveButtonClicked();

		/**
		* @brief プレイボタンが押された場合に発生するシグナルです。
		*/
		cnoid::SignalProxy<void()> PlayButtonClicked();
#endif

	private:

		/**
		* @brief 読み込んだファイル名を表示するラベルです。
		*/
		QLabel *m_fileNameDisplay;

		/**
		* @brief ロードボタンです。
		*/	
		cnoid::PushButton* m_loadButton;

		/**
		* @brief セーブボタンです。
		*/
                cnoid::PushButton* m_saveButton;
		
		/**
		* @brief プレイボタンです。
		*/
		cnoid::PushButton* m_playButton;
	};
}

#endif
