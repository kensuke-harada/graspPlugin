#ifndef LINE_EDIT_PART_H
#define LINE_EDIT_PART_H

#include <QLayout>

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <cnoid/SignalProxy>
#include <boost/signal.hpp>
#include <boost/signals.hpp>
#else
#include <cnoid/Signal>
#endif

namespace cnoid
{
	class PushButton;
}

class QWidget;
class QLineEdit;
class QPushButton;
class QSpinBox;

namespace motionedit
{
	/**
	* @brief 動作指示編集ラインの追加を行う部分を表すクラスです。
	* 動作指示編集ラインを増加させるボタンを有します。 	 
	*/
	class LineEditPart : public QGridLayout
	{
	public:

		/**
		* @brief コンストラクタです。
		*/
		LineEditPart(QWidget*);
			
		 /**
		 * @brief 追加する行数を得るメンバ関数です。
		 */
		int GetAppendingLineCount();

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	         /**
		 * @brief 行追加ボタンが押された場合に発生するシグナルです。
		 */
		cnoid::SignalProxy<boost::signal<void(bool)> > AppendButtonClicked();
#elif defined(CNOID_15)
	         /**
		 * @brief 行追加ボタンが押された場合に発生するシグナルです。
		 */
		cnoid::SignalProxy<void(bool)> AppendButtonClicked();
#else
		cnoid::SignalProxy<void()> AppendButtonClicked();
#endif

	private:
		
		/**
		* @brief 行追加数を選択するためのスピンボックスです。
		*/
		QSpinBox* m_LineAppendingSpinBox;

		/**
		* @brief 行追加ボタンです。
		*/
		cnoid::PushButton* m_LineAppendingButton;

	};
}

#endif
