#include"LineEditPart.h"

#include <cnoid/Button>

#include <QLabel>
#include <QLayout>
#include <QMessageBox>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>

using namespace cnoid;
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
using namespace boost;
#endif
using namespace motionedit;

 /**
 * @brief コンストラクタの実装です。
 */
LineEditPart::LineEditPart(QWidget* parent) : QGridLayout(parent)
{
	// 行追加スピンボックスを作成します。
	m_LineAppendingSpinBox = new QSpinBox();
	{
		m_LineAppendingSpinBox->setSizePolicy(
			QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed)
			);
	}

	// 行追加ボタンを作成します。
	m_LineAppendingButton = new cnoid::PushButton("&Append");
	{
		m_LineAppendingButton->setSizePolicy(
			QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed)
			);
	}

	// 作成したウィジェットを自身に登録していきます。
	{
		addItem(new QSpacerItem(40, 20), 1, 0);
	
		addWidget(m_LineAppendingSpinBox, 1, 1);

		addItem(new QSpacerItem(40, 20, QSizePolicy::Preferred), 1, 2);

		addWidget(m_LineAppendingButton, 1, 3);

		addItem(new QSpacerItem(40, 20, QSizePolicy::Expanding), 1, 4);
	}
}

/**
* @brief 追加する行数を得るメンバ関数です。
*/
int LineEditPart::GetAppendingLineCount()
{
	return m_LineAppendingSpinBox->value();
}

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
/**
* @brief 行追加ボタンが押された場合に発生するシグナルです。
*/
SignalProxy<signal<void(bool)> >LineEditPart::AppendButtonClicked()
{
	return m_LineAppendingButton->sigClicked();
}
#elif defined(CNOID_15)
/**
* @brief 行追加ボタンが押された場合に発生するシグナルです。
*/
SignalProxy<void(bool)>LineEditPart::AppendButtonClicked()
{
	return m_LineAppendingButton->sigClicked();
}
#else
SignalProxy<void()>LineEditPart::AppendButtonClicked()
{
	return m_LineAppendingButton->sigClicked();
}
#endif


