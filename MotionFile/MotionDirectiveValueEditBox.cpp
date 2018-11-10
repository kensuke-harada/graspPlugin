#include "MotionDirectiveValueEditBox.h"
#include <QValidator>

using namespace motionedit;

/**
 * @brief コンストラクタの実装です。
 */
MotionDirectiveValueEditBox::MotionDirectiveValueEditBox(QWidget* parent, int id) :QDoubleSpinBox(parent)
{
	m_Id = id;

	// 入力ボックスから入力される値の上限と下限を設定します。
	const double limit = 1000.0;
	this->setRange(-limit, limit);

	// 入力ボックスの小数点以下の桁数です。
	const int decimals = 5;
	this->setDecimals(decimals);

	this->setSizePolicy(
		QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed)
		);
}

/**
 * @brief 自身の番号を得るメンバ関数の実装です。
 */
int MotionDirectiveValueEditBox::GetId()
{
	return m_Id;
}
