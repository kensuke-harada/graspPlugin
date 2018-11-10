#ifndef MOTIONDIRECTIVE_VALUE_EDIT_BOX_H
#define MOTIONDIRECTIVE_VALUE_EDIT_BOX_H

#include <QDoubleSpinBox>

namespace motionedit
{
	/**
	 * @brief 動作指示数値設定用のボックスクラスです。
	 */
	class MotionDirectiveValueEditBox :public QDoubleSpinBox
	{

	public:

		/**
		 * @brief コンストラクタです。
		 */
		MotionDirectiveValueEditBox(QWidget*, int);

		/**
		 * @brief 自身の番号を得るメンバ関数です。
		 */
		int GetId();

	private:

		/**
		 * @brief 自身の番号です。
		 */
		int m_Id;
	};
}

#endif
