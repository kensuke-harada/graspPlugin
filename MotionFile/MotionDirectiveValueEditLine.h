#ifndef MOTIONDIRECTIVE_VALUE_EDIT_LINE_H
#define MOTIONDIRECTIVE_VALUE_EDIT_LINE_H

#include <vector>

#include <QWidget>
#include <QFrame>
#include <QLayout>
#include <QLineEdit>
#include <QPushButton>
#include <QComboBox>

#include <Eigen/Dense>

namespace motionedit
{
	class MotionDirectiveValueEditBox;

	/**
	 * @birsf 動作指示の数値列を表すクラスです。 
	 */
	class MotionDirectiveValueEditLine : public QGridLayout
	{
		//Q_OBJECT

	public:

		/**
		 * @birsf コンストラクタです。
		 */
		MotionDirectiveValueEditLine(QWidget* parent);

		/**
		 * @birsf 数値入力ボックスを全て削除するメンバ関数です。
		 */
		void DeleteValueEditBoxList();

		/**
		 * @birsf 与えられた数値リストをもとに数値入力ボックスを作成するメンバ関数です。
		 */
		void MakeValueBoxes(const std::vector<double>&);

		/**
		 * @birsf 数値設定ボックスの要素数を返すメンバ関数です。
		 */
		int GetMotionDirectiveValueCount();

		/**
		 * @birsf 数値設定ボックスの値を返すメンバ関数です。
		 */
		double GetValue(int);

		/**
		 * @birsf 列内の数値が変化された場合に発生させるシグナルです。
		 */
		void MotionDirectiveValueChanged(int, double);

	private:

		/**
		 * @birsf 数値設定ボックスのリストです。
	 	 */
		std::vector<MotionDirectiveValueEditBox* > m_valueEditBoxList;
	};
}


#endif
