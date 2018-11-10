#include <QSpinBox>
#include <QLayout>

#include "MotionDirectiveValueEditLine.h"
#include "MotionDirectiveValueEditBox.h"

#include "MotionUtil.h"

#include<iostream>

using namespace std;
using namespace motionedit;

/**
 * @birsf コンストラクタの実装です。
 */
MotionDirectiveValueEditLine::MotionDirectiveValueEditLine(QWidget* parent) : QGridLayout(parent)
{}

/**
 * @birsf 数値設定ボックスの要素数を返すメンバ関数です。
 */
int MotionDirectiveValueEditLine::GetMotionDirectiveValueCount()
{
	return m_valueEditBoxList.size();
}

/**
 * @birsf 数値入力ボックスを全て削除するメンバ関数です。
 */
void MotionDirectiveValueEditLine::DeleteValueEditBoxList()
{
	// 数値設定ボックス列が空の場合は何もしません。
	if (m_valueEditBoxList.empty())
	{
		return;
	}
	int removingCount = m_valueEditBoxList.size();
	
	for (int i = 0; i < removingCount; i++)
	{
		MotionDirectiveValueEditBox* removingBox = m_valueEditBoxList[m_valueEditBoxList.size() - 1];
		m_valueEditBoxList.erase(--m_valueEditBoxList.end());
	}

	util::ClearLayout(this);

	return;
}

/**
 * @birsf 与えられた数値リストをもとに数値入力ボックスを作成するメンバ関数の実装です。
 */
void MotionDirectiveValueEditLine::MakeValueBoxes(const vector<double>& valueList)
{
	// 最初に全ての数値設定ボックスを削除します。
	this->DeleteValueEditBoxList();

	int pos = 0;

	for (int i = 0; i < valueList.size(); i++, pos += 2)
	{
		// 数値入力ボックスを作成し Id を与えます。
		// さらに、引数で与えられた数値をボックスに設定します。
		MotionDirectiveValueEditBox* valueBox = new MotionDirectiveValueEditBox(this->parentWidget(), i);
		{
			valueBox->setValue(valueList[i]);
		}

		// 数値設定ボックスを自身のフィールド
		// 及びレイアウトに追加します。
		m_valueEditBoxList.push_back(valueBox);
		addWidget(valueBox, 1, pos);

		addItem(new QSpacerItem(20, 20, QSizePolicy::Fixed), 1, pos + 1);
	}

	return;
}

/**
 * @birsf 数値設定ボックスの値を返すメンバ関数の実装です。
 */
double MotionDirectiveValueEditLine::GetValue(int i)
{
	return m_valueEditBoxList[i]->value();
}


