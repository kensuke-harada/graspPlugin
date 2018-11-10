#include "MotionUtil.h"

#include <QMessageBox>

#include <iostream>
#include <sstream>

using namespace std;
using namespace motionedit;

/**
* @brief 与えられた文字列リストの中から特定の文字リストに含まれる文字を検索する関数の実装です。
*/
size_t util::Find(
	const string& baseString,
	const vector<string>& searchWordList,
	std::string& outFoundWord
	)
{
	outFoundWord == "";

	for (vector<string>::const_iterator iter = searchWordList.begin(); iter != searchWordList.end(); iter++)
	{
		size_t foundPos = baseString.find(*iter);

		if (foundPos != string::npos)
		{
			outFoundWord = *iter;

			return foundPos;
		}
	}

	return string::npos;
}

/**
* @brief ベクトル型データをコンテナ型データに変換する関数の実装です。
*/
vector<double> util::Convert(const Eigen::VectorXd& valueVector)
{
	vector<double> valueList;

	for (int i = 0; i < valueVector.size(); i++)
	{
		valueList.push_back(valueVector[i]);
	}

	return valueList;
}

/**
* @brief std::string 型の文字列データを Qstring 型に変換する関数の実装です。
*/
QString util::ConvertToQString(const string& s)
{
	return QString::fromStdString(s);
}

/**
* @brief QString 型の文字列データを std::string 型に変換する関数の実装です。
*/
string util::ConvertToString(const QString& s)
{
	std::string current_locale_text = s.toLocal8Bit().constData();

	return current_locale_text;
}

/**
 * @brief フルパスからファイル名を取り出す関数の実装です。
 */
string util::GetFileName(const string &path)
{
	size_t pos1;

	pos1 = path.rfind('\\');
	if (pos1 != string::npos){
		return path.substr(pos1 + 1, path.size() - pos1 - 1);
	}

	pos1 = path.rfind('/');
	if (pos1 != string::npos){
		return path.substr(pos1 + 1, path.size() - pos1 - 1);
	}

	return path;
}

/**
* @brief レイアウトの中身をすべて削除する関数の実装です。
*/
void util::ClearLayout(QLayout* layout)
{

 QLayoutItem *item;
    while((item = layout->takeAt(0))) {
        if (item->layout()) {
            ClearLayout(item->layout());
            delete item->layout();
        }
        if (item->widget()) {
            delete item->widget();
        }
        delete item;
    }
}

string util::GetFormattedDoubleString(double value)
{
        int precision = 5;
	stringstream ss;
	ss << value;
	ss.precision(precision);
	return ss.str();
}

