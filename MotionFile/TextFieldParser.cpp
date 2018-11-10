
#include "TextFieldParser.h"
#include <sstream>
#include <boost/algorithm/string.hpp>   
#include <iostream>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>

using namespace std;
using namespace boost;
using namespace motionedit;

/**
 * @brief テキスト パーサ クラスの実装クラスです。
 */
class TextFieldParser::Impl : public cnoid::Referenced
{
public:

	/**
	 * @brief パースの対象となるストリームです。
	 */
	std::istream* istream;

	/**
	 * @brief デリミリタ文字の集合です。
	 */
	string delimiterList;

	/**
	 * @brief コメント行であることを表すトークン文字の集合です。
	 */
	string commentTokenList;

	/**
	 * @brief 与えられた文字列を分割した結果を返すメンバ関数です。
	 */
	vector<string> Split(string& line);

	/**
	 * @brief 与えられた文字列がコメント行を表すかどうかを判定するメンバ関数です。
	 */
	bool IsCommentLine(string& line);

	/**
	 * @brief 与えられた文字列がスキップすべき行かどうかを判定するメンバ関数です。
	 */
	bool IsSkipLine(string& line);
};

/***
 * @brief 与えられた文字列を区切り文字を用いて分割するメンバ関数の実装です。 
 */
vector<string> TextFieldParser::Impl::Split(string& line)
{
	vector<string> splitStringList;

	typedef char_separator<char>     BOOST_CHAR_SEP;
	typedef tokenizer< BOOST_CHAR_SEP > BOOST_TOKENIZER;

	BOOST_CHAR_SEP sep(delimiterList.c_str());
	BOOST_TOKENIZER tokens(line, sep);

	BOOST_FOREACH(string s, tokens) {
		splitStringList.push_back(s);
	}

	return splitStringList;
}

/***
 * @brief 与えられた文字列がコメント行を表すかどうかを判定するメンバ関数です。
 */
bool TextFieldParser::Impl::IsCommentLine(string& line)
{
	// ホワイトスペースではない最初の文字をえます。
	size_t pos = line.find_first_not_of(" \t\n");
	// 文字列に空白以外のものが見つからなかった場合はコメントラインではないと判定します。(スキップラインの可能性はあり)
	if (pos == string::npos)
	{
		return false;
	}
	// パーサーオブジェクトが持つコメント判定文字が見つかるかどうかで判定します。
	return commentTokenList.find(line[pos]) != string::npos;
}

/***
* @brief 与えられた文字列がスッキプラインかどうかを判定するメンバ関数です。
*/
bool TextFieldParser::Impl::IsSkipLine(string& line)
{
	// 一行に空白しか含まれていない場合はスキップする行と判定します。
	return line.find_first_not_of(" \t\n") == string::npos;
}

/***
* @brief 一連の文字列のパースを行うクラスです。
*/
TextFieldParser::TextFieldParser() : mImpl(new Impl())
{
	mImpl->istream = 0;
}

// パースの対象となるストリームをセットするメンバ関数の実装です。
void TextFieldParser::SetStream(istream* stream)
{
	mImpl->istream = stream;
}

// 区切り文字を追加するメンバ関数の実装です。
void TextFieldParser::AddDelimiter(string delimiiter)
{
	this->mImpl->delimiterList += delimiiter;
}

// コメント判定用の文字列を追加するメンバ関数の実装です。
void TextFieldParser::AddCommentToken(string comentToken)
{
	this->mImpl->commentTokenList += comentToken;
}

// ファイル読み込の最後の行に達したかどうかを判定するメンバ関数の実装です。
bool TextFieldParser::HasReachedEndOfData()
{
	return mImpl->istream->eof();
}

/***
* @brief パーサーの現在の位置にある文字列をパースした結果の文字列リストを返すメンバ関数です。
*/
vector<string> TextFieldParser::ReadFiels()
{
	// パースの対象となる文字列を格納する変数です。
	string line;

	// コメント行及び空白行ではない最初の行の文字列をえます。
	while (getline(*mImpl->istream, line))
	{
		if ((mImpl->IsSkipLine(line)) || (mImpl->IsCommentLine(line)))
		{
			continue;
		}
		break;
	}

	//読み込んだ行が空の場合はからのコンテナを返します。
	if (line.empty())
	{
		return vector<string>();
	}

	// 得られた文字列を分解したものを返します。
	return mImpl->Split(line);
}


