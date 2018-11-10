#ifndef TEXTFILE_PARSER_H
#define TEXTFILE_PARSER_H

#include <iostream>
#include <string>
#include <vector>

#include <cnoid/Referenced>

namespace motionedit
{
	/**
	* @brief ストリーム内のデータをパースするインターフェースです。
	*　自身がメンバとして持つ区切り文字リスト及びコメントアウトトークンリストをもとに
        * 文字列のラインをパースしていきます。
	*/
	class ITextFieldParser : public cnoid::Referenced
	{
	public:

		ITextFieldParser(){ }

		/**
		 * @brief パースの対象となるストリームを設定する純粋仮想関数です。
		 */
		virtual void SetStream(std::istream*) = 0;

		/**
		 * @brief ストリーム内のデータをパースする純粋仮想関数です。
		 */
		virtual std::vector<std::string> ReadFiels() = 0;

		/**
		 * @brief ストリームの最後に到達したかどうかを判定する純粋仮想関数です。
		 */
		virtual bool HasReachedEndOfData() = 0;
	};

	/**
	 * @brief ストリーム内のデータをパースするクラスです。
	 */
	class TextFieldParser : public ITextFieldParser
	{
	public:

		TextFieldParser();

		/**
		 * @brief パースの対象となるストリームを設定するメンバ関数です。
		 */
		void SetStream(std::istream*);

		/**
		 * @brief ストリーム内のデータをパースするメンバ関数数です。
		 */
		std::vector<std::string> ReadFiels();

		/**
		 * @brief ストリームの最後に到達したかどうかを判定するメンバ関数です。
		 */
		bool HasReachedEndOfData();

		/**
		 * @brief デリミタを追加するメンバ関数です。
		 */
		void AddDelimiter(std::string);

		/**
		 * @brief コメント行であることを表す文字を追加するメンバ関数です。
		 */
		void AddCommentToken(std::string);

	private :

		/**
     　	 * @brief 実装クラスです。
		 */
		class Impl;

		/**
		 * @brief 実装へのポインタです。
		 */
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		boost::intrusive_ptr<Impl> mImpl;
#else
		cnoid::ref_ptr<Impl> mImpl;
#endif
	};
}

#endif
