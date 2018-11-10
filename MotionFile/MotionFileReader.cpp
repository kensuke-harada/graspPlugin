#include <iostream>
#include <fstream>
#include "MotionFileReader.h"
#include "MotionFileData.h"
#include <exception>

#include <iostream>

using namespace std;
using namespace motionedit;

/**
* @brief １ 単位の動作情報を表すパースされた文字列の中から、他の動作指示ファイルの名前を取り出すメンバ関数です。
*/
string MotionFileReader::Impl::GetMotionFileName(vector<vector<string> >& parsedStringLineList)
{
	// 想定されるファイル位置のパースされた文字列の数をえます。
	int fileNameCount = parsedStringLineList[motionFileNamePos].size();

	// 該当する文字列のまとまりが 1 でない場合は対象の文字列動作指示ファイル名を表しません。
	if (fileNameCount != 1)
	{
		// 他のファイル名としてから文字列を返します。
		return "";
	}

	// 他の動作指示ファイル名と考えられる文字列の 1 かたまりを返します。
	return parsedStringLineList[motionFileNamePos][0];
}

/**
* @brief パースされた文字列の複数のラインら動作指示情報リストを得るメンバ関数です。
*/
vector<MotionDirectiveInfo> MotionFileReader::Impl::GetMotionDirectiveInfoList(
	vector<vector<string> >& parsedStringLineList
	)
{
        // 文字列のラインのうち、動作指示情報を読み込むインデックスをえます。
	int motionDirectiveInfoStartPos = 0;
	{
                // 行の一番最初のラインの文字列のかたまりの数が 1 でない場合は
		// その行から動作指示情報を読み込んでいきます。
		//　一番最初のラインの文字列のかたまりの数が 1 の場合は
		// そのラインの単語は他の動作指示列の名前を表すと判断します。
		if(parsedStringLineList[motionFileNamePos].size() != 1)
		{
			motionDirectiveInfoStartPos = motionFileNamePos;
		}
	}

	// 最初の読み込み行に含まれる文字の固まりの数をえます、
	int fileNameCount = parsedStringLineList[motionFileNamePos].size();

	// 動作指示情報のリストを作成します。
	vector<MotionDirectiveInfo> motionDirectiveInfoList;

	// パースされた文字列のラインのうち上記で得られた動作指示情報読み込み開始インデックスから、
	// 各ラインを走査し、各ラインの文字のかたまりを動作指示情報データに変換していきます。
	for (vector<vector<string> > ::iterator iter = parsedStringLineList.begin() += motionDirectiveInfoStartPos;
		iter != parsedStringLineList.end();
		iter++)
	{
		if ((*iter).size() == 0)
		{
			continue;
		}

		motionDirectiveInfoList.push_back(toMotionDirectiveConverter->Convert(*iter));
	}

	return motionDirectiveInfoList;
}

/**
* @brief １ ファイル読み込みクラスのコンストラクタの実装です。
* パーサ と 文字列リストを動作指示情報データに変換するコンバータを引数に取ります。
*/
MotionFileReader::MotionFileReader(
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	boost::intrusive_ptr<ITextFieldParser> textFieldParser,
	boost::intrusive_ptr<IToMotionDirectiveInfoConverter> toMotionDirectiveInfoConverter
	):mImpl(boost::intrusive_ptr<Impl>(new Impl))
#else
	cnoid::ref_ptr<ITextFieldParser> textFieldParser,
	cnoid::ref_ptr<IToMotionDirectiveInfoConverter> toMotionDirectiveInfoConverter
	):mImpl(cnoid::ref_ptr<Impl>(new Impl))
#endif
{
	mImpl->textFieldParser = textFieldParser;
	mImpl->toMotionDirectiveConverter = toMotionDirectiveInfoConverter;
}

/**
* @brief 引数として与えられた文字列を名前として持つファイルを読み込み
* 動作指示情報と他の動作指示情報を記載するファイルの名前を得るメンバ関数の実装です。
* 読み込んだ文字列のパースとパースした文字列から動作指示情報を取り出す機能を
* 別のインターフェースに任せます。
* 読み込みに失敗した場合は例外を投げます。
*/
MotionFileData MotionFileReader::Read(std::string fileName)
{
	// 与えられたファイル名を持つファイルのストリームを開きます。
	ifstream stream(fileName.c_str());

	if (stream.fail())
	{
		throw std::exception();
	}
	
	// メンバのパーサー にストリームをセットします。
	mImpl->textFieldParser->SetStream(&stream);

	// パースされた文字列を格納するコンテナを作成します。
	vector<vector<string> > parsedStringLineList;

	try
        {
		// パーサーがファイルの終了点に達するまで各ラインを解析し、結果を用意したコンテナに格納していきます。
		while (!mImpl->textFieldParser->HasReachedEndOfData())
		{
			parsedStringLineList.push_back(mImpl->textFieldParser->ReadFiels());
		}
		 
		// ファイル読込結果を格納する変数を作成します。
		MotionFileData motionFileData;

		// ファイルの中に存在した動作指示ファイル名を格納します。(存在しない場合は空の文字列)
		motionFileData.motionFileName = 
			mImpl->GetMotionFileName(parsedStringLineList);
	
		// 得られた動作指示リストを格納します。
	 	motionFileData.motionDirectiveInfoList =
			mImpl->GetMotionDirectiveInfoList(parsedStringLineList);

		stream.close();

		return motionFileData;
        }
        catch(const std::exception& e)
        {
		stream.close();
		throw e;
        }
}


