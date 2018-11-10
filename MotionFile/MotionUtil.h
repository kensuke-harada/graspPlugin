#ifndef MOTIONUTIL_H
#define MOTIONUTIL_H

#include<QString>
#include<QLayout>

#include <string>
#include <vector>
#include <map>
#include <ostream>

#include<Eigen/Dense>

namespace motionedit
{
	// 動作指示編集プラグイン実装に使用するユーティリティ関数を定義した名前空間です。
	class util
	{
	private:
		util() { }
		virtual ~util() { }
	public:
		/**
		* @brief マップのキーのリストを得る関数です。
		*/
		template<typename T1, typename T2>
		static std::vector<T1> GetKeyList(const std::map<T1, T2>& map)
		{

			std::vector<T1> vector;

			for(typename std::map<T1, T2>::const_iterator iter = map.begin(); iter != map.end(); iter++)
			{
				vector.push_back(iter->first);
			}

			return vector;
		}

		/**
		* @brief 与えられた文字列リストの中から特定の文字リストに含まれる文字を検索する関数です。
		*/
		static size_t Find(const std::string&, const std::vector<std::string>&, std::string&);

		/**
		* @brief ベクトル型データをコンテナ型データに変換する関数です。
		*/
		static std::vector<double> Convert(const Eigen::VectorXd&);

		/**
		* @brief std::string 型の文字列データを Qstring 型に変換する関数です。
		*/
		static QString ConvertToQString(const std::string&);
		
		/**
		* @brief Qstring 型の文字列データを std::string 型に変換する関数です。
		*/
		static std::string ConvertToString(const QString&);

		/**
		* @brief 絶対パスの中からファイル名を取り出す関数です。
		*/
		static std::string GetFileName(const std::string&);

		/**
		* @brief レイアウトの中身をすべて削除する関数です。
		*/
		static void ClearLayout(QLayout*);

		static std::string GetFormattedDoubleString(double);
	};
}

#endif
