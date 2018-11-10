#ifndef MOTIONEDIT_ROBOT_H
#define MOTIONEDIT_ROBOT_H

#include <vector>
#include <string>
#include <map>

#include <cnoid/Referenced>

#include "MotionDirectiveInfo.h"

namespace motionedit
{

	class IMotionDirectiveTypeConverter;

	class IUsingHandTypeConverter;

	/**
	 * @brief 動作編集の際に使用するロボットの情報を得るためのインターフェースです。
	 */
	class IRobotInMotionEdit : public cnoid::Referenced
	{
	public:

		virtual ~IRobotInMotionEdit(){}

		/**
		* @brief ロボットの名前を得る純粋仮想関数です。
		*/
		virtual std::string GetName() = 0;

		/**
		* @brief 使用可能な手の種類のリストを返す純粋仮想関数です。
		*/
		virtual std::vector<UsingHandType> GetHands() = 0;

		/**
		* @brief 動作の種類と、それぞれの動作に対応するデフォルトの数値列を対応付けたマップをかえす純粋仮想関数です。
		*/ 
		virtual std::map<std::string, std::vector<double> > GetMotionAndDefaultValuesMap() = 0;

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		/**
		 * @brief 動作の種類と対応する単語を相互に変換するコンバータを設定する純粋仮想関数です。 
		 */ 
		virtual void SetMotionDirectiveTypeConverter(boost::intrusive_ptr<IMotionDirectiveTypeConverter>) = 0;
		/**
		 * @brief 使用する腕の種類(右手、左手など) とそれに対応する単語を相互に変換するコンバータを設定する純粋仮想関数です。
		 */ 
		virtual void SetUsingHandTypeConverter(boost::intrusive_ptr<IUsingHandTypeConverter>) = 0;
#else
		/**
		 * @brief 動作の種類と対応する単語を相互に変換するコンバータを設定する純粋仮想関数です。 
		 */ 
		virtual void SetMotionDirectiveTypeConverter(cnoid::ref_ptr<IMotionDirectiveTypeConverter>) = 0;
		/**
		 * @brief 使用する腕の種類(右手、左手など) とそれに対応する単語を相互に変換するコンバータを設定する純粋仮想関数です。
		 */ 
		virtual void SetUsingHandTypeConverter(cnoid::ref_ptr<IUsingHandTypeConverter>) = 0;
#endif
 	};

	/**
	*  @brief デフォルトで動作編集タブに設定されるロボット情報クラスです。
	*/
	class DefaultRobotInMotionEdit : public IRobotInMotionEdit
	{
	public:
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		DefaultRobotInMotionEdit(boost::intrusive_ptr<IMotionDirectiveTypeConverter>, boost::intrusive_ptr<IUsingHandTypeConverter>);
#else
		DefaultRobotInMotionEdit(cnoid::ref_ptr<IMotionDirectiveTypeConverter>, cnoid::ref_ptr<IUsingHandTypeConverter>);
#endif
		/**
		* @brief  ロボットの名前を得る関数です。
		*/	
      		std::string GetName();
		/**
		* @brief ロボットの使用可能な手の種類を返す関数です。
		*/
		std::vector<UsingHandType> GetHands();
		/**
		* @brief 動作の種類と、それぞれの動作に対応するデフォルトの数値列を対応付けたマップを返す関数です。
		*/
		std::map<std::string, std::vector<double> > GetMotionAndDefaultValuesMap();
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		/**
		* @brief 動作の種類と対応する単語を相互に変換するコンバータを設定する関数です。
		*/
		void SetMotionDirectiveTypeConverter(boost::intrusive_ptr<IMotionDirectiveTypeConverter>);
		/**
		* @brief 使用する腕の種類(右手、左手など) とそれに対応する単語を相互に変換するコンバータを設定する関数です。
		*/
		void SetUsingHandTypeConverter(boost::intrusive_ptr<IUsingHandTypeConverter>);
#else
		/**
		* @brief 動作の種類と対応する単語を相互に変換するコンバータを設定する関数です。
		*/
		void SetMotionDirectiveTypeConverter(cnoid::ref_ptr<IMotionDirectiveTypeConverter>);
		/**
		* @brief 使用する腕の種類(右手、左手など) とそれに対応する単語を相互に変換するコンバータを設定する関数です。
		*/
		void SetUsingHandTypeConverter(cnoid::ref_ptr<IUsingHandTypeConverter>);
#endif

	private:
		/**
		* @brief 使用可能な腕のリストメンバです。
		*/
		std::vector<UsingHandType> m_handList;
		/**
		* @brief 動作の種類と、それぞれの動作に対応するデフォルトの数値列を対応付けたマップです。
		*/
		std::map<std::string, std::vector<double> > m_motionAndDefaultValueMap;
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		/**
		* @brief 動作の種類と対応する単語を相互に変換するコンバータを設定する関数です。
		*/
		boost::intrusive_ptr<IMotionDirectiveTypeConverter> m_motionDirectiveTypeConverter;
		/**
		* @brief 使用する腕の種類(右手、左手など) とそれに対応する単語を相互に変換するコンバータを設定する関数です。
		*/
		boost::intrusive_ptr<IUsingHandTypeConverter> m_usingHandTypeConverter;
#else
		/**
		* @brief 動作の種類と対応する単語を相互に変換するコンバータを設定する関数です。
		*/
		cnoid::ref_ptr<IMotionDirectiveTypeConverter> m_motionDirectiveTypeConverter;
		/**
		* @brief 使用する腕の種類(右手、左手など) とそれに対応する単語を相互に変換するコンバータを設定する関数です。
		*/
		cnoid::ref_ptr<IUsingHandTypeConverter> m_usingHandTypeConverter;
#endif
		//  ロボットの各部位の関節数です。
		/**
		* @brief すべての関節数です。
		*/
		const static int m_jointCount = 32;
		/**
		* @brief 右腕の関節数です。
		*/
		const static int m_rightArmJointCount = 6;
		/**
		* @brief 右手の関節数です。
		*/
		const static int m_rightHandJointCount = 10;
		/**
		* @brief 左腕の関節数です。
		*/
		const static int m_leftArmJointCount = m_rightArmJointCount;
		/**
		* @brief 左手の関節数です。
		*/
		const static int m_leftHandJointCount = m_rightHandJointCount;
	};

}
#endif
