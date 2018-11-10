#ifndef MOTIONDIRECTIVEINFO_H
#define MOTIONDIRECTIVEINFO_H

#include <map>
#include <Eigen/Dense>

namespace motionedit
{
	/**
	 * @brief 座標系の種類を表す列挙体です。
	 */
	enum CoordinateType 
	{
		CoordinateType_None,
		CoordinateType_Abs,
		CoordinateType_Rel,
	};

	/**
	 * @brief 動作指示の種類を表す列挙体です。
	 */
	enum MotionDirectiveType 
	{
		MotionType_None,
		MotionType_Pause,
		MotionType_HeadForward,
		MotionType_Joint,	
		MotionType_Arm_Xyz,
		MotionType_Arm_Linear,
		MotionType_Arm_Jnt,
		MotionType_Arm_Object,
		MotionType_Hand_Jnt_Open,
		MotionType_Hand_Jnt_Close,
		MotionType_Hand_Grasp,
	};

	/**
	 * @brief 使用する腕の種類を表す列挙体です。
	 */
	enum UsingHandType 
	{
		UsingHandType_None,
		UsingHandType_Right,
		UsingHandType_Left,
	};

	/**
         * @brief 動作指示の内容を格納するクラスです。
         */
	class MotionDirectiveInfo
	{
	public:
	
		/**
	 　  * @brief 動作の開始時間を得るメンバ関数です。
	     */
		double GetStartTime() const;
		
		/**
	   　* @brief 動作の終了時間を得るメンバ関数です。
	　   */
		double GetEndTime() const;
		
		UsingHandType GetUsingHandType() const;

		/**
		 * @brief 動作指示の種類を得るメンバ関数です。
		 */
		MotionDirectiveType GetMotionDirectiveType() const;

		/**
		 * @brief 座標系を得るメンバ関数です。
		 */
		CoordinateType GetCoordinateType() const;

		/**
	     * @brief 動作指示の際に設定する値の数を得るメンバ関数です。
	     */
		int GetDirectiveValueCount() const;
		
		/**
	     * @brief 動作指示の際に設定する値のうち、index 番目の値を得るメンバ関数です。
	     */
		double GetDirectiveValue(int index) const;
		
		/**
		 * @brief 動作指示の際に設定する値をベクトルとして得るメンバ関数です。
		 */
		Eigen::VectorXd GetDirectiveValueVector() const;

		/**
		 * @brief 動作の開始時間をセットするメンバ関数です。
		 */
		void SetStartTime(double);

		/**
		 * @brief 動作の終了時間をセットするメンバ関数です。
		 */
		void SetEndTime(double);

		void SetUsingHandType(UsingHandType);

		/**
		 * @brief 動作指示の種類をセットするメンバ関数です。
		 */
		void SetMotionDirectiveType(MotionDirectiveType);

		/**
		 * @brief 座標系をセットするメンバ関数です。
		 */
		void SetCoordinateType(CoordinateType);

		/**
		 * @brief 動作指示の際に設定する値をまとめたベクトルをセットするメンバ関数です。
		 */
		void SetDirectiveValueVector(Eigen::VectorXd&);

		/**
		 * @brief 動作指示の際に設定する値をセットするメンバ関数です。
		 */
		void SetMotionDirectiveValue(int, double);

		/**
		 * @brief 自身の持つ情報を文字列に変換するメンバ関数です。
		 */
		std::string ToString();

	private:

		/**
		 * @brief 動作の開始時間です。
		 */
		double m_startTime;

		/**
		 * @brief 動作の終了時間です。
		 */
		double m_endTime;

		UsingHandType m_usingHandType;

		/**
	  　 * @brief 動作指示の種類です。
		 */
		MotionDirectiveType m_motionDirectiveType;

		/**
    	 * @brief 座標系です。
		 */
		CoordinateType m_coordinateType;

		/**
		 * @brief 動作指示の際に設定する値をまとめたベクトルです。
		 */
		Eigen::VectorXd m_directiveValueVector;
	};
}

#endif
