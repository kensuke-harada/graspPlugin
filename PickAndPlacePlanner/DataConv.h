/*
 * DataConv.h
 *
 * cnoid::Vector3 / Matrix3 と、RTCインターフェースの DblSequence3 / DblSequence9 を相互に変換するメソッド集
 *
 * 使い方:
 *  // const GraspPlanStart::DblSequence3& objPos, const GraspPlanStart::DblSequence9& objOri を、
 *  // cnoid::Vector3 pos, cnoid::Matrix3 ori に変換
 *	DataConv::toVector3(objPos, pos);
 *	DataConv::toMatrix3(objOri, ori);
 *
 *  // cnoid::Vector3 pos, cnoid::Matrix3 ori を、
 *  // GraspPlanResult::DblSequence3& objPos, GraspPlanResurt::DblSequence9& objOri に変換
 *	DataConv::toDs3<GraspPlanResult::DblSequence3>(pos, objPos);
 *	DataConv::toDs9<GraspPlanResurt::DblSequence9>(ori, objOri);
 *
 *  // テンプレートの型指定は省略もできるが、型チェックになるのでぜひ書くべき
 *
 *  Created on: 2011/07/15
 *      Author: ASAHI,Michiharu
 */

#ifndef DATACONV_H_
#define DATACONV_H_


namespace DataConv {
	/**
	 * cnoid::Vector3 と *::DblSequence3 を相互に変換する。
	 * @return 変換結果の参照
	 */
	template<typename T>
	static inline cnoid::Vector3& toVector3(T& src, cnoid::Vector3& dst) {
		for (int i = 0; i < 3; i++) {
			dst[i] = src[i];
		}
		return dst;
	}

	/**
	 * cnoid::Vector3 と *::DblSequence3 を相互に変換する。
	 * @return 変換結果の参照
	 */
	template<typename T>
	static inline T& toDs3(cnoid::Vector3& src, T& dst) {
		dst.length(3);
		for (int i = 0; i < 3; i++) {
			dst[i] = src[i];
		}
		return dst;
	}

	/**
	 * *::DblSequence9 を、cnoid::Matrix3 に変換する。
	 * @return 変換結果の参照
	 */
	template<typename T>
	static inline cnoid::Matrix3& toMatrix3(T& src, cnoid::Matrix3& dst) {
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				dst(i, j) = src[i*3+j];
			}
		}
		return dst;
	}

	/**
	 * cnoid::Matrix3 を、*::DblSequence9 に変換する。
	 * @return 変換結果の参照
	 */
	template<typename T>
	static inline T& toDs9(cnoid::Matrix3& src, T& dst) {
		dst.length(9);
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				dst[i*3+j] = src(i, j);
			}
		}
		return dst;
	}

}



#endif /* DATACONV_H_ */
