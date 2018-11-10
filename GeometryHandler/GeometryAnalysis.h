#ifndef __GEOMTRY_ANALYSIS__H
#define __GEOMTRY_ANALYSIS__H

#include "GeometryHandle.h"
#include "ObjectShape.h"
#include<Eigen/StdVector>

namespace grasp {

//一つの平面におけるくびれ部分を把持する際の情報
//例えば１つのPPCが，（1,2,3）３つの楕円体から成っており，くびれを構成している楕円体の組み合わせが（1-2, 1-3）の２つの場合，BoundaryCreateIdは{ {1,2}, {1,3} }となる
class BoundaryData : public Triangle
{
public:
	std::vector<int> idBoundary;		//どの二次曲面との交面なのか、二次曲面のidの格納
	double co[4];							//平面の式の係数
	int check;								//同じidBoundaryを持つかチェック
	int checkSort;							//同じ要素をもつかチェック
	
	std::vector<cnoid::Vector3> targetPoint;			//把持目標とする点
	std::vector<std::vector<cnoid::Vector3> > graspVec;	//approachVecとfingerVecに用いるベクトル;
	std::vector<std::vector<int> > BoundaryCreateId;		//くびれを生成している2つのidを格納
	std::vector<int> BoundaryShape;					//くびれを生成している二次曲面の形を格納　1->複数の楕円体・楕円柱面,2->一葉双曲面,0->その他

	void calcApproximatedPlane(int listNum, int id, std::vector<VertexLink*> &boundaryList);	//１周する（連続している）店群を平面近似する
}; 

class DepartData : public Triangle
{
public:
	std::vector<int> surface_id;
	std::vector<cnoid::VectorXd> surface_qc;
	std::vector<int> surface_shape;
	std::vector<cnoid::Vector3> meshe_center;	//メッシュの重心
	std::vector<cnoid::Vector3> boundary_center;	//メッシュの境界点群の重心
	std::vector< std::vector<cnoid::Vector3> > ellipseParameter;	//楕円の長軸，短軸ベクトルを格納

	std::vector< std::vector<cnoid::Vector3> > grasp_point;					//はさむ基準点
	std::vector< std::vector< std::vector<cnoid::Vector3> > > approachVec;	//グリッパをアプローチする直線のパラメータを格納
	std::vector< std::vector< std::vector<cnoid::Vector3> > > fingerVec;		//グリッパをはさむ直線のパラメータを格納
	

	void createLineEllEll(int id1, int id2, cnoid::Vector3& sub_midle_point, std::vector<cnoid::Vector3>& tmp_approachVec, std::vector<cnoid::Vector3>& tmp_fingerVec);	//楕円体-楕円体の組み合わせでアプローチ軸と開閉軸を導出
	void createLineEllCyl(int id1, int id2, cnoid::Vector3& sub_midle_point, std::vector<cnoid::Vector3>& tmp_approachVec, std::vector<cnoid::Vector3>& tmp_fingerVec);	//楕円体-楕円柱の組み合わせでアプローチ軸と開閉軸を導出
	void createLineEllHyp(int id1, int id2, cnoid::Vector3& sub_midle_point, std::vector<cnoid::Vector3>& tmp_approachVec, std::vector<cnoid::Vector3>& tmp_fingerVec);	//楕円体-一葉双極面の組み合わせでアプローチ軸と開閉軸を導出
	void createLineEllPla(int id1, int id2, cnoid::Vector3& sub_midle_point, std::vector<cnoid::Vector3>& tmp_approachVec, std::vector<cnoid::Vector3>& tmp_fingerVec);	//楕円体-平面の組み合わせでアプローチ軸と開閉軸を導出
	void createLineCylCyl(int id1, int id2, cnoid::Vector3& sub_midle_point, std::vector<cnoid::Vector3>& tmp_approachVec, std::vector<cnoid::Vector3>& tmp_fingerVec);	//楕円柱-楕円柱の組み合わせでアプローチ軸と開閉軸を導出
	void createLineCylHyp(int id1, int id2, cnoid::Vector3& sub_midle_point, std::vector<cnoid::Vector3>& tmp_approachVec, std::vector<cnoid::Vector3>& tmp_fingerVec);	//楕円柱-一葉双極面の組み合わせでアプローチ軸と開閉軸を導出
	void createLineCylPla(int id1, int id2, cnoid::Vector3& sub_midle_point, std::vector<cnoid::Vector3>& tmp_approachVec, std::vector<cnoid::Vector3>& tmp_fingerVec);	//楕円柱-平面の組み合わせでアプローチ軸と開閉軸を導出
	void createLineHypHyp(int id1, int id2, cnoid::Vector3& sub_midle_point, std::vector<cnoid::Vector3>& tmp_approachVec, std::vector<cnoid::Vector3>& tmp_fingerVec);	//一葉双極面-一葉双極面の組み合わせでアプローチ軸と開閉軸を導出
	void createLineHypPla(int id1, int id2, cnoid::Vector3& sub_midle_point, std::vector<cnoid::Vector3>& tmp_approachVec, std::vector<cnoid::Vector3>& tmp_fingerVec);	//一葉双極面-平面の組み合わせでアプローチ軸と開閉軸を導出
	void createLinePlaPla(int id1, int id2, cnoid::Vector3& sub_midle_point, std::vector<cnoid::Vector3>& tmp_approachVec, std::vector<cnoid::Vector3>& tmp_fingerVec);	//一葉双極面-平面の組み合わせでアプローチ軸と開閉軸を導出

}; 


class ClusterNeighbor
{
	std::vector<int> nbrId;		//隣のクラスタのidを格納するベクター
	int check;					//そのクラスタのidが既に格納されているかのフラグ
	void calcEllipseParameter( std::vector<cnoid::Vector3>& v1 );	//二次曲面を楕円で近似し，長軸，短軸，法線を導出する

	void calcApproximatedPlane(int listNum, int id, BoundaryData &sub_boundarydata);	//１周する（連続している）点軍を平面近似する
};


class ObjectAnalysis 
{
	public:
		
	ObjectAnalysis() { } 
	~ObjectAnalysis(){ } 
	
	ObjectShape *object;
	//uto
	void createBoundaryData();	//boundaryDateを作成する
	
	int calcClusterBoundaryNbr(ClusterImpl *c, std::vector<std::vector<VertexLink*> > &boundaryList);	//隣のidをVertexLink::nbrIdに格納する
	
	void createDepartData();	//departDataを作成する
	
	std::vector<BoundaryData> boundarydata;	//境界面の情報であるBoundaryを格納したベクター
	void sortBoundary();	//boundarydataの中身を整理する
	void calcConstriction(int dataNum);	//楕円体の組み合わせや一葉双曲面等の情報を計算し，くびれ判定を行う
									//くびれていると判断されたくびれの情報（2軸ベクトル，半径等）を格納する
	int checkQuadricSurface(cnoid::VectorXd quadCoefficient);	//二次曲面の形状を判定する
//	void calcBoundaryArea(double co[4], cnoid::VectorXd quadCoefficient, cnoid::Matrix3& rot, cnoid::Vector2& rad, cnoid::Vector3& gap, std::vector<double>& S);	//平面と二次曲面の共通領域を導出する関数
	void calcBoundaryArea(double co[4], cnoid::VectorXd quadCoefficient, cnoid::Vector3& sub_targetPoint, std::vector<cnoid::Vector3>& sub_graspVec, std::vector<double>& S);
	
//	void calcHyperboloid(cnoid::VectorXd quadCoefficient, cnoid::Matrix3& rot, cnoid::Vector2& rad, cnoid::Vector3& gap);	//一葉双曲面の情報を計算する関数
	void calcHyperboloid(cnoid::VectorXd quadCoefficient, cnoid::Vector3& sub_targetPoint, std::vector<cnoid::Vector3>& sub_graspVec);	//一葉双曲面の情報を計算する関数

	bool checkConstriction(std::vector<double>& S, std::vector<double>& d_S);	//くびれているかチェックする関数
	void calcBoundaryMainAxis(int id, cnoid::Vector3& v1,  cnoid::Vector3& v2);	//メッシュの境界点群を楕円に近似し,長軸,短軸を導出
	cnoid::Vector3 calcMesheCenter(int id);		//メッシュの重心を計算する
	cnoid::Vector3 calcBoundaryCenter(int id);	//メッシュの境界点群の重心を計算する
	
	void LimitedInitialClustersFromOneFace();	//ある限定的な範囲でObjectShape::initialClustersFromOneFaceを行う

/*
	case 'B':
		if(!object){
			loadObjectShape();
			//object->transform(getPointedSceneLink()->R,Vector3(0,0,0));
			object->initialClustersFromOneFace();
			//object->alpha = 0.5;
		}
		//loadObjectShape();
		//object->initialClustersFromOneFace();
//		cout << "check1" << endl;
		object->createBoundaryData();
//		cout << "check2" << endl;

		return handled;
*/

};
}
#endif
