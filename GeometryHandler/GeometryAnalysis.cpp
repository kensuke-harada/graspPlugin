// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-

//#include </opt/intel/composerxe-2011.0.084/mkl/include/mkl.h>

#include "GeometryHandle.h"
#include <algorithm>
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include <iterator>
#include <list>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <Eigen/Eigenvalues>

#include "../Grasp/PlanBase.h"
#include "../Grasp/VectorMath.h"
#include "GeometryAnalysis.h"

using namespace grasp;
using namespace cnoid;
using namespace std;


//////////////////////////////////////////////////////////
///   createBoundaryDateボタンが押したときに実行する関数   ///
//////////////////////////////////////////////////////////
void ObjectAnalysis::createBoundaryData(){

	for(int i=0 ; i<object->nTriangles ; i++) object->clusterNodes[object->triangles[i].idCluster].triangleList.push_back(&object->triangles[i]);
	
	for(int i=0 ;i<object->nClusterNodes ; i++){
		if(object->clusterNodes[i].isAliveNode){
			
			std::vector<std::vector<VertexLink*> > boundaryList;
			//隣のidをVertexLink::nbrIdに格納する
			//boundaryList[2][○]->idBoundaryについて,boundaryが2周分あることを示しており，○は1周内に存在する境界点群の数
			calcClusterBoundaryNbr(&object->clusterNodes[i], boundaryList);
			
		
			//１周する（連続している）点群を平面近似する
			for(int j=0 ; j< boundaryList.size() ; j++){
				BoundaryData sub_boundarydata;
				for(int k=0 ; k<4 ; k++) sub_boundarydata.co[k]=0;
				sub_boundarydata.calcApproximatedPlane(j, i, boundaryList[j]);
//				cout << "co : " << sub_boundarydata.co[0] << "	" <<  sub_boundarydata.co[1] << "	" << sub_boundarydata.co[2] << "	" << sub_boundarydata.co[3] << endl;
				if(sub_boundarydata.idBoundary.size()>0) boundarydata.push_back(sub_boundarydata);
			}
		}
	}
	
	//メッシュ同士の境界情報を整理する
	/*idBoundaryに格納されているidが同じboundaryDataを消去する*/
	sortBoundary();
	
	///////////////////////////
	///   複数の楕円体について   ///
	///////////////////////////
	//くびれていると判断されたくびれの情報（2軸ベクトル，半径等）を格納する
	for(int i=0 ; i<boundarydata.size() ; i++) calcConstriction(i);

	/*
	//一葉双極面のデータ作成
	std::vector<int> hy_id;
#ifdef WIN32
	std::vector<cnoid::Vector2,Eigen::aligned_allocator<cnoid::Vector2> > hy_rad;
#else
	std::vector<cnoid::Vector2> hy_rad;
#endif
	std::vector<cnoid::Vector3> hy_gap;
	std::vector<cnoid::Matrix3> hy_rot;
	
	for(int i=0;i<object->nClusterNodes;i++){
		if(object->clusterNodes[i].isAliveNode){
			if(checkQuadricSurface(object->clusterNodes[i].quadCoefficient) ==3){
				Matrix3 tmp_rot;
				Vector2 tmp_rad;
				Vector3 tmp_gap;
				calcHyperboloid(object->clusterNodes[i].quadCoefficient, tmp_rot, tmp_rad, tmp_gap);
				
				hy_id.push_back(object->clusterNodes[i].id);
				hy_rot.push_back(tmp_rot);
				hy_rad.push_back(tmp_rad);
				hy_gap.push_back(tmp_gap);
			}
		}
	}
*/
	//////////////////////////
	///   ファイルへの書き込み   ///
	//////////////////////////
	cout << "surface dataをファイルに書き込みます" << endl;
	std::ofstream ofs1( "surfaceData.yaml" );
	
	for(int i=0;i<object->nClusterNodes;i++){
		if(object->clusterNodes[i].isAliveNode){
			ofs1 << "id : " << object->clusterNodes[i].id;
			if(checkQuadricSurface(object->clusterNodes[i].quadCoefficient) ==1) ofs1 << "	shape : ellipsoid" << endl;
			else if(checkQuadricSurface(object->clusterNodes[i].quadCoefficient) ==3) ofs1 << "	shape : hyperboloid" << endl;
			else ofs1 << "	shape : else" << endl;
		}
	}
	ofs1.close();
	cout << "surfaceData.yamlへの書き込みが完了しました" << endl << endl;

	

	cout << "boundary dataをファイルに書き込みます" << endl;
	std::ofstream ofs( "boundaryData.yaml" );
	VectorXd co(4);
	VectorXd rot(9);
	
	for(int i=0 ; i<boundarydata.size() ; i++){

		//CCPの情報を格納
		for(int k=0 ; k<4 ; k++) co[k] = boundarydata[i].co[k];
		ofs << "boundaryNum : " << i << endl;
		ofs << " Common Cutting Plan：\n    " << co.transpose() << endl << endl;
			
		//targetPointやgraspVecを書き込み
		for(int j=0 ; j<boundarydata[i].BoundaryShape.size() ; j++){
				
			ofs << " shapeNum : " << j << endl;
			if(boundarydata[i].BoundaryShape[j] == 1) ofs << "  shape : \n    ellipsoids" << endl << endl; 
			else if(boundarydata[i].BoundaryShape[j] == 2) ofs << "  shape : \n    hyperboloid" << endl << endl;

			ofs << "  id : \n    " << boundarydata[i].BoundaryCreateId[j][0] << "     " << boundarydata[i].BoundaryCreateId[j][1] << endl << endl;
			ofs << "  targetPoint : \n    " << boundarydata[i].targetPoint[j].transpose() << endl << endl;
			ofs << "  graspVec : " << endl;
			for(int k=0 ; k<boundarydata[i].graspVec[j].size() ; k++) ofs << "  " << boundarydata[i].graspVec[j][k].transpose() << endl;
			ofs << endl;
				
		}
	}
	
	ofs.close();
	cout << "boundaryData.yamlへの書き込みが完了しました" << endl;
}



///////////////////////////////////////////
///   隣のidをVertexLink::nbrIdに格納する   ///
///////////////////////////////////////////
int ObjectAnalysis::calcClusterBoundaryNbr(ClusterImpl *c, std::vector<std::vector<VertexLink*> > &boundaryList){
	Triangle *t,*tn;
	VertexLink *v;
	int check;
	
	std::vector<VertexLink*> boundaryVertices;

	for(unsigned int i=0;i<c->triangleList.size();i++){
		t = c->triangleList[i];
		for(int j=0;j<3;j++){
			tn = t->nbr[j];
#ifndef HEURISTICS_FOR_PCL 
			if(tn == NULL) continue;
#endif
			if(tn != NULL && tn->idCluster == c->id) continue;
			t->ver[j]->next = t->ver[(j+1)%3];
			boundaryVertices.push_back(t->ver[j]);
			t->ver[j]->check = c->id;
			
			t->ver[j]->nbrId = tn->idCluster;
/*			if(nbrId.size()==0) nbrId.push_back(tn->idCluster);
			else{
				check=0;
				for(unsigned int k=0 ; k<nbrId.size() ; k++)
					if(nbrId[k]==tn->idCluster) check=1;
				if(check==0) nbrId.push_back(tn->idCluster);
			}
*/		}
	}
	
	for(unsigned int k=0; k<boundaryVertices.size(); k++)
		boundaryVertices[k]->check = 1;

	boundaryList.clear();

	for(unsigned int k=0; k<boundaryVertices.size(); k++){

		v = boundaryVertices[k];
		vector<VertexLink*> b;
		
		while(v->check == 1){
			b.push_back(v);
			v->check = -1;
			v = v->next;
		}
		
		if(b.size()>0)
			boundaryList.push_back(b);
	}


	return 0;
}



////////////////////////////////////////////
///   １周する（連続している）店群を平面近似する   ///
////////////////////////////////////////////
void BoundaryData::calcApproximatedPlane(int listNum, int id, std::vector<VertexLink*> &boundaryList){
	for(int i=0 ; i<4 ; i++) co[i]=0;
	
	///////////////////////////////
	///   点群が閾値以上ならば近似   ///
	///////////////////////////////
//	cout << "host_id : " << id <<endl;
//	for(int i=0 ; i<boundaryList.size() ; i++) cout << "nbrId : " << boundaryList[i]->nbrId << endl;

	if(boundaryList.size()>10){
		idBoundary.push_back(id);
		unsigned int count=0;
		unsigned int i, j, k, l;
		int id_check=0;
		Vector3 center(0, 0, 0);
		Matrix3 VCM;
		
		///////////////////////////
		///   各点の平均座標を導出   ///
		///////////////////////////
		for(i=0 ; i<boundaryList.size() ; i++){
			center += boundaryList[i]->pos;
			
			/*境界を生成する二次曲面の全てのidをidBoundaryに格納する*/
			/*すでに格納されているidと*boundaryList[i]->nbrIdが同じならばスルー，新たなidなら格納*/
			for(j=0 ; j<idBoundary.size() ; j++)
				if(idBoundary[j] == boundaryList[i]->nbrId) check=1;
		
			if(check==0) idBoundary.push_back(boundaryList[i]->nbrId);
			check=0;
		}
		center = center/boundaryList.size();
		
		
		///////////////////////
		///   共分散行列を導出 ///
		///////////////////////
		VCM << 0, 0, 0, 0, 0, 0, 0, 0, 0;	//分散共分散行列
			

		for(i=0 ; i<3 ; i++)
			for(j=0 ; j<3 ; j++)
				for(k=0 ; k<boundaryList.size() ; k++) VCM(i, j)+=(center[i]-boundaryList[k]->pos[i])*(center[j]-boundaryList[k]->pos[j]);
		VCM = VCM/boundaryList.size();
		
		dmatrix evec = MatrixXd::Zero(3,3);
		dvector eval(3);
		double info = calcEigenVectors(VCM, evec, eval); //行列"VCM"の固有ベクトル"evec"と固有値"evel"を求める関数

	
		///////////////////////////////////////////////////////////////
		///   固有値を小さい順に導出し，その順どおりに固有ベクトルを並べ替える   ///
		///////////////////////////////////////////////////////////////
		Vector3 a[3];		//固有ベクトルの入れ物
		for(i=0 ; i<3 ; i++)
			for(j=0 ; j<3 ; j++) a[j][i] = evec(i, j);
		
		for(i=0 ; i<3 ; i++){
			j=i%2;
			if(eval[j] < eval[j+1]){
				double temp=eval[j];
				Vector3 tvec=a[j];
				
				eval[j] = eval[j+1];
				a[j] = a[j+1];
				eval[j+1] = temp;
				a[j] = tvec;
			}
		}
		
		
		/////////////////
		///   coに代入   ///
		/////////////////
		for(i=0 ; i<3 ; i++){
			co[i] = a[2][i];	//法線ベクトル
			co[3] += a[2][i]*center[i];
		}
	}
//	cout << "host_id : " << id << ", CCP : " << co[0] << "*x + ( " << co[1] << " )*y + ( " << co[2] << " )*z = " << co[3] << endl;
}



////////////////////////////////////
///   boundarydataの中身を整理する   ///
////////////////////////////////////
void ObjectAnalysis::sortBoundary(){
	
	for(int i=0 ; i<boundarydata.size() ; i++) boundarydata[i].checkSort = 1;

	//idBoundaryをソートする
	double tmp_id;
	for(int i=0 ; i<boundarydata.size() ; i++){
		for(int j=0 ; j<boundarydata[i].idBoundary.size()-1 ; j++){
			for(int k=j+1 ; k<boundarydata[i].idBoundary.size() ; k++){
				if(boundarydata[i].idBoundary[j] > boundarydata[i].idBoundary[k]){
					tmp_id = boundarydata[i].idBoundary[j];
					boundarydata[i].idBoundary[j] = boundarydata[i].idBoundary[k];
					boundarydata[i].idBoundary[k] = tmp_id;
				}
			}
		}
	}

	//かぶっているBoundaryDataについてcheckSortにフラグを立てる
	//checkSort==0：かぶりあり
	//chekcSort==1：かぶりなし
	int check;
	vector<int> checkoutNum;
	for(int i=0 ; i<boundarydata.size()-1 ; i++){
		for(int j=i+1 ; j<boundarydata.size() ; j++){
			if(boundarydata[i].idBoundary.size() == boundarydata[j].idBoundary.size()){
				check=0;
				for(int k=0 ; k<boundarydata[i].idBoundary.size() ; k++){
					if(boundarydata[i].idBoundary[k] != boundarydata[j].idBoundary[k]) check=1;
				}
				if(check==0){
					boundarydata[j].checkSort = check;
					break;
				}
			}
		}
	}

	//かぶってない要素を前に，かぶっている要素を後に移動する
	int count=0;
	BoundaryData tmp_boundary;

	for(int i=0 ; i<boundarydata.size() ; i++)
		if(boundarydata[i].checkSort == 1) count++;
	
	for(int i=0 ; i<boundarydata.size() ; i++){
		if(boundarydata[i].checkSort == 0){
			for(int j=boundarydata.size() ; j>=count ; j--){
				if(boundarydata[j].checkSort == 1){
					tmp_boundary = boundarydata[i];
					boundarydata[i] = boundarydata[j];
					boundarydata[j] = tmp_boundary;
				}
			}
		}
	}
					
	//後にまとめられたかぶっている要素を消去する
	count = boundarydata.size() - count;
	for(int j=0 ; j<count ; j++) boundarydata.pop_back();		
}



//////////////////////////////////////////////////////////////////
///   楕円体の組み合わせや一葉双曲面等の情報を計算し，くびれ判定を行う     ///
///   くびれていると判断されたくびれの情報（2軸ベクトル，半径等）を格納する   ///
//////////////////////////////////////////////////////////////////
void ObjectAnalysis::calcConstriction(int dataNum){

	///////////////////////////////////
	///   楕円体・楕円柱面かどうかの判定   ///
	///////////////////////////////////
	VectorXd quadCoefficient;
	vector<int> id_Ell;		//楕円体のidBoundaryの添字が格納
	vector<int> id_Hyp;		//一葉双曲面のidBoundaryの添字が格納

	for(int i=0 ; i<boundarydata[dataNum].idBoundary.size() ; i++){
		quadCoefficient = object->clusterNodes[boundarydata[dataNum].idBoundary[i]].quadCoefficient;
		if( checkQuadricSurface(quadCoefficient)==1 ) id_Ell.push_back(i);	//楕円体の場合はidQuadricSurfaceへidBoudaryの添字を格納
		else if( checkQuadricSurface(quadCoefficient)==3 ) id_Hyp.push_back(i);	//一葉双曲面の場合はidHyperboloidへidBoudaryの添字を格納
	}

	//楕円体が２つ以上あれば，把持姿勢生成のための情報を導出する
	if(id_Ell.size()>=2){
		for(int i=0 ; i<id_Ell.size()-1 ; i++){
			double co[4];
			Vector3 sub_targetPoint_1;
			vector<Vector3> sub_graspVec_1;
			vector<double> S;

			//１つ目の楕円体に対し，CCPとの交面の面積を求める
			quadCoefficient = object->clusterNodes[ boundarydata[dataNum].idBoundary[ id_Ell[i] ] ].quadCoefficient;
			for(int k=0 ; k<4 ; k++) co[k] = boundarydata[dataNum].co[k];
			calcBoundaryArea(co, quadCoefficient, sub_targetPoint_1, sub_graspVec_1, S);

			//２つ目の楕円体に対し，CCPとの交面の面積を求める
			for(int j=i+1 ; j<id_Ell.size() ; j++){
				Vector3 sub_targetPoint_2;
				vector<Vector3> sub_graspVec_2;
				vector<double> d_S;

				quadCoefficient = object->clusterNodes[ boundarydata[dataNum].idBoundary[ id_Ell[j] ] ].quadCoefficient;
				calcBoundaryArea(co, quadCoefficient, sub_targetPoint_2, sub_graspVec_2, d_S);

				//くびれ判定を行う
				if( checkConstriction(S, d_S) ){
					
					//CCPと楕円体の交面が楕円であることが前提
					if(S[1]!=0 && d_S[1]!=0){
						//targetPointとgraspVecを格納
						vector<Vector3> sub_graspVec;
						for(int k=0 ; k<2 ; k++) sub_graspVec.push_back( sub_graspVec_1[k] );						
						for(int k=0 ; k<2 ; k++) sub_graspVec.push_back( sub_graspVec_2[k] );						
						boundarydata[dataNum].targetPoint.push_back( (sub_targetPoint_1 + sub_targetPoint_2)/2 );
						boundarydata[dataNum].graspVec.push_back( sub_graspVec );
											
						std::vector<int> id_Ells; //くびれを生成している2つのidを格納
						id_Ells.push_back( boundarydata[dataNum].idBoundary[ id_Ell[i] ]);
						id_Ells.push_back( boundarydata[dataNum].idBoundary[ id_Ell[j] ]);
						boundarydata[dataNum].BoundaryCreateId.push_back( id_Ells );
						boundarydata[dataNum].BoundaryShape.push_back(1);
					}
				}
			}
		}
	}
			
	//一葉双曲面だったidに対して軸ベクトル,長径,短径,中心からのズレ,接触面積を導出
	if(id_Hyp.size()>0){
		for(int i=0 ; i<id_Hyp.size() ; i++){

			double co[4];
			Vector3 sub_targetPoint;
			vector<Vector3> sub_graspVec;

			//一葉双曲面と平面の接触領域を導出する
			quadCoefficient = object->clusterNodes[ boundarydata[dataNum].idBoundary[ id_Hyp[i] ] ].quadCoefficient;
			for(int k=0 ; k<4 ; k++) co[k] = boundarydata[dataNum].co[k];
			calcHyperboloid(quadCoefficient, sub_targetPoint, sub_graspVec);

			boundarydata[dataNum].targetPoint.push_back(sub_targetPoint);
			boundarydata[dataNum].graspVec.push_back(sub_graspVec);

			vector<int> sub_id_Hyp;
			sub_id_Hyp.push_back( boundarydata[dataNum].idBoundary[ id_Hyp[i] ] );
			sub_id_Hyp.push_back(-1);
			boundarydata[dataNum].BoundaryCreateId.push_back(sub_id_Hyp);
			boundarydata[dataNum].BoundaryShape.push_back(2);
			
		}
	}	
}



/////////////////////////////////
///   楕二次曲面の形状を判定する   ///
/////////////////////////////////
//1：楕円体
//2：楕円柱
//3：一葉双曲面
//4:平面
//0：その他
int ObjectAnalysis::checkQuadricSurface(cnoid::VectorXd quadCoefficient){

	Matrix3 M3;
	Vector3 ghj;
	double k;
	dmatrix evec(3, 3);
	dvector eval(3);
	int info;

	bool allzero=true;
	for(int k=0 ; k<6 ; k++)
		if( fabs(quadCoefficient[k]) > 1.0e-10) allzero = false;	//最初の6項全てが"1.0e-10"より小さければ"allzero"は"true"
	if(allzero==true) return 4;

	
	M3 << quadCoefficient[0], quadCoefficient[3]/2, quadCoefficient[5]/2, quadCoefficient[3]/2, quadCoefficient[1], quadCoefficient[4]/2, quadCoefficient[5]/2, quadCoefficient[4]/2, quadCoefficient[2];
	ghj << quadCoefficient[6], quadCoefficient[7], quadCoefficient[8];
	k = quadCoefficient[9];
	info = calcEigenVectors(M3, evec, eval);
	
	ghj = ghj.transpose() * evec;
	k = ghj[0]*ghj[0]/(4*eval[0]) + ghj[1]*ghj[1]/(4*eval[1]) + ghj[2]*ghj[2]/(4*eval[2]) - k;
	
//	cout << eval[0] << "*x^2 + ( " << eval[1] << " )*y^2 + ( " << eval[2] << " )*z^2 = " << k << endl << endl;
//	for(int i=0 ; i<3 ; i++) cout << "eval[" << i << "] : " << eval[i] << endl;

	//evalが小さい順に添字を格納する
	list<int> num;
	for(int i=0 ; i<3 ; i++){
		list<int>::iterator  it = num.begin();
	
		if(i==0) num.push_back(0);
		else{
			if( fabs(eval(*it)) < fabs(eval[i]) ) num.push_back( i ) ;
			else if( fabs(eval(*it)) > fabs(eval[i]) ) num.push_front( i );
		}
	}
	
	int min, mid;
	list<int>::iterator  it = num.begin();
	for(int i=0 ; i<3 ; i++){
//		cout << *it << endl;
		if(i==0) min = *it;
		else if( i==1 ) mid = *it;
		++it;
	}

	
	if( (eval[0]/k>0) && (eval[1]/k>0) && ( fabs(eval[2])/fabs(eval[mid]) < 0.001) ) return 2;
	else if( (eval[0]/k>0) && ( fabs(eval[1])/fabs(eval[mid]) < 0.001 ) && (eval[2]/k>0) ) return 2;
	else if( ( fabs(eval[0])/fabs(eval[mid]) < 0.001 ) && (eval[1]/k>0) && (eval[2]/k>0) ) return 2;
	else if( eval[0]/k>0 && eval[1]/k>0 && eval[2]/k>0 ) return 1;
	else if( (eval[0]/k>0) && (eval[1]/k>0) && (eval[2]/k<0) ) return 3;
	else if( (eval[0]/k>0) && (eval[1]/k<0) && (eval[2]/k>0) ) return 3;
	else if( (eval[0]/k<0) && (eval[1]/k>0) && (eval[2]/k>0) ) return 3;
	else return 0;
}



////////////////////////////////////////////
///   平面と二次曲面の共通領域を導出する関数   ///
////////////////////////////////////////////
void ObjectAnalysis::calcBoundaryArea(double co[4], cnoid::VectorXd quadCoefficient, Vector3& sub_targetPoint, std::vector<cnoid::Vector3>& sub_graspVec, std::vector<double>& S){

	////////////////////////////////////////////////////////////////////////
	///   平面の法線ベクトルを(0,0,1)へ回転 & 同様の回転行列を用いて二次曲面も回転   ///
	////////////////////////////////////////////////////////////////////////

	//平面の法線ベクトルを（0,0,1）へ回転させる行列の導出
	double sita=0.0;
	Vector3 vec1, vec2, vec3;
	Matrix3 rod;
	vec1 << co[0], co[1], co[2];
	vec2 << 0, 0, 1;
	vec3 = vec1.cross(vec2);
	rod << 1, 0, 0, 0, 1, 0, 0, 0, 1;

//	cout << "before_vec1：" << vec1 << endl;
	if(!( fabs(vec1[0])<1.0e-5 && fabs(vec1[1])<1.0e-5 && vec1[2]==1)){ 
		sita = acos(  dot(vec1, vec2)  / ( norm2(vec1)*norm2(vec2) ) );
		vec3 = vec3 / norm2(vec3);

		rod = rodrigues(vec3, sita);
		vec1=rod*vec1;
	}
//	cout << "after_vec1：" << vec1 << endl;
	
	//二次曲線の回転
	Vector3 ghj, sub_ghj;
	Matrix3 M3, sub_M3;
	double k, w;
	ghj << quadCoefficient[6], quadCoefficient[7], quadCoefficient[8];
	M3 <<  quadCoefficient[0], quadCoefficient[3]/2, quadCoefficient[5]/2, quadCoefficient[3]/2, quadCoefficient[1], quadCoefficient[4]/2, quadCoefficient[5]/2, quadCoefficient[4]/2, quadCoefficient[2];

	sub_M3 = rod * M3 * rod.transpose();
	sub_ghj = ghj.transpose() * rod.transpose();
//	cout << "変換後rot" << endl << d_rod.inverse();
	
//	for(int i=0 ; i<4 ; i++) cout << "co[" << i << "]：" << co[i] << endl;

	
	//////////////////////////////////////
	///   平面と二次曲面の接触領域面を導出   ///
	//////////////////////////////////////

	for(int i=0 ; i<3 ; i++){
		//くびれ判別を行うために３つの平面を準備する
		if(i==0) w=co[3]+0.001;
		if(i==1) w=co[3];
		if(i==2) w=co[3]-0.001;

		//z'=wを代入
		M3 << sub_M3(0,0), (sub_M3(0,1)+sub_M3(1,0))/2, 0, (sub_M3(0,1)+sub_M3(1,0))/2, sub_M3(1,1), 0, 0, 0, 0;
		ghj[0] = (sub_M3(2,0)+sub_M3(0,2))*w+sub_ghj[0];
		ghj[1] = (sub_M3(2,1)+sub_M3(1,2))*w+sub_ghj[1];
		ghj[2] = 0;
		k = sub_M3(2,2)*w*w + sub_ghj[2]*w + quadCoefficient[9];



		//接触領域面から楕円の場合を抽出
		Vector2 de,sub_de;
		Matrix2 M2,sub_M2;
		dmatrix evec(2, 2);
		dvector eval(2);
		int info;

		sub_de << ghj[0], ghj[1];
		sub_M2 << M3(0,0), M3(0,1), M3(1,0), M3(1,1);
		info = calcEigenVectors(sub_M2, evec, eval);
	
		de[0] = sub_de[0]*evec(0,0) + sub_de[1]*evec(1,0);
		de[1] = sub_de[0]*evec(0,1) + sub_de[1]*evec(1,1);
		k = k - de[0]*de[0]/(4*eval[0]) - de[1]*de[1]/(4*eval[1]);
	

		//平面を動かしていないときはSに加えてaxis,gap,radもそれぞれ求める
		if(i==1){

			//接触領域が楕円でない場合
			if( (-1.0*k/eval[0])<0 || (-1.0*k/eval[1])<0 ){
				sub_targetPoint << 0, 0, 0;
				S.push_back(0);
			}

			//接触領域が楕円の場合
			else{ 
				Vector3 x(1, 0, 0);
				Vector3 y(0, 1, 0);
				Vector3 gap;
				Matrix3 ext_evec;
				
				gap << -de[0]/(2*eval[0]), -de[1]/(2*eval[1]), co[3];
				ext_evec << evec(0,0), evec(0,1), 0, evec(1,0), evec(1,1), 0, 0, 0, 1;

				//x軸,y軸合わせのために回転させた分(evec)逆回転				
				x = ext_evec*x;
				y = ext_evec*y;
				gap = ext_evec * gap;

				//平面,二次曲面のz軸合わせのために回転させた分逆回転
				x = rod.inverse()*x;
				y = rod.inverse()*y;
				sub_graspVec.push_back(x);
				sub_graspVec.push_back(y);
				sub_targetPoint = rod.inverse()*gap;
				

				//面積の導出
				S.push_back( sqrt(-1.0*k/eval[0])*sqrt(-1.0*k/eval[1]) );
			}
		}		
	
		//平面を動かしたときは面積のみを求める
		else{

			//接触領域が楕円でない場合
			if( (-1.0*k/eval[0])<0 || (-1.0*k/eval[1])<0 ) S.push_back(0);
	
			//接触領域が楕円の場合
			else S.push_back( sqrt(-1.0*k/eval[0])*sqrt(-1.0*k/eval[1]) );
		}
	}
}



////////////////////////////////////
///   一葉双曲面の情報を計算する関数   ///
////////////////////////////////////
void ObjectAnalysis::calcHyperboloid(cnoid::VectorXd quadCoefficient, Vector3& sub_targetPoint, std::vector<cnoid::Vector3>& sub_graspVec){
	double k=0;
	Vector3 ghj;
	Matrix3 M3;
	
	M3 << quadCoefficient[0], quadCoefficient[3]/2, quadCoefficient[5]/2, quadCoefficient[3]/2, quadCoefficient[1], quadCoefficient[4]/2, quadCoefficient[5]/2, quadCoefficient[4]/2, quadCoefficient[2];
	ghj << quadCoefficient[6], quadCoefficient[7], quadCoefficient[8];
	k = quadCoefficient[9];

	//一葉双曲面の中心の軸を合わせる
	dmatrix evec(3, 3);
	dvector eval(3);
	int info;
	info = calcEigenVectors(M3, evec, eval);
	if(evec.determinant()<0) evec = -evec;
		
	ghj = ghj.transpose() * evec;
	k = k-(ghj[0]*ghj[0])/(4*eval[0])-(ghj[1]*ghj[1])/(4*eval[1])-(ghj[2]*ghj[2])/(4*eval[2]);

	Vector3 gap;
	gap << -ghj[0]/(2*eval[0]), -ghj[1]/(2*eval[1]), -ghj[2]/(2*eval[2]);
	sub_targetPoint = evec*gap;
//	rot = evec;

//	for(int i=0 ; i<3 ; i++) cout << "-k/eval[" << i << "]：" << endl << -k/eval[i] << endl;

	/////////////////////////////////////
	///   中心にある楕円の半径を格納   ///
	/////////////////////////////////////
	Vector3 x(1, 0, 0); 
	Vector3 y(0, 1, 0); 
	Vector3 z(0, 0, 1);
	
	if(-k/eval[0]<0){
		sub_graspVec.push_back(evec*y);
		sub_graspVec.push_back(evec*z);
	}

	else if(-k/eval[1]<0){
		sub_graspVec.push_back(evec*x);
		sub_graspVec.push_back(evec*z);
	}
		
	else if(-k/eval[2]<0){
		sub_graspVec.push_back(evec*x);
		sub_graspVec.push_back(evec*y);
	}
		
}



//////////////////////////////////
///   くびれているかチェックする関数   ///
//////////////////////////////////
bool ObjectAnalysis::checkConstriction(std::vector<double>& S, std::vector<double>& d_S){

//	for(int i=0 ; i<3 ; i++) cout << "S[" << i << "]：" << S[i] << endl;
//	for(int i=0 ; i<3 ; i++) cout << "d_S[" << i << "]：" << d_S[i] << endl;

	if(S[0]>=S[1] || S[1]>=S[2]){
		if(d_S[2]>=d_S[1] || d_S[1]>=d_S[0]) return true;
		else return false;
	}
	else if(S[2]>=S[1] || S[1]>=S[0]){
		if(d_S[0]>=d_S[1] || d_S[1]>=d_S[2]) return true;
		else return false;
	}


}

///////////////////////////////////////////////////////
///   createDepartDateボタンが押したときに実行する関数   ///
///////////////////////////////////////////////////////
void ObjectAnalysis::createDepartData(){
	int i, j, k;
	DepartData departData;
	
	for(i=0 ; i<object->nTriangles ; i++)
		object->clusterNodes[object->triangles[i].idCluster].triangleList.push_back(&object->triangles[i]);
	
	
	//二次曲面のidと形状の格納
	for(i=0;i<object->nClusterNodes;i++){
		if(object->clusterNodes[i].isAliveNode){
			
//			for(j=0 ; j<object->clusterNodes[i].triangleList.size() ; j++) cout << " id : " <<object->clusterNodes[i].triangleList[j]->id << endl;
			
			vector<Vector3> v1_ellipseParameter;
			Vector3 v1,v2;
//			object->clusterNodes[i].calcMainAxis(v1,v2);
			calcBoundaryMainAxis(object->clusterNodes[i].id, v1, v2);
			v1_ellipseParameter.push_back(v1);
			v1_ellipseParameter.push_back(v2);
			
			departData.surface_id.push_back( object->clusterNodes[i].id );
			departData.surface_qc.push_back( object->clusterNodes[i].quadCoefficient );
			departData.surface_shape.push_back( checkQuadricSurface(object->clusterNodes[i].quadCoefficient) );
			departData.boundary_center.push_back( calcBoundaryCenter(object->clusterNodes[i].id) );
			departData.meshe_center.push_back( calcMesheCenter(object->clusterNodes[i].id) );
			departData.ellipseParameter.push_back( v1_ellipseParameter );
	
//			cout << "shape[" << i << "] : " << checkQuadricSurface(clusterNodes[i].quadCoefficient) << endl;
		}
	}
	
//	for(i=0 ; i<departData.surface_id.size() ; i++) cout << "shape[" << i << "] : " << departData.surface_shape[i] << endl;

	
	//グリッパのアプローチ軸と開閉軸の導出
	//引数のiの順番に注意
	//Ell -> 1, Cyl -> 2, Hyp ->3
	cnoid::Vector3 v0_approachVec;
	cnoid::Vector3 v0_fingerVec;
	v0_approachVec << 0, 0, 0;
	v0_fingerVec << 0, 0, 0;
	
	for(i=0 ; i<departData.surface_id.size() ; i++){
		std::vector< cnoid::Vector3 > v1_grasp_point;
		std::vector< std::vector<cnoid::Vector3> > v2_approachVec;
		std::vector< std::vector<cnoid::Vector3> > v2_fingerVec;
		
//		cout << endl << "departData.surface_shape[" << i << "]1 : " <<departData.surface_shape[i] << endl;

		for(j=i+1 ; j<departData.surface_id.size() ; j++){
			cnoid::Vector3 grasp_point;
			std::vector<cnoid::Vector3> v1_approachVec;
			std::vector<cnoid::Vector3> v1_fingerVec;
			
//			cout << "departData.surface_shape[" << j << "]2 : " <<departData.surface_shape[j] << endl;

			
			if(departData.surface_shape[i]==1){
				if(departData.surface_shape[j]==1) departData.createLineEllEll(i, j, grasp_point, v1_approachVec, v1_fingerVec);
				else if(departData.surface_shape[j]==2) departData.createLineEllCyl(i, j, grasp_point, v1_approachVec, v1_fingerVec);
				else if(departData.surface_shape[j]==3) departData.createLineEllHyp(i, j, grasp_point, v1_approachVec, v1_fingerVec);
				else if(departData.surface_shape[j]==4) departData.createLineEllPla(i, j, grasp_point, v1_approachVec, v1_fingerVec);
				else if(departData.surface_shape[j]==0){
					v1_approachVec.push_back( v0_approachVec );
					v1_fingerVec.push_back( v0_fingerVec );
				}
			}
			else if(departData.surface_shape[i]==2){
				if(departData.surface_shape[j]==1) departData.createLineEllCyl(j, i, grasp_point, v1_approachVec, v1_fingerVec);
				else if(departData.surface_shape[j]==2) departData.createLineCylCyl(j, i, grasp_point, v1_approachVec, v1_fingerVec);
				else if(departData.surface_shape[j]==3) departData.createLineCylHyp(i, j, grasp_point, v1_approachVec, v1_fingerVec);
				else if(departData.surface_shape[j]==4) departData.createLineCylPla(i, j, grasp_point, v1_approachVec, v1_fingerVec);
				else if(departData.surface_shape[j]==0){
					v1_approachVec.push_back( v0_approachVec );
					v1_fingerVec.push_back( v0_fingerVec );
				}
			}
			else if(departData.surface_shape[i]==3){
				if(departData.surface_shape[j]==1) departData.createLineEllHyp(j, i, grasp_point, v1_approachVec, v1_fingerVec);
				else if(departData.surface_shape[j]==2) departData.createLineCylHyp(j, i, grasp_point, v1_approachVec, v1_fingerVec);
				else if(departData.surface_shape[j]==3) departData.createLineHypHyp(i, j, grasp_point, v1_approachVec, v1_fingerVec);
				else if(departData.surface_shape[j]==4) departData.createLineHypPla(i, j, grasp_point, v1_approachVec, v1_fingerVec);
				else if(departData.surface_shape[j]==0){
					v1_approachVec.push_back( v0_approachVec );
					v1_fingerVec.push_back( v0_fingerVec );
				}
			}
			else if(departData.surface_shape[i]==4){
				if(departData.surface_shape[j]==1) departData.createLineEllPla(j, i, grasp_point, v1_approachVec, v1_fingerVec);
				else if(departData.surface_shape[j]==2) departData.createLineCylPla(j, i, grasp_point, v1_approachVec, v1_fingerVec);
				else if(departData.surface_shape[j]==3) departData.createLineHypPla(j, i, grasp_point, v1_approachVec, v1_fingerVec);
				else if(departData.surface_shape[j]==4) departData.createLinePlaPla(i, j, grasp_point, v1_approachVec, v1_fingerVec);
				else if(departData.surface_shape[j]==0){
					v1_approachVec.push_back( v0_approachVec );
					v1_fingerVec.push_back( v0_fingerVec );
				}
			}
			else if(departData.surface_shape[i]==0){
				v1_approachVec.push_back( v0_approachVec );
				v1_fingerVec.push_back( v0_fingerVec );
			}
			
			//grasp_point, v1_approachVec, v1_fingerVecを格納
			v1_grasp_point.push_back(grasp_point);
			v2_approachVec.push_back(v1_approachVec);
			v2_fingerVec.push_back(v1_fingerVec);
		}
		
		//v2_approachVec, v2_fingerVecを格納
		departData.grasp_point.push_back(v1_grasp_point);
		departData.approachVec.push_back(v2_approachVec);
		departData.fingerVec.push_back(v2_fingerVec);
	}
	
	//ファイルへかきこみ
	cout << "depart dataをファイルに書き込みます" << endl;
	std::ofstream ofs( "DepartData.yaml" );
	
	for(i=0 ; i<departData.approachVec.size() ; i++){
		for(j=0 ; j<departData.approachVec[i].size() ; j++){
			
//			cout << "check " << i << "-" << j << endl;
//			cout << "shape1 : " << departData.surface_shape[i] << endl;
//			cout << "shape2 : " << departData.surface_shape[j] << endl;
			
			if(departData.surface_shape[i]==1){
				if(departData.surface_shape[j+i+1]==1) ofs << "1.楕円体" << endl << "2.楕円体" << endl;
				else if(departData.surface_shape[j+i+1]==2) ofs << "1.楕円体" << endl << "2.楕円柱" << endl;
				else if(departData.surface_shape[j+i+1]==3) ofs << "1.楕円体" << endl << "2.一葉双極面" << endl;
				else if(departData.surface_shape[j+i+1]==4) ofs << "1.楕円体" << endl << "2.平面" << endl;
				else if(departData.surface_shape[j+i+1]==0) ofs << "1.楕円体" << endl << "2.その他" << endl;
			}
			else if(departData.surface_shape[i]==2){
				if(departData.surface_shape[j+i+1]==1) ofs << "1.楕円柱" << endl << "2.楕円体" << endl;
				else if(departData.surface_shape[j+i+1]==2) ofs << "1.楕円柱" << endl << "2.楕円柱" << endl;
				else if(departData.surface_shape[j+i+1]==3) ofs << "1.楕円柱" << endl << "2.一葉双極面" << endl;
				else if(departData.surface_shape[j+i+1]==4) ofs << "1.楕円柱" << endl << "2.平面" << endl;
				else if(departData.surface_shape[j+i+1]==0) ofs << "1.楕円柱" << endl << "2.その他" << endl;
			}
			else if(departData.surface_shape[i]==3){
				if(departData.surface_shape[j+i+1]==1) ofs << "1.一葉双極面" << endl << "2.楕円体" << endl;
				else if(departData.surface_shape[j+i+1]==2) ofs << "1.一葉双極面" << endl << "2.楕円柱" << endl;
				else if(departData.surface_shape[j+i+1]==3) ofs << "1.一葉双極面" << endl << "2.一葉双極面" << endl;
				else if(departData.surface_shape[j+i+1]==4) ofs << "1.一葉双極面" << endl << "2.平面" << endl;
				else if(departData.surface_shape[j+i+1]==0) ofs << "2.その他" << endl;
			}
			else if(departData.surface_shape[i]==4){
				if(departData.surface_shape[j+i+1]==1) ofs << "1.平面" << endl << "2.楕円体" << endl;
				else if(departData.surface_shape[j+i+1]==2) ofs << "1.平面" << endl << "2.楕円柱" << endl;
				else if(departData.surface_shape[j+i+1]==3) ofs << "1.平面" << endl << "2.一葉双極面" << endl;
				else if(departData.surface_shape[j+i+1]==4) ofs << "1.平面" << endl << "2.平面" << endl;
				else if(departData.surface_shape[j+i+1]==0) ofs << "2.その他" << endl;
			}
			else if(departData.surface_shape[i]==0){
				ofs << "1.その他" << endl << "2.その他" << endl;
			}

			ofs << "midle point" << endl << "	" << departData.grasp_point[i][j].transpose() << endl;
			ofs << "approachVec" << endl;
			for(k=0 ; k<departData.approachVec[i][j].size() ; k++) ofs << "	" << departData.approachVec[i][j][k].transpose() << endl;
			ofs << "fingerVec" << endl;
			for(k=0 ; k<departData.fingerVec[i][j].size() ; k++) ofs << "	" << departData.fingerVec[i][j][k].transpose() << endl;
			ofs << endl;
//			cout << "approachVec : " << departData.approachVec[i][j].transpose() << endl;
//			cout << "fingerVec : " << departData.fingerVec[i][j].transpose() << endl;
		}
//		ofs << "--------------------------------------------------------" << endl << endl;
		ofs << "----------" << endl << endl;
	}
					
	ofs.close();
	cout << "DepartData.yamlへの書き込みが完了しました" << endl;
		
}



//////////////////////////////////////////////////
///   メッシュの境界点群を楕円に近似し,長軸,短軸を導出   ///
//////////////////////////////////////////////////
void ObjectAnalysis::calcBoundaryMainAxis(int id, cnoid::Vector3& v1,  cnoid::Vector3& v2){
	int i, j, k;
	vector<Vector3> pos;
	
//	cout << " check " << endl;
	for(i=0 ; i<object->nTriangles ; i++)
		object->clusterNodes[object->triangles[i].idCluster].triangleList.push_back(&object->triangles[i]);

	//隣のidをVertexLink::nbrIdに格納する
	std::vector<std::vector<VertexLink*> > boundaryList;
	calcClusterBoundaryNbr(&object->clusterNodes[id], boundaryList);
		
//	cout << endl << "id : " << id << endl;
	for(i=0 ; i<boundaryList.size() ; i++){
		for(j=0 ; j<boundaryList[i].size() ; j++){
			pos.push_back(boundaryList[i][j]->pos);
//			cout << "boundaryList[" << i << "].size() : " << boundaryList[i].size() << endl;
//			cout << "pos[" << i << "][" << j << "] : " << boundaryList[i][j]->pos.transpose() << endl;
//			cout << boundaryList[i][j]->pos[0] << "	" << boundaryList[i][j]->pos[1] << "	" << boundaryList[i][j]->pos[2] << endl;
		}
	}
	
	
	//点群posに対して長軸，短軸を導出する.
	Vector3 center(0, 0, 0);
	Matrix3 VCM;
	VCM << 0, 0, 0, 0, 0, 0, 0, 0, 0;	//分散共分散行列
	
	for(i=0 ; i<pos.size() ; i++) center+=pos[i];
	center = center/pos.size();
	
	for(i=0 ; i<3 ; i++)
		for(j=0 ; j<3 ; j++)
			for(k=0 ; k<pos.size() ; k++) VCM(i,j)+=(center[i]-pos[k][i])*(center[j]-pos[k][j]);
	VCM = VCM/pos.size();
	
	dvector eval(3);
	dmatrix evec = MatrixXd::Zero(3,3);
	double info = calcEigenVectors(VCM, evec, eval); //行列"M3"の固有ベクトル"evec"と固有値"evel"を求める関数

	//固有値を小さい順に導出し，その順どおりに固有ベクトルを並べ替える
	Vector3 a[3];
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++) 
			a[i][j] = evec(i,j);

	
	for( i=0 ; i<3 ; i++){
		j=i%2;
		if(eval[j] < eval[j+1]){
			double temp=eval[j];
			Vector3 tvec=a[j];
				
			eval[j] = eval[j+1];
			a[j] = a[j+1];
			eval[j+1] = temp;
			a[j+1] = tvec;
		}
	}
	v1 = a[0];
	v2 = a[1];

//	cout << "v1 : " << v1.transpose() << endl;
//	cout << "v2 : " << v2.transpose() << endl;
	
}



/////////////////////////////////////////
///   メッシュの境界点群の重心を計算する   ///
/////////////////////////////////////////
cnoid::Vector3 ObjectAnalysis::calcBoundaryCenter(int id){
	int i, j;
	Vector3 boundary_center(0, 0, 0);
	
//	cout << " check " << endl;
	for(i=0 ; i<object->nTriangles ; i++) object->clusterNodes[object->triangles[i].idCluster].triangleList.push_back(&object->triangles[i]);

	//隣のidをVertexLink::nbrIdに格納する
	std::vector<std::vector<VertexLink*> > boundaryList;
	calcClusterBoundaryNbr(&object->clusterNodes[id], boundaryList);
		
	for(i=0 ; i<boundaryList.size() ; i++){
		for(j=0 ; j<boundaryList[i].size() ; j++){
//			cout << "clusterNodes[" << id << "].boundaryList[" << i << "][" << j << "]: " << clusterNodes[id].boundaryList[i][j]->pos.transpose() << endl;
			boundary_center+=boundaryList[i][j]->pos;
		}
		boundary_center = boundary_center/boundaryList[i].size();
	}
//	cout << "boundary_center : " << boundary_center.transpose() << endl;
//	cout << endl;
	
	return boundary_center;
	
}



///////////////////////////////
///   メッシュの重心を計算する   ///
///////////////////////////////
cnoid::Vector3 ObjectAnalysis::calcMesheCenter(int id){
	int i, j;
	Vector3 meshe_center(0, 0, 0);
	
	for(i=0 ; i<object->nTriangles ; i++) object->clusterNodes[object->triangles[i].idCluster].triangleList.push_back(&object->triangles[i]);
		
	for(i=0 ; i<object->clusterNodes[id].triangleList.size() ; i++)
		for(j=0 ; j<3 ; j++) meshe_center += object->clusterNodes[id].triangleList[i]->ver[j]->pos;

	meshe_center = meshe_center/(object->clusterNodes[id].triangleList.size()*3);
	
	return meshe_center;
	
}

//平面の法線ベクトルのなす角度に対するしきい値
int angle_threshold = 5;
////////////////////////////////////////////////////////
///   楕円体-楕円体の組み合わせでアプローチ軸と開閉軸を導出   ///
////////////////////////////////////////////////////////
void DepartData::createLineEllEll(int id1, int id2, cnoid::Vector3& sub_grasp_point, std::vector<cnoid::Vector3>& tmp_approachVec, std::vector<cnoid::Vector3>& tmp_fingerVec){
	int i, j, k;
	int id;	//二次曲面のid
	Vector3 center[2];	//二次曲面の中心
	Vector3 v0_approachVec;
	Vector3 v0_fingerVec;

	//二つの楕円体の中心centerをそれぞれ導出
	dmatrix evec1 = MatrixXd::Zero(3,3);
	dmatrix evec2 = MatrixXd::Zero(3,3);
	dvector eval(3);

	for(i=0 ; i<2 ; i++){
		if(i==0) id=id1;
		else if(i==1) id=id2;
		
		dmatrix M3 = MatrixXd::Zero(3,3);
		Vector3 ghj;
		double k;
		
		M3(0,0) = surface_qc[id][0];
		M3(1,1) = surface_qc[id][1];
		M3(2,2) = surface_qc[id][2];
		M3(0,1) = surface_qc[id][3];
		M3(1,2) = surface_qc[id][4];
		M3(0,2) = surface_qc[id][5];
		ghj << surface_qc[id][6], surface_qc[id][7], surface_qc[id][8];
		k = surface_qc[id][9];

		for(j=0 ; j<3 ; j++){
			for(k=0 ; k<3 ; k++){
				double m = ( M3(k,j) + M3(j,k) )/2;
				M3(k,j) = M3(j,k) = m;
			}	
		}

		dmatrix evec = MatrixXd::Zero(3,3);
		double info = calcEigenVectors(M3, evec, eval); //行列"M3"の固有ベクトル"evec"と固有値"evel"を求める関数
				
		if(evec.determinant() < 0){
			evec = -evec;
		}
		ghj = ghj.transpose()*evec;
		center[i] << -ghj[0]/(2*eval[0]), -ghj[1]/(2*eval[1]), -ghj[2]/(2*eval[2]);
		center[i] = evec*center[i];
		
		if(i==0) evec1 = evec;
		else if(i==1) evec2 = evec;
	}

//	cout << "evec1 : " << endl << evec1 << endl;
//	cout << "evec2 : " << endl << evec2 << endl;
	//２つのcenterの中心sub_midle_centerを導出
	for(i=0 ; i<3 ; i++) sub_grasp_point[i] = (center[0][i]+center[1][i])/2;
	
	//approachVecを求める
	for(i=0 ; i<2 ; i++){
		//楕円体の主軸3軸を導出
		Vector3 vec[3];
		vec[0] << 1, 0, 0;
		vec[1] << 0, 1, 0;
		vec[2] << 0, 0, 1;
		
		for(j=0 ; j<3 ; j++){
			if(i==0) vec[j] = evec1*vec[j];
			else if(i==1) vec[j] = evec2*vec[j];
		
		//midel_centerをとおり，3つのvecベクトルと並行な直線をapproachVecに格納する
//		v0_approachVec << vec[j][0], vec[j][0]*midle_center[0], vec[j][1], vec[j][1]*midle_center[1], vec[j][2], vec[j][2]*midle_center[2];
		v0_approachVec = vec[j];
		tmp_approachVec.push_back(v0_approachVec);
		}
	}
	
	
	//graspVecを求める
	v0_fingerVec = center[0]-center[1];
	tmp_fingerVec.push_back(v0_fingerVec);
}



////////////////////////////////////////////////////////
///   楕円体-楕円柱の組み合わせでアプローチ軸と開閉軸を導出   ///
////////////////////////////////////////////////////////
void DepartData::createLineEllCyl(int id1, int id2, cnoid::Vector3& sub_grasp_point, std::vector<cnoid::Vector3>& tmp_approachVec, std::vector<cnoid::Vector3>& tmp_fingerVec){
	Vector3 v0_approachVec;
	Vector3 v0_fingerVec;

	sub_grasp_point << 0, 0, 0;
	v0_approachVec << 0, 0, 0;
	v0_fingerVec << 0, 0, 0;
	
	tmp_approachVec.push_back(v0_approachVec);
	tmp_fingerVec.push_back(v0_fingerVec);
}
////////////////////////////////////////////////////////////
///   楕円体-一葉双極面の組み合わせでアプローチ軸と開閉軸を導出   ///
////////////////////////////////////////////////////////////
void DepartData::createLineEllHyp(int id1, int id2, cnoid::Vector3& sub_grasp_point, std::vector<cnoid::Vector3>& tmp_approachVec, std::vector<cnoid::Vector3>& tmp_fingerVec){
	Vector3 v0_approachVec;
	Vector3 v0_fingerVec;

	sub_grasp_point << 0, 0, 0;
	v0_approachVec << 0, 0, 0;
	v0_fingerVec << 0, 0, 0;
	
	tmp_approachVec.push_back(v0_approachVec);
	tmp_fingerVec.push_back(v0_fingerVec);

}
////////////////////////////////////////////////////////////
///   楕円体-平面の組み合わせでアプローチ軸と開閉軸を導出   ///
////////////////////////////////////////////////////////////
void DepartData::createLineEllPla(int id1, int id2, cnoid::Vector3& sub_grasp_point, std::vector<cnoid::Vector3>& tmp_approachVec, std::vector<cnoid::Vector3>& tmp_fingerVec){

	int i, j, k;
	int id;	//二次曲面のid
	Vector3 v0_approachVec;
	Vector3 v0_fingerVec;

	VectorXd qc[2];	//平面の係数
	qc[0]= VectorXd::Zero(10);
	qc[1]= VectorXd::Zero(4);
	for(i=0 ; i<10 ; i++) qc[0][i] = surface_qc[id1][i];		//楕円体の係数を格納
	for(i=0 ; i<4 ; i++) qc[1][i] = surface_qc[id2][i+6];	//平面の係数を格納
//	cout << "qc : " << qc[0].transpose() << endl;

	//２つの曲面のメッシュ境界点群の重心を通る直線の式を導出する
	double l, m, o, p, q, r;
	Vector3 line_vec;
	if(boundary_center[id1] == boundary_center[id2]) line_vec = meshe_center[id1] - meshe_center[id2];	//境界の重心を結ぶベクトル
	else line_vec = boundary_center[id1] - boundary_center[id2];											//メッシュの重心を結ぶベクトル
	
	l = line_vec[0];
	m = line_vec[1];
	o = line_vec[2];
	p = boundary_center[id2][0];
	q = boundary_center[id2][1];
	r = boundary_center[id2][2];

	//上記直線と楕円体の交点（2点）を導出し，その2点間のベクトルと，平面から楕円体へのベクトルが同じ時，交点の先にある方の点を導出する
	double A, B, C, t[2];
	
	A=qc[0][0]*l*l+qc[0][1]*m*m+qc[0][2]*o*o+qc[0][3]*l*m+qc[0][4]*m*o+qc[0][5]*l*o;
	B=2*qc[0][0]*p*l+2*qc[0][1]*q*m+2*qc[0][2]*r*o+qc[0][3]*(m*p+l*q)+qc[0][4]*(o*q+m*r)+qc[0][5]*(o*p+l*r)+qc[0][6]*l+qc[0][7]*m+qc[0][8]*o;
	C=qc[0][0]*p*p+qc[0][1]*q*q+qc[0][2]*r*r+qc[0][3]*p*q+qc[0][4]*q*r+qc[0][5]*p*r+qc[0][6]*p+qc[0][7]*q+qc[0][8]*r+qc[0][9];

	t[0] = (-B+sqrt(B*B-4*A*C)) / (2*A);
	t[1] = (-B-sqrt(B*B-4*A*C)) / (2*A);

	Vector3 tmp_point[2], point;
	for(i=0 ; i<2 ; i++) tmp_point[i] << t[i]*l+p, t[i]*m+q, t[i]*o+r;
//	cout << "tmp_point[0] : " << tmp_point[0].transpose() << endl;
//	cout << "tmp_point[1] : " << tmp_point[1].transpose() << endl;
	if( (tmp_point[0]-tmp_point[1]) == line_vec ) point  = tmp_point[0];
	else point = tmp_point[1];
	
	//上部で求めたpointにおける楕円体に対する接平面の法線ベクトルを導出する
	Vector3 n1, n2;
	n1 << 2*qc[0][0]*point[0]+qc[0][3]*point[1]+qc[0][5]*point[2]+qc[0][6], 2*qc[0][1]*point[1]+qc[0][3]*point[0]+qc[0][4]*point[2]+qc[0][7], 2*qc[0][2]*point[2]+qc[0][4]*point[1]+qc[0][5]*point[0]+qc[0][8];	//接平面の法線ベクトル
	n2 << qc[1][0], qc[1][1], qc[1][2];	//平面の法線ベクトル
	
	//2ベクトルの角度を導出する
	double angle;
	angle = acos( dot(n1, n2)/(norm2(n1)*norm2(n2)) );
	if(angle > M_PI/2) angle = M_PI-angle;
//	cout << "angle : " << angle << endl;
	
	//angleがthreshold以下のときに把持姿勢を生成する
//	int threshold = 5;
//	if( angle > threshold*M_PI/180 || angle < -threshold*M_PI/180 ){		
	if( angle > angle_threshold*M_PI/180 || angle < -angle_threshold*M_PI/180 ){		
		sub_grasp_point << 0, 0, 0;
		v0_approachVec << 0, 0, 0;
		v0_fingerVec << 0, 0, 0;
		tmp_approachVec.push_back(v0_approachVec);
		tmp_fingerVec.push_back(v0_fingerVec);
	}
	else{
		//sub_grasp_pointの格納
		//片方の平面のboundary_centerから法線を伸ばし，もう片方の平面との交点を導出し，その交点とboundary_centerとの中点をsub_grasp_pointとする
//		cout << "point : " << point.transpose() << endl;
//		cout << "mesh_center : " << boundary_center[id2].transpose() << endl;
		sub_grasp_point = (point + boundary_center[id2])/2;

		//tmp_fingerVecの格納
		//平面の法線ベクトル
//		v0_fingerVec = boundary_center[id1] - boundary_center[id2];
		v0_fingerVec = n2;
		tmp_fingerVec.push_back(v0_fingerVec);
		
		//tmp_approachVecの格納		
		for(i=0 ; i<4 ; i++){
//			cout << "ellipseParameter[" << i/2 <<"] : " << ellipseParameter[id1][i/2] << endl;
			if(i%2==0) v0_approachVec = ellipseParameter[id1][i/2];
			else if(i%2==1) v0_approachVec = -1*ellipseParameter[id1][i/2];
			tmp_approachVec.push_back(v0_approachVec);
		}			
		
	}
}
////////////////////////////////////////////////////////
///   楕円柱-楕円柱の組み合わせでアプローチ軸と開閉軸を導出   ///
////////////////////////////////////////////////////////
void DepartData::createLineCylCyl(int id1, int id2, cnoid::Vector3& sub_grasp_point, std::vector<cnoid::Vector3>& tmp_approachVec, std::vector<cnoid::Vector3>& tmp_fingerVec){
	Vector3 v0_approachVec;
	Vector3 v0_fingerVec;

	sub_grasp_point << 0, 0, 0;
	v0_approachVec << 0, 0, 0;
	v0_fingerVec << 0, 0, 0;
	
	tmp_approachVec.push_back(v0_approachVec);
	tmp_fingerVec.push_back(v0_fingerVec);
}
////////////////////////////////////////////////////////////
///   楕円柱-一葉双極面の組み合わせでアプローチ軸と開閉軸を導出   ///
////////////////////////////////////////////////////////////
void DepartData::createLineCylHyp(int id1, int id2, cnoid::Vector3& sub_grasp_point, std::vector<cnoid::Vector3>& tmp_approachVec, std::vector<cnoid::Vector3>& tmp_fingerVec){
	Vector3 v0_approachVec;
	Vector3 v0_fingerVec;

	sub_grasp_point << 0, 0, 0;
	v0_approachVec << 0, 0, 0;
	v0_fingerVec << 0, 0, 0;
	
	tmp_approachVec.push_back(v0_approachVec);
	tmp_fingerVec.push_back(v0_fingerVec);
}
////////////////////////////////////////////////////////////
///   楕円柱-平面の組み合わせでアプローチ軸と開閉軸を導出   ///
////////////////////////////////////////////////////////////
void DepartData::createLineCylPla(int id1, int id2, cnoid::Vector3& sub_grasp_point, std::vector<cnoid::Vector3>& tmp_approachVec, std::vector<cnoid::Vector3>& tmp_fingerVec){
	int i, j, k;
	int id;	//二次曲面のid
	Vector3 v0_approachVec;
	Vector3 v0_fingerVec;

	VectorXd qc[2];	//平面の係数
	qc[0]= VectorXd::Zero(10);
	qc[1]= VectorXd::Zero(4);
	for(i=0 ; i<10 ; i++) qc[0][i] = surface_qc[id1][i];		//楕円柱の係数を格納
	for(i=0 ; i<4 ; i++) qc[1][i] = surface_qc[id2][i+6];	//平面の係数を格納

	//２つの曲面のメッシュ境界点群の重心を通る直線の式を導出する
	double l, m, o, p, q, r;
	Vector3 line_vec;
	if(boundary_center[id1] == boundary_center[id2]) line_vec = meshe_center[id1] - meshe_center[id2];	//境界の重心を結ぶベクトル
	else line_vec = boundary_center[id1] - boundary_center[id2];								//メッシュの重心を結ぶベクトル
	
	l = line_vec[0];
	m = line_vec[1];
	o = line_vec[2];
	p = boundary_center[id2][0];
	q = boundary_center[id2][1];
	r = boundary_center[id2][2];

	//上記直線と楕円体の交点（2点）を導出し，その2点間のベクトルと，平面から楕円体へのベクトルが同じ時，交点の先にある方の点を導出する
	double A, B, C, t[2];
	
	A=qc[0][0]*l*l+qc[0][1]*m*m+qc[0][2]*o*o+qc[0][3]*l*m+qc[0][4]*m*o+qc[0][5]*l*o;
	B=2*qc[0][0]*p*l+2*qc[0][1]*q*m+2*qc[0][2]*r*o+qc[0][3]*(m*p+l*q)+qc[0][4]*(o*q+m*r)+qc[0][5]*(o*p+l*r)+qc[0][6]*l+qc[0][7]*m+qc[0][8]*o;
	C=qc[0][0]*p*p+qc[0][1]*q*q+qc[0][2]*r*r+qc[0][3]*p*q+qc[0][4]*q*r+qc[0][5]*p*r+qc[0][6]*p+qc[0][7]*q+qc[0][8]*r+qc[0][9];

	t[0] = (-B+sqrt(B*B-4*A*C)) / (2*A);
	t[1] = (-B-sqrt(B*B-4*A*C)) / (2*A);

	Vector3 tmp_point[2], point;
	for(i=0 ; i<2 ; i++) tmp_point[i] << t[i]*l+p, t[i]*m+q, t[i]*o+r;
	if( (tmp_point[0]-tmp_point[1]) == (boundary_center[id1] - boundary_center[id2]) ) point  = tmp_point[0];
	else point = tmp_point[1];
	
	//上部で求めたpointにおける楕円柱に対する接平面の法線ベクトルを導出する
	Vector3 n1, n2;
	n1  << 2*qc[0][0]*point[0]+qc[0][3]*point[1]+qc[0][5]*point[2]+qc[0][6], 2*qc[0][1]*point[1]+qc[0][3]*point[0]+qc[0][4]*point[2]+qc[0][7], 2*qc[0][2]*point[2]+qc[0][4]*point[1]+qc[0][5]*point[0]+qc[0][8];	//接平面の法線ベクトル
	n2 << qc[1][0], qc[1][1], qc[1][2];	//平面の法線ベクトル
	
	//2ベクトルの角度を導出する
	double angle;
	angle = acos( dot(n1, n2)/(norm2(n1)*norm2(n2)) );
	if(angle > M_PI/2) angle = M_PI-angle;
	cout << "angle : " << angle << endl;
	
	//angleがthreshold以下のときに把持姿勢を生成する
	if( angle > angle_threshold*M_PI/180 || angle < -angle_threshold*M_PI/180 ){		
		sub_grasp_point << 0, 0, 0;
		v0_approachVec << 0, 0, 0;
		v0_fingerVec << 0, 0, 0;
		tmp_approachVec.push_back(v0_approachVec);
		tmp_fingerVec.push_back(v0_fingerVec);
	}
	else{
		//sub_grasp_pointの格納
		//片方の平面のboundary_centerから法線を伸ばし，もう片方の平面との交点を導出し，その交点とboundary_centerとの中点をsub_grasp_pointとする
		sub_grasp_point = (point + boundary_center[id2])/2;

		//tmp_fingerVecの格納
		//平面の法線ベクトル
//		v0_fingerVec = boundary_center[id1] - boundary_center[id2];
		v0_fingerVec = n2;
		tmp_fingerVec.push_back(v0_fingerVec);
		
		//tmp_approachVecの格納		
		for(i=0 ; i<4 ; i++){
//			cout << "ellipseParameter[" << i/2 <<"] : " << ellipseParameter[id1][i/2] << endl;
			if(i%2==0) v0_approachVec = ellipseParameter[id2][i/2];
			else if(i%2==1) v0_approachVec = -1*ellipseParameter[id2][i/2];
			tmp_approachVec.push_back(v0_approachVec);
		}			
	}
}
///////////////////////////////////////////////////////////////
///   一葉双極面-一葉双極面の組み合わせでアプローチ軸と開閉軸を導出   ///
///////////////////////////////////////////////////////////////
void DepartData::createLineHypHyp(int id1, int id2, cnoid::Vector3& sub_grasp_point, std::vector<cnoid::Vector3>& tmp_approachVec, std::vector<cnoid::Vector3>& tmp_fingerVec){
	Vector3 v0_approachVec;
	Vector3 v0_fingerVec;

	sub_grasp_point << 0, 0, 0;
	v0_approachVec << 0, 0, 0;
	v0_fingerVec << 0, 0, 0;
	
	tmp_approachVec.push_back(v0_approachVec);
	tmp_fingerVec.push_back(v0_fingerVec);
}
///////////////////////////////////////////////////////////////
///   一葉双極面-平面の組み合わせでアプローチ軸と開閉軸を導出   ///
///////////////////////////////////////////////////////////////
void DepartData::createLineHypPla(int id1, int id2, cnoid::Vector3& sub_grasp_point, std::vector<cnoid::Vector3>& tmp_approachVec, std::vector<cnoid::Vector3>& tmp_fingerVec){
	Vector3 v0_approachVec;
	Vector3 v0_fingerVec;

	sub_grasp_point << 0, 0, 0;
	v0_approachVec << 0, 0, 0;
	v0_fingerVec << 0, 0, 0;
	
	tmp_approachVec.push_back(v0_approachVec);
	tmp_fingerVec.push_back(v0_fingerVec);
}
///////////////////////////////////////////////////////////////
///   平面-平面の組み合わせでアプローチ軸と開閉軸を導出   ///
///////////////////////////////////////////////////////////////
void DepartData::createLinePlaPla(int id1, int id2, cnoid::Vector3& sub_grasp_point, std::vector<cnoid::Vector3>& tmp_approachVec, std::vector<cnoid::Vector3>& tmp_fingerVec){
	int i, j, k;
	int id;	//二次曲面のid
	Vector3 v0_approachVec;
	Vector3 v0_fingerVec;

	VectorXd qc[2];	//平面の係数
	qc[0]= VectorXd::Zero(4);
	qc[1]= VectorXd::Zero(4);
	for(i=0 ; i<4 ; i++){
		qc[0][i] = surface_qc[id1][i+6];
		qc[1][i] = surface_qc[id2][i+6];
	}
	
	//２つの平面の角度を導出
	double angle;
	Vector3 n1(qc[0][0], qc[0][1], qc[0][2]);
	Vector3 n2(qc[1][0], qc[1][1], qc[1][2]);
	angle = acos( dot(n1, n2)/(norm2(n1)*norm2(n2)) );
	if(angle > M_PI/2) angle = M_PI-angle;
//	cout << "angle : " << angle << endl;
	
	//angleがthreshold以下のときに把持姿勢を生成する
//	int threshold = 5;
//	if( angle > threshold*M_PI/180 || angle < -threshold*M_PI/180 ){		
	if( angle > angle_threshold*M_PI/180 || angle < -angle_threshold*M_PI/180 ){		
		sub_grasp_point << 0, 0, 0;
		v0_approachVec << 0, 0, 0;
		v0_fingerVec << 0, 0, 0;
		tmp_approachVec.push_back(v0_approachVec);
		tmp_fingerVec.push_back(v0_fingerVec);
	}
	else{
		//sub_grasp_pointの格納
		//片方の平面のboundary_centerから法線を伸ばし，もう片方の平面との交点を導出し，その交点とboundary_centerとの中点をsub_grasp_pointとする
/*		double p, q, r, A, B, C, D, t;
		Vector3 point;

		p = boundary_center[id1][0];
		q = boundary_center[id1][1];
		r = boundary_center[id1][2];
		A = n2[0];
		B = n2[1];
		C = n2[2];
		D = dot(n2, boundary_center[id2]);
		t = ( D-A*p-B*q-C*r )/(A*n1[0]+B*n1[1]+C*n1[2]);
		point << t*n1[0]+p, t*n1[1]+q, t*n1[2]+r;
		
//		cout << "boundary_center[" << id1 << "] : " << boundary_center[id1] << endl;
//		cout << "boundary_center[" << id2 << "] : " << boundary_center[id2] << endl << endl;
		sub_grasp_point = (boundary_center[id1] + point)/2;
*/		sub_grasp_point = (boundary_center[id1] + boundary_center[id2])/2;
		
		//tmp_fingerVecの格納
//		v0_fingerVec = boundary_center[id1] - boundary_center[id2];
		v0_fingerVec = n1;
		tmp_fingerVec.push_back(v0_fingerVec);
		
		//tmp_approachVecの格納		
		for(i=0 ; i<4 ; i++){
//			cout << "ellipseParameter[" << i/2 <<"] : " << ellipseParameter[id1][i/2] << endl;
			if(i%2==0) v0_approachVec = ellipseParameter[id1][i/2];
			else if(i%2==1) v0_approachVec = -1*ellipseParameter[id1][i/2];
			tmp_approachVec.push_back(v0_approachVec);
		}			
	}
}



//////////////////////////////////////////////////////////////////
///   ある限定的な範囲でobjectShape::initialClustersFromOneFaceを行う   ///
//////////////////////////////////////////////////////////////////
void ObjectAnalysis::LimitedInitialClustersFromOneFace(){

//	clusterNodes = new ClusterImpl[nTriangles*2];
//	nClusterNodes=0;
	object->clusterNodes = new ClusterImpl[object->nTriangles*2];
	object->nClusterNodes=0;

//	for(int i=0;i<nTriangles*2;i++){
//		clusterNodes[i].id = i;
//	}
	for(int i=0;i<object->nTriangles*2;i++){
		object->clusterNodes[i].id = i;
	}
	
//	cout << "initial clusterNode" << endl;
//	for(int i=0;i<nTriangles;i++){
//		clusterNodes[i].addTriangleToCluster(triangles[i]);
//		clusterNodes[i].calcQuadricAreaParameter();
		
//		if(nTriangles>100)  if(  ( i % (nTriangles /100) ) == 0 ) cout << "\r" << (i / (nTriangles /100)) << "% " << flush;
//	}
//	nClusterNodes=nTriangles;
	
//	cout <<endl << "initial cluster pair heap" << endl;
	
	//限定する範囲の指定
	//default
//	double lim_x[2] = {-0.5, 0.5};
//	double lim_y[2] = {-0.5, 0.5};
//	double lim_z[2] = {-0.5, 0.5};
	//red-pet-btl_1
//	double lim_x[2] = {-0.441, 0.629};
//	double lim_y[2] = {-0.441, 0.659};
//	double lim_z[2] = {-0.26, 0.14};

	double lim_x[2] = {-0.24, 0.24};
	double lim_y[2] = {-1.1, 0.29};
	double lim_z[2] = {0.19, 0.84};

//	Vector3 shift(-0.367,0.102,-1.44);
	int limited_check;
	int limited_num=0;
		
	for(int i=0;i<object->nTriangles;i++){
		limited_check=0;
		
		for(int j=0 ; j<3 ; j++){
//			object->triangles[i].ver[j]->pos += shift;
		
			if(object->triangles[i].ver[j]->pos[0] > lim_x[0] && object->triangles[i].ver[j]->pos[0] < lim_x[1] && 
				object->triangles[i].ver[j]->pos[1] > lim_y[0] && object->triangles[i].ver[j]->pos[1] < lim_y[1] &&
					object->triangles[i].ver[j]->pos[2] > lim_z[0] && object->triangles[i].ver[j]->pos[2] < lim_z[1])
						limited_check=1;
		}
				
		if(limited_check==1){
			object->clusterNodes[i].addTriangleToCluster(object->triangles[i]);
#ifdef CENTER_SHIFT
			object->clusterNodes[i].calcQuadricAreaParameterLow();
#endif
			object->clusterNodes[i].calcQuadricAreaParameter();

			limited_num++;
		}
		else{
			object->triangles[i].idCluster=-1;
		}
			
		if(object->nTriangles>100)  if(  ( i % (object->nTriangles /100) ) == 0 ) cout << "\r" << (i / (object->nTriangles /100)) << "% " << flush;
	}

	object->nTriangles=limited_num;
	object->nClusterNodes=limited_num;
	cout << "limited_num : " << limited_num << endl;
	
	cout <<endl << "limited initial cluster pair heap" << endl;
	
#ifdef THREAD
	ClusterPair** cpLink = new ClusterPair*[nTriangles*3];
	//ClusterPair* cpLink[nTriangles*3];
	int cnt=0;
	boost::thread_group* thr_grp = new boost::thread_group();
	for(int i=0;i<nTriangles;i++){
		int idc = triangles[i].idCluster;
		for(int j=0;j<3;j++){
			if(triangles[i].nbr[j]==NULL) continue;
			clusterNodes[idc].neighborClusters.push_back(&clusterNodes[triangles[i].nbr[j]->idCluster]);
			if(idc < triangles[idc].nbr[j]->idCluster) thr_grp->create_thread( boost::bind(&thread_process,  &cpLink[cnt++] , &clusterNodes[idc], &clusterNodes[triangles[i].nbr[j]->idCluster]));
			if(  cnt%16 ==0 ){
				thr_grp->join_all();
				delete thr_grp;
				thr_grp=  new boost::thread_group();
			}
		}
		if(nTriangles>100)  if(  (i % (nTriangles /100)) == 0 ) cout << "\r" << (i / (nTriangles /100)) << "% " << flush;
	}
	thr_grp->join_all();
	delete thr_grp;
	
	for(int i=0;i<cnt;i++){
		clusterPairHeap.push_back( cpLink[i] );
//		push_heap ( clusterPairHeap.begin(), clusterPairHeap.end(), ClusterPair::compClusterHeap );
	}
	delete[] cpLink;
#else
	for(int i=0;i<object->nTriangles;i++){
		if(object->triangles[i].idCluster==-1) continue;
		int idc = object->triangles[i].idCluster;
		for(int j=0;j<3;j++){
			if(object->triangles[i].nbr[j]==NULL) continue;
			if(object->triangles[i].nbr[j]->idCluster == -1) continue;
			object->clusterNodes[idc].neighborClusters.push_back(&object->clusterNodes[object->triangles[i].nbr[j]->idCluster]);
			if(idc < object->triangles[idc].nbr[j]->idCluster) object->clusterPairHeap.push_back( new ClusterPair( object->clusterNodes[idc], object->clusterNodes[object->triangles[i].nbr[j]->idCluster] ) );
		}
		if(object->nTriangles>100)  if(  (i % (object->nTriangles /100)) == 0 ) cout << "\r" << (i / (object->nTriangles /100)) << "% " << flush;
	}
#endif	
	
	make_heap ( object->clusterPairHeap.begin(), object->clusterPairHeap.end(), ClusterPair::compClusterHeap ); 

	
	cout << endl <<"the number of cluster pair "<< object->clusterPairHeap.size() << endl;
	for(int i=0; i < object->clusterPairHeap.size(); i++){
//		clusterPairHeap[i].id= i;
		object->clusterPairHeap[i]->id= i;
//		clusterPairHeap[i].error = clusterPairHeap[i].clusters[0]->area + clusterPairHeap[i].clusters[1]->area;
	}
	
}
