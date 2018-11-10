/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#ifndef _ConvexAnalysis_H
#define _ConvexAnalysis_H

#include <iostream> 
#include <vector>
#include <cnoid/EigenTypes>

#if defined(__cplusplus)
extern "C"
{
#endif
#include <stdio.h>
#include <stdlib.h>
#if (defined(WIN32) || defined(_WIN32))
#include <libqhull/libqhull.h>
#include <libqhull/mem.h>
#include <libqhull/qset.h>
#include <libqhull/geom.h>
#include <libqhull/merge.h>
#include <libqhull/poly.h>
#include <libqhull/io.h>
#include <libqhull/stat.h>
#else
#include <qhull/qhull.h>
#include <qhull/mem.h>
#include <qhull/qset.h>
#include <qhull/geom.h>
#include <qhull/merge.h>
#include <qhull/poly.h>
#include <qhull/io.h>
#include <qhull/stat.h>
#endif
#if defined(__cplusplus)
}
#endif

#include "exportdef.h"

namespace grasp{
class EXCADE_API ConvexAnalysis
{

public :
	ConvexAnalysis();
	~ConvexAnalysis();

	static void calcConvexHull(int dim, std::vector<double>& pt, std::vector<double>& pt_out, bool fq);
	static void calcConvexHull(int dim, std::vector<double>& pt, std::vector<double>& pt_out, std::vector<int>& index_out, bool fq);
	static double calcConvexHull2(int dim, std::vector<double>& pt, std::vector<double>& pt_out, cnoid::VectorXd w, bool fq);
	//static void calcHalfSpace(std::vector<double>& unitTwist, std::vector<double>& spanVectors, int dim);
	static void outputConvexHull(int dim, std::vector<double>& pt, bool fq);

private :


};
}
#endif
