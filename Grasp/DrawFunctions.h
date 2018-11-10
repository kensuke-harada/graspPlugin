/**
 * @file   DrawFunctions.h
 * @author Akira Ohchi
 */

#ifndef _GRASP_DRAWFUNCTIONS_H_
#define _GRASP_DRAWFUNCTIONS_H_

#include <vector>

#include <cnoid/EigenTypes>
#include <cnoid/SceneGraph>
#include <cnoid/SceneDrawables>
#include <cnoid/MeshGenerator>
#include <cnoid/MeshExtractor>
#include <cnoid/Link>
#include <cnoid/LinkTraverse>
#include <cnoid/Body>

#include "exportdef.h"

namespace grasp {
	namespace draw {
		class EXCADE_API ShapeInfo {
		public:
			ShapeInfo();
			ShapeInfo(const cnoid::Vector3& pos_, const cnoid::Matrix3& R_,
								const cnoid::Vector3& color_ = cnoid::Vector3(0.5, 0.5, 0.5),
								double alpha_ = 1.0,
								const cnoid::Vector3& scale_ = cnoid::Vector3::Ones());

			cnoid::Vector3 pos;
			cnoid::Matrix3 R;
			cnoid::Vector3 color;
			double alpha;
			cnoid::Vector3 scale;
		};

		class EXCADE_API LineInfo {
		public:
			LineInfo();
			LineInfo(const cnoid::Vector3& start, const cnoid::Vector3& end,
							 const cnoid::Vector3& color_ = cnoid::Vector3(0.5, 0.5, 0.5));
			cnoid::Vector3 start_pos;
			cnoid::Vector3 end_pos;
			cnoid::Vector3 color;
		};

		class EXCADE_API TriangleInfo {
		public:
			TriangleInfo(const cnoid::Vector3& v1, const cnoid::Vector3& v2, const cnoid::Vector3& v3);
			cnoid::Vector3 ver[3];
		};

		typedef std::vector<LineInfo> LineInfos;
		typedef std::vector<TriangleInfo> TriangleInfos;

		EXCADE_API cnoid::SgNode* generateBoxNode(const cnoid::Vector3& pos, const cnoid::Matrix3& R, const cnoid::Vector3& length,
																							const cnoid::Vector3& color = cnoid::Vector3(0.5, 0.5, 0.5),
																							double alpha = 1.0);
		EXCADE_API cnoid::SgNode* generateCylinderNode(const cnoid::Vector3& pos, const cnoid::Vector3& dir, double radius, double length,
																									 const cnoid::Vector3& color = cnoid::Vector3(0.5, 0.5, 0.5),
																									 double alpha = 1.0);
		EXCADE_API cnoid::SgNode* generateConeNode(const cnoid::Vector3& pos, const cnoid::Matrix3& R, double radius, double length,
																							 const cnoid::Vector3& color = cnoid::Vector3(0.5, 0.5, 0.5),
																							 double alpha = 1.0);
		EXCADE_API cnoid::SgNode* generateSphereNode(const cnoid::Vector3& pos, double radius,
																								 const cnoid::Vector3& color = cnoid::Vector3(0.5, 0.5, 0.5),
																								 double alpha = 1.0);
		EXCADE_API cnoid::SgNode* generateEllipsoidNode(const cnoid::Vector3& pos, const cnoid::Matrix3& R, const cnoid::Vector3& length,
																										const cnoid::Vector3& color = cnoid::Vector3(0.5, 0.5, 0.5),
																										double alpha = 1.0);
		EXCADE_API cnoid::SgNode* generateArrowNode(const cnoid::Vector3& start, const cnoid::Vector3& end, double width,
																								double coneLengthRatio = 0.1, double coneWidthRatio = 2.5,
																								const cnoid::Vector3& color = cnoid::Vector3(0.5, 0.5, 0.5),
																								double alpha = 1.0);
		EXCADE_API cnoid::SgNode* generateLinesNode(const LineInfos& lines, double width = 0.01);
		EXCADE_API cnoid::SgNode* generateCoordinateAxisNode(const cnoid::Vector3& pos, const cnoid::Matrix3& R, double size,
																												 const cnoid::Vector3& color = cnoid::Vector3(0.5, 0.5, 0.5));
		EXCADE_API cnoid::SgNode* generateCoordinateAxisArrowNode(const cnoid::Vector3& pos, const cnoid::Matrix3& R, double size,
																															double alpha = 1.0);
		EXCADE_API cnoid::SgNode* generateTrianglesNode(const TriangleInfos& trinagles,
																										const cnoid::Vector3& color = cnoid::Vector3(0.5, 0.5, 0.5),
																										double alpha = 1.0);
		EXCADE_API cnoid::SgNode* generateTrianglesNode(const cnoid::Vector3& pos, const cnoid::Matrix3& R, const TriangleInfos& triangles,
																										const cnoid::Vector3& color = cnoid::Vector3(0.5, 0.5, 0.5),
																										double alpha = 1.0);
		EXCADE_API cnoid::SgNode* generateLinkCurrentPosNode(cnoid::Link* link,
																												 const cnoid::Vector3& color = cnoid::Vector3(0.5, 0.5, 0.5),
																												 double alpha = 1.0);
		EXCADE_API cnoid::SgNode* generateLinkNode(cnoid::Link* link, const cnoid::Vector3& pos, const cnoid::Matrix3& R,
																							 const cnoid::Vector3& color = cnoid::Vector3(0.5, 0.5, 0.5),
																							 double alpha = 1.0);
		EXCADE_API cnoid::SgNode* generateLinkTraverseCurrentPosNode(cnoid::LinkTraverse* links,
																																 const cnoid::Vector3& color = cnoid::Vector3(0.5, 0.5, 0.5),
																																 double alpha = 1.0);
		EXCADE_API cnoid::SgNode* generateBodyCurrentPosNode(const cnoid::BodyPtr& body,
																												 const cnoid::Vector3& color = cnoid::Vector3(0.5, 0.5, 0.5),
																												 double alpha = 1.0);
		EXCADE_API cnoid::SgNode* generateShapeNode(cnoid::SgMesh* mesh, const ShapeInfo& info);
	}
}

#endif /* _GRASP_DRAWFUNCTIONS_H_ */
