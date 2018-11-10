#include "DrawFunctions.h"

#include "VectorMath.h"

namespace grasp {
	namespace draw {
		ShapeInfo::ShapeInfo() {
			pos = cnoid::Vector3::Zero();
			R = cnoid::Matrix3::Identity();
			color = cnoid::Vector3(0.5, 0.5, 0.5);
			alpha = 1.0;
			scale = cnoid::Vector3::Ones();
		}

		ShapeInfo::ShapeInfo(const cnoid::Vector3& pos_, const cnoid::Matrix3& R_,
												 const cnoid::Vector3& color_, double alpha_,
												 const cnoid::Vector3& scale_) {
			pos = pos_;
			R = R_;
			color = color_;
			alpha = alpha_;
			scale = scale_;
		}

		LineInfo::LineInfo() {
		}

		LineInfo::LineInfo(const cnoid::Vector3& start, const cnoid::Vector3& end,
											 const cnoid::Vector3& color_) {
			start_pos = start;
			end_pos = end;
			color = color_;
		}

		TriangleInfo::TriangleInfo(const cnoid::Vector3& v1, const cnoid::Vector3& v2, const cnoid::Vector3& v3) {
			ver[0] = v1;
			ver[1] = v2;
			ver[2] = v3;
		}

		/**
		 * @brief generate a box node
		 * @param[in] pos    center
		 * @param[in] R      rotation matrix
		 * @param[in] length size of box
		 * @param[in] color  RGB
		 * @param[in] alpha  alpha (1:fully opaque, 0: fully transparent)
		 */
		cnoid::SgNode* generateBoxNode(const cnoid::Vector3& pos, const cnoid::Matrix3& R, const cnoid::Vector3& length,
																	 const cnoid::Vector3& color, double alpha) {
			ShapeInfo info(pos, R, color, alpha);

			cnoid::MeshGenerator generator;
			cnoid::SgMesh* shape = generator.generateBox(length);

			return generateShapeNode(shape, info);
		}

		/**
		 * @brief generate a cylinder node
		 * @param[in] pos    center
		 * @param[in] dir    direction
		 * @param[in] radius radius
		 * @param[in] length height
		 * @param[in] color  RGB
		 * @param[in] alpha  alpha (1:fully opaque, 0: fully transparent)
		 */
		cnoid::SgNode* generateCylinderNode(const cnoid::Vector3& pos, const cnoid::Vector3& dir, double radius, double length,
																				const cnoid::Vector3& color, double alpha) {
			cnoid::Vector3 y = cnoid::Vector3::UnitY();
			cnoid::Vector3 d = dir;
			cnoid::Matrix3 R = grasp::rotFromTwoVecs(y, d);
			if (y == -d) {
				R << 0, 0, 1, 0, -1, 0, 1, 0, 0;
			}
			ShapeInfo info(pos, R, color, alpha);

			cnoid::MeshGenerator generator;
			cnoid::SgMesh* shape = generator.generateCylinder(radius, length);

			return generateShapeNode(shape, info);
		}

		/**
		 * @brief generate a cone node. (When R=Identity Matrix and center=zero, an its apex is located at (0, height/2, 0) and its bottom is located at (0, -height/2, 0))
		 * @param[in] pos    center
		 * @param[in] R      rotation matrix 
		 * @param[in] radius radius of bottom
		 * @param[in] length height
		 * @param[in] color  RGB
		 * @param[in] alpha  alpha (1:fully opaque, 0: fully transparent)
		 */
		cnoid::SgNode* generateConeNode(const cnoid::Vector3& pos, const cnoid::Matrix3& R, double radius, double length,
																		const cnoid::Vector3& color, double alpha) {
			cnoid::Matrix3 yToz;
			yToz << 1, 0, 0, 0, 0, -1, 0, 1, 0;
			cnoid::Matrix3 Rt = R * yToz;

			ShapeInfo info(pos, R, color, alpha);

			cnoid::MeshGenerator generator;
			cnoid::SgMesh* shape = generator.generateCone(radius, length);

			return generateShapeNode(shape, info);
		}

		/**
		 * @brief generate a sphere node
		 * @param[in] pos    center
		 * @param[in] radius radius
		 * @param[in] color  RGB
		 * @param[in] alpha  alpha (1:fully opaque, 0: fully transparent)
		 */
		cnoid::SgNode* generateSphereNode(const cnoid::Vector3& pos, double radius,
																			const cnoid::Vector3& color, double alpha) {
			ShapeInfo info(pos, cnoid::Matrix3::Identity(), color, alpha);

			cnoid::MeshGenerator generator;
			cnoid::SgMesh* shape = generator.generateSphere(radius);

			return generateShapeNode(shape, info);
		}

		/**
		 * @brief generate an ellipsoid node
		 * @param[in] pos    center
		 * @param[in] R      rotaion matirx
		 * @param[in] length length (Vector3)
		 * @param[in] color  RGB
		 * @param[in] alpha  alpha (1:fully opaque, 0: fully transparent)
		 */
		cnoid::SgNode* generateEllipsoidNode(const cnoid::Vector3& pos, const cnoid::Matrix3& R, const cnoid::Vector3& length,
																				 const cnoid::Vector3& color, double alpha) {
			ShapeInfo info(pos, cnoid::Matrix3::Identity(), color, alpha, length);

			cnoid::MeshGenerator generator;
			cnoid::SgMesh* shape = generator.generateSphere(1.0);

			return generateShapeNode(shape, info);
		}

		/**
		 * @brief generate an arrow node
		 * @param[in] start           start positon 
		 * @param[in] end             end postion
		 * @param[in] width           diameter of cylinder part
		 * @param[in] coneLengthRatio cone part proportion of arrow length
		 * @param[in] coneWidthRatio  ratio of cone radius to cylinder radius
		 * @param[in] color           RGB
		 * @param[in] alpha           alpha (1:fully opaque, 0: fully transparent)
		 */
		cnoid::SgNode* generateArrowNode(const cnoid::Vector3& start, const cnoid::Vector3& end, double width,
																		 double coneLengthRatio, double coneWidthRatio,
																		 const cnoid::Vector3& color, double alpha) {
			cnoid::Vector3 d = end - start;
			cnoid::Vector3 y = cnoid::Vector3::UnitY();
			cnoid::Matrix3 R = grasp::rotFromTwoVecs(y, d);
			if (y == -d) {
				R << 0, 0, 1, 0, -1, 0, 1, 0, 0;
			}

			double len = d.norm() * (1-coneLengthRatio);
			cnoid::Vector3 pos = start + 0.5 * (1-coneLengthRatio) * d;

			ShapeInfo info(pos, R, color, alpha);

			cnoid::MeshGenerator generator;
			double ratio = coneLengthRatio/(1-coneLengthRatio);
			cnoid::SgMesh* shape = generator.generateArrow(len, width, ratio, coneWidthRatio);

			return generateShapeNode(shape, info);
		}

		/**
		 * @brief generate lines node
		 * @param[in] lines  line infos
		 * @param[in] width  line width
		 */
		cnoid::SgNode* generateLinesNode(const LineInfos& lines, double width) {
			cnoid::SgLineSet* line_set = new cnoid::SgLineSet();
			cnoid::SgVertexArray* ver_array = line_set->getOrCreateVertices();
			cnoid::SgColorArray* color_array = line_set->getOrCreateColors();

			ver_array->resize(2*lines.size());
			color_array->resize(2*lines.size());

			for (size_t i = 0; i < lines.size(); i++) {
				int b = 2 * i;
				ver_array->at(b) = lines[i].start_pos.cast<float>();
				ver_array->at(b+1) = lines[i].end_pos.cast<float>();
				line_set->addLine(b, b+1);
				color_array->at(b) = lines[i].color.cast<float>();
				color_array->at(b+1) = lines[i].color.cast<float>();
			}

			line_set->normalIndices().clear();
			line_set->colorIndices().clear();
			line_set->setLineWidth(width);

			return line_set;
		}

		/**
		 * @brief generate coordinate axis node
		 * @param[in] pos   coordinate origin
		 * @param[in] R     rotation matrix
		 * @param[in] size  length of axis
		 * @param[in] color RGB
		 */
		cnoid::SgNode* generateCoordinateAxisNode(const cnoid::Vector3& pos, const cnoid::Matrix3& R, double size,
																							const cnoid::Vector3& color) {
			LineInfos axis(3);
			for (int i = 0; i < 3; i++) {
				LineInfo& line = axis[i];
				line.start_pos = pos;
				line.end_pos = pos + grasp::col(R, i) * size;
				line.color = color;
			}

			return generateLinesNode(axis);
		}

		/**
		 * @brief generate coordinate axis node
		 * @param[in] pos   coordinate origin
		 * @param[in] R     rotation matrix
		 * @param[in] size  length of axis
		 * @param[in] alpha alpha (1:fully opaque, 0: fully transparent)
		 */
		cnoid::SgNode* generateCoordinateAxisArrowNode(const cnoid::Vector3& pos, const cnoid::Matrix3& R, double size,
																									 double alpha) {
			cnoid::SgGroup* coordinate = new cnoid::SgGroup();
			for (int i = 0; i < 3; i++) {
				cnoid::Vector3 color = 0.2 * cnoid::Vector3::Ones();
				color(i) = 1.0;
				cnoid::SgNode* axis = generateArrowNode(pos, pos + grasp::col(R, i) * size, 0.1 * size, 0.3, 2, color, alpha);
				coordinate->addChild(axis);
			}

			return coordinate;
		}

		/**
		 * @brief generate Triangle node
		 * @param[in] triangles  triangle infos
		 * @param[in] color RGB
		 * @param[in] alpha alpha (1:fully opaque, 0: fully transparent)
		 */
		cnoid::SgNode* generateTrianglesNode(const TriangleInfos& triangles,
																				 const cnoid::Vector3& color, double alpha) {
			return generateTrianglesNode(cnoid::Vector3::Zero(), cnoid::Matrix3::Identity(), triangles, color, alpha);
		}

		/**
		 * @brief generate Triangle node
		 * @param[in] pos  offset
		 * @param[in] R  rotation matrix
		 * @param[in] triangles  triangle infos
		 * @param[in] color RGB
		 * @param[in] alpha alpha (1:fully opaque, 0: fully transparent)
		 */
		cnoid::SgNode* generateTrianglesNode(const cnoid::Vector3& pos, const cnoid::Matrix3& R, const TriangleInfos& triangles,
																				 const cnoid::Vector3& color, double alpha) {
			ShapeInfo info(pos, R, color, alpha);

			cnoid::SgMesh* mesh = new cnoid::SgMesh();

			cnoid::SgVertexArray* vertices = mesh->getOrCreateVertices();
			vertices->resize(3 * triangles.size());
			mesh->setNumTriangles(triangles.size());
			for (size_t i = 0; i < triangles.size(); i++) {
				int b = 3 * i;
				for (int j = 0; j < 3; j++) {
					vertices->at(b+j) = triangles[i].ver[j].cast<float>();
				}
				mesh->setTriangle(i, b, b+1, b+2);
			}

			return generateShapeNode(mesh, info);
		}

		cnoid::SgNode* generateLinkCurrentPosNode(cnoid::Link* link,
																							const cnoid::Vector3& color, double alpha) {
			return generateLinkNode(link, link->p(), link->R(), color, alpha);
		}

		cnoid::SgNode* generateLinkNode(cnoid::Link* link, const cnoid::Vector3& pos, const cnoid::Matrix3& R,
																		const cnoid::Vector3& color, double alpha) {
			ShapeInfo info(pos, R, color, alpha);

			cnoid::MeshExtractor extract;
			cnoid::SgMesh* mesh = extract.integrate(link->shape());

			return generateShapeNode(mesh, info);
		}

		cnoid::SgNode* generateLinkTraverseCurrentPosNode(cnoid::LinkTraverse* links,
																											const cnoid::Vector3& color, double alpha) {
			cnoid::SgGroup* linknodes = new cnoid::SgGroup();
			for (int i = 0; i < links->numLinks(); i++) {
				linknodes->addChild(generateLinkCurrentPosNode(links->link(i), color, alpha));
			}
			return linknodes;
		}

		cnoid::SgNode* generateBodyCurrentPosNode(const cnoid::BodyPtr& body,
																							const cnoid::Vector3& color, double alpha) {
			cnoid::SgGroup* bodynode = new cnoid::SgGroup();
			for (int i = 0; i < body->numLinks(); i++) {
				bodynode->addChild(generateLinkCurrentPosNode(body->link(i), color, alpha));
			}
			return bodynode;
		}

		cnoid::SgNode* generateShapeNode(cnoid::SgMesh* mesh, const ShapeInfo& info) {
			cnoid::SgShape* shape = new cnoid::SgShape();
			shape->setMesh(mesh);

			cnoid::SgMaterial* material = shape->getOrCreateMaterial();
			material->setTransparency(1-info.alpha);
			material->setDiffuseColor(info.color);
			material->setEmissiveColor(cnoid::Vector3f::Zero());
			material->setSpecularColor(cnoid::Vector3f::Zero());

			cnoid::SgScaleTransform* scale = NULL;
			bool do_scale = !(info.scale.isOnes());
			if (do_scale) {
				scale = new cnoid::SgScaleTransform();
				scale->setScale(info.scale);
				scale->addChild(shape);
			}

			cnoid::SgPosTransform* pos = new cnoid::SgPosTransform();
			pos->setTranslation(info.pos);
			pos->setRotation(info.R);
			if (do_scale) {
				pos->addChild(scale);
			} else {
				pos->addChild(shape);
			}

			return pos;
		}
	}
}
