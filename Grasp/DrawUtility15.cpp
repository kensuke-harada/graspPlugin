// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-

#include "DrawUtility.h"

#include <math.h>

#include <limits>

#include <QtGui>

#include <cnoid/ItemManager>
#include <cnoid/ItemTreeView>
#include <cnoid/MessageView>
#include <cnoid/Archive>
//#include "../GeometryHandler/GeometryHandle.h"

//void write_vrml_data(char *file,grasp::ObjectShape *wo);

using namespace std;
using namespace boost;
using namespace cnoid;

DrawUtility* DrawUtility::instance()
{
    static DrawUtility* instance = new DrawUtility();
    return instance;
}

DrawUtility::DrawUtility() : os (MessageView::mainInstance()->cout() )
{
    rgb = Vector3(1,0,0);
}

DrawUtility::~DrawUtility()
{
}

bool DrawUtility::displayCylinders()
{
	cnoid::SgGroupPtr node;
	return displayCylinders(node);
}

bool DrawUtility::displayCylinders(cnoid::SgGroupPtr& node)
{
	if (cylinders.empty()) return true;
	node = new SgGroup();
	addCylinderNodes(node);
	sgnodes_.push_back(node);
	SceneView::instance()->scene()->addChild(node, true);
    return true;
}

bool DrawUtility::addCylinderNodes(cnoid::SgGroupPtr node)
{
	Vector3 y = Vector3::UnitY();
    for(int i = 0; i < cylinders.size(); i++){
        Vector3 c = cylinders[i].pos;
        Vector3 d = cylinders[i].dir;
        double r = cylinders[i].radius;
        double l = cylinders[i].length;

		MeshGenerator generator;
		SgMeshPtr mesh = generator.generateCylinder(r, l);

		SgShapePtr shape = new SgShape();
		shape->setMesh(mesh);

		SgMaterialPtr material = shape->getOrCreateMaterial();
		material->setTransparency(1.0 - cylinders[i].alpha);
		material->setDiffuseColor(cylinders[i].rgb);
		material->setEmissiveColor(cylinders[i].rgb);

		SgPosTransform* pos = new SgPosTransform();
		pos->setTranslation(c);
		Matrix3 rot = grasp::rotFromTwoVecs(y, d);
		if (y == -d) {
			rot << 0, 0, 1,
				0, -1, 0,
				1, 0, 0;
		}
		pos->setRotation(rot);

		pos->addChild(shape);

		node->addChild(pos, false);
    }
	return true;
}

bool DrawUtility::displayCones()
{
	cnoid::SgGroupPtr node;
	return displayCones(node);
}
bool DrawUtility::displayCones(cnoid::SgGroupPtr& node)
{
	if (cones.empty()) return true;
	node = new SgGroup();
	addConeNodes(node);
	sgnodes_.push_back(node);
	SceneView::instance()->scene()->addChild(node, true);
    return true;
}

bool DrawUtility::addConeNodes(cnoid::SgGroupPtr node)
{
	Matrix3 yToz;
	yToz << 1, 0, 0, 0, 0, -1, 0, 1, 0;
    for(int i = 0; i < cones.size(); i++){
        double r = cones[i].radius;
        double l = cones[i].length;
        Vector3 color = cones[i].rgb;
        double alpha = 1.0 - cones[i].alpha;
        Matrix3 R = cones[i].R * yToz;
		Vector3 c = cones[i].pos;

		MeshGenerator generator;
		SgMeshPtr mesh = generator.generateCone(r, l);

		SgShapePtr shape = new SgShape();
		shape->setMesh(mesh);

		SgMaterialPtr material = shape->getOrCreateMaterial();
        material->setTransparency(alpha);
		material->setDiffuseColor(color);
		material->setEmissiveColor(color);

		SgPosTransform* pos = new SgPosTransform();
		pos->setTranslation(c);
		pos->setRotation(R);
		pos->addChild(shape);

		node->addChild(pos, false);
    }
	return true;
}

bool DrawUtility::displayBoxes()
{
	cnoid::SgGroupPtr node;
	return displayBoxes(node);
}

bool DrawUtility::displayBoxes(cnoid::SgGroupPtr& node)
{
	if (boxes.empty()) return true;
	node = new SgGroup();
	addBoxNodes(node);
	sgnodes_.push_back(node);
	SceneView::instance()->scene()->addChild(node, true);
    return true;
}

bool DrawUtility::addBoxNodes(cnoid::SgGroupPtr node) {
    for(int i = 0; i < boxes.size(); i++){
        Vector3 c = boxes[i].pos;
        Vector3 l = boxes[i].length;
        Vector3 color = boxes[i].rgb;
        Matrix3 R = boxes[i].R;

		MeshGenerator generator;
		SgMeshPtr mesh = generator.generateBox(l);

		SgShapePtr shape = new SgShape();
		shape->setMesh(mesh);

		SgMaterialPtr material = shape->getOrCreateMaterial();
		material->setTransparency(1.0 - boxes[i].alpha);
		material->setDiffuseColor(color);
		material->setEmissiveColor(color);

		SgPosTransform* pos = new SgPosTransform();
		pos->setTranslation(c);
		pos->setRotation(R);
		pos->addChild(shape);

		node->addChild(pos, false);
    }
	return true;
}

bool DrawUtility::displaySpheres()
{
	cnoid::SgGroupPtr node;
	return displaySpheres(node);
}

bool DrawUtility::displaySpheres(cnoid::SgGroupPtr& node)
{
	if (spheres.empty()) return true;
	node = new SgGroup();
	addSphereNodes(node);
	sgnodes_.push_back(node);
	SceneView::instance()->scene()->addChild(node, true);
    return true;
}

bool DrawUtility::addSphereNodes(cnoid::SgGroupPtr node)
{
	for(int i = 0; i < spheres.size(); i++){
        Vector3 c = spheres[i].pos;
        double r = spheres[i].radius;
        Vector3 color = spheres[i].rgb;
        double a = 1.0 - spheres[i].alpha;

		MeshGenerator generator;
		SgMeshPtr mesh = generator.generateSphere(r);

		SgShapePtr shape = new SgShape();
		shape->setMesh(mesh);

		SgMaterialPtr material = shape->getOrCreateMaterial();
        material->setTransparency(a);
		material->setDiffuseColor(color);
		material->setEmissiveColor(color);

		SgPosTransform* pos = new SgPosTransform();
		pos->setTranslation(c);
		pos->addChild(shape);

		node->addChild(pos, false);
    }
	return true;
}

bool DrawUtility::displayEllipsoids()
{
	cnoid::SgGroupPtr node;
	return displayEllipsoids(node);
}

bool DrawUtility::displayEllipsoids(cnoid::SgGroupPtr& node)
{
	if (ellipsoids.empty()) return true;
	node = new SgGroup();
	addEllipsoidNodes(node);
	sgnodes_.push_back(node);
	SceneView::instance()->scene()->addChild(node, true);
    return true;
}

bool DrawUtility::addEllipsoidNodes(cnoid::SgGroupPtr node)
{
	for(int i = 0; i < ellipsoids.size(); i++){
        Vector3 c = ellipsoids[i].pos;
        Vector3 l = ellipsoids[i].length;
        Vector3 color = ellipsoids[i].rgb;
        Matrix3 R = ellipsoids[i].R;
        double a = 1.0 - ellipsoids[i].alpha;

		MeshGenerator generator;
		SgMeshPtr mesh = generator.generateSphere(1);

		SgShapePtr shape = new SgShape();
		shape->setMesh(mesh);

		SgMaterialPtr material = shape->getOrCreateMaterial();
        material->setTransparency(a);
		material->setDiffuseColor(color);
		material->setEmissiveColor(color);

		SgScaleTransform* scale = new SgScaleTransform();
		scale->setScale(l);
		scale->addChild(shape);

		SgPosTransform* pos = new SgPosTransform();
		pos->setTranslation(c);
		pos->setRotation(R);
		pos->addChild(scale);

		node->addChild(pos, false);
    }
	return true;
}

bool DrawUtility::displayShapes()
{
	cnoid::SgGroupPtr node;
	return displayShapes(node);
}

bool DrawUtility::displayShapes(cnoid::SgGroupPtr& node)
{
	if (cylinders.empty() && cones.empty() && boxes.empty() &&
		spheres.empty() && ellipsoids.empty() ) return true;
	node = new SgGroup();
	addCylinderNodes(node);
	addConeNodes(node);
	addBoxNodes(node);
	addSphereNodes(node);
	addEllipsoidNodes(node);
	sgnodes_.push_back(node);
	SceneView::instance()->scene()->addChild(node, true);
	return true;
}

bool DrawUtility::displayPoints()
{
	cnoid::SgGroupPtr node;
	return displayPoints(node);
}

bool DrawUtility::displayPoints(cnoid::SgGroupPtr& node)
{
	if (points.empty()) return true;
	node = new SgGroup();
	addPointNodes(node);
	sgnodes_.push_back(node);
	SceneView::instance()->scene()->addChild(node, true);
    return true;
}

bool DrawUtility::addPointNodes(cnoid::SgGroupPtr node)
{
	SgPointSet* point_set = new SgPointSet();
	SgVertexArrayPtr ver_array = new SgVertexArray(points.size());
	SgColorArrayPtr color_array = new SgColorArray(colors.size());
	for (int i = 0; i < points.size(); i++) {
		ver_array->at(i) = points[i].cast<float>();
	}

	for (int i = 0; i < colors.size(); i++) {
		color_array->at(i) = colors[i].cast<float>();
	}

	point_set->setVertices(ver_array);
	point_set->setColors(color_array);
	point_set->normalIndices().clear();
	point_set->colorIndices().clear();
	point_set->setPointSize(1);

	node->addChild(point_set, false);

	return true;
}

bool DrawUtility::displayLines(const double alpha, const double lineWidth)
{
	cnoid::SgGroupPtr node;
	displayCoordinateAxes(alpha, lineWidth);
	return displayLines(node, alpha, lineWidth);
}

bool DrawUtility::displayLines(cnoid::SgGroupPtr& node, const double alpha, const double lineWidth)
{
	if (points.empty()) return true;
	node = new SgGroup();
	addLineNodes(node, alpha, lineWidth);
	sgnodes_.push_back(node);
	SceneView::instance()->scene()->addChild(node, true);
	return true;
}

bool DrawUtility::addLineNodes(cnoid::SgGroupPtr node, const double alpha, const double lineWidth)
{
	SgLineSet* line_set = new SgLineSet();
	SgVertexArrayPtr ver_array = new SgVertexArray();
	SgColorArrayPtr color_array = new SgColorArray();

	int index = 0;
	for (size_t i = 0; i < points.size(); i++) {
		ver_array->push_back(points[i].cast<float>());
		if (i % 2 == 1) line_set->addLine(i-1, i);
	}
	for (size_t i = 0; i < colors.size(); i++) {
		color_array->push_back(colors[i].cast<float>());
	}

	line_set->setVertices(ver_array);
	line_set->setColors(color_array);
	line_set->normalIndices().clear();
	line_set->colorIndices().clear();
	line_set->setLineWidth(lineWidth);

	SgMaterialPtr material = new SgMaterial();
    material->setTransparency(1.0 - alpha);

	line_set->setMaterial(material);
	node->addChild(line_set, false);

	return true;
}

bool DrawUtility::displayCoordinateAxes(const double alpha, const double lineWidth)
{
	cnoid::SgGroupPtr node;
	return displayCoordinateAxes(node, alpha, lineWidth);
}

bool DrawUtility::displayCoordinateAxes(cnoid::SgGroupPtr& node, const double alpha, const double lineWidth)
{
	if (axes.empty()) return true;
	node = new SgGroup();
	addCoordinateAxeNodes(node, alpha, lineWidth);
	sgnodes_.push_back(node);
	SceneView::instance()->scene()->addChild(node, true);
	return true;
}

bool DrawUtility::addCoordinateAxeNodes(cnoid::SgGroupPtr node, const double alpha, const double lineWidth)
{
	SgLineSet* line_set = new SgLineSet();
	SgVertexArrayPtr ver_array = new SgVertexArray();
	SgColorArrayPtr color_array = new SgColorArray();

	int index = 0;
	for (size_t i = 0; i < axes.size(); i++) {
		for (int j = 0; j < 3; j++) {
			ver_array->push_back(axes[i].pos.cast<float>());
			Vector3f v = (axes[i].pos + grasp::col(axes[i].R, j) * axes[i].size).cast<float>();
			ver_array->push_back(v);
			line_set->addLine(index, index+1);
			index += 2;
			color_array->push_back(axes[i].rgb.cast<float>());
			color_array->push_back(axes[i].rgb.cast<float>());
		}
	}

	line_set->setVertices(ver_array);
	line_set->setColors(color_array);
	line_set->normalIndices().clear();
	line_set->colorIndices().clear();
	line_set->setLineWidth(lineWidth);

	SgMaterialPtr material = new SgMaterial();
    material->setTransparency(1.0 - alpha);

	line_set->setMaterial(material);
	node->addChild(line_set, false);

	return true;
}

bool DrawUtility::displayTriangles(const double alpha)
{
	cnoid::SgGroupPtr node;
    return displayTriangles(alpha, node);
}

bool DrawUtility::displayTriangles(const double alpha, cnoid::SgGroupPtr& node)
{
	if (triangles.empty()) return true;
	node = new SgGroup();
	addTriangleNode(node, alpha);
	sgnodes_.push_back(node);
	SceneView::instance()->scene()->addChild(node, true);
    return true;
}

bool DrawUtility::addTriangleNode(cnoid::SgGroupPtr node, const double alpha)
{
    SgMeshPtr mesh = new SgMesh();

    SgVertexArray* vertices = mesh->getOrCreateVertices();
	for (int i = 0; i < points.size(); i++) {
        vertices->push_back(Vector3f(points[i][0], points[i][1], points[i][2]));
    }

    SgIndexArray& indices = mesh->triangleVertices();
    SgColorArray* colorArray = mesh->getOrCreateColors();
    for (int i = 0; i < triangles.size(); i++) {
        for (int j = 0; j < triangles[i].size(); j++) {
			colorArray->push_back(Vector3f(colors[i][0], colors[i][1], colors[i][2]));
            indices.push_back(triangles[i][j]);
        }
    }

    mesh->texCoordIndices() = mesh->triangleVertices();

    SgShapePtr shape = new SgShape();
    shape->setMesh(mesh);

    SgMaterial* material = shape->getOrCreateMaterial();
    material->setTransparency(1.0 - alpha);

    mesh->updateBoundingBox();

    MeshNormalGenerator normalGenerator;
    normalGenerator.generateNormals(mesh, 0);

	node->addChild(shape, false);

    return true;
}

void DrawUtility::clear()
{
    os <<  "Clear" << endl;

	for (int i = 0; i < sgnodes_.size(); i++) {
	    cnoid::SceneView::instance()->scene()->removeChild(sgnodes_[i], false);
	}
	redraw();
	sgnodes_.clear();

    points.clear();
    colors.clear();
    cylinders.clear();
    boxes.clear();
    ellipsoids.clear();
    cones.clear();
    triangles.clear();
	spheres.clear();
    axes.clear();
}

/**
 * @brief DrawUtility::deleteanobj remove the obj from the objList, if redraw
 *          is called, the object will not display in the view (deleted).
 *          However, if redraw is not called, the object remains there
 * @param obj a pointer to the OSGSceneObject that you would like to remove
 */
void DrawUtility::deleteanobj(cnoid::SgGroupPtr plothandle)
{
	cnoid::SceneView::instance()->scene()->removeChild(plothandle, true);
}

/**
 * @brief DrawUtility::redraw replots the OSGSceneObjectPtrs in the objList
 */
void DrawUtility::redraw()
{
	cnoid::SceneView::instance()->scene()->notifyUpdate();
}

