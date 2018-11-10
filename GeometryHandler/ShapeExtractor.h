#ifndef _GEOMETRYHANDLER_SHAPEEXTRACTOR_H_
#define _GEOMETRYHANDLER_SHAPEEXTRACTOR_H_

#include <vector>

#ifdef CNOID_15
#include <cnoid/SceneVisitor>
#endif
#include <cnoid/SceneDrawables>
#include <boost/bind.hpp>
#include <boost/function.hpp>

#ifdef CNOID_GE_16
#include <cnoid/PolymorphicFunctionSet>
#endif

namespace grasp {
#ifdef CNOID_GE_16
	class ShapeExtractorImpl;
#endif

	class ShapeExtractor
#ifdef CNOID_15
		: public cnoid::SceneVisitor
#endif
	{
	public:
		ShapeExtractor();

    bool extract(cnoid::SgNode* node, boost::function<void()> callback);

    cnoid::SgShape* currentShape() const;

		void collect(cnoid::SgNode* node, std::vector<cnoid::SgShape*>& shapes);

#ifdef CNOID_GE_16
		cnoid::PolymorphicFunctionSet<cnoid::SgNode>& visitFunctions();
#endif

	protected:
    virtual void visitShape(cnoid::SgShape* shape);
    virtual void visitPointSet(cnoid::SgPointSet* pointSet);
    virtual void visitLineSet(cnoid::SgLineSet* lineSet);
    virtual void visitLight(cnoid::SgLight* light);
    virtual void visitCamera(cnoid::SgCamera* camera);

	private:
    boost::function<void()> callback;
    cnoid::SgShape* currentShape_;
		bool shapeFound;

#ifdef CNOID_GE_16
		ShapeExtractorImpl* impl;
#endif
	};
}

#endif
