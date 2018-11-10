#include "ShapeExtractor.h"

using namespace grasp;


#ifdef CNOID_GE_16
namespace grasp {
class ShapeExtractorImpl
{
public:

	cnoid::PolymorphicFunctionSet<cnoid::SgNode> functions;
	boost::function<void()> callback;
	cnoid::SgShape* currentShape;
	bool shapeFound;

	ShapeExtractorImpl();
	void visitGroup(cnoid::SgGroup* group);
	void visitSwitch(cnoid::SgSwitch* switchNode); 
	void visitTransform(cnoid::SgTransform* transform);
	void visitPosTransform(cnoid::SgPosTransform* transform);
	void visitShape(cnoid::SgShape* shape);
};
}

ShapeExtractorImpl::ShapeExtractorImpl() {
	functions.setFunction<cnoid::SgGroup>(
																 [&](cnoid::SgNode* node){ visitGroup(static_cast<cnoid::SgGroup*>(node)); });
	functions.setFunction<cnoid::SgSwitch>(
																	[&](cnoid::SgNode* node){ visitSwitch(static_cast<cnoid::SgSwitch*>(node)); });
	functions.setFunction<cnoid::SgTransform>(
																		 [&](cnoid::SgNode* node){ visitTransform(static_cast<cnoid::SgTransform*>(node)); });
	functions.setFunction<cnoid::SgPosTransform>(
																				[&](cnoid::SgNode* node){ visitPosTransform(static_cast<cnoid::SgPosTransform*>(node)); });
	functions.setFunction<cnoid::SgShape>(
																 [&](cnoid::SgNode* node){ visitShape(static_cast<cnoid::SgShape*>(node)); });
	functions.updateDispatchTable();
}

void ShapeExtractorImpl::visitGroup(cnoid::SgGroup* group) {
	for (cnoid::SgGroup::const_iterator p = group->begin(); p != group->end(); ++p) {
		functions.dispatch(*p);
	}
}

void ShapeExtractorImpl::visitSwitch(cnoid::SgSwitch* switchNode) {
	if(switchNode->isTurnedOn()){
		visitGroup(switchNode);
	}
}

void ShapeExtractorImpl::visitTransform(cnoid::SgTransform* transform) {
	visitGroup(transform);
}

void ShapeExtractorImpl::visitPosTransform(cnoid::SgPosTransform* transform) {
	visitGroup(transform);
}

void ShapeExtractorImpl::visitShape(cnoid::SgShape* shape) {
	currentShape = shape;
	shapeFound = true;
	callback();
	currentShape = 0;
}
#endif

ShapeExtractor::ShapeExtractor() {
#ifdef CNOID_GE_16
	impl = new ShapeExtractorImpl;
#endif
}

void ShapeExtractor::visitShape(cnoid::SgShape* shape) {
	currentShape_ = shape;
	shapeFound = true;
	callback();
	currentShape_ = 0;
}


void ShapeExtractor::visitPointSet(cnoid::SgPointSet* pointSet) {
}


void ShapeExtractor::visitLineSet(cnoid::SgLineSet* lineSet) {
}


void ShapeExtractor::visitLight(cnoid::SgLight* light) {
}


void ShapeExtractor::visitCamera(cnoid::SgCamera* camera) {
}


bool ShapeExtractor::extract(cnoid::SgNode* node, boost::function<void()> callback) {
#ifdef CNOID_GE_16
		impl->callback = callback;
		impl->currentShape = 0;
		impl->shapeFound = false;
		impl->functions.dispatch(node);
		return impl->shapeFound;
#else
    this->callback = callback;
    currentShape_ = 0;
    shapeFound = false;
    node->accept(*this);
    return shapeFound;
#endif
}

cnoid::SgShape* ShapeExtractor::currentShape() const {
#ifdef CNOID_GE_16
	return impl->currentShape;
#else
	return currentShape_;
#endif
}

static void collectShapes(ShapeExtractor* extractor, std::vector<cnoid::SgShape*>& shapes) {
	cnoid::SgShape* shape = extractor->currentShape();
	if (shape) {
		shapes.push_back(shape);
	}
}

void ShapeExtractor::collect(cnoid::SgNode* node, std::vector<cnoid::SgShape*>& shapes) {
	extract(node, boost::bind(collectShapes, this, boost::ref(shapes)));
}

#ifdef CNOID_GE_16
cnoid::PolymorphicFunctionSet<cnoid::SgNode>& ShapeExtractor::visitFunctions() {
	return impl->functions;
}
#endif
