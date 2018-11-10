// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include <iostream>
#include <fstream>
#include "GeometryBar.h"
#include <cnoid/ItemTreeView>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/MessageView>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/Archive>	/* modified by qtconv.rb 0th rule*/

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <cnoid/ScenePieces>
//#include <cnoid/SceneBodyManager>
  #ifdef CNOID_ENABLE_OSG
  #include <cnoid/OSGSceneBodyManager>
  #include "GeometryHandleSceneBody.h"
  #endif
#else
#include "../Grasp/ColdetConverter.h"
#endif

#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
// #include <glibmm/i18n.h>	/* modified by qtconv.rb 5th rule*/


#include "../Grasp/GraspController.h"
#include "../Grasp/PlanInterface.h"

//uto
#include "GeometryHandle.h"
#include "SgVisitShapeGet.h"

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

int GeometryBar::count = 0;

ClusteringDialog::ClusteringDialog() : QDialog(cnoid::MainWindow::instance()) {

	setWindowTitle("clustering dialog");

	QVBoxLayout* vbox = new QVBoxLayout();
	setLayout(vbox);

	QHBoxLayout* hbox = new QHBoxLayout();
	hbox->addWidget(new QLabel("Clutering target number"));
	targetNumber.setAlignment(Qt::AlignCenter);
	targetNumber.setRange(1, 100);
	targetNumber.setValue(1);
	hbox->addWidget(&targetNumber);
	hbox->addStretch();
	vbox->addLayout(hbox);

	vbox->addStretch();

	cnoid::PushButton* okButton = new cnoid::PushButton("&OK");
	okButton->setDefault(true);
	connect(okButton, SIGNAL(clicked()), this, SLOT(accept()));
	okButton->sigClicked().connect(boost::bind(&ClusteringDialog::okClicked, this));

	vbox->addWidget(okButton);
}


void ClusteringDialog::okClicked(){

	if(GeometryBar::instance()->targetBodyItems.size()!=1){
		GeometryBar::instance()->os <<  "Please selecet one bodyitem" << endl;
		return;
	}
	BodyItemPtr targetBodyItem = GeometryBar::instance()->targetBodyItems[0];
	SgClusterRenderer::SgLastRenderer(0,true);
	SgGroupPtr node  = (SgGroup*)targetBodyItem->body()->link(0)->shape();
	SgVisitShapeGet visit;
#ifdef CNOID_GE_16
	visit.getShapes(node);
#else
	node->accept(visit);
#endif
	if(visit.shape.size()==0){
		MessageView::mainInstance()->cout()  << "no shape node"  << visit.shape.size() << endl;
		return;
	}

	SgClusterRenderer* cr = SgClusterRenderer::SgLastRenderer(0,false);
	ObjectShape* object = NULL;
	if(cr){
		object = cr->object;
	}
	else{
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		object = new ObjectShape(targetBodyItem->body()->link(0)->coldetModel());
#else
		object = new ObjectShape(
			ColdetConverter::ConvertFrom(targetBodyItem->body()->link(0)->collisionShape()));
#endif
		object->makeNbr();
		object->initialClustersFromOneFace();
	}
	object->calcClusterBinaryTree(targetNumber.value());

	if(!cr){
		cr = new SgClusterRenderer(object);
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		visit.shape[0]->mesh()->triangles().clear();
#else
		visit.shape[0]->mesh()->triangleVertices().clear();
#endif
		node->addChild(cr);
	}
	ItemTreeView::mainInstance()->checkItem(targetBodyItem,false);
	MessageView::mainInstance()->flush();
	ItemTreeView::mainInstance()->checkItem(targetBodyItem,true);
	MessageView::mainInstance()->flush();
}


SaveClusteringDialog::SaveClusteringDialog() : QDialog(cnoid::MainWindow::instance()) {

	setWindowTitle("Save clusters diarlog");

	QVBoxLayout* vbox = new QVBoxLayout();
	setLayout(vbox);

	QHBoxLayout* hbox = new QHBoxLayout();
	hbox->addWidget(new QLabel("Save Clutering depth"));
	targetNumber.setAlignment(Qt::AlignCenter);
	targetNumber.setRange(1, 100);
	targetNumber.setValue(1);
	hbox->addWidget(&targetNumber);
	hbox->addStretch();
	vbox->addLayout(hbox);

	vbox->addStretch();

	cnoid::PushButton* okButton = new cnoid::PushButton("&OK");
	okButton->setDefault(true);
	connect(okButton, SIGNAL(clicked()), this, SLOT(accept()));
	okButton->sigClicked().connect(boost::bind(&SaveClusteringDialog::okClicked, this));

	vbox->addWidget(okButton);
}


void SaveClusteringDialog::okClicked(){

	if(GeometryBar::instance()->targetBodyItems.size()!=1){
		GeometryBar::instance()->os <<  "Please selecet one bodyitem" << endl;
		return;
	}
	BodyItemPtr targetBodyItem = GeometryBar::instance()->targetBodyItems[0];
	SgClusterRenderer::SgLastRenderer(0,true);
	SgGroupPtr node  = (SgGroup*)targetBodyItem->body()->link(0)->shape();
	SgVisitShapeGet visit;
#ifdef CNOID_GE_16
	visit.getShapes(node);
#else
	node->accept(visit);
#endif
	if(visit.shape.size()==0){
		MessageView::mainInstance()->cout()  << "no shape node"  << visit.shape.size() << endl;
		return;
	}

	SgClusterRenderer* cr = SgClusterRenderer::SgLastRenderer(0,false);
	ObjectShape* object = NULL;
	if(cr){
		object = cr->object;
	}
	else{
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		object = new ObjectShape(targetBodyItem->body()->link(0)->coldetModel());
#else
		object = new ObjectShape(
			ColdetConverter::ConvertFrom(targetBodyItem->body()->link(0)->collisionShape()));
#endif
		object->makeNbr();
		object->initialClustersFromOneFace();
	}

#ifdef CNOID_10_11
	boost::filesystem::path fullpath( targetBodyItem->modelFilePath() );
#elif defined(CNOID_10_11_12_13) || defined(CNOID_14)
	boost::filesystem::path fullpath( targetBodyItem->lastAccessedFilePath() );
#else
	boost::filesystem::path fullpath( targetBodyItem->filePath() );
#endif
//	cout << fullpath.string() << endl;
	int pos = fullpath.string().rfind(".");
	string yamlname = fullpath.string().substr(0,pos) + ".yaml";
	cout << yamlname << endl;
	ofstream fout(yamlname.c_str());
	fout << "modelFile: " <<fullpath.filename().string() << endl;

	object->saveClusters(fout, targetNumber.value());
	fout.close();
}


GeometryBar* GeometryBar::instance()
{
	static GeometryBar* instance = new GeometryBar();
	return instance;
}



GeometryBar::GeometryBar()
	: ToolBar("GeometryBar"),
	  mes(*MessageView::mainInstance()),
   	os (MessageView::mainInstance()->cout() )
{

	addSeparator();

	addLabel(("=Geometry="));

	addButton(("Clustering"), ("clustering of the selected base item"))->
		sigClicked().connect(bind(&GeometryBar::onClusteringButtonClicked, this));

	addButton(("Target number clustering"), ("clustering of the selected base item"))->
		sigClicked().connect(bind(&QDialog::show, &clustering));

	addButton(("Step clustering"), ("1 step clustering of the selected base item"))->
		sigClicked().connect(bind(&GeometryBar::onStepButtonClicked, this));

	addButton(("Save clusters"), ("Save clusters of the selected base item"))->
		sigClicked().connect(bind(&QDialog::show, &saveClustering));

	addButton(("Load clusters"), ("Load clusters from yaml file of the selected model"))->
		sigClicked().connect(bind(&GeometryBar::onLoadClustersButtonClicked, this));

   addButton(("CalClustering"), ("clustering & calc feature value"))->
    sigClicked().connect(bind(&GeometryBar::onCalcClusteringButtonClicked, this));

	addSeparator();
	// show_all_children();	/* modified by qtconv.rb 7th rule*/

	ItemTreeView::mainInstance()->sigSelectionChanged().connect(
		bind(&GeometryBar::onItemSelectionChanged, this, _1));
	count++;
}


GeometryBar::~GeometryBar()
{
	connectionOfCurrentBodyItemDetachedFromRoot.disconnect();
	count--;
}


/**
   \todo ItemTreeView::sigSelectionChanged() should be emitted
   after the final selection state has been determined.
*/
bool GeometryBar::makeSingleSelection(BodyItemPtr bodyItem)
{
	ItemTreeView* tree = ItemTreeView::mainInstance()->mainInstance();

	ItemList<BodyItem> prevSelected = selectedBodyItems_;

	for(size_t i=0; i < prevSelected.size(); ++i){
#ifdef CNOID_10_11_12_13
		BodyItem* item = prevSelected[i];
#else
		BodyItem* item = prevSelected.get(i);
#endif
		if(item != bodyItem && tree->isItemSelected(item)){
			tree->selectItem(item, false);
		}
	}

	bool isSelected = tree->isItemSelected(bodyItem);
	if(!isSelected){
		isSelected = tree->selectItem(bodyItem, true);
	}

	return isSelected;
}


void GeometryBar::onItemSelectionChanged(const ItemList<BodyItem>& bodyItems)
{
	bool selectedBodyItemsChanged = false;
	if(count < 1) return;
	if(selectedBodyItems_ != bodyItems){
		selectedBodyItems_ = bodyItems;
		selectedBodyItemsChanged = true;
	}

	BodyItemPtr firstItem = bodyItems.toSingle();

	if(firstItem && firstItem != currentBodyItem_){
		currentBodyItem_ = firstItem;
		connectionOfCurrentBodyItemDetachedFromRoot.disconnect();
		connectionOfCurrentBodyItemDetachedFromRoot = currentBodyItem_->sigDetachedFromRoot().connect(
			bind(&GeometryBar::onBodyItemDetachedFromRoot, this));
		sigCurrentBodyItemChanged_(currentBodyItem_.get());
	}

	if(selectedBodyItemsChanged){
		sigBodyItemSelectionChanged_(selectedBodyItems_);
	}

	targetBodyItems.clear();
	if(selectedBodyItems_.empty()){
//		if(currentBodyItem_){
//			targetBodyItems.push_back(currentBodyItem_);
//		}
	} else {
		targetBodyItems = selectedBodyItems_;
	}
}


void GeometryBar::onBodyItemDetachedFromRoot()
{
	currentBodyItem_ = 0;
	connectionOfCurrentBodyItemDetachedFromRoot.disconnect();
	sigCurrentBodyItemChanged_(0);
}


void GeometryBar::onObjectButtonClicked()
{
	if(targetBodyItems.size()==1){
		PlanBase::instance()->SetGraspedObject(targetBodyItems[0]);
		os << PlanBase::instance()->targetObject->bodyItemObject->name() << " is grasped object"<< endl;
	}else{
		os <<  "Please selecet one bodyitem" << endl;
	}

}

void GeometryBar::onClusteringButtonClicked(){

	if(targetBodyItems.size()!=1){
		os <<  "Please selecet one bodyitem" << endl;
		return;
	}

	SgClusterRenderer::SgLastRenderer(0,true);
	SgGroupPtr node  = (SgGroup*)targetBodyItems[0]->body()->link(0)->shape();
	SgVisitShapeGet visit;
#ifdef CNOID_GE_16
	visit.getShapes(node);
#else
	node->accept(visit);
#endif
	if(visit.shape.size()==0){
		os  << "no shape node"  << visit.shape.size() << endl;
		return;
	}

	SgClusterRenderer* cr = SgClusterRenderer::SgLastRenderer(0,false);
	ObjectShape* object = NULL;
	if(cr){
		object = cr->object;
	}
	else{
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		object = new ObjectShape(targetBodyItems[0]->body()->link(0)->coldetModel());
#else
		object = new ObjectShape(
			ColdetConverter::ConvertFrom(targetBodyItems[0]->body()->link(0)->collisionShape()));
#endif
		object->makeNbr();
		object->initialClustersFromOneFace();
	}
	object->calcClusterBinaryTree();

	if(!cr){
		cr = new SgClusterRenderer(object);
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		visit.shape[0]->mesh()->triangles().clear();
#else
		visit.shape[0]->mesh()->triangleVertices().clear();
#endif
		node->addChild(cr);
	}
	ItemTreeView::mainInstance()->checkItem(targetBodyItems[0],false);
	MessageView::mainInstance()->flush();
	ItemTreeView::mainInstance()->checkItem(targetBodyItems[0],true);
	MessageView::mainInstance()->flush();
}

void GeometryBar::onCalcClusteringButtonClicked(){

	if(targetBodyItems.size()!=1){
		os <<  "Please selecet one bodyitem" << endl;
		return;
	}

	SgClusterRendererFunc::SgLastRenderer(0,true);
	SgGroupPtr node  = (SgGroup*)targetBodyItems[0]->body()->link(0)->shape();
	SgVisitShapeGet visit;
#ifdef CNOID_GE_16
	visit.getShapes(node);
#else
	node->accept(visit);
#endif
	if(visit.shape.size()==0){
		os  << "no shape node"  << visit.shape.size() << endl;
		return;
	}

	SgClusterRendererFunc* cr = SgClusterRendererFunc::SgLastRenderer(0,false);
	ObjectShape* object = NULL;
	if(cr){
		object = cr->object;
	}
	else{
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		object = new ObjectShape(targetBodyItems[0]->body()->link(0)->coldetModel());
#else
		object = new ObjectShape(
			ColdetConverter::ConvertFrom(targetBodyItems[0]->body()->link(0)->collisionShape()));
#endif
		object->makeNbr();
		object->initialClustersFromOneFace();
	}
  int targetnum=0;
  std::cout << "target number clustering =" << '\n';
  std::cin >> targetnum;
	object->calcClusterBinaryTreeFV(targetnum);

	if(!cr){
		cr = new SgClusterRendererFunc(object);
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		visit.shape[0]->mesh()->triangles().clear();
#else
		visit.shape[0]->mesh()->triangleVertices().clear();
#endif
	   node->addChild(cr);
	}
	ItemTreeView::mainInstance()->checkItem(targetBodyItems[0],false);
	MessageView::mainInstance()->flush();
	ItemTreeView::mainInstance()->checkItem(targetBodyItems[0],true);
	MessageView::mainInstance()->flush();
}



void GeometryBar::onStepButtonClicked(){

	if(targetBodyItems.size()!=1){
		os <<  "Please selecet one bodyitem" << endl;
		return;
	}
	BodyItemPtr targetBodyItem = targetBodyItems[0];
	SgClusterRenderer::SgLastRenderer(0,true);
	SgGroupPtr node  = (SgGroup*)targetBodyItem->body()->link(0)->shape();
	SgVisitShapeGet visit;
#ifdef CNOID_GE_16
	visit.getShapes(node);
#else
	node->accept(visit);
#endif
	if(visit.shape.size()==0){
		MessageView::mainInstance()->cout()  << "no shape node"  << visit.shape.size() << endl;
		return;
	}

	SgClusterRenderer* cr = SgClusterRenderer::SgLastRenderer(0,false);
	ObjectShape* object = NULL;
	if(cr){
		object = cr->object;
	}
	else{
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		object = new ObjectShape(targetBodyItem->body()->link(0)->coldetModel());
#else
		object = new ObjectShape(
			ColdetConverter::ConvertFrom(targetBodyItem->body()->link(0)->collisionShape()));
#endif
		object->makeNbr();
		object->initialClustersFromOneFace();
	}
	object->calcClusterBinaryTreeStep();

	if(!cr){
		cr = new SgClusterRenderer(object);
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		visit.shape[0]->mesh()->triangles().clear();
#else
		visit.shape[0]->mesh()->triangleVertices().clear();
#endif
		node->addChild(cr);
	}
	ItemTreeView::mainInstance()->checkItem(targetBodyItem,false);
	MessageView::mainInstance()->flush();
	ItemTreeView::mainInstance()->checkItem(targetBodyItem,true);
	MessageView::mainInstance()->flush();


}

void GeometryBar::onLoadClustersButtonClicked(){

	if(targetBodyItems.size()!=1){
		os <<  "Please selecet one bodyitem" << endl;
		return;
	}

	SgClusterRenderer::SgLastRenderer(0,true);
	SgGroupPtr node  = (SgGroup*)targetBodyItems[0]->body()->link(0)->shape();
	SgVisitShapeGet visit;
#ifdef CNOID_GE_16
	visit.getShapes(node);
#else
	node->accept(visit);
#endif
	if(visit.shape.size()==0){
		os  << "no shape node"  << visit.shape.size() << endl;
		return;
	}

	SgClusterRenderer* cr = SgClusterRenderer::SgLastRenderer(0,false);
	ObjectShape* object = NULL;
	if(cr){
		object = cr->object;
	}
	else{
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		object = new ObjectShape(targetBodyItems[0]->body()->link(0)->coldetModel());
#else
		object = new ObjectShape(
			ColdetConverter::ConvertFrom(targetBodyItems[0]->body()->link(0)->collisionShape()));
#endif
		object->makeNbr();
	}

	const cnoid::Mapping& clusterSettings =  *targetBodyItems[0]->body()->info();
	if( ! object->loadClusters(clusterSettings) ){
		os << "no cluster settings" << endl;
		return;
	}

	if(!cr){
		cr = new SgClusterRenderer(object);
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		visit.shape[0]->mesh()->triangles().clear();
#else
		visit.shape[0]->mesh()->triangleVertices().clear();
#endif
		node->addChild(cr);
	}
	ItemTreeView::mainInstance()->checkItem(targetBodyItems[0],false);
	MessageView::mainInstance()->flush();
	ItemTreeView::mainInstance()->checkItem(targetBodyItems[0],true);
	MessageView::mainInstance()->flush();
}

bool GeometryBar::storeState(Archive& archive)
{
	return true;
}


bool GeometryBar::restoreState(const Archive& archive)
{
	return true;
}
