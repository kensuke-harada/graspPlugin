// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include <iostream>
#include "GraspBar.h"
#include <cnoid/ItemTreeView>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/MessageView>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/SceneView>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/Archive>	/* modified by qtconv.rb 0th rule*/
#include <boost/bind.hpp>
#include <boost/format.hpp>
// #include <glibmm/i18n.h>	/* modified by qtconv.rb 5th rule*/

#include "GraspController.h"
#include "PlanInterface.h"

#include "GraspBodyItem.h"
#include "ForceClosureTest.h"
#include "SgStringRenderer.h"
#include "Camera.h"
#include "CameraSelectDialog.h"
#include "HandSelectDialog.h"
#if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)
#include <cnoid/SceneShape>
#endif

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

int GraspBar::count = 0;

SaveGraspPatternDialog::SaveGraspPatternDialog() : QDialog(cnoid::MainWindow::instance()) {

    setWindowTitle("Save grasp pattern");

    QVBoxLayout* vbox = new QVBoxLayout();
    setLayout(vbox);

    QHBoxLayout* hbox = new QHBoxLayout();
    hbox->addWidget(new QLabel("Rotational direction [x: 0, y: 1, z: 2]: "));
    rotationDirection.setAlignment(Qt::AlignCenter);
    rotationDirection.setRange(0, 2);
    rotationDirection.setValue(0);
    hbox->addWidget(&rotationDirection);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout();
    hbox->addWidget(new QLabel("Rotational angle [0-180]: "));
    rotationAngle.setAlignment(Qt::AlignCenter);
    rotationAngle.setRange(0, 180);
    rotationAngle.setValue(0);
    hbox->addWidget(&rotationAngle);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout();
    hbox->addWidget(new QLabel("Translational direction [x: 0, y: 1, z: 2]: " ));
    translationDirection.setAlignment(Qt::AlignCenter);
    translationDirection.setRange(0, 2);
    translationDirection.setValue(0);
    hbox->addWidget(&translationDirection);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout();
    hbox->addWidget(new QLabel( "Translation length:" ));
    translationLength.setAlignment(Qt::AlignCenter);
    translationLength.setRange(0, 1.0);
    translationLength.setSingleStep(0.01);
    translationLength.setValue(0);
    hbox->addWidget(&translationLength);
    hbox->addStretch();
    vbox->addLayout(hbox);

    rotationYaxis = new QCheckBox("Rotate about y axis for 180[deg]? [y: check]");
    vbox->addWidget(rotationYaxis);

    vbox->addStretch();

    cnoid::PushButton* okButton = new cnoid::PushButton("&OK");
    okButton->setDefault(true);
    connect(okButton, SIGNAL(clicked()), this, SLOT(accept()));
    okButton->sigClicked().connect(boost::bind(&SaveGraspPatternDialog::okClicked, this));

    vbox->addWidget(okButton);
}

void SaveGraspPatternDialog::okClicked(){
//	rotationDirection;
    GraspController::instance()->
        saveGraspPattern(rotationDirection.value(), rotationAngle.value(), translationDirection.value(), translationLength.value(), rotationYaxis->isChecked());
}


SelectArmDialog::SelectArmDialog() : QDialog(cnoid::MainWindow::instance()) {

    setWindowTitle("Multiple arms");

    QVBoxLayout* vbox = new QVBoxLayout();
    setLayout(vbox);

    QHBoxLayout* hbox = new QHBoxLayout();
    hbox->addWidget(new QLabel("Select a main arm"));
    armNumber.setAlignment(Qt::AlignCenter);
    armNumber.setRange(0, PlanBase::instance()->armsList.size()-1);
    armNumber.setValue(0);
    hbox->addWidget(&armNumber);
    hbox->addStretch();
    vbox->addLayout(hbox);

    cnoid::PushButton* okButton = new cnoid::PushButton("&OK");
    okButton->setDefault(true);
    connect(okButton, SIGNAL(clicked()), this, SLOT(accept()));
    okButton->sigClicked().connect(boost::bind(&SelectArmDialog::okClicked, this));

    vbox->addWidget(okButton);
}


void SelectArmDialog::okClicked(){
    PlanBase::instance()->targetArmFinger = PlanBase::instance()->armsList[armNumber.value()];
    MessageView::mainInstance()->cout() << PlanBase::instance()->targetArmFinger->name << " is target arm"<< endl;
}

GraspBar* GraspBar::instance()
{
    static GraspBar* instance = new GraspBar();
    return instance;
}



GraspBar::GraspBar()
    : ToolBar("GraspBar"),
      mes(*MessageView::mainInstance()),
    os (MessageView::mainInstance()->cout() )
{
#if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)
	setVisibleByDefault(true);
#endif

    addSeparator();

    addLabel(("=Planner="));

    addButton(("SetObject"), ("Set Selected bodyitem as a grasped object"))->
        sigClicked().connect(bind(&GraspBar::onObjectButtonClicked, this));	/* modified by qtconv.rb 6th rule*/

    addButton(("SetRobot"), ("Set the preset bodyitem as a grasping robot"))->
        sigClicked().connect(bind(&GraspBar::onRobotButtonClicked, this));	/* modified by qtconv.rb 6th rule*/

    addButton(("Grasp"), ("Grasp planning start"))->
        sigClicked().connect(bind(&GraspBar::onGraspButtonClicked, this));	/* modified by qtconv.rb 6th rule*/

    addButton(("Place"), ("Place planning start"))->
        sigClicked().connect(bind(&GraspBar::onPlaceButtonClicked, this));	/* modified by qtconv.rb 6th rule*/

    addButton(("Pick&Place"), ("Pick and place planning start"))->
        sigClicked().connect(bind(&GraspBar::onPickAndPlaceButtonClicked, this));	/* modified by qtconv.rb 6th rule*/

    addButton(("Stop"), ("Grasp planning stop"))->
        sigClicked().connect(bind(&GraspBar::onStopButtonClicked, this));	/* modified by qtconv.rb 6th rule*/

    addSeparator();

    addButton(("SetEnv"), ("Set the preset bodyitem as a sorrounding environment"))->
        sigClicked().connect(bind(&GraspBar::onEnvironButtonClicked, this));	/* modified by qtconv.rb 6th rule*/

    addButton(("RemoveEnv"), ("Remove the preset bodyitem from a sorrounding environment"))->
        sigClicked().connect(bind(&GraspBar::onRemoveEnvButtonClicked, this));	/* modified by qtconv.rb 6th rule*/

    addSeparator();

    addButton(("SaveGraspPattern"), ("Save grasp pattern"))->
        sigClicked().connect(bind(&QDialog::show, &saveGraspPattern));
        //sigClicked().connect(bind(&GraspBar::onSaveGPButtonClicked, this));

    addButton(("SelectGraspPattern"), ("Load selected grasp pattern"))->
        sigClicked().connect(bind(&GraspBar::onSelectGraspPattern, this));	/* modified by qtconv.rb 6th rule*/

    addButton(("GetInterObject"), ("output Inter object information to terminal"))->
        sigClicked().connect(bind(&GraspBar::onDisplayGRCPositionButtonClicked, this));	/* modified by qtconv.rb 6th rule*/

    addButton(("CloseFingers"), ("Close Fingers"))->
        sigClicked().connect(bind(&GraspBar::onCloseButtonClicked, this));	/* modified by qtconv.rb 6th rule*/

	addSeparator();

	addButton(("SetCamera"), ("Select a camera"))->
        sigClicked().connect(bind(&GraspBar::onSelectCameraButtonClicked, this));

    addSeparator();

	addButton(("AttachHand"), ("Attach hand"))->
		sigClicked().connect(bind(&GraspBar::onAttachHandButtonClicked, this));

	addButton(("DetachHand"), ("Detach hand"))->
		sigClicked().connect(bind(&GraspBar::onDetachHandButtonClicked, this));

	addSeparator();

	addButton(("AddAssemblyObj"), ("Append the object to assembly object list"))->
		sigClicked().connect(bind(&GraspBar::onSetAssemblyObjButtonClicked, this));

	addButton(("RemoveAsseblyObj"), ("Remove the object from assembly object list"))->
		sigClicked().connect(bind(&GraspBar::onRemoveAssemblyObjButtonClicked, this));

	addSeparator();

#ifdef TEST_FUNCTION
    addButton(("Test"), ("ExtSceneBodyTest"))->
        sigClicked().connect(bind(&GraspBar::onExtSceneBodyTestClicked, this));
#endif

    ItemTreeView::mainInstance()->sigSelectionChanged().connect(
        bind(&GraspBar::onItemSelectionChanged, this, _1));
    count++;
}


GraspBar::~GraspBar()
{
    connectionOfCurrentBodyItemDetachedFromRoot.disconnect();
    count--;
}


/**
   \todo ItemTreeView::sigSelectionChanged() should be emitted
   after the final selection state has been determined.
*/
bool GraspBar::makeSingleSelection(BodyItemPtr bodyItem)
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


void GraspBar::onItemSelectionChanged(const ItemList<BodyItem>& bodyItems)
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
            bind(&GraspBar::onBodyItemDetachedFromRoot, this));
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


void GraspBar::onBodyItemDetachedFromRoot()
{
    currentBodyItem_ = 0;
    connectionOfCurrentBodyItemDetachedFromRoot.disconnect();
    sigCurrentBodyItemChanged_(0);
}


void GraspBar::onObjectButtonClicked()
{
    if(targetBodyItems.size()==1){
        PlanBase::instance()->SetGraspedObject(targetBodyItems[0]);
        os << PlanBase::instance()->targetObject->bodyItemObject->name() << " is grasped object"<< endl;
    }else{
        os <<  "Please select one bodyitem." << endl;
    }
}

void GraspBar::onRobotButtonClicked()
{
    if(targetBodyItems.size()!=1){
        os <<  "Please select one bodyitem." << endl;
        return;
    }
    if( !(PlanBase::instance()->targetArmFinger) || (PlanBase::instance()->bodyItemRobot()!=targetBodyItems[0])  ){
        if( PlanBase::instance()->SetGraspingRobot(targetBodyItems[0]) ){
            os << PlanBase::instance()->bodyItemRobot()->name() << " is grasping robot"<< endl;
        } else {
            os << targetBodyItems[0]->name()  << " does not have graspping robot setting"<< endl;
            return;
        }
    }
    if(PlanBase::instance()->armsList.size() > 1 ){
        SelectArmDialog* SADialog = new SelectArmDialog ();
        SADialog->show();
    }
}



void GraspBar::onEnvironButtonClicked()
{
    if(targetBodyItems.size()>0){
        for(unsigned int i=0;i<targetBodyItems.size();i++){
            PlanBase::instance()->SetEnvironment(targetBodyItems[i]);
            os << targetBodyItems[i]->name() << " is sorrounding environment"<< endl;
        }
    }else{
        os <<  "Please select more than one bodyitem." << endl;
    }
}

void GraspBar::onRemoveEnvButtonClicked()
{
    if(targetBodyItems.size()>0){
        for(unsigned int i=0;i<targetBodyItems.size();i++){
            PlanBase::instance()->RemoveEnvironment(targetBodyItems[i]);
            os << targetBodyItems[i]->name() << " is removed from environment"<< endl;
        }
    }else{
        os <<  "Please select more than one bodyitem." << endl;
    }
}


void GraspBar::onGraspButtonClicked()
{
    bool init = PlanBase::instance()->initial();
    if(!init){
        os << "Failed: Grasp Planning Initial" << endl;
        return;
    }

    try{
        PlanInterface::instance()->doGraspPlanning();
    }
    catch(int number){
        PlanBase::instance()->stopFlag=false;
        os <<  "Grasp Planning is stopped" << endl;
    }
    PlanBase::instance()->flush();
//	cout << "test" << endl;
}

void GraspBar::onPlaceButtonClicked()
{
    try{
        PlanInterface::instance()->doPlacePlanning();
    }
    catch(int number){
        PlanBase::instance()->stopFlag=false;
        os <<  "Place Planning is stopped" << endl;
    }
    PlanBase::instance()->flush();
}

void GraspBar::onPickAndPlaceButtonClicked()
{
    try{
        PlanInterface::instance()->doPickAndPlacePlanning();
    }
    catch(int number){
        PlanBase::instance()->stopFlag=false;
        os <<  "Grasp Planning is stopped" << endl;
    }
    PlanBase::instance()->flush();
}

void GraspBar::onStopButtonClicked()
{
    PlanBase::instance()->stopFlag = true;
    os <<  "Stop button is pressed" << endl;
}


void GraspBar::onSaveGPButtonClicked(){
    GraspController::instance()->saveGraspPattern();
}

void GraspBar::onSelectGraspPattern(){
    bool init = PlanBase::instance()->initial();
    if(!init){
        os << "Failed: Grasp Planning Initial" << endl;
        return;
    }
    GraspController::instance()->loadAndSelectGraspPattern();
}

void GraspBar::onDisplayGRCPositionButtonClicked(){
//	GraspController::instance()->doDisplayGRCPosition();
    PlanBase* pb = PlanBase::instance();
    for(int i=0;i<pb->interObjectList.size();i++){
        pb->interObjectList[i].outputRelativePosition();
    }
}
void GraspBar::onExtSceneBodyTestClicked(){
	
/*	Vector3 op,fp,normal;
	cout << "area "<< GraspController::getContactArea(PlanBase::instance()->fingers(0)->linkObjPair[0],op,fp,normal) << endl;
	cout << op.transpose() <<endl;
	cout << fp.transpose() <<endl;
	cout << normal.transpose() <<endl;
	
	return;
*/
	if(targetBodyItems.size()!=1){
		os <<  "Please selecet one bodyitem" << endl;
		return;
	}
	static SgStringRenderer * cr=NULL;
	if(!cr){
		SgGroupPtr node  = (SgGroup*)targetBodyItems[0]->body()->link(0)->shape();
		cr = new SgStringRenderer ("test", Vector3(0,0,0.5));
		node->addChild(cr);
	}
	static int count=0;
	stringstream ss;
	ss << "count\n "<<count++; 
	cr->setString(ss.str());
	
	ItemTreeView::mainInstance()->checkItem(targetBodyItems[0],false);
	MessageView::mainInstance()->flush();
	ItemTreeView::mainInstance()->checkItem(targetBodyItems[0],true);
	MessageView::mainInstance()->flush();
	return ;

        SgVertexArrayPtr vertices = new SgVertexArray();
        vertices->reserve(10);
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
        SgVector3 vertex = SgVector3(1,1,1);
#else
		Vector3f vertex(1, 1, 1);
#endif
        vertices->push_back(vertex);
		SgPointSetPtr pointSet = new SgPointSet();
        pointSet->setVertices(vertices);

        if(pointSet){
            SgInvariantGroupPtr group = new SgInvariantGroup();
            group->addChild(pointSet);

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
        	SceneView::instance()->addEntity(group);
#else
			SceneView::instance()->scene()->addChild(group, true);
#endif
            os << pointSet->vertices()->size() << " points have been added.";
//		SceneView::instance()->removeEntity(group);
        }
         return;

    if(targetBodyItems.size()!=1){
        os <<  "Please selecet one bodyitem" << endl;
        return;
    }
    //ForceClosureTest::instance()->forceClosureTestOnly(targetBodyItems[0]->body()->link(0)->coldetModel());
    //return;

//	SgPointSetPtr pointSet = cnoid::loadPCD(filename);




    GraspBodyItem::factory(TestSceneBody::create);

    BodyItemPtr temp = new GraspBodyItem(*targetBodyItems[0]);
    targetBodyItems[0]->parentItem()->addChildItem (temp);

    ItemTreeView::mainInstance()->checkItem(targetBodyItems[0],false);
    ItemTreeView::mainInstance()->checkItem(temp,true);
}

void GraspBar::onCloseButtonClicked()
{
    GraspController::instance()->closeFingers();
}

void GraspBar::onSelectCameraButtonClicked()
{
	CameraSelectDialog* CSDialog = new CameraSelectDialog();
	CSDialog->exec();
	delete CSDialog;
}

void GraspBar::onAttachHandButtonClicked()
{
	if ((!PlanBase::instance()->robotBody())) {
		os << "There is no robot!" << endl;
		return;
	}
	HandSelectDialog* HSDialog = new HandSelectDialog();
	if (HSDialog->hasAttachHandList()) {
		HSDialog->setAttachMode();
		HSDialog->exec();
	} else {
		os <<  "There are no attachable hands!" << endl;
	}
	delete HSDialog;
}

void GraspBar::onDetachHandButtonClicked()
{
	if (!(PlanBase::instance()->robotBody())) {
		os << "There is no robot!" << endl;
		return;
	}
	HandSelectDialog* HSDialog = new HandSelectDialog();
	if (HSDialog->hasDetachHandList()) {
		HSDialog->setDetachMode();
		HSDialog->exec();
	} else {
		os <<  "There are no detachable hands!" << endl;
	}
	delete HSDialog;
}

void GraspBar::onSetAssemblyObjButtonClicked()
{
    if(targetBodyItems.size()==1){
        if (PlanBase::instance()->AppendAssembleObject(targetBodyItems[0])) {
			os << targetBodyItems[0]->name() << " is assembly object"<< endl;
		}
    }else{
        os <<  "Please select one bodyitem" << endl;
    }
}

void GraspBar::onRemoveAssemblyObjButtonClicked()
{
    if(targetBodyItems.size()==1){
        if (PlanBase::instance()->RemoveAssembleObject(targetBodyItems[0])) {
			os << targetBodyItems[0]->name() << " is removed from assembly objects"<< endl;
		}
    }else{
        os <<  "Please select one bodyitem" << endl;
    }
}

bool GraspBar::storeState(Archive& archive)
{
    PlanBase* gc = PlanBase::instance();
    if(gc->targetArmFinger){
       archive.writeItemId("graspRobot", gc->bodyItemRobot());
    }
    if(gc->targetObject){
       archive.writeItemId("graspObject", gc->targetObject->bodyItemObject);
    }
    if( gc->bodyItemEnv.size() ) {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
        YamlSequence& qs = *archive.createFlowStyleSequence("graspEnv");
#else
        Listing& qs = *archive.createFlowStyleListing("graspEnv");
#endif
        list<BodyItemPtr>::iterator it = gc->bodyItemEnv.begin();
        for(int i=0;i< gc->bodyItemEnv.size();i++){
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
            qs.append(archive.getItemId(*it), 10, i);
#else
            qs.append(archive.getItemId(*it)->toInt(), 10, i);
#endif
            it++ ;
        }
    }
    if( gc->objTag2Item.size() ) {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
        YamlSequence& qs = *archive.createFlowStyleSequence("objTag2Item");
#else
        Listing& qs = *archive.createFlowStyleListing("objTag2Item");
#endif
        map<string,BodyItemPtr>::iterator it = gc->objTag2Item.begin();
        for(int i=0;i< gc->objTag2Item.size();i++){
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
            qs.append(archive.getItemId(it->second), 10, i);
#else
            qs.append(archive.getItemId(it->second)->toInt(), 10, i);
#endif
            it++;
        }
    }

	archive.write("camera", CameraHandler::instance()->getTargetCameraID());

    return true;
}

bool GraspBar::restoreState(const Archive& archive) {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	return restoreStateProc(archive);
#else
	archive.addPostProcess(boost::bind(&GraspBar::restoreStateProc, this, boost::ref(archive)));
	return true;
#endif
}

bool GraspBar::restoreStateProc(const Archive& archive)
{
    PlanBase* gc = PlanBase::instance();

    BodyItemPtr bodyItem = archive.findItem<BodyItem>("graspRobot");
    if(bodyItem) gc->SetGraspingRobot(bodyItem);

    bodyItem = archive.findItem<BodyItem>("graspObject");
    if(bodyItem) gc->SetGraspedObject(bodyItem);
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
    const YamlSequence& qs = *archive.findSequence("graspEnv");
#else
    const Listing& qs = *archive.findListing("graspEnv");
#endif
    if(qs.isValid()){
        gc->bodyItemEnv.clear();
        for(int i=0; i < qs.size(); ++i){
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
            BodyItemPtr bodyItem = archive.findItem<BodyItem>(qs[i].toInt());
#else
            BodyItemPtr bodyItem = archive.findItem<BodyItem>(qs.at(i));
#endif
            gc->bodyItemEnv.push_back( bodyItem );
        }
    }
    gc->initialCollision();
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
    const YamlSequence& qs2 = *archive.findSequence("objTag2Item");
#else
    const Listing& qs2 = *archive.findListing("objTag2Item");
#endif
    if(qs2.isValid()){
        gc->objTag2Item.clear();
        for(int i=0; i < qs2.size(); ++i){
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
            BodyItemPtr bodyItem = archive.findItem<BodyItem>(qs2[i].toInt());
#else
            BodyItemPtr bodyItem = archive.findItem<BodyItem>(qs2.at(i));
#endif
            string tagId;
            if(bodyItem != NULL) {
                tagId = bodyItem->name();
			} else {
				continue;
			}
            gc->objTag2Item.insert( pair <string,BodyItemPtr>(tagId, bodyItem) );
        }
    }
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
    const YamlSequence& qs3 = *archive.findSequence("GraspPlanning");
#else
    const Listing& qs3 = *archive.findListing("GraspPlanning");
#endif
    if(qs3.isValid()){
        bool init = PlanBase::instance()->initial();
        if(!init){
            os << "Failed: Grasp Planning Initial" << endl;
            return true;
        }

        try{
            PlanInterface::instance()->doGraspPlanning();
        }
        catch(int number){
            PlanBase::instance()->stopFlag=false;
            os <<  "Grasp Planning is stopped" << endl;
        }
        PlanBase::instance()->flush();
    }

	int camera_id = 0;
	if (archive.read("camera", camera_id)) {
		if (gc->targetArmFinger != NULL) {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
			CameraHandler::instance()->init(gc->body(), gc->bodyItemRobot()->lastAccessedFilePath());
#else
			CameraHandler::instance()->init(gc->body(), gc->bodyItemRobot()->filePath());
#endif
		}
		CameraHandler::instance()->setTargetCamera(camera_id);
	}

    return true;
}
