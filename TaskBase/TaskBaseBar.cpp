/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include "TaskBaseBar.h"
#include <cnoid/ItemTreeView>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/MessageView>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/Archive>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/RootItem>	/* modified by qtconv.rb 0th rule*/  
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <../src/PoseSeqPlugin/PoseSeqItem.h>
#include "../Grasp/VectorMath.h"

#include "RobotMotionGenerator.h"
#include "ExportAssemblyGraphDialog.h"

// #include <glibmm/i18n.h>	/* modified by qtconv.rb 5th rule*/  
//#include "TaskBasePlanner.h"

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

int TaskBaseBar::count = 0;

TaskBaseBar* TaskBaseBar::instance()
{
	static TaskBaseBar* instance = new TaskBaseBar();
	return instance;
}

TaskBaseBar::TaskBaseBar()
	: ToolBar("TaskBaseBar"),
      mes(*MessageView::mainInstance()),
    os (MessageView::mainInstance()->cout() )
{

    addSeparator();

    addLabel(("=TaskBase="));

    addButton(("Start"), ("Path planning start"))->sigClicked().connect(bind(&TaskBaseBar::onTaskBasePlanButtonClicked, this));
    addButton(("NewTask"), ("New task"))->sigClicked().connect(bind(&TaskBaseBar::onNewTaskItemButtonClicked, this));
    addButton(("NewAction"), ("New action"))->sigClicked().connect(bind(&TaskBaseBar::onNewActionItemButtonClicked, this));
	addButton(("SaveAction"), ("Save Action"))->sigClicked().connect(bind(&TaskBaseBar::onSaveActionItemButtonClicked, this));
	addButton(("SetMain"), ("Set main item"))->sigClicked().connect(bind(&TaskBaseBar::onSetMainActionItemButtonClicked, this));
	addButton(("SetSub"), ("Set sub item"))->sigClicked().connect(bind(&TaskBaseBar::onSetSubActionItemButtonClicked, this));
	addButton(("SetActionState"), ("Set state"))->sigClicked().connect(bind(&TaskBaseBar::onSetActionStateButtonClicked, this));
	
	addSeparator();

	addButton(("Plan"), (""))->sigClicked().connect(bind(&TaskBaseBar::onPlanButtonClicked, this));

	addSeparator();

	addButton(("ExportAssemblyGraph"), ("export assembly graph"))->sigClicked().connect(bind(&TaskBaseBar::onExportAssemblyGraphClicked, this));

	addSeparator();

	ItemTreeView::mainInstance()->sigSelectionChanged().connect(bind(&TaskBaseBar::onItemSelectionChanged, this, _1));
	
	count++;
    action = NULL;
    task = NULL;
}



TaskBaseBar::~TaskBaseBar()
{
	connectionOfCurrentBodyItemDetachedFromRoot.disconnect();
}

void TaskBaseBar::onTaskBasePlanButtonClicked()
{

	bool ret = true;
	os << targetBodyItems.size() << endl;
	if(targetBodyItems.size()!=1){
		os <<  "Please selecet one bodyitem" << endl;
		return;
	}
}

 void TaskBaseBar::onNewTaskItemButtonClicked()
 {
      task = new TaskItem;

      cnoid::ItemList<TaskItem> taskItemlist;
      taskItemlist.extractChildItems(cnoid::RootItem::mainInstance());

      stringstream ss;
      ss << "task" << taskItemlist.size();
      os << "task name: =" << ss.str() << endl;
      task->setName( ss.str() );

      cnoid::RootItem::mainInstance()->addChildItem(task);
 }

void TaskBaseBar::onNewActionItemButtonClicked(){

    action = new ActionItem;
    if (task == NULL){
        os <<  "Please selecet one taskitem" << endl;
        return;
    }

    // 名前の設定
    stringstream ss;
    ss << "action" << task->actionSeqSize();
    os << "action name: =" << ss.str() << endl;
    action->setName( ss.str() );

    if (task->actionSeqSize() == 0){
        action->setStartTime(0);
    }
    else{
        TaskItem::ActionSet& lastActionSet = task->getLastActionSet();
        action->setStartTime(lastActionSet.startTime + lastActionSet.action->getDurationTime());
    }
    task->addAction(action->getStartTime(), action);

    os << "action start time = " << action->getStartTime() << endl;
}

void TaskBaseBar::onSaveActionItemButtonClicked(){
	
}

void TaskBaseBar::onSetMainActionItemButtonClicked(){
	if(action == NULL){
		os << "Generate or open action item" << endl;
		return;
	}
	if(targetBodyItems.size()!=1){
		os <<  "Please selecet one bodyitem" << endl;
		return;
    }
    action->setMainItem(targetBodyItems[0]);
    os << "set MainItem: " << action->getMainItem()->name() << endl;
}

void TaskBaseBar::onSetSubActionItemButtonClicked(){
	if(action == NULL){
		os << "Generate or open action item" << endl;
		return;
	}
	if(targetBodyItems.size()!=1){
		os <<  "Please selecet one bodyitem" << endl;
		return;
	}
    action->setSubItem(targetBodyItems[0]);
    os << "set MainItem: " << action->getSubItem()->name() << endl;
}

void TaskBaseBar::onSetActionStateButtonClicked(){
	if(action == NULL){
		os << "Generate or open action item" << endl;
		return;
	}
    action->setNewActionState();
}


void TaskBaseBar::onItemSelectionChanged(const ItemList<Item>& items)
{
    if (items.size() != 1){
        // TODO
        return;
    }
    else{
        Item* item = items.get(0);

        TaskItemPtr selectedTask = dynamic_cast<TaskItem*>(item);
        if(selectedTask && selectedTask != task){
            task = selectedTask;
            return;
        }

        BodyItemPtr selectedBody = dynamic_cast<BodyItem*>(item);
        if(selectedBody){
            ItemList<BodyItem> bodyItems = ItemList<BodyItem>();
            bodyItems.push_back(selectedBody);

            // just copied from old ver.
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
                    bind(&TaskBaseBar::onBodyItemDetachedFromRoot, this));
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
            return;
        }
    }
}


void TaskBaseBar::onBodyItemDetachedFromRoot()
{
	currentBodyItem_ = 0;
	connectionOfCurrentBodyItemDetachedFromRoot.disconnect();
	sigCurrentBodyItemChanged_(0);
}



bool TaskBaseBar::storeState(Archive& archive)
{
	return true;
}


bool TaskBaseBar::restoreState(const Archive& archive)
{
	return true;
}

void TaskBaseBar::onPlanButtonClicked() {
	RobotMotionGenerator rmg;
	TaskItem* taskitem = cnoid::ItemTreeView::instance()->selectedItem<TaskItem>(true);
	if (taskitem != NULL) {
		rmg.plan(taskitem);
		return;
	}

	ActionItem* actionitem = cnoid::ItemTreeView::instance()->selectedItem<ActionItem>(true);
	if (actionitem != NULL) {
		os << "Start planning: action " << actionitem->name() << std::endl;
		rmg.plan(actionitem);
		return;
	}

	os <<  "Please select one Task/Action Item" << endl;
}

void TaskBaseBar::onExportAssemblyGraphClicked() {
	TaskItem* taskitem = cnoid::ItemTreeView::instance()->selectedItem<TaskItem>(true);
	if (taskitem != NULL) {
		ExportAssemblyGraphDialog* eagd = new ExportAssemblyGraphDialog();
		eagd->setTaskItem(taskitem);
		eagd->exec();
		delete eagd;
		return;
	}

	os << "Please select one Task Item" << endl;
}
