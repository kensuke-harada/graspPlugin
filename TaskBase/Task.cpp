
#include "Task.h"
#include <cnoid/BodyMotion>
#include <cnoid/YAMLReader>
#include <cnoid/YAMLWriter>
#include <cnoid/ParametricPathProcessor>
#include <cnoid/FileUtil>

#include "../../../src/PoseSeqPlugin/PoseSeq.h"
#include <cnoid/EigenArchive>
#include "../../../src/Util/ValueTree.h"
#include "../Grasp/VectorMath.h"

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

TaskItem::TaskItem()
    : mes(*cnoid::MessageView::mainInstance()),
      os (cnoid::MessageView::mainInstance()->cout())
{
	sigSubTreeChanged().connect(boost::bind(&TaskItem::onSubTreeChanged, this));
}

void TaskItem::initializeClass(cnoid::ExtensionManager *ext){

    static bool initialized = false;

    if(!initialized){
        cnoid::ItemManager& im = ext->itemManager();
        im.registerClass<TaskItem>(N_("TaskItem"));
#ifdef CNOID_GE_16
        im.addLoaderAndSaver<TaskItem>(_("TaskItem File"), "TASK-YAML", "yaml;yml", std::bind(&TaskItem::loadFromFile, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), std::bind(&TaskItem::saveToFile, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
#else
        im.addLoaderAndSaver<TaskItem>(_("TaskItem File"), "TASK-YAML", "yaml;yml", boost::bind(TaskItem::loadFromFile, _1, _2, _3), boost::bind(TaskItem::saveToFile, _1, _2, _3));
#endif
        initialized = true;
    }
}

bool TaskItem::saveToFile(TaskItem *item, const std::string &filename, std::ostream &os){
    cnoid::YAMLWriter writer(filename);
    writer.setKeyOrderPreservationMode(true);

    cnoid::MappingPtr archive = new cnoid::Mapping();
    archive->setDoubleFormat("%.9g");

    cnoid::Listing& refsNode = *archive->createListing("sequence");

    for(actionSeqIterator itr = item->actionSeq.begin(); itr != item->actionSeq.end(); itr++)
    {
        ActionSet &set = *itr;
        // double startTime = set.startTime;
        ActionItemPtr action = set.action;
				double startTime = action->getStartTime();

        cnoid::MappingPtr refNode = refsNode.newMapping();

        action->overwrite(true);

        refNode->write("name",action->name());
				// replace choreonoid directory path to ${PROGRAM_TOP}
				std::string parameterized_filepath = cnoid::ParametricPathProcessor::instance()->parameterize(action->filePath());
				refNode->write("filename", parameterized_filepath);
        // refNode->write("filename", action->filePath());
        refNode->write("startTime", startTime);
    }

    writer.putComment("Assembly Task data format version 0.1\n");
    writer.putNode(archive);
    return true;
}

bool TaskItem::loadFromFile(TaskItem *item, const std::string &filename, std::ostream &os){
    // 名前の設定
    item->setName(filesystem::basename(filesystem::path(filename)));

    cnoid::YAMLReader reader;
    const cnoid::MappingPtr archive = reader.loadDocument(filename)->toMapping();

    const cnoid::Listing& refs = *archive->findListing("sequence");
    if(!refs.isValid()){
        return false;
    }

    for(int i=0; i < refs.size(); i++){
        ActionSet actionSet;
        double startTime;
        ActionItemPtr action = new ActionItem();

        const cnoid::Mapping& ref = *refs[i].toMapping();
        std::string actionName;
        std::string actionFileName;
        ref.read("name", actionName);
        ref.read("filename", actionFileName);
        ref.read("startTime", startTime);

				// replace ${PROGRAM_TOP} to the choreonoid directory
				boost::optional<std::string> expanded_filename = cnoid::ParametricPathProcessor::instance()->expand(actionFileName);
				if (expanded_filename) {
					actionFileName = cnoid::getPathString(cnoid::getAbsolutePath(*expanded_filename));
				}

        // デバック用
//        os << "name = " << actionName << endl;
//        os << "filename = " << actionFileName << endl;
//        os << "startTime = " << startTime << endl;

        //current = item->actionSeq.insert(current, actionTaskSet);

        action->setStartTime(startTime);
        ActionItem::loadFromFile(action, actionFileName, item->os);
        actionSet.startTime = startTime;
        actionSet.action = action;

        item->actionSeq.push_back(actionSet);
        item->addChildItem(action);
    }
    return true;
}

bool TaskItem::store(cnoid::Archive &archive){
    if(overwrite(true)){
        archive.writeRelocatablePath("filename", filePath());
        archive.write("format", fileFormat());
        return true;
    }
    return false;
}

bool TaskItem::restore(const cnoid::Archive &archive){
    std::string filename, formatId;
    if(archive.readRelocatablePath("filename", filename) && archive.read("format", formatId)){
        if(load(filename, formatId)){
            return true;
        }
    }
    else{
        os << "cannot open " << filename << " by format " << formatId << endl;
    }
    return false;
}

void TaskItem::addAction(double startTime, ActionItemPtr action)
{
    ActionSet ref;
    ref.startTime = startTime;
    ref.action = action;
    actionSeq.push_back(ref);

    this->addChildItem(action);
}

TaskItem::ActionSet& TaskItem::getLastActionSet()
{
    actionSeqIterator it = actionSeq.end();
    --it;
		if (it != actionSeq.end()) (*it).startTime = (*it).action->getStartTime();
    return *it;
}

TaskItem::actionSeqIterator TaskItem::begin() {
	return actionSeq.begin();
}

TaskItem::actionSeqIterator TaskItem::end() {
	return actionSeq.end();
}

TaskItem::actionSeqConstIterator TaskItem::cbegin() const {
	return actionSeq.begin();
}

TaskItem::actionSeqConstIterator TaskItem::cend() const {
	return actionSeq.end();
}

void TaskItem::onSubTreeChanged() {
	// reconstrcut actionSeq
	actionSeq.clear();

	cnoid::Item* next = childItem();
	std::stack<cnoid::Item*> s_item;
	if (next != NULL) s_item.push(next);
	while (!s_item.empty()) {
		next = s_item.top();
		s_item.pop();

		ActionItem* action_item = dynamic_cast<ActionItem*>(next);
		if (action_item != NULL) {
			ActionSet action_set;
			action_set.action = action_item;
			action_set.startTime = action_item->getStartTime();
			actionSeq.push_back(action_set);
		}

		if (next->nextItem() != NULL) {
			s_item.push(next->nextItem());
		}
		if (next->childItem() != NULL) {
			s_item.push(next->childItem());
		}
	}
}
