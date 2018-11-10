/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#ifndef TASKITEM_H
#define TASKITEM_H

#include <iostream>
#include <stack>
#include <boost/bind.hpp>

#include <cnoid/Archive>
#include <cnoid/ItemTreeView>
#include <cnoid/ItemManager>
#include <cnoid/RootItem>

#include <cnoid/MessageView>
#include <cnoid/BodyItem>
#include <../src/PoseSeqPlugin/PoseSeqItem.h>
#include <boost/filesystem.hpp>
#include <boost/optional.hpp>

#include "exportdef.h"
//#include "gettext.h"
#include "Action.h"


namespace grasp{

class TaskItem;
typedef cnoid::ref_ptr<TaskItem> TaskItemPtr;

class EXCADE_API TaskItem :  public cnoid::Item
{

    public :
        struct ActionSet {
            double startTime;
            ActionItemPtr action;
        };

        typedef std::list<ActionSet>::iterator actionSeqIterator;
				typedef std::list<ActionSet>::const_iterator actionSeqConstIterator;

        TaskItem();

        static void initializeClass(cnoid::ExtensionManager* ext);
        static bool loadFromFile(TaskItem* item, const std::string& filename, std::ostream& os);
        static bool saveToFile(TaskItem *item, const std::string &filename, std::ostream &os);

        virtual bool store(cnoid::Archive& archive);
        virtual bool restore(const cnoid::Archive& archive);

        inline std::list<ActionSet>::size_type actionSeqSize() const {
            return actionSeq.size();
        }

        void addAction(double startTime, ActionItemPtr action);
        void removeAction(ActionItemPtr action);
        ActionSet& getLastActionSet();

				actionSeqIterator begin();
				actionSeqIterator end();
				actionSeqConstIterator cbegin() const;
				actionSeqConstIterator cend() const;

    protected:
				void onSubTreeChanged();

    private:
        cnoid::MessageView& mes;
        std::ostream& os;
        std::list<ActionSet> actionSeq;
};

}


#endif // TASKITEM_H
