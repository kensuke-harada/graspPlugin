#ifndef _PCL_RECOGNIZEDOBJECT_H_
#define _PCL_RECOGNIZEDOBJECT_H_

#include <vector>
#include <string>

#include <cnoid/FolderItem>
#include <cnoid/BodyItem>
#include <cnoid/RootItem>
#include <cnoid/ItemList>
#include <cnoid/ItemTreeView>

#include "exportdef.h"

class EXCADE_API RecognizedObjectManager {
 public:
	static void getOrCreateRecognizedObjectBodyItems(int n, const cnoid::BodyItemPtr& target,
																									 std::vector<cnoid::BodyItemPtr>& items);
};

#endif /* _PCL_RECOGNIZEDOBJECT_H_ */
