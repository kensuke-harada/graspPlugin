/**
   @author Takashi Kitagawa (AIST)
*/


#include "MotionFileBar.h"
#include "MotionFileControl.h"

#include <cnoid/ItemTreeView>
#include <cnoid/Archive>
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <cnoid/ExecutablePath>

#include <iostream>
#include <fstream>


using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

MotionFileBar* MotionFileBar::instance()
{
	static MotionFileBar* instance = new MotionFileBar();
	return instance;
}

MotionFileBar::MotionFileBar()
	: ToolBar("MotionFileBar"),
	  mes(*MessageView::mainInstance()),
   	os (MessageView::mainInstance()->cout() )
{
#if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)
	setVisibleByDefault(true);
#endif
	addSeparator();

	addLabel(("=MotionFile="));

	addButton(("Play"), ("Motion File Play"))->
		sigClicked().connect(bind(&MotionFileBar::onLoadButtonClicked, this));

	addButton(("Clear"), ("Motion File Clear"))->
		sigClicked().connect(bind(&MotionFileBar::onClearButtonClicked, this));

	addButton(("SavePosition"), ("Save Position to DB (PrePlanning File)"))->
		sigClicked().connect(bind(&MotionFileBar::onSavePositionButtonClicked, this));
}


MotionFileBar::~MotionFileBar()
{

}


/* --------------- */
/* Loadボタンの処理 */
/* --------------- */
void MotionFileBar::onLoadButtonClicked()
{
	string  motionfile, basepath;
	basepath = cnoid::executableTopDirectory() + "/extplugin/graspPlugin/MotionFile/data/";
	motionfile = basepath + "motion.dat";    // default
	FILE *fp;
	if( (fp = fopen((basepath+"motionfile.dat").c_str(), "r")) != NULL ){
		ifstream motionfilePath((basepath + "motionfile.dat").c_str());
		string line;
		while(getline(motionfilePath, line)){
			if(line.empty() || line.compare(0,1,"#")==0) continue;
			motionfile = line;
			break;
		}
		fclose(fp);
	}
	MotionFileControl::instance()->LoadFromMotionFile(motionfile);
}

/* --------------- */
/* Clearボタンの処理 */
/* --------------- */
void MotionFileBar::onClearButtonClicked()
{
	MotionFileControl::instance()->ClearMotionFile();
}

/* --------------- */
/* Save Positionボタンの処理 */
/* --------------- */
void MotionFileBar::onSavePositionButtonClicked()
{
	MotionFileControl::instance()->SavePositionToDB();
}


/* end */

