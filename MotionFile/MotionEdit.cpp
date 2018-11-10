
#include "MotionEdit.h"
#include <cnoid/TimeBar>
#include <cnoid/ConnectionSet>
#include <cnoid/MenuManager>
#include <cnoid/Archive>
#include <cnoid/ItemTreeView>
#include <cnoid/MessageView>
#include <cnoid/LazyCaller>
#include <cnoid/Sleep>
#include <QEvent>
#include <boost/bind.hpp>

#include <QApplication> 
#include <QGraphicsView> 
#include <QGraphicsScene> 
#include <QGraphicsItem> 
#include <QPrinter> 
#include <QImage> 

#include <iostream>

#include <QWidget>
#include <QScrollArea>
#include <QLayout>
#include <QMessageBox>

#include "MainView.h"
#include "FileIOAndExecutingPart.h"
#include "MotionDirectiveEditPart.h"
#include "MotionDirectiveEditLine.h"
#include "LineEditPart.h"

#include "MotionFileReader.h"
#include "MotionDirectiveInfoWriter.h"
#include "MotionDirectiveInfo.h"
#include "MotionUtil.h"
#include "TextFieldParser.h"
#include "MotionDirectiveTypeConverter.h"

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <boost/shared_ptr.hpp>
#include <boost/signal.hpp>
#include <boost/signals.hpp>
#else
#include <cnoid/Referenced>
#include <cnoid/Signal>
#endif

#include<cnoid/Button>
#include<cnoid/ToolBar>
#include"../Grasp/GraspBar.h"
#include"MotionFileControl.h"

#include <cnoid/Plugin>
#include <cnoid/MessageView>

#if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)
#include <cnoid/ViewManager>
#endif

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;
using namespace motionedit;

#if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)
void MotionEdit::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<MotionEdit>(
        "MotionEdit", "Motion", ViewManager::SINGLE_DEFAULT);
}
#endif

MotionEdit::MotionEdit()
{
	setName("MotionEdit");
    	setDefaultLayoutArea(View::CENTER);

	/******************************
	* motion FileReader を作成します。
	******************************/

	string delimiliterList = " ,";

	string comentTokenList = "#";

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	boost::intrusive_ptr<TextFieldParser>textFieldParser(new TextFieldParser());
	{
		textFieldParser->AddDelimiter(delimiliterList);
		textFieldParser->AddCommentToken(comentTokenList);
	}
	boost::intrusive_ptr<MotionDirectiveTypeConverter> motionDirectiveTypeConverter(new MotionDirectiveTypeConverter);
	boost::intrusive_ptr<UsingHandTypeConverter> usingHandTypeConverter(new UsingHandTypeConverter);
	boost::intrusive_ptr<CoordinateTypeConverter> coordinateTypeConverter(new CoordinateTypeConverter);

	boost::intrusive_ptr<MotionDirectiveWordAnalyzer> motionDirectiveAnalyzer(
		new MotionDirectiveWordAnalyzer(
		motionDirectiveTypeConverter,
		usingHandTypeConverter,
		coordinateTypeConverter
		));

	boost::intrusive_ptr<ToMotionDirectiveInfoConverter> toMotionDirectiveConverter(
		new ToMotionDirectiveInfoConverter(motionDirectiveAnalyzer));
		boost::intrusive_ptr<MotionFileReader> motionFileReader(new MotionFileReader(
		textFieldParser,
		toMotionDirectiveConverter
		));
#else
	cnoid::ref_ptr<TextFieldParser>textFieldParser(new TextFieldParser());
	{
		textFieldParser->AddDelimiter(delimiliterList);
		textFieldParser->AddCommentToken(comentTokenList);
	}

	cnoid::ref_ptr<MotionDirectiveTypeConverter> motionDirectiveTypeConverter(new MotionDirectiveTypeConverter);
	cnoid::ref_ptr<UsingHandTypeConverter> usingHandTypeConverter(new UsingHandTypeConverter);
	cnoid::ref_ptr<CoordinateTypeConverter> coordinateTypeConverter(new CoordinateTypeConverter);

	cnoid::ref_ptr<MotionDirectiveWordAnalyzer> motionDirectiveAnalyzer(
		new MotionDirectiveWordAnalyzer(
		motionDirectiveTypeConverter,
		usingHandTypeConverter,
		coordinateTypeConverter
		));

	cnoid::ref_ptr<ToMotionDirectiveInfoConverter> toMotionDirectiveConverter(
		new ToMotionDirectiveInfoConverter(motionDirectiveAnalyzer));

	cnoid::ref_ptr<MotionFileReader> motionFileReader(new MotionFileReader(
		textFieldParser,
		toMotionDirectiveConverter
		));
#endif

	MainView* mainView = new MainView(0);
	mainView->SetReader(motionFileReader);

	QHBoxLayout* layout = new QHBoxLayout;
	layout->addWidget(mainView);
	this->setLayout(layout);
}


MotionEdit::~MotionEdit()
{
}

