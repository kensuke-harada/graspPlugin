/**
c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#ifndef _CONSTRAINTIK_CONSTRAINTIKBAR_H_
#define _CONSTRAINTIK_CONSTRAINTIKBAR_H_

#ifndef  CNOID_10_11_12_13
#include <cnoid/ItemList>
#endif
#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/ToolBar>	/* modified by qtconv.rb 0th rule*/  

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <cnoid/SignalProxy>
#else
#include <cnoid/Signal>
#endif

#include "exportdef.h"

#include <cnoid/MainWindow>
#include <cnoid/SpinBox>
#include <cnoid/ComboBox>
#include <cnoid/Button>

#include <QDialog>
#include <QCheckBox>
#include <QLayout>
#include <QPushButton>

namespace cnoid {
  class MessageView;
}

namespace grasp {
  class SaveObstacleShapeDialog;

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
  class EXCADE_API ConstraintIKBar : public cnoid::ToolBar, public boost::signals::trackable
#else
  class EXCADE_API ConstraintIKBar : public cnoid::ToolBar
#endif
  {
  public:

    static ConstraintIKBar* instance();
    virtual ~ConstraintIKBar();

  protected:

    virtual bool storeState(cnoid::Archive& archive);
    virtual bool restoreState(const cnoid::Archive& archive);

  private:

    ConstraintIKBar();
		
    cnoid::MessageView& mes;
    std::ostream& os;

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
    boost::signal<void(const cnoid::ItemList<cnoid::BodyItem>& selectedBodyItems)> sigBodyItemSelectionChanged_;
    boost::signal<void(cnoid::BodyItem* currentBodyItem)> sigCurrentBodyItemChanged_;
#else
    cnoid::Signal<void(const cnoid::ItemList<cnoid::BodyItem>& selectedBodyItems)> sigBodyItemSelectionChanged_;
    cnoid::Signal<void(cnoid::BodyItem* currentBodyItem)> sigCurrentBodyItemChanged_;
#endif

    void onSaveObstacle();
    void onDisplayObstacle();
    void onHideObstacle();
    void onSetCamera();
  };

  class SaveObstacleShapeDialog : public QDialog
  {
  public:
    SaveObstacleShapeDialog();

    void showClicked();
    void writeClicked();
    void rbVolumeRatioToggled();
  protected:
    cnoid::SpinBox       clusterNumber;
    cnoid::DoubleSpinBox overlapVolumeRatio;
    cnoid::DoubleSpinBox modelScale;
    cnoid::RadioButton   rbNumber, rbVolumeRatio;
    cnoid::RadioButton   rbEllipsoid, rbBbox;
  };
}

#endif /* _CONSTRAINTIK_CONSTRAINTIKBAR_H_ */
