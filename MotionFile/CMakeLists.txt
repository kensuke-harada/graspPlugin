set(libname CnoidMotionFilePlugin)

set(sources
  MotionFilePlugin.cpp
  MotionFileControl.cpp
  MotionFileBar.cpp
  MotionEdit.cpp
  MotionDirectiveInfo.cpp
  FileIOAndExecutingPart.cpp
  LineEditPart.cpp
  MainView.cpp
  MotionDirectiveEditLine.cpp
  MotionDirectiveEditPart.cpp
  MotionDirectiveValueEditBox.cpp
  MotionDirectiveValueEditLine.cpp
  MotionDirectiveWordAnalyzer.cpp
  MotionFileReader.cpp
  MotionUtil.cpp
  TextFieldParser.cpp
  MotionDirectiveInfoFormatter.cpp
  MotionDirectiveTypeConverter.cpp
  ToMotionDirectiveInfoConverter.cpp
  UsingHandTypeConverter.cpp
  CoordinateTypeConverter.cpp
  MotionDirectiveInfoWriter.cpp
  RobotInMotionEdit.cpp
  )

set(headers
  MotionEdit.h
  MotionDirectiveInfo.h
  FileIOAndExecutingPart.h
  LineEditPart.h
  MainView.h
  MotionDirectiveEditLine.h
  MotionDirectiveEditPart.h
  MotionDirectiveValueEditBox.h
  MotionDirectiveValueEditLine.h
  MotionDirectiveWordAnalyzer.h
  MotionFileData.h
  MotionFileReader.h
  MotionUtil.h
  TextFieldParser.h
  MotionDirectiveInfoFormatter.h
  MotionDirectiveTypeConverter.h
  ToMotionDirectiveInfoConverter.h
  UsingHandTypeConverter.h
  CoordinateTypeConverter.h
  MotionDirectiveInfoWriter.h
  RobotInMotionEdit.h
  MotionFileControl.h
  )

if(USE_QT5)
  find_package(Qt5PrintSupport)
  include_directories(${Qt5PrintSupport_INCLUDE_DIRS})
endif()

add_cnoid_plugin(${libname} SHARED ${sources} ${headers})

add_definitions(-g)

set_target_properties(${libname} PROPERTIES VERSION 0.0.0 SOVERSION 0)

target_link_libraries(${libname} CnoidBodyPlugin CnoidGraspPlugin CnoidPRMPlugin)

apply_common_setting_for_plugin(${libname})
