set(target PyGraspPCLPlugin)

add_cnoid_python_module(${target}
  PyGraspPCLPluginModule.cpp
  PyPointCloudHolder.cpp
  PyObjectPoseEstimator.cpp
  PySegmenter.cpp
  PyPointSetItemDrawer.cpp
  )

target_link_libraries(${target} CnoidGraspPCLPlugin CnoidPyBase)
