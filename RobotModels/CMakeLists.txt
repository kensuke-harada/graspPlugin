if(ENABLE_INSTALL_RPATH)
  set(CMAKE_INSTALL_RPATH "$ORIGIN/..")
endif()

set(GRASP_ROBOT_MODEL_PLUGINS "PA10/Plugin" CACHE STRING
  "Sub-directories containing plugins to compile in the each directory")

foreach(subdirectory ${GRASP_ROBOT_MODEL_PLUGINS})
  add_subdirectory(${subdirectory})
endforeach()


