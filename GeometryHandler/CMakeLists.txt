
option(BUILD_GEOMETRY_EXE "Building geometry handling exes" OFF)

if(BUILD_GEOMETRY_EXE)

  set(program3 stl2vrml)

  set(sources3
    stl2vrml.cpp
    Cluster.cpp
    OverlapVolumeCalculator.cpp
    ObjectShape.cpp
    ShapeElements.cpp
    )

  add_executable(${program3} ${sources3})

  if(UNIX)
    target_link_libraries(
      ${program3}
      CnoidBodyPlugin CnoidGraspPlugin
      boost_thread glut GLU GL Xmu Xi Xext X11)
  elseif(WIN32)
    set_target_properties(${program3} PROPERTIES DEBUG_POSTFIX d )
    target_link_libraries(${program3}
      CnoidBodyPlugin CnoidChoreographyPlugin boost_thread
      ${OMNIORB_LIBRARIES})
  endif()

  set(procsv2stl csv2stl)

  set(srccsv2stl
    csv2stl.cpp
    Cluster.cpp
    OverlapVolumeCalculator.cpp
    ObjectShape.cpp
    ShapeElements.cpp
    )

  add_executable(${procsv2stl} ${srccsv2stl})

  if(UNIX)
    target_link_libraries( ${procsv2stl}  CnoidBodyPlugin CnoidGraspPlugin boost_thread)
  elseif(WIN32)
    set_target_properties(${procsv2stl} PROPERTIES DEBUG_POSTFIX d )
    target_link_libraries(${procsv2stl}
      CnoidBodyPlugin CnoidChoreographyPlugin boost_thread
      ${OMNIORB_LIBRARIES})
  endif()


  set(program4 matchTest)

  set(sources4
    matchTest.cpp
    Cluster.cpp
    OverlapVolumeCalculator.cpp
    ObjectShape.cpp
    ShapeElements.cpp
    )

  add_executable(${program4} ${sources4})

  if(UNIX)
    target_link_libraries(
      ${program4}
      CnoidBodyPlugin CnoidGraspPlugin boost_thread)
  elseif(WIN32)
    set_target_properties(${program4} PROPERTIES DEBUG_POSTFIX d )
    target_link_libraries(
      ${program4}
      CnoidBodyPlugin CnoidGraspPlugin)
  endif()

endif()
#BUILD_GEOMETRY_EXE

#set(OPENHRP_LIBRARY_VERSION 3.1 )
#add_definitions(-g)

#target_link_libraries(
#    ${program3}
#)


set(libname CnoidGeometryHandlerPlugin)

set(sources2
  GeometryHandlerPlugin.cpp
  Cluster.cpp
  ObjectShape.cpp
  ShapeElements.cpp
  OverlapVolumeCalculator.cpp
  MatchClusters.cpp
  FindParallelPlane.cpp
  AssemblyPlan.cpp
  ObjEnvContact.cpp
  ClusterParameter.cpp
  GeometryBar.cpp
  GeometryAnalysis.cpp
  EnCalculator.cpp
  )

set(headers2
  GeometryHandle.h
  FindParallelPlane.h
  MatchClusters.h
  AssemblyPlan.h
  ObjEnvContact.h
  ClusterParameter.h
  GeometryBar.h
  EnCalculator.h
  )

if((CNOID_VERSION VERSION_EQUAL 1.5) OR (CNOID_VERSION VERSION_GREATER 1.5))
  set(sources2 ${sources2}
    GeometryEventEater.cpp
    GeometryExecutor.cpp
    ShapeExtractor.cpp
    )
  set(headers2 ${headers2}
    GeometryEventEater.h
    GeometryExecutor.h
    ShapeExtractor.h
    )
endif()

if((CNOID_VERSION VERSION_EQUAL 1.6) OR (CNOID_VERSION VERSION_GREATER 1.6))
  set(sources2 ${sources2}
    SgVisitShapeGet.cpp
    )
endif()

if(ENABLE_OSG)
  add_cnoid_plugin(${libname} SHARED ${sources2} OSGGeometrySceneBody.cpp ${headers2})
else()
  add_cnoid_plugin(${libname} SHARED ${sources2} ${headers2})
endif()

set_target_properties(${libname} PROPERTIES VERSION 0.0.0 SOVERSION 0)
#set_target_properties(${libname} PROPERTIES DEFINE_SYMBOL "EXCADE_SAMPLE1_MAKE_DLL")

if(UNIX)

  target_link_libraries(${libname} CnoidBodyPlugin CnoidGraspPlugin boost_thread )
  apply_common_setting_for_plugin(${libname})
  #  add_definitions(-g)
  option(CENTER_SHIFT_ON "Define CENTER_SHIFT" OFF)
  if(CENTER_SHIFT_ON)
    add_definitions(-DCENTER_SHIFT)
  endif()

elseif(MSVC)
  add_definitions(-DEXCADE_GEOMETRYHANDLER_MAKE_DLL)

  source_group("Header Files" FILES ${headers})

  #target_link_libraries(${libname} Excade${EXCADE_VERSION_SUFFIX} ExcadeRobotics${EXCADE_VERSION_SUFFIX} )
  target_link_libraries(${libname} CnoidBodyPlugin CnoidGraspPlugin)

  set_target_properties(${libname} PROPERTIES COMPILE_FLAGS "/wd4250 /wd4275")

  install(TARGETS ${libname}
    RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
    LIBRARY DESTINATION lib CONFIGURATIONS Release Debug)

  apply_common_setting_for_plugin(${libname})

  find_package(Boost 1.36.0 REQUIRED COMPONENTS thread date_time)
  install_external_libraries(${Boost_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS}
    ${Boost_THREAD_LIBRARY}   ${Boost_DATE_TIME_LIBRARY} )

endif(UNIX)

#add_subdirectory(project)
