cmake_minimum_required(VERSION 2.6)

project(PartialCloudGenerator)

set(CMAKE_BUILD_TYPE Debug)

find_package(PCL 1.6 REQUIRED)

if(UNIX)
  # remove vtkporj4 from link libraries because libpcl1.7 on Ubunut 16.04 has bug.
  # see https://bugs.launchpad.net/ubuntu/+source/vtk6/+bug/1573234
  execute_process(COMMAND lsb_release -is
    TIMEOUT 4
    OUTPUT_VARIABLE LINUX_DISTRO
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )
  execute_process(COMMAND lsb_release -rs
    TIMEOUT 4
    OUTPUT_VARIABLE LINUX_RELEASE
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )
  if((${LINUX_DISTRO} STREQUAL "Ubuntu") AND (${LINUX_RELEASE} STREQUAL "16.04"))
    list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
  endif()
endif()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Boost 1.40 REQUIRED program_options)

set(target partial_cloud_genrator)

set(sources
  PartialCloudGenerator.cpp
  )

add_executable(${target}  ${sources})
target_link_libraries(${target} ${PCL_LIBRARIES} ${Boost_PROGRAM_OPTIONS_LIBRARY})
