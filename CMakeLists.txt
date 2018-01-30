# Configure CMake settings
#cmake_minimum_required(VERSION 2.8.9) # needed for Qt5-related settings
cmake_minimum_required(VERSION 2.8.7) # for 12.04 backwards compatibility
set(BUILD_CORE_ONLY False)
if (${CMAKE_VERSION} VERSION_LESS "2.8.9") # needed for Qt5-related settings
  set(BUILD_CORE_ONLY True)
  message(STATUS "sightedturtlesim: only building core msg/srv since cmake version < 2.8.9")
else()
  message(STATUS "sightedturtlesim: building core and gui")
endif()

project(sightedturtlesim)

set(QT5_PATH "/opt/Qt5.2.1/5.2.1/gcc_64/" CACHE STRING "Qt5 installation path (i.e. containing ./bin/qmake)")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -std=c++0x")



# Set sources and headers
set(GUI_SOURCES
  src/AbstractImageServer.cpp
  src/SingleImageServer.cpp
  src/Turtle.cpp
  src/VisionTurtle.cpp
  src/TurtleFrame.cpp
  src/QImageWidget.cpp
  src/QTurtleGUI.cpp
  src/QTurtleNode.cpp
)

set(GUI_QT_HEADERS
  include/sightedturtlesim/QSpawnRobotDialog.hpp
  include/sightedturtlesim/QImageWidget.hpp
  include/sightedturtlesim/QTurtleGUI.hpp
)

set(GUI_OTHER_HEADERS
  include/sightedturtlesim/AbstractImageServer.hpp
  include/sightedturtlesim/SingleImageServer.hpp
  include/sightedturtlesim/Turtle.hpp
  include/sightedturtlesim/VisionTurtle.hpp
  include/sightedturtlesim/TurtleFrame.hpp
)

set(TESTER_SOURCES
  src/AbstractImageServer.cpp
  src/SingleImageServer.cpp
  src/ImageServerTester.cpp
)

set(TESTER_HEADERS
  include/sightedturtlesim/AbstractImageServer.hpp
  include/sightedturtlesim/SingleImageServer.hpp
)


# Configure ROS & catkin
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  message_generation
  )

add_message_files(
  FILES
  PoseXYZ.msg
  ImageWithPoseXYZ.msg
  VelocityXYZ.msg
  DriveXYZ.msg
  TurtleParams.msg
  MapSettings.msg
  )

add_service_files(
  FILES
  LoadSingleImageMap.srv
  Spawn.srv
  Kill.srv
  QueryGeolocatedImage.srv
  TeleportAbsoluteXYZ.srv
  SetTurtleParams.srv
  )

generate_messages(
  DEPENDENCIES
  sensor_msgs std_msgs
  )

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge image_transport message_runtime
  )


if(${BUILD_CORE_ONLY})
else()
# Load other libraries and dependencies
find_package(OpenCV REQUIRED)
if (${OpenCV_VERSION} VERSION_LESS "3.0") # OpenCV3 split certain HighGUI features into ImgCodecs and VideoIO
  find_package(OpenCV 2.2 REQUIRED COMPONENTS core imgproc highgui)
  message(STATUS "sightedturtlesim: using OpenCV ${OpenCV_VERSION} with highgui")
else ()
  find_package(OpenCV 3.0 REQUIRED COMPONENTS core imgproc highgui imgcodecs)
  message(STATUS "sightedturtlesim: using OpenCV ${OpenCV_VERSION} with highgui+imgcodecs")
endif ()

set(Boost_USE_MULTITHREADED ON)
find_package(Boost 1.46 REQUIRED COMPONENTS date_time system thread)

set(Qt5Widgets_DIR "${QT5_PATH}/lib/cmake/Qt5Widgets/")
set(CMAKE_INCLUDE_CURRENT_DIR ON)
#set(CMAKE_AUTOMOC ON)
find_package(Qt5Widgets)
add_definitions(${Qt5Widgets_DEFINITIONS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}") # NOTE: double-quote needed here; not recommended below
set(QT_INCLUDE_DIRS ${Qt5Widgets_INCLUDE_DIRS})
set(QT_LIBRARIES ${Qt5Widgets_LIBRARIES})



# Configure compiler and linker directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  )

#link_directories(
#  )


# Build teleop_sighted_turtle_key
add_executable(teleop_sighted_turtle_key src/teleop_sighted_turtle_key.cpp)
target_link_libraries(teleop_sighted_turtle_key ${catkin_LIBRARIES})
add_dependencies(teleop_sighted_turtle_key ${PROJECT_NAME}_generate_messages_cpp)

# Build internal tester project
add_executable(tester ${TESTER_SOURCES})
target_link_libraries(tester ${Boost_LIBS} ${OpenCV_LIBS})
qt5_use_modules(tester Widgets)

# Build test_query_geolocated_image
add_executable(test_query_geolocated_image src/TestQueryGeolocatedImage.cpp)
target_link_libraries(test_query_geolocated_image ${Boost_LIBS} ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_dependencies(test_query_geolocated_image ${PROJECT_NAME}_generate_messages_cpp)

# Build GUI project
add_executable(${PROJECT_NAME} ${GUI_SOURCES} ${GUI_QT_HEADERS} ${GUI_OTHER_HEADERS})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp) # Note: sometimes AUTOMOC does not ...
set_target_properties(${PROJECT_NAME} PROPERTIES AUTOMOC TRUE) # ... respect the above dependency
target_link_libraries(${PROJECT_NAME} ${Boost_LIBS} ${OpenCV_LIBS} ${catkin_LIBRARIES} ${QT_LIBRARIES})
qt5_use_modules(${PROJECT_NAME} Widgets)



# Install binaries and headers
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
  )
endif()
