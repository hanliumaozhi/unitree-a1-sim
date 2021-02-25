cmake_minimum_required(VERSION 3.0.2)
project(song_wbc)

# for drake
option(RUN_X11_TESTS "Run tests that require X11" OFF)

if(APPLE)
    set(FIND_PYTHON_EXECUTABLE_PATHS /usr/local/opt/python@3.8/bin)
    set(FIND_PYTHON_INTERP_VERSION 3.8)
else()
    set(FIND_PYTHON_EXECUTABLE_PATHS /usr/bin)
    set(FIND_PYTHON_INTERP_VERSION 3.8)
endif()
find_program(PYTHON_EXECUTABLE NAMES python3
        PATHS "${FIND_PYTHON_EXECUTABLE_PATHS}"
        NO_DEFAULT_PATH
        )

find_package(PythonInterp ${FIND_PYTHON_INTERP_VERSION} MODULE REQUIRED)

execute_process(COMMAND ${PYTHON_EXECUTABLE}-config --exec-prefix
        OUTPUT_VARIABLE PYTHON_EXEC_PREFIX
        OUTPUT_STRIP_TRAILING_WHITESPACE
        )
set(CMAKE_PREFIX_PATH "/opt/drake")
list(APPEND CMAKE_PREFIX_PATH "${PYTHON_EXEC_PREFIX}")
find_package(PythonLibs ${FIND_PYTHON_INTERP_VERSION} MODULE REQUIRED)

find_package(drake CONFIG REQUIRED)

get_filename_component(PYTHONPATH
        "${drake_DIR}/../../python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages"
        REALPATH
        )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  song_msgs
        message_filters
        roslib
)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)


catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(wbc_stand_node src/WbcStand.cpp src/Utils.cpp src/OscTrackingData.cpp src/OscStandController.cpp)

target_link_libraries(wbc_stand_node ${catkin_LIBRARIES} drake::drake)
