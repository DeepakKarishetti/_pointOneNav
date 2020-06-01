find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

set(EXECUTABLE_OUTPUT_PATH "../bin")
add_executable(ekf main.cpp pointOneNav.hpp matplotlibcpp.h)

find_package(PythonLibs 2.7)
target_include_directories(ekf PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(ekf ${PYTHON_LIBRARIES})
