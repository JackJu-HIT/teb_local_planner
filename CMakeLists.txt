cmake_minimum_required(VERSION 3.17)
project(teb)

set(CMAKE_CXX_STANDARD 14)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)

#list(APPEND CMAKE_MODULE_PATH /home/juchunyu/20231013/tebNoRos/teb_local_planner/cmake_modules)


find_package(Boost REQUIRED COMPONENTS system thread graph)
find_package(SUITESPARSE REQUIRED)
find_package(G2O REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
        /usr/include/eigen3
        ${SUITESPARSE_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)
link_directories(/usr/include/python2.7)
include_directories(${G2O_INCLUDE_DIRS})

add_library(lib_teb
        src/teb_config.cpp
        src/obstacles.cpp
        src/visualization.cpp
        src/optimal_planner.cpp
        src/timed_elastic_band.cpp
)

add_executable(teb main.cpp)

target_link_libraries(
        teb
        lib_teb
        ${SUITESPARSE_LIBRARIES}
        ${G2O_LIBRARIES}
        -lboost_system -lboost_thread -lboost_graph
        ${OpenCV_LIBS}
        python2.7
        ${PYTHON_LIBRARIES}
)

#target_link_libraries(PID  python2.7)


#target_link_libraries(g2o_demo  ${catkin_LIBRARIES} g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension)

