project(LearnToLocalize)
cmake_minimum_required(VERSION 3.16)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(learntolocalize script/kalmanfiltermultiD.cpp)
target_link_libraries(learntolocalize Eigen3::Eigen)

add_executable(mclexecutable script/montecarlolocalization.cpp)
target_link_libraries(mclexecutable)