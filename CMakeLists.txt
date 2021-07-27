project(LearnToLocalize)
cmake_minimum_required(VERSION 3.16)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(learntolocalize script/kalmanfiltermultiD.cpp)
target_link_libraries(learntolocalize Eigen3::Eigen)