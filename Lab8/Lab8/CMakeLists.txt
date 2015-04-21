cmake_minimum_required(VERSION 2.8) 
project( ParticleFilter ) 
SET(COMPILE_DEFINITIONS -Werror) 
#OPENCV 
find_package( OpenCV REQUIRED ) 
add_executable(ParticleFilter test.cpp) 
target_link_libraries(ParticleFilter wiringPi ${OpenCV_LIBS})
target_link_libraries(ParticleFilter ${OpenCV_LIBS})