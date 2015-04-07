cmake_minimum_required(VERSION 2.8) 
project( TestServo ) 
SET(COMPILE_DEFINITIONS -Werror) 
#OPENCV find_package( OpenCV REQUIRED ) 
add_executable(TestServo testservo.cpp) 
target_link_libraries(TestServo /opt/vc/lib/libmmal_core.so /opt/vc/lib/libmmal_util.so /opt/vc/lib/libmmal_vc_client.so /opt/vc/lib/libvcos.so /opt/vc/lib/libbcm_host.so wiringPi pthread ${OpenCV_LIBS})


