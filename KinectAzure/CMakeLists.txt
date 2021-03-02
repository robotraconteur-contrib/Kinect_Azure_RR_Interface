    
cmake_minimum_required(VERSION 3.3)
project(KinectV2)

set (CMAKE_CXX_STANDARD 11)
set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH} )



find_package(RobotRaconteur REQUIRED)
find_package(RobotRaconteur_standard_robdef_cpp REQUIRED)
find_package(RobotRaconteurCompanion REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)


include_directories(${RobotRaconteur_INCLUDE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

ROBOTRACONTEUR_GENERATE_THUNK(RR_THUNK_SRCS RR_THUNK_HDRS
    edu.rpi.robotics.kinect.robdef
	MASTER_HEADER
	IMPORT_DIRS ${CMAKE_CURRENT_SOURCE_DIR} ${RobotRaconteur_STANDARD_ROBDEF_DIRS}
    IMPORT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../robdef
	AUTO_IMPORT
	)
if (MSVC)
   add_definitions(-D_USE_MATH_DEFINES)
   SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif()



add_executable(KinectV2
	${RR_THUNK_HDRS}
	${RR_THUNK_SRCS}
	main.cpp
	kinect2_impl.cpp
	kinect2_impl.h
	packages.config
	)
	

target_link_libraries(KinectV2 yaml-cpp RobotRaconteurCore RobotRaconteurCompanion ${Boost_LIBRARIES} ${RobotRaconteurCore_Extra_libraries} 
    ${CMAKE_BINARY_DIR}/packages/Microsoft.Azure.Kinect.Sensor.1.3.0/build/native/Microsoft.Azure.Kinect.Sensor.targets
    ${CMAKE_BINARY_DIR}/packages/Microsoft.Azure.Kinect.BodyTracking.0.9.5/build/native/Microsoft.Azure.Kinect.BodyTracking.targets
	${CMAKE_BINARY_DIR}/packages/Microsoft.Azure.Kinect.BodyTracking.Dependencies.0.9.1/build/native/Microsoft.Azure.Kinect.BodyTracking.Dependencies.targets
	${CMAKE_BINARY_DIR}/packages/Microsoft.Azure.Kinect.BodyTracking.Dependencies.cuDNN.0.9.1/build/native/Microsoft.Azure.Kinect.BodyTracking.Dependencies.cuDNN.targets

)


configure_file(${CMAKE_CURRENT_SOURCE_DIR}/packages.config ${CMAKE_CURRENT_BINARY_DIR}/packages.config COPYONLY)
