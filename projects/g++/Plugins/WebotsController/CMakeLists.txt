cmake_minimum_required(VERSION 2.8)

project(WebotsControllerPlugin)

add_definitions( -DWEBOTS_EXPORT )

# Link with the Webots controller library.
link_directories($ENV{WEBOTS_HOME}/lib/controller)
set (WEBOTS_LIBRARIES ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})
include_directories($ENV{WEBOTS_HOME}/include/controller/c $ENV{WEBOTS_HOME}/include/controller/cpp)


file(
	GLOB_RECURSE
	source_files
	${PLUGIN_SOURCE_DIR}/WebotsController/*.cpp
	${PLUGIN_SOURCE_DIR}/WebotsController/*.h
)

add_library(
	webotsController
	SHARED
	${source_files}
)

target_link_libraries (webotsController mengeCore ${WEBOTS_LIBRARIES} )

