cmake_minimum_required(VERSION 3.14)

if(POLICY CMP0074)
	cmake_policy(SET CMP0074 NEW)
endif()

set_property(GLOBAL PROPERTY USE_FOLDERS TRUE)

project(midair-balloon-haptics)

add_subdirectory(deps/dynamic-manipulation)
add_subdirectory(src)