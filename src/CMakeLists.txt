cmake_minimum_required(VERSION 3.21)
project(IKSoftener)
include($ENV{DEVKIT_LOCATION}/cmake/pluginEntry.cmake)

set(
	SOURCE_FILES
	"pluginMain.cpp"
	"IKSoftenerNode.h"
	"IKSoftenerNode.cpp"
)

set(
	LIBRARIES
	OpenMaya
	OpenMayaAnim
	Foundation
)

build_plugin()