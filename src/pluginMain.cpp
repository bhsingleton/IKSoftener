//
// File: pluginMain.cpp
//
// Author: Benjamin H. Singleton
//

#include "IKSoftenerNode.h"
#include "IKEmulatorNode.h"
#include <maya/MFnPlugin.h>


MStatus initializePlugin(MObject obj) 
{

	MStatus status;

	MFnPlugin plugin(obj, "Ben Singleton", "2017", "Any");
	status = plugin.registerNode("ikSoftener", IKSoftener::id, IKSoftener::creator, IKSoftener::initialize);
	
	if (!status) 
	{

		status.perror("registerNode");
		return status;

	}

	status = plugin.registerNode("ikEmulator", IKEmulator::id, IKEmulator::creator, IKEmulator::initialize);

	if (!status)
	{

		status.perror("registerNode");
		return status;

	}

	return status;

}

MStatus uninitializePlugin(MObject obj) 
{

	MStatus status;

	MFnPlugin plugin(obj);
	status = plugin.deregisterNode(IKSoftener::id);

	if (!status) 
	{

		status.perror("deregisterNode");
		return status;

	}

	status = plugin.deregisterNode(IKEmulator::id);

	if (!status)
	{

		status.perror("deregisterNode");
		return status;

	}

	return status;

}
