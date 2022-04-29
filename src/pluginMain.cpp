//
// File: pluginMain.cpp
//
// Author: Benjamin H. Singleton
//

#include "SoftIKNode.h"

#include <maya/MFnPlugin.h>


MStatus initializePlugin(MObject obj) 
{

	MStatus status;

	MFnPlugin plugin(obj, "Ben Singleton", "2017", "Any");
	status = plugin.registerNode("softIK", SoftIK::id, SoftIK::creator, SoftIK::initialize);
	
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
	status = plugin.deregisterNode(SoftIK::id);

	if (!status) 
	{

		status.perror("deregisterNode");
		return status;

	}

	return status;

}
