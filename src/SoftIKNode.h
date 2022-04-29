#ifndef _SoftIKNode
#define _SoftIKNode
//
// File: SoftIKNode.h
//
// Dependency Graph Node: SoftIK
//
// Author: Benjamin H. Singleton
//

#include <maya/MPxNode.h>
#include <maya/MObject.h>
#include <maya/MPlug.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MMatrix.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MVector.h>
#include <maya/MMatrix.h>
#include <maya/MString.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnNumericData.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MTypeId.h> 
#include <maya/MGlobal.h>

 
class SoftIK : public MPxNode
{

public:

						SoftIK();
	virtual				~SoftIK();

	virtual MStatus		compute(const MPlug& plug, MDataBlock& data);

	static  void*		creator();
	static  MStatus		initialize();

public:

	static	MObject		envelope;
	static	MObject		startMatrix;
	static	MObject		endMatrix;
	static	MObject		softDistance;
	static	MObject		chainLength;
	static	MObject		parentInverseMatrix;

	static	MObject		outputTranslate;
	static	MObject		outputTranslateX;
	static	MObject		outputTranslateY;
	static	MObject		outputTranslateZ;
	static	MObject		softScale;
	static	MObject		distance;

	static	MTypeId		id;
	static	MString		outputCategory;

};

#endif