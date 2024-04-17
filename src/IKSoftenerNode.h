#ifndef _IK_SOFTENER_NODE
#define _IK_SOFTENER_NODE
//
// File: IKSoftenerNode.h
//
// Dependency Graph Node: ikSoftener
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
#include <maya/MDistance.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnNumericData.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MTypeId.h> 
#include <maya/MGlobal.h>

#include <iostream>
#include <cmath>

 
class IKSoftener : public MPxNode
{

public:

						IKSoftener();
	virtual				~IKSoftener();

	virtual MStatus		compute(const MPlug& plug, MDataBlock& data);

	static	MMatrix		createPositionMatrix(const MPoint& position);
	static	MPoint		matrixToPosition(const MMatrix& matrix);

	static  void*		creator();
	static  MStatus		initialize();

public:

	static	MObject		envelope;
	static	MObject		startMatrix;
	static	MObject		endMatrix;
	static	MObject		radius;
	static	MObject		chainLength;
	static	MObject		parentInverseMatrix;

	static	MObject		outTranslate;
	static	MObject		outTranslateX;
	static	MObject		outTranslateY;
	static	MObject		outTranslateZ;
	static	MObject		outVector;
	static	MObject		outVectorX;
	static	MObject		outVectorY;
	static	MObject		outVectorZ;
	static	MObject		outMatrix;
	static	MObject		outWorldMatrix;
	static	MObject		softScale;
	static	MObject		softDistance;

	static	MString		inputCategory;
	static	MString		outputCategory;

	static	MTypeId		id;

};

#endif