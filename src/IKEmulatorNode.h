#ifndef _IK_EMULATOR_NODE
#define _IK_EMULATOR_NODE
//
// File: IKEmulatorNode.h
//
// Dependency Graph Node: ikEmulator
//
// Author: Benjamin H. Singleton
//

#include <maya/MPxNode.h>
#include <maya/MObject.h>
#include <maya/MPlug.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MArrayDataBuilder.h>
#include <maya/MString.h>
#include <maya/MDistance.h>
#include <maya/MAngle.h>
#include <maya/MVector.h>
#include <maya/MEulerRotation.h>
#include <maya/MQuaternion.h>
#include <maya/MMatrix.h>
#include <maya/MMatrixArray.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnNumericData.h>
#include <maya/MFnMatrixData.h>
#include <maya/MTypeId.h> 
#include <maya/MGlobal.h>

#include <iostream>
#include <cmath>
#include <vector>


enum class Axis
{

	X = 0,
	Y = 1,
	Z = 2

};


enum class PoleType
{

	Align = 0,  // Uses `poleVector` as an alignment vector
	LookAt = 1,  // Uses `poleVector` as a look-at target
	Matrix = 2,  // Uses `poleMatrix` as a look-at target
	Goal = 3  // Uses `upAxis` to get a normalized row from `goal` to use as an alignment vector

};


struct IKSettings
{

	MMatrix goal = MMatrix::identity;
	Axis forwardAxis = Axis::X;
	bool forwardAxisFlip = false;
	Axis upAxis = Axis::Y;
	bool upAxisFlip = true;
	MVector poleVector = MVector::zAxis;
	MPoint polePosition = MPoint::origin;
	MAngle twist = MAngle(0.0);
	bool segmentScaleCompensate = true;

};


class IKEmulator : public MPxNode
{

public:

										IKEmulator();
	virtual								~IKEmulator();

	virtual MStatus						compute(const MPlug& plug, MDataBlock& data);

	virtual	MPxNode::SchedulingType		schedulingType() const;

	static	MStatus						getRestMatrices(MDataBlock& data, MMatrixArray& restMatrices, MMatrix& parentRestMatrix);
	static	MMatrixArray				scaleRestMatrices(MMatrixArray& restMatrices, const double scale);
	static	MStatus						pinRestMatrices(MMatrixArray& restMatrices, const IKSettings& settings, const double& envelope);

	static	MMatrixArray				solve(const MMatrixArray& restMatrices, const IKSettings& settings);
	static	MMatrixArray				solve1Bone(const MMatrix& startRestMatrix, const MMatrix& endRestMatrix, const IKSettings& settings);
	static	MMatrixArray				solve2Bone(const MMatrix& startRestMatrix, const MMatrix& midRestMatrix, const MMatrix& endRestMatrix, const IKSettings& settings);
	static	MMatrixArray				solveNBone(const MMatrixArray& restMatrices, const IKSettings& settings);

	static	MMatrix						createPositionMatrix(const double x, const double y, const double z);
	static	MMatrix						createPositionMatrix(const MVector& position);
	static	MMatrix						createPositionMatrix(const MMatrix& matrix);
	static	MMatrix						createPositionMatrix(const Axis axis, const bool flip, const MMatrix& matrix);

	static	MMatrix						createRotationMatrix(const double x, const double y, const double z, const MEulerRotation::RotationOrder order);
	static	MMatrix						createRotationMatrix(const double x, const double y, const double z);
	static	MMatrix						createRotationMatrix(const MVector& radians, const MEulerRotation::RotationOrder order);
	static	MMatrix						createRotationMatrix(const MVector& radians);
	static	MMatrix						createRotationMatrix(const MMatrix& matrix);

	static	MStatus						createAimMatrix(const MVector& forwardVector, const Axis forwardAxis, const bool forwardAxisFlip, const MVector& upVector, const Axis upAxis, const bool upAxisFlip, const MPoint& origin, MMatrix& matrix);
	static	MStatus						createAimMatrix(const Axis forwardAxis, const bool forwardAxisFlip, const Axis upAxis, const bool upAxisFlip, MMatrix& matrix);

	static	MMatrix						createTwistMatrix(const Axis axis, const bool flip, const MAngle angle);

	static	MMatrix						createScaleMatrix(const double x, const double y, const double z);
	static	MMatrix						createScaleMatrix(const MVector& scale);
	static	MMatrix						createScaleMatrix(const MMatrix& matrix);

	static	MMatrix						composeMatrix(const MVector& xAxis, const MVector& yAxis, const MVector& zAxis, const MPoint& position);
	static	MStatus						reorientMatrices(const Axis forwardAxis, const bool forwardAxisFlip, const Axis upAxis, const bool upAxisFlip, MMatrixArray& matrices);
	static	MMatrixArray				localizeMatrices(const MMatrixArray& matrices, const MMatrix& parentInverseMatrix);
	static	MMatrixArray				globalizeMatrices(const MMatrixArray& matrices, const bool segmentScaleCompensate);

	static	MPoint						matrixToPosition(const MMatrix& matrix);
	static	MEulerRotation				matrixToEulerRotation(const MMatrix& matrix, const MEulerRotation::RotationOrder order);
	static	MQuaternion					matrixToQuaternion(const MMatrix& matrix);
	static	MVector						matrixToScale(const MMatrix& matrix);

	static	double						distanceTo(const MMatrix& startMatrix, const MMatrix& endMatrix);
	static	double						length(const MMatrixArray& matrices);

	static	MObject						createMatrixData(const MMatrix& matrix);
	static	MObject						createMatrixData(const MTransformationMatrix& matrix);
	static	MMatrix						getMatrixData(const MObject& matrixData);
	static	MTransformationMatrix		getTransformData(const MObject& matrixData);
	static	MVector						getMatrixRow(const MMatrix& matrix, const Axis axis, const bool flip, const bool normalize);

	static	void						displayInfo(const MString& variable, const MMatrix& matrix);
	static	void						displayInfo(const MString& variable, const MPoint& point);
	static	void						displayInfo(const MString& variable, const MVector& vector);
	static	void						displayInfo(const MString& variable, const double value);

	static  void*						creator();
	static  MStatus						initialize();

public:

	static	MObject						envelope;
	static	MObject						forwardAxis;
	static	MObject						forwardAxisFlip;
	static	MObject						upAxis;
	static	MObject						upAxisFlip;
	static	MObject						restMatrix;
	static	MObject						parentRestMatrix;
	static	MObject						poleType;
	static	MObject						poleVector;
	static	MObject						poleVectorX;
	static	MObject						poleVectorY;
	static	MObject						poleVectorZ;
	static	MObject						poleMatrix;
	static	MObject						twist;
	static	MObject						stretch;
	static	MObject						soften;
	static	MObject						pin;
	static	MObject						goal;
	static	MObject						segmentScaleCompensate;
	static	MObject						parentInverseMatrix;

	static	MObject						softOrigin;
	static	MObject						softOriginX;
	static	MObject						softOriginY;
	static	MObject						softOriginZ;
	static	MObject						softGoal;
	static	MObject						softGoalX;
	static	MObject						softGoalY;
	static	MObject						softGoalZ;
	static	MObject						softMatrix;
	static	MObject						softVector;
	static	MObject						softVectorX;
	static	MObject						softVectorY;
	static	MObject						softVectorZ;
	static	MObject						softDistance;
	static	MObject						softScale;
	static	MObject						outMatrix;
	static	MObject						outWorldMatrix;

	static	MObject						identityMatrixData;

	static	MString						inputCategory;
	static	MString						outputCategory;
	static	MString						softputCategory;

	static	MTypeId						id;

};

#endif