//
// File: IKEmulatorNode.cpp
//
// Dependency Graph Node: ikEmulator
//
// Author: Benjamin H. Singleton
//

#include "IKEmulatorNode.h"
#include "IKSoftenerNode.h"

MObject		IKEmulator::envelope;
MObject		IKEmulator::forwardAxis;
MObject		IKEmulator::forwardAxisFlip;
MObject		IKEmulator::upAxis;
MObject		IKEmulator::upAxisFlip;
MObject		IKEmulator::restMatrix;
MObject		IKEmulator::parentRestMatrix;
MObject		IKEmulator::poleType;
MObject		IKEmulator::poleVector;
MObject		IKEmulator::poleVectorX;
MObject		IKEmulator::poleVectorY;
MObject		IKEmulator::poleVectorZ;
MObject		IKEmulator::poleMatrix;
MObject		IKEmulator::twist;
MObject		IKEmulator::stretch;
MObject		IKEmulator::soften;
MObject		IKEmulator::pin;
MObject		IKEmulator::goal;
MObject		IKEmulator::segmentScaleCompensate;
MObject		IKEmulator::parentInverseMatrix;

MObject		IKEmulator::softOrigin;
MObject		IKEmulator::softOriginX;
MObject		IKEmulator::softOriginY;
MObject		IKEmulator::softOriginZ;
MObject		IKEmulator::softGoal;
MObject		IKEmulator::softGoalX;
MObject		IKEmulator::softGoalY;
MObject		IKEmulator::softGoalZ;
MObject		IKEmulator::softMatrix;
MObject		IKEmulator::softVector;
MObject		IKEmulator::softVectorX;
MObject		IKEmulator::softVectorY;
MObject		IKEmulator::softVectorZ;
MObject		IKEmulator::softDistance;
MObject		IKEmulator::softScale;
MObject		IKEmulator::outMatrix;
MObject		IKEmulator::outWorldMatrix;

MString		IKEmulator::inputCategory("Input");
MString		IKEmulator::outputCategory("Output");
MString		IKEmulator::softputCategory("Softput");

MObject		IKEmulator::identityMatrixData(IKEmulator::createMatrixData(MMatrix::identity));

MTypeId		IKEmulator::id(0x0013b1ef);


template<class N> N lerp(const N start, const N end, const double weight)
/**
Linearly interpolates the two given numbers using the supplied weight.

@param start: The start number.
@param end: The end number.
@param weight: The amount to blend.
@return: The interpolated value.
*/
{

	return (start * (1.0 - weight)) + (end * weight);

};


IKEmulator::IKEmulator() {}
IKEmulator::~IKEmulator() {}


MStatus IKEmulator::compute(const MPlug& plug, MDataBlock& data)
/**
This method should be overridden in user defined nodes.
Recompute the given output based on the nodes inputs.
The plug represents the data value that needs to be recomputed, and the data block holds the storage for all of the node's attributes.
The MDataBlock will provide smart handles for reading and writing this node's attribute values.
Only these values should be used when performing computations!

@param plug: Plug representing the attribute that needs to be recomputed.
@param data: Data block containing storage for the node's attributes.
@return: Return status.
*/
{

	MStatus status;

	// Evaluate requested attribute
	//
	MObject attribute = plug.attribute(&status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	MFnAttribute fnAttribute(attribute, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	bool isSoftput = fnAttribute.hasCategory(IKEmulator::softputCategory);
	bool isOutput = fnAttribute.hasCategory(IKEmulator::outputCategory);
	
	if (isSoftput)
	{

		// Get input data handles
		//
		MArrayDataHandle restMatrixArrayHandle = data.inputArrayValue(IKEmulator::restMatrix, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle parentRestMatrixHandle = data.inputValue(IKEmulator::parentRestMatrix, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle stretchHandle = data.inputValue(IKEmulator::stretch, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle softenHandle = data.inputValue(IKEmulator::soften, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle goalHandle = data.inputValue(IKEmulator::goal, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle segmentScaleCompensateHandle = data.inputValue(IKEmulator::segmentScaleCompensate, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		// Get input values
		//
		double stretch = stretchHandle.asDouble();
		double soften = softenHandle.asDouble();
		MMatrix goalMatrix = IKEmulator::getMatrixData(goalHandle.data());
		MPoint goal = IKEmulator::matrixToPosition(goalMatrix);
		bool segmentScaleCompensate = segmentScaleCompensateHandle.asBool();

		// Get rest origin
		//
		MMatrixArray restMatrices;
		MMatrix parentRestMatrix;

		status = IKEmulator::getRestMatrices(data, restMatrices, parentRestMatrix);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		unsigned int matrixCount = restMatrices.length();
		MPoint origin = (matrixCount > 0) ? IKEmulator::matrixToPosition(restMatrices[0]) : MPoint::origin;
		
		MMatrixArray worldRestMatrices = IKEmulator::globalizeMatrices(restMatrices, segmentScaleCompensate);
		double chainLength = IKEmulator::length(worldRestMatrices);

		// Calculate soft IK goal
		//
		MPoint softGoal;
		MVector softVector;
		double softScale, softDistance;

		IKSoftener::solve(origin, goal, chainLength, soften, softGoal, softVector, softDistance, softScale);

		MPoint stretchGoal = lerp(softGoal, goal, stretch);
		MMatrix stretchMatrix = IKEmulator::createScaleMatrix(goalMatrix) * IKEmulator::createRotationMatrix(goalMatrix) * IKEmulator::createPositionMatrix(stretchGoal);
		double stretchScale = lerp(1.0, softScale, stretch);
		double stretchDistance = stretchGoal.distanceTo(origin);
		
		// Update output handles
		//
		MDataHandle softOriginHandle = data.outputValue(IKEmulator::softOrigin, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle softGoalHandle = data.outputValue(IKEmulator::softGoal, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle softMatrixHandle = data.outputValue(IKEmulator::softMatrix, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle softVectorHandle = data.outputValue(IKEmulator::softVector, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle softDistanceHandle = data.outputValue(IKEmulator::softDistance, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle softScaleHandle = data.outputValue(IKEmulator::softScale, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		softOriginHandle.setMVector(origin);
		softOriginHandle.setClean();

		softGoalHandle.setMVector(stretchGoal);
		softGoalHandle.setClean();

		softMatrixHandle.setMObject(IKEmulator::createMatrixData(stretchMatrix));
		softMatrixHandle.setClean();

		softVectorHandle.setMVector(softVector);
		softVectorHandle.setClean();

		softDistanceHandle.setDouble(stretchDistance);
		softDistanceHandle.setClean();

		softScaleHandle.setDouble(stretchScale);
		softScaleHandle.setClean();

		// Mark plug as clean
		//
		status = data.setClean(plug);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		return MS::kSuccess;

	}
	else if (isOutput)
	{
		
		// Get input data handles
		//
		MDataHandle envelopeHandle = data.inputValue(IKEmulator::envelope, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle forwardAxisHandle = data.inputValue(IKEmulator::forwardAxis, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle forwardAxisFlipHandle = data.inputValue(IKEmulator::forwardAxisFlip, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle upAxisHandle = data.inputValue(IKEmulator::upAxis, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle upAxisFlipHandle = data.inputValue(IKEmulator::upAxisFlip, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MArrayDataHandle restMatrixArrayHandle = data.inputArrayValue(IKEmulator::restMatrix, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle parentRestMatrixHandle = data.inputValue(IKEmulator::parentRestMatrix, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle poleTypeHandle = data.inputValue(IKEmulator::poleType, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle poleVectorHandle = data.inputValue(IKEmulator::poleVector, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle poleVectorXHandle = poleVectorHandle.child(IKEmulator::poleVectorX);
		MDataHandle poleVectorYHandle = poleVectorHandle.child(IKEmulator::poleVectorY);
		MDataHandle poleVectorZHandle = poleVectorHandle.child(IKEmulator::poleVectorZ);

		MDataHandle poleMatrixHandle = data.inputValue(IKEmulator::poleMatrix, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle twistHandle = data.inputValue(IKEmulator::twist, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle stretchHandle = data.inputValue(IKEmulator::stretch, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle pinHandle = data.inputValue(IKEmulator::pin, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle softMatrixHandle = data.inputValue(IKEmulator::softMatrix, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle softScaleHandle = data.inputValue(IKEmulator::softScale, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle segmentScaleCompensateHandle = data.inputValue(IKEmulator::segmentScaleCompensate, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle parentInverseMatrixHandle = data.inputValue(IKEmulator::parentInverseMatrix, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		// Get input values
		//
		double envelope = envelopeHandle.asDouble();

		Axis forwardAxis = Axis(forwardAxisHandle.asShort());
		bool forwardAxisFlip = forwardAxisFlipHandle.asBool();
		Axis upAxis = Axis(upAxisHandle.asShort());
		bool upAxisFlip = upAxisFlipHandle.asBool();

		MAngle twistAngle = twistHandle.asAngle();
		double stretch = stretchHandle.asDouble();
		double pin = pinHandle.asDouble();
		MMatrix softMatrix = IKEmulator::getMatrixData(softMatrixHandle.data());
		MPoint softGoal = IKEmulator::matrixToPosition(softMatrix);
		double softScale = softScaleHandle.asDouble();

		bool segmentScaleCompensate = segmentScaleCompensateHandle.asBool();
		MMatrix parentInverseMatrix = IKEmulator::getMatrixData(parentInverseMatrixHandle.data());

		// Get rest matrices
		//
		MMatrixArray restMatrices;
		MMatrix parentRestMatrix;

		status = IKEmulator::getRestMatrices(data, restMatrices, parentRestMatrix);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MMatrixArray scaledRestMatrices = IKEmulator::scaleRestMatrices(restMatrices, softScale);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		unsigned int matrixCount = scaledRestMatrices.length();
		MPoint origin = (matrixCount > 0) ? IKEmulator::matrixToPosition(scaledRestMatrices[0]) : MPoint::origin;

		MMatrixArray worldRestMatrices = IKEmulator::globalizeMatrices(scaledRestMatrices, segmentScaleCompensate);
		double chainLength = IKEmulator::length(worldRestMatrices);

		// Get pole vector
		//
		PoleType poleType = PoleType(poleTypeHandle.asShort());

		MVector poleVector;
		MPoint polePosition;
		MMatrix poleMatrix;

		switch (poleType)
		{

		case PoleType::Align:
			poleVector = poleVectorHandle.asVector();
			polePosition = origin + poleVector;
			break;

		case PoleType::LookAt:
			polePosition = poleVectorHandle.asVector();
			poleVector = (MVector(polePosition) - MVector(origin)).normal();
			break;

		case PoleType::Matrix:
			poleMatrix = IKEmulator::getMatrixData(poleMatrixHandle.data());
			polePosition = IKEmulator::matrixToPosition(poleMatrix);
			poleVector = (MVector(polePosition) - MVector(origin)).normal();
			break;

		case PoleType::Goal:
			poleVector = IKEmulator::getMatrixRow(softMatrix, upAxis, upAxisFlip, true);
			polePosition = origin + poleVector;
			break;

		}

		// Stretch or pin rest matrices
		//
		IKSettings settings = { softMatrix, forwardAxis, forwardAxisFlip, upAxis, upAxisFlip, poleVector, polePosition, twistAngle, segmentScaleCompensate };

		if (pin > 0.0 && matrixCount == 3)
		{

			status = IKEmulator::pinRestMatrices(scaledRestMatrices, settings, pin);
			CHECK_MSTATUS_AND_RETURN_IT(status);

		}

		// Solve IK system
		//
		MMatrixArray localIKMatrices = IKEmulator::solve(scaledRestMatrices, settings);
		bool isIdentity = MMatrix::identity.isEquivalent(parentInverseMatrix);

		if (matrixCount > 0 && !isIdentity)
		{
			
			localIKMatrices[0] = localIKMatrices[0] * parentInverseMatrix;

		}

		MMatrixArray worldIKMatrices = IKEmulator::globalizeMatrices(localIKMatrices, segmentScaleCompensate);

		// Update output array handles
		//
		MArrayDataHandle outWorldMatrixArrayHandle = data.outputArrayValue(IKEmulator::outWorldMatrix, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MArrayDataBuilder outWorldMatrixArrayBuilder(&data, IKEmulator::outWorldMatrix, matrixCount, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MArrayDataHandle outMatrixArrayHandle = data.outputArrayValue(IKEmulator::outMatrix, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MArrayDataBuilder outMatrixArrayBuilder(&data, IKEmulator::outMatrix, matrixCount, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle outWorldMatrixHandle, outMatrixHandle;
		MObject outWorldMatrixData, outMatrixData;

		for (unsigned int i = 0; i < matrixCount; i++)
		{

			outWorldMatrixHandle = outWorldMatrixArrayBuilder.addElement(i, &status);
			CHECK_MSTATUS_AND_RETURN_IT(status);

			outWorldMatrixData = IKEmulator::createMatrixData(worldIKMatrices[i]);

			outWorldMatrixHandle.setMObject(outWorldMatrixData);
			outWorldMatrixHandle.setClean();

			outMatrixHandle = outMatrixArrayBuilder.addElement(i, &status);
			CHECK_MSTATUS_AND_RETURN_IT(status);

			outMatrixData = IKEmulator::createMatrixData(localIKMatrices[i]);

			outMatrixHandle.setMObject(outMatrixData);
			outMatrixHandle.setClean();

		}

		status = outWorldMatrixArrayHandle.set(outWorldMatrixArrayBuilder);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		status = outWorldMatrixArrayHandle.setAllClean();
		CHECK_MSTATUS_AND_RETURN_IT(status);
		
		status = outMatrixArrayHandle.set(outMatrixArrayBuilder);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		status = outMatrixArrayHandle.setAllClean();
		CHECK_MSTATUS_AND_RETURN_IT(status);

		// Mark plug as clean
		//
		status = data.setClean(plug);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		return MS::kSuccess;

	}
	else
	{

		return MS::kUnknownParameter;

	}

};


MPxNode::SchedulingType IKEmulator::schedulingType() const
/**
When overridden this method controls the degree of parallelism supported by the node during threaded evaluation.

@return: The scheduling type to be used for this node.
*/
{

	return MPxNode::SchedulingType::kParallel;

};


MStatus IKEmulator::getRestMatrices(MDataBlock& data, MMatrixArray& restMatrices, MMatrix& parentRestMatrix)
{

	MStatus status;

	// Resize passed array
	//
	MArrayDataHandle restMatrixArrayHandle = data.inputArrayValue(IKEmulator::restMatrix, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	MDataHandle parentRestMatrixHandle = data.inputValue(IKEmulator::parentRestMatrix, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	unsigned matrixCount = restMatrixArrayHandle.elementCount(&status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	status = restMatrices.setLength(matrixCount);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// Iterate through element handles
	//
	MDataHandle restMatrixHandle;
	MMatrix restMatrix, parentInverseScaleMatrix;

	for (unsigned int i = 0; i < matrixCount; i++)
	{

		status = restMatrixArrayHandle.jumpToArrayElement(i);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		restMatrixHandle = restMatrixArrayHandle.inputValue(&status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		restMatrices[i] = IKEmulator::getMatrixData(restMatrixHandle.data());

	}

	// Update parent space
	//
	parentRestMatrix = IKEmulator::getMatrixData(parentRestMatrixHandle.data());
	bool isIdentity = MMatrix::identity.isEquivalent(parentRestMatrix);

	if (matrixCount > 0 && !isIdentity)
	{

		restMatrices[0] = restMatrices[0] * parentRestMatrix;

	}

	return status;

};


MMatrixArray IKEmulator::scaleRestMatrices(MMatrixArray& restMatrices, const double scale)
{

	MMatrixArray scaledMatrices = MMatrixArray(restMatrices);
	unsigned int matrixCount = scaledMatrices.length();

	MVector scaledPosition;
	MMatrix scaledRestMatrix;

	for (unsigned int i = 1; i < matrixCount; i++)
	{

		scaledPosition = IKEmulator::matrixToPosition(restMatrices[i].matrix) * scale;
		scaledRestMatrix = IKEmulator::createScaleMatrix(restMatrices[i]) * IKEmulator::createRotationMatrix(restMatrices[i]) * IKEmulator::createPositionMatrix(scaledPosition);
		
		scaledMatrices[i] = scaledRestMatrix;

	}

	return scaledMatrices;

};


MStatus IKEmulator::pinRestMatrices(MMatrixArray& restMatrices, const IKSettings& settings, const double& envelope)
{

	unsigned int jointCount = restMatrices.length();

	if (jointCount != 3)
	{

		return MS::kFailure;

	}

	MMatrixArray worldRestMatrices = IKEmulator::globalizeMatrices(restMatrices, settings.segmentScaleCompensate);
	MPoint origin = IKEmulator::matrixToPosition(restMatrices[0]);

	double initialStartDistance = IKEmulator::distanceTo(worldRestMatrices[0], worldRestMatrices[1]);
	double preferredStartDistance = origin.distanceTo(settings.polePosition);
	double startScale = lerp(1.0, preferredStartDistance / initialStartDistance, envelope);

	MPoint midRestPosition = IKEmulator::matrixToPosition(restMatrices[1]) * startScale;
	MMatrix midRestMatrix = IKEmulator::createScaleMatrix(restMatrices[1]) * IKEmulator::createRotationMatrix(restMatrices[1]) * IKEmulator::createPositionMatrix(midRestPosition);
	restMatrices[1] = midRestMatrix;

	double initialEndDistance = IKEmulator::distanceTo(worldRestMatrices[1], worldRestMatrices[2]);
	double preferredEndDistance = settings.polePosition.distanceTo(IKEmulator::matrixToPosition(settings.goal));
	double endScale = lerp(1.0, preferredEndDistance / initialEndDistance, envelope);

	MPoint endRestPosition = IKEmulator::matrixToPosition(restMatrices[2]) * endScale;
	MMatrix endRestMatrix = IKEmulator::createScaleMatrix(restMatrices[2]) * IKEmulator::createRotationMatrix(restMatrices[2]) * IKEmulator::createPositionMatrix(endRestPosition);
	restMatrices[2] = endRestMatrix;

	return MS::kSuccess;

};


MMatrixArray IKEmulator::solve(const MMatrixArray& restMatrices, const IKSettings& settings)
/**
Returns an IK solution for the supplied joint chain.
The solution uses x-forward and y-up as the default axes in world-space!

@param goal: The IK goal in world space.
@param poleVector: The up vector to orient the joint chain.
@param twistAngle: The twist value along the aim vector.
@param joints: The joints in their respective parent spaces.
@return: The IK solution.
*/
{

	unsigned int matrixCount = restMatrices.length();

	switch (matrixCount)
	{

	case 0: case 1:
		return MMatrixArray();

	case 2:
		return IKEmulator::solve1Bone(restMatrices[0], restMatrices[1], settings);

	case 3:
		return IKEmulator::solve2Bone(restMatrices[0], restMatrices[1], restMatrices[2], settings);

	default:
		return IKEmulator::solveNBone(restMatrices, settings);

	}

};


MMatrixArray IKEmulator::solve1Bone(const MMatrix& startRestMatrix, const MMatrix& endRestMatrix, const IKSettings& settings)
{

	MStatus status;

	// Calculate forward vector
	//
	MMatrixArray restMatrices = MMatrixArray(2);
	restMatrices[0] = startRestMatrix;
	restMatrices[1] = endRestMatrix;

	MMatrixArray globalRestMatrices = IKEmulator::globalizeMatrices(restMatrices, settings.segmentScaleCompensate);

	MPoint origin = IKEmulator::matrixToPosition(startRestMatrix);
	MPoint goal = IKEmulator::matrixToPosition(settings.goal);
	MVector forwardVector = (goal - origin).normal();


	MPoint tip = IKEmulator::matrixToPosition(globalRestMatrices[1]);
	MVector tipVector = (tip - origin).normal();
	MVector restPoleVector = IKEmulator::getMatrixRow(globalRestMatrices[0], settings.upAxis, settings.upAxisFlip, true);

	MMatrix tipAimMatrix;

	status = IKEmulator::createAimMatrix(tipVector, settings.forwardAxis, settings.forwardAxisFlip, restPoleVector, settings.upAxis, settings.upAxisFlip, origin, tipAimMatrix);
	CHECK_MSTATUS_AND_RETURN(status, restMatrices);

	MMatrix offsetMatrix = startRestMatrix * tipAimMatrix.inverse();

	// Calculate axis vectors
	//
	MMatrix aimMatrix;

	status = IKEmulator::createAimMatrix(forwardVector, settings.forwardAxis, settings.forwardAxisFlip, settings.poleVector, settings.upAxis, settings.upAxisFlip, origin, aimMatrix);
	CHECK_MSTATUS_AND_RETURN(status, restMatrices);

	MMatrix twistMatrix = IKEmulator::createTwistMatrix(settings.forwardAxis, settings.forwardAxisFlip, settings.twist);
	MMatrix twistedAimMatrix = twistMatrix * aimMatrix;

	MMatrix startMatrix = offsetMatrix * twistedAimMatrix;

	// Populate matrix array
	//
	MMatrixArray matrices = MMatrixArray(restMatrices);
	matrices[0] = IKEmulator::createScaleMatrix(startRestMatrix) * IKEmulator::createRotationMatrix(startMatrix) * IKEmulator::createPositionMatrix(startRestMatrix);

	return matrices;

};


MMatrixArray IKEmulator::solve2Bone(const MMatrix& startRestMatrix, const MMatrix& midRestMatrix, const MMatrix& endRestMatrix, const IKSettings& settings)
{

	MStatus status;

	// Calculate min/max limb length
	//
	MMatrixArray restMatrices = MMatrixArray(3);
	restMatrices[0] = startRestMatrix;
	restMatrices[1] = midRestMatrix;
	restMatrices[2] = endRestMatrix;

	MMatrixArray globalRestMatrices = IKEmulator::globalizeMatrices(restMatrices, settings.segmentScaleCompensate);

	double startLength = IKEmulator::distanceTo(globalRestMatrices[0], globalRestMatrices[1]);
	double endLength = IKEmulator::distanceTo(globalRestMatrices[1], globalRestMatrices[2]);

	double maxDistance = startLength + endLength;
	double minDistance = std::fabs(endLength - startLength);

	// Calculate aim vector
	//
	MPoint origin = IKEmulator::matrixToPosition(startRestMatrix);
	MPoint goal = IKEmulator::matrixToPosition(settings.goal);

	MVector aimVector = goal - origin;
	MVector forwardVector = aimVector.normal();
	double distance = aimVector.length();

	// Calculate angles using law of cosines
	//
	double startRadian = 0.0;
	double endRadian = 0.0;

	if (distance < (minDistance + 1e-3)) // Collapsed
	{

		endRadian = 0.0;

	}
	else if (distance > (maxDistance - 1e-3))  // Hyper-extended
	{

		endRadian = M_PI;

	}
	else  // Default
	{

		startRadian = acos((pow(startLength, 2.0) + pow(distance, 2.0) - pow(endLength, 2.0)) / (2.0 * startLength * distance));
		endRadian = acos((pow(endLength, 2.0) + pow(startLength, 2.0) - pow(distance, 2.0)) / (2.0 * endLength * startLength));

	}
	
	// Calculate initial aim matrix
	//
	MMatrix initialAimMatrix;

	status = IKEmulator::createAimMatrix(forwardVector, Axis::X, false, settings.poleVector, Axis::Y, false, origin, initialAimMatrix);
	CHECK_MSTATUS_AND_RETURN(status, restMatrices);
	
	MMatrix twistMatrix = IKEmulator::createTwistMatrix(Axis::X, false, settings.twist);
	MMatrix aimMatrix = twistMatrix * initialAimMatrix;
	
	// Compose IK matrices
	//
	MMatrixArray worldIKMatrices = MMatrixArray(restMatrices);
	worldIKMatrices[0] = IKEmulator::createRotationMatrix(0.0, 0.0, startRadian) * aimMatrix;
	worldIKMatrices[1] = (IKEmulator::createRotationMatrix(0.0, 0.0, -(M_PI - endRadian)) * IKEmulator::createPositionMatrix(startLength, 0.0, 0.0)) * worldIKMatrices[0];
	worldIKMatrices[2] = IKEmulator::createPositionMatrix(endLength, 0.0, 0.0) * worldIKMatrices[1];
	
	status = IKEmulator::reorientMatrices(settings.forwardAxis, settings.forwardAxisFlip, settings.upAxis, settings.upAxisFlip, worldIKMatrices);
	CHECK_MSTATUS_AND_RETURN(status, restMatrices);

	MMatrixArray localIKMatrices = IKEmulator::localizeMatrices(worldIKMatrices, MMatrix::identity);

	// Populate matrix array
	//
	MMatrixArray matrices = MMatrixArray(restMatrices);
	matrices[0] = IKEmulator::createScaleMatrix(startRestMatrix) * IKEmulator::createRotationMatrix(localIKMatrices[0]) * IKEmulator::createPositionMatrix(startRestMatrix);
	matrices[1] = IKEmulator::createScaleMatrix(midRestMatrix) * IKEmulator::createRotationMatrix(localIKMatrices[1]) * IKEmulator::createPositionMatrix(midRestMatrix);
	matrices[2] = IKEmulator::createScaleMatrix(endRestMatrix) * IKEmulator::createRotationMatrix(localIKMatrices[2]) * IKEmulator::createPositionMatrix(endRestMatrix);

	return matrices;

};


MMatrixArray IKEmulator::solveNBone(const MMatrixArray& restMatrices, const IKSettings& settings)
{

	return MMatrixArray(restMatrices); // TODO: Implement N-bone solver!

};


MMatrix IKEmulator::createPositionMatrix(const double x, const double y, const double z)
/**
Returns a position matrix from the supplied XYZ values.

@param x: The X value.
@param x: The Y value.
@param x: The Z value.
@return: The new position matrix.
*/
{

	double rows[4][4] = {
		{ 1.0, 0.0, 0.0, 0.0 },
		{ 0.0, 1.0, 0.0, 0.0 },
		{ 0.0, 0.0, 1.0, 0.0 },
		{ x, y, z, 1.0 },
	};

	return MMatrix(rows);

};


MMatrix IKEmulator::createPositionMatrix(const MVector& position)
/**
Returns a position matrix from the supplied vector.

@param position: The vector to convert.
@return: The new position matrix.
*/
{

	return IKEmulator::createPositionMatrix(position.x, position.y, position.z);

};


MMatrix IKEmulator::createPositionMatrix(const MMatrix& matrix)
/**
Returns a position matrix from the supplied transform matrix.

@param position: The transform matrix to convert.
@return: The new position matrix.
*/
{

	return IKEmulator::createPositionMatrix(matrix(3, 0), matrix(3, 1), matrix(3, 2));

};


MMatrix IKEmulator::createPositionMatrix(const Axis axis, const bool flip, const MMatrix& matrix)
{

	MPoint position = IKEmulator::matrixToPosition(matrix);
	double distance = MPoint::origin.distanceTo(position);

	unsigned int index = static_cast<unsigned int>(axis);

	MVector vector = MVector::zero;
	vector[index] = (flip) ? -distance : distance;

	return IKEmulator::createPositionMatrix(vector);

};


MMatrix IKEmulator::createRotationMatrix(const double x, const double y, const double z, const MEulerRotation::RotationOrder order)
/**
Creates a rotation matrix from the supplied angles and axis order.

@param x: The X angle in radians.
@param y: The Y angle in radians.
@param z: The Z angle in radians.
@param axisOrder: The axis order.
@return: The new rotation matrix.
*/
{

	return MEulerRotation(x, y, z, order).asMatrix();

};


MMatrix IKEmulator::createRotationMatrix(const double x, const double y, const double z)
/**
Creates a rotation matrix from the supplied angles.

@param x: The X angle in radians.
@param y: The Y angle in radians.
@param z: The Z angle in radians.
@return: The new rotation matrix.
*/
{

	return IKEmulator::createRotationMatrix(x, y, z, MEulerRotation::RotationOrder::kXYZ);

};


MMatrix IKEmulator::createRotationMatrix(const MVector& radians, const MEulerRotation::RotationOrder order)
/**
Creates a rotation matrix from the supplied radians and rotation order.

@param radians: The XYZ values in radians.
@param axisOrder: The rotation order.
@return: The new rotation matrix.
*/
{

	return IKEmulator::createRotationMatrix(radians.x, radians.y, radians.z, order);

};


MMatrix IKEmulator::createRotationMatrix(const MVector& radians)
/**
Creates a rotation matrix from the supplied angles.

@param radians: The XYZ values in radians.
@return: The new rotation matrix.
*/
{

	return IKEmulator::createRotationMatrix(radians, MEulerRotation::RotationOrder::kXYZ);

};


MMatrix IKEmulator::createRotationMatrix(const MMatrix& matrix)
/**
Creates a rotation matrix from the supplied transformation matrix.

@param matrix: The transform matrix to convert.
@return: The new rotation matrix.
*/
{

	MVector xAxis = MVector(matrix[0]).normal();
	MVector yAxis = MVector(matrix[1]).normal();
	MVector zAxis = MVector(matrix[2]).normal();

	double rows[4][4] = {
		{ xAxis.x, xAxis.y, xAxis.z, 0.0 },
		{ yAxis.x, yAxis.y, yAxis.z, 0.0 },
		{ zAxis.x, zAxis.y, zAxis.z, 0.0 },
		{ 0.0, 0.0, 0.0, 1.0 },
	};

	return MMatrix(rows);

};


MStatus IKEmulator::createAimMatrix(const MVector& forwardVector, const Axis forwardAxis, const bool forwardAxisFlip, const MVector& upVector, const Axis upAxis, const bool upAxisFlip, const MPoint& origin, MMatrix& matrix)
/**
	Creates an aim matrix from the supplied parameters.
	Both of the supplied axes must be unique!

	@param forwardVector: The forward-vector.
	@param forwardAxis: The forward-axis.
	@param forwardAxisFlip: Determines if the forward-axis should be flipped.
	@param upVector: The up-vector.
	@param upAxis: The up-axis.
	@param upAxisFlip: Determines if the up-axis should be flipped.
	@param origin: The point of origin.
	@param matrix: The passed matrix to populate.
	@return: Return status.
	*/
{

	MStatus status;

	// Evaluate forward-axis
	//
	MVector xAxis = MVector::xAxis;
	MVector yAxis = MVector::yAxis;
	MVector zAxis = MVector::zAxis;

	switch (forwardAxis)
	{

	case Axis::X:
	{

		xAxis = forwardAxisFlip ? MVector(-forwardVector) : MVector(forwardVector);

		if (upAxis == Axis::Y)
		{

			zAxis = xAxis ^ (upAxisFlip ? MVector(-upVector) : MVector(upVector));
			yAxis = zAxis ^ xAxis;

		}
		else if (upAxis == Axis::Z)
		{

			yAxis = (upAxisFlip ? MVector(-upVector) : MVector(upVector)) ^ xAxis;
			zAxis = xAxis ^ yAxis;

		}
		else
		{

			return MS::kFailure;

		}

	}
	break;

	case Axis::Y:
	{

		yAxis = forwardAxisFlip ? MVector(-forwardVector) : MVector(forwardVector);

		if (upAxis == Axis::X)
		{

			zAxis = (upAxisFlip ? MVector(-upVector) : MVector(upVector)) ^ yAxis;
			xAxis = yAxis ^ zAxis;

		}
		else if (upAxis == Axis::Z)
		{

			xAxis = yAxis ^ (upAxisFlip ? MVector(-upVector) : MVector(upVector));
			zAxis = xAxis ^ yAxis;

		}
		else
		{

			return MS::kFailure;

		}

	}
	break;

	case Axis::Z:
	{

		zAxis = forwardAxisFlip ? MVector(-forwardVector) : MVector(forwardVector);

		if (upAxis == Axis::X)
		{

			yAxis = zAxis ^ (upAxisFlip ? MVector(-upVector) : MVector(upVector));
			xAxis = yAxis ^ zAxis;

		}
		else if (upAxis == Axis::Y)
		{

			xAxis = (upAxisFlip ? MVector(-upVector) : MVector(upVector)) ^ zAxis;
			yAxis = zAxis ^ xAxis;

		}
		else
		{

			return MS::kFailure;

		}

	}
	break;

	default:
	{

		return MS::kFailure;

	}

	}

	// Normalize axis vectors
	//
	status = xAxis.normalize();
	CHECK_MSTATUS_AND_RETURN_IT(status);

	status = yAxis.normalize();
	CHECK_MSTATUS_AND_RETURN_IT(status);

	status = zAxis.normalize();
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// Compose aim matrix
	//
	matrix = IKEmulator::composeMatrix(xAxis, yAxis, zAxis, origin);

	return MS::kSuccess;

};


MStatus IKEmulator::createAimMatrix(const Axis forwardAxis, const bool forwardAxisFlip, const Axis upAxis, const bool upAxisFlip, MMatrix& matrix)
/**
Creates an aim matrix from the supplied parameters.
Both of the supplied axes must be unique!

@param forwardAxis: The forward axis.
@param forwardAxisFlip: Determines if the forward axis is inversed.
@param upAxis: The up axis.
@param upAxisFlip: Determines if the up axis is inversed.
@param matrix: The passed matrix to populate.
@return: Return status.
*/
{

	return IKEmulator::createAimMatrix(MVector::xAxis, forwardAxis, forwardAxisFlip, MVector::yAxis, upAxis, upAxisFlip, MPoint::origin, matrix);

};


MMatrix IKEmulator::createTwistMatrix(const Axis axis, const bool flip, const MAngle angle)
{

	unsigned int index = static_cast<unsigned int>(axis);

	MEulerRotation eulerRotation = MEulerRotation::identity;
	eulerRotation[index] = (flip) ? -angle.asRadians() : angle.asRadians();

	return eulerRotation.asMatrix();

};


MMatrix IKEmulator::createScaleMatrix(const double x, const double y, const double z)
/**
Returns a scale matrix from the supplied XYZ values.

@param x: The X value.
@param x: The Y value.
@param x: The Z value.
@return: The new scale matrix.
*/
{

	double rows[4][4] = {
		{ x, 0.0, 0.0, 0.0 },
		{ 0.0, y, 0.0, 0.0 },
		{ 0.0, 0.0, z, 0.0 },
		{ 0.0, 0.0, 0.0, 1.0 },
	};

	return MMatrix(rows);

};


MMatrix IKEmulator::createScaleMatrix(const MVector& scale)
/**
Returns a scale matrix from the supplied scale vector.

@param scale: The vector to convert.
@return: The new scale matrix.
*/
{

	return IKEmulator::createScaleMatrix(scale.x, scale.y, scale.z);

};


MMatrix IKEmulator::createScaleMatrix(const MMatrix& matrix)
/**
Returns a scale matrix from the supplied transformation matrix.

@param matrix: The transform matrix to convert.
@return: The new scale matrix.
*/
{

	return IKEmulator::createScaleMatrix(matrixToScale(matrix));

};


MMatrix IKEmulator::composeMatrix(const MVector& xAxis, const MVector& yAxis, const MVector& zAxis, const MPoint& position)
/**
Returns a matrix from the supplied axis vectors and position.

@param xAxis: The x-axis vector.
@param yAxis: The y-axis vector.
@param zAxis: The z-axis vector.
@param position: The positional value.
@return: Transform matrix.
*/
{

	double rows[4][4] = {
		{xAxis.x, xAxis.y, xAxis.z, 0.0},
		{yAxis.x, yAxis.y, yAxis.z, 0.0},
		{zAxis.x, zAxis.y, zAxis.z, 0.0},
		{position.x, position.y, position.z, 1.0}
	};

	return MMatrix(rows);

};


MStatus IKEmulator::reorientMatrices(const Axis forwardAxis, const bool forwardAxisFlip, const Axis upAxis, const bool upAxisFlip, MMatrixArray& matrices)
/**
Re-orients the passed matrices based on the supplied axis alignments.

@param matrices: The matrices to re-orient.
@param forwardAxis: The forward-axis.
@param forwardAxisFlip: Determines if the forward-axis should be flipped.
@param upAxis: The up-axis.
@param upAxisFlip: Determines if the up-axis should be flipped.
@return: The re-oriented matrices.
*/
{

	MStatus status;

	// Compose aim matrix
	//
	MMatrix aimMatrix;

	status = IKEmulator::createAimMatrix(forwardAxis, forwardAxisFlip, upAxis, upAxisFlip, aimMatrix);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// Re-orient matrices
	//
	unsigned int numMatrices = matrices.length();

	for (unsigned int i = 0; i < numMatrices; i++)
	{

		matrices[i] = aimMatrix * matrices[i];

	}

	return MS::kSuccess;

};


MMatrixArray IKEmulator::localizeMatrices(const MMatrixArray& matrices, const MMatrix& parentInverseMatrix)
/**
Returns an array of matrices where each matrix is converted into local space using the preceding matrix as its parent space.

@param matrices: The matrices to localize.
@return: The matrices in local space.
*/
{

	MMatrixArray newMatrices = MMatrixArray(matrices);
	unsigned int numMatrices = newMatrices.length();

	MMatrix previousInverseMatrix;

	for (unsigned int i = 0; i < numMatrices; i++)
	{

		previousInverseMatrix = (i == 0) ? parentInverseMatrix : matrices[i - 1].inverse();
		newMatrices[i] = matrices[i] * previousInverseMatrix;

	}

	return newMatrices;

};


MMatrixArray IKEmulator::globalizeMatrices(const MMatrixArray& matrices, const bool segmentScaleCompensate)
/**
Returns an array of matrices where each matrix is converted into global space using the first matrix as world space.

@param matrices: The matrices to globalize.
@return: The matrices in global space.
*/
{

	MMatrixArray worldMatrices = MMatrixArray(matrices);
	unsigned int matrixCount = worldMatrices.length();

	MMatrix matrix, inverseScaleMatrix;

	for (unsigned int i = 1; i < matrixCount; i++)
	{

		if (segmentScaleCompensate)
		{

			inverseScaleMatrix = IKEmulator::createScaleMatrix(matrices[i - 1]).inverse();
			matrix = IKEmulator::createScaleMatrix(matrices[i]) * IKEmulator::createRotationMatrix(matrices[i]) * inverseScaleMatrix * IKEmulator::createPositionMatrix(matrices[i]);
			
			worldMatrices[i] = matrix * worldMatrices[i - 1];

		}
		else
		{

			worldMatrices[i] = matrices[i] * worldMatrices[i - 1];

		}

	}

	return worldMatrices;

};


MPoint IKEmulator::matrixToPosition(const MMatrix& matrix)
/**
Extracts the position component from the supplied transform matrix.

@param matrix: The transform matrix to extract from.
@return: The position value.
*/
{

	return MPoint(matrix[3]);

};


MEulerRotation IKEmulator::matrixToEulerRotation(const MMatrix& matrix, const MEulerRotation::RotationOrder order)
/**
Converts the supplied transform matrix into an euler rotation using the specified rotation order.

@param matrix: The matrix to convert.
@param axis: The rotation order for the euler angles.
@return: The euler rotation.
*/
{

	MEulerRotation eulerRotation;
	eulerRotation = IKEmulator::createRotationMatrix(matrix);
	eulerRotation.reorderIt(order);

	return eulerRotation;

};


MQuaternion IKEmulator::matrixToQuaternion(const MMatrix& matrix)
/**
Converts the supplied transform matrix into a quaternion.

@param matrix: The matrix to convert.
@return: The transform orientation as a quaternion.
*/
{

	MQuaternion quat;
	quat = IKEmulator::createRotationMatrix(matrix);

	return quat;

};


MVector IKEmulator::matrixToScale(const MMatrix& matrix)
/**
Extracts the scale component from the supplied transform matrix.

@param matrix: The transform matrix to extract from.
@return: The scale value.
*/
{

	MVector xAxis = MVector(matrix[0]);
	MVector yAxis = MVector(matrix[1]);
	MVector zAxis = MVector(matrix[2]);

	return MVector(xAxis.length(), yAxis.length(), zAxis.length());

};


double IKEmulator::distanceTo(const MMatrix& startMatrix, const MMatrix& endMatrix)
{
	
	return IKEmulator::matrixToPosition(startMatrix).distanceTo(IKEmulator::matrixToPosition(endMatrix));

};


double IKEmulator::length(const MMatrixArray& matrices)
{

	unsigned int matrixCount = matrices.length();
	double length = 0.0;

	for (unsigned int i = 1; i < matrixCount; i++)
	{

		length += IKEmulator::distanceTo(matrices[i - 1], matrices[i]);

	}

	return length;

};


MObject IKEmulator::createMatrixData(const MMatrix& matrix)
/**
Returns a matrix data object from the supplied matrix.

@param matrix: The matrix to convert.
@param status: Status code.
@return: Matrix data object.
*/
{

	MStatus status;

	// Create new matrix data
	//
	MFnMatrixData fnMatrixData;

	MObject matrixData = fnMatrixData.create(&status);
	CHECK_MSTATUS_AND_RETURN(status, MObject::kNullObj);

	// Assign identity matrix
	//
	status = fnMatrixData.set(matrix);
	CHECK_MSTATUS_AND_RETURN(status, MObject::kNullObj);

	return matrixData;

};


MObject IKEmulator::createMatrixData(const MTransformationMatrix& transform)
/**
Returns a matrix data object from the supplied transformation matrix.

@param matrix: The transformation matrix to convert.
@param status: Status code.
@return: Matrix data object.
*/
{

	MStatus status;

	// Create new matrix data
	//
	MFnMatrixData fnMatrixData;

	MObject matrixData = fnMatrixData.create(&status);
	CHECK_MSTATUS_AND_RETURN(status, MObject::kNullObj);

	// Assign identity matrix
	//
	status = fnMatrixData.set(transform);
	CHECK_MSTATUS_AND_RETURN(status, MObject::kNullObj);

	return matrixData;

};


MMatrix	IKEmulator::getMatrixData(const MObject& matrixData)
/**
Returns the matrix value from the supplied matrix data object.

@param matrixData: The matrix data object.
@param status: Status code.
@return: The matrix value.
*/
{

	MStatus status;

	MFnMatrixData fnMatrixData(matrixData, &status);
	CHECK_MSTATUS_AND_RETURN(status, MMatrix::identity);

	bool isTransformation = fnMatrixData.isTransformation(&status);
	CHECK_MSTATUS_AND_RETURN(status, MMatrix::identity);

	MMatrix matrix;

	if (isTransformation)
	{

		MTransformationMatrix transformationMatrix = fnMatrixData.transformation(&status);
		CHECK_MSTATUS_AND_RETURN(status, MMatrix::identity);

		matrix = transformationMatrix.asMatrix();

	}
	else
	{

		matrix = fnMatrixData.matrix(&status);
		CHECK_MSTATUS_AND_RETURN(status, MMatrix::identity);

	}

	return matrix;

};


MTransformationMatrix IKEmulator::getTransformData(const MObject& matrixData)
/**
Returns the transformation matrix from the supplied matrix data object.

@param matrixData: The matrix data object.
@param status: Status code.
@return: The matrix value.
*/
{

	MStatus status;

	MFnMatrixData fnMatrixData(matrixData, &status);
	CHECK_MSTATUS_AND_RETURN(status, MTransformationMatrix::identity);

	bool isTransformation = fnMatrixData.isTransformation(&status);
	CHECK_MSTATUS_AND_RETURN(status, MTransformationMatrix::identity);

	MTransformationMatrix transformationMatrix;

	if (isTransformation)
	{

		transformationMatrix = fnMatrixData.transformation(&status);
		CHECK_MSTATUS_AND_RETURN(status, MTransformationMatrix::identity);

	}
	else
	{

		MMatrix matrix = fnMatrixData.matrix(&status);
		CHECK_MSTATUS_AND_RETURN(status, MTransformationMatrix::identity);

		transformationMatrix = MTransformationMatrix(matrix);

	}

	return transformationMatrix;

};


MVector IKEmulator::getMatrixRow(const MMatrix& matrix, const Axis axis, const bool flip, const bool normalize)
{

	unsigned int row = static_cast<unsigned int>(axis);
	MVector vector = MVector(matrix(row, 0), matrix(row, 1), matrix(row, 2));

	if (flip)
	{

		vector *= -1.0;

	}

	if (normalize)
	{

		vector.normalize();

	}

	return vector;

};


void IKEmulator::displayInfo(const MString& variable, const MMatrix& matrix)
{

	MString debug;
	debug += variable;
	debug += " = ";
	debug += "MMatrix({";
	debug += "{";
	debug += matrix(0, 0);
	debug += ", ";
	debug += matrix(0, 1);
	debug += ", ";
	debug += matrix(0, 2);
	debug += ", ";
	debug += matrix(0, 3);
	debug += "}, ";
	debug += "{";
	debug += matrix(1, 0);
	debug += ", ";
	debug += matrix(1, 1);
	debug += ", ";
	debug += matrix(1, 2);
	debug += ", ";
	debug += matrix(1, 3);
	debug += "}, ";
	debug += "{";
	debug += matrix(2, 0);
	debug += ", ";
	debug += matrix(2, 1);
	debug += ", ";
	debug += matrix(2, 2);
	debug += ", ";
	debug += matrix(2, 3);
	debug += "}, ";
	debug += "{";
	debug += matrix(3, 0);
	debug += ", ";
	debug += matrix(3, 1);
	debug += ", ";
	debug += matrix(3, 2);
	debug += ", ";
	debug += matrix(3, 3);
	debug += "}})";

	MGlobal::displayInfo(debug);

};


void IKEmulator::displayInfo(const MString& variable, const MPoint& point)
{

	MString debug;
	debug += variable;
	debug += " = ";
	debug += "MPoint(";
	debug += point.x;
	debug += ", ";
	debug += point.y;
	debug += ", ";
	debug += point.z;
	debug += ", ";
	debug += point.w;
	debug += ")";

	MGlobal::displayInfo(debug);

};


void IKEmulator::displayInfo(const MString& variable, const MVector& vector)
{

	MString debug;
	debug += variable;
	debug += " = ";
	debug += "MVector(";
	debug += vector.x;
	debug += ", ";
	debug += vector.y;
	debug += ", ";
	debug += vector.z;
	debug += ")";

	MGlobal::displayInfo(debug);

};


void IKEmulator::displayInfo(const MString& variable, const double value)
{

	MString debug;
	debug += variable;
	debug += " = ";
	debug += value;

	MGlobal::displayInfo(debug);

};


void* IKEmulator::creator() 
/**
This function is called by Maya when a new instance is requested.
See pluginMain.cpp for details.

@return: IKEmulator
*/
{

	return new IKEmulator();

};


MStatus IKEmulator::initialize()
/**
This function is called by Maya after a plugin has been loaded.
Use this function to define any static attributes.

@return: MStatus
*/
{
	
	MStatus status;

	// Initialize function sets
	//
	MFnNumericAttribute fnNumericAttr;
	MFnEnumAttribute fnEnumAttr;
	MFnUnitAttribute fnUnitAttr;
	MFnTypedAttribute fnTypedAttr;
	MFnMatrixAttribute fnMatrixAttr;

	// Input attributes:
	// Initialize `envelope` attribute
	//
	IKEmulator::envelope = fnNumericAttr.create("envelope", "env", MFnNumericData::kDouble, 1.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setMin(0.0));
	CHECK_MSTATUS(fnNumericAttr.setMax(1.0));
	CHECK_MSTATUS(fnNumericAttr.setSoftMin(0.0));
	CHECK_MSTATUS(fnNumericAttr.setSoftMax(1.0));
	CHECK_MSTATUS(fnNumericAttr.setKeyable(true));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKEmulator::inputCategory));

	// Initialize `forwardAxis` attribute
	//
	IKEmulator::forwardAxis = fnEnumAttr.create("forwardAxis", "fa", short(0), &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnEnumAttr.addField(MString("x"), 0));
	CHECK_MSTATUS(fnEnumAttr.addField(MString("y"), 1));
	CHECK_MSTATUS(fnEnumAttr.addField(MString("z"), 2));
	CHECK_MSTATUS(fnEnumAttr.addToCategory(IKEmulator::inputCategory));

	// Initialize `forwardAxisFlip` attribute
	//
	IKEmulator::forwardAxisFlip = fnNumericAttr.create("forwardAxisFlip", "faf", MFnNumericData::kBoolean, false, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKEmulator::inputCategory));

	// Initialize `upAxis` attribute
	//
	IKEmulator::upAxis = fnEnumAttr.create("upAxis", "ua", short(1), &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnEnumAttr.addField(MString("x"), 0));
	CHECK_MSTATUS(fnEnumAttr.addField(MString("y"), 1));
	CHECK_MSTATUS(fnEnumAttr.addField(MString("z"), 2));
	CHECK_MSTATUS(fnEnumAttr.addToCategory(IKEmulator::inputCategory));

	// Initialize `upAxisFlip` attribute
	//
	IKEmulator::upAxisFlip = fnNumericAttr.create("upAxisFlip", "uaf", MFnNumericData::kBoolean, false, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKEmulator::inputCategory));

	// Initialize `restMatrix` attribute
	//
	IKEmulator::restMatrix = fnTypedAttr.create("restMatrix", "rm", MFnData::kMatrix, IKEmulator::identityMatrixData, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnTypedAttr.setArray(true));
	CHECK_MSTATUS(fnTypedAttr.addToCategory(IKEmulator::inputCategory));

	// Initialize `parentRestMatrix` attribute
	//
	IKEmulator::parentRestMatrix = fnTypedAttr.create("parentRestMatrix", "prm", MFnData::kMatrix, IKEmulator::identityMatrixData, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnTypedAttr.addToCategory(IKEmulator::inputCategory));

	// Initialize `poleType` attribute
	//
	IKEmulator::poleType = fnEnumAttr.create("poleType", "pt", short(0), & status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnEnumAttr.addField(MString("Align"), 0));
	CHECK_MSTATUS(fnEnumAttr.addField(MString("Look-At"), 1));
	CHECK_MSTATUS(fnEnumAttr.addField(MString("Matrix"), 2));
	CHECK_MSTATUS(fnEnumAttr.addField(MString("Goal"), 3));
	CHECK_MSTATUS(fnEnumAttr.addToCategory(IKEmulator::inputCategory));

	// Initialize `poleVectorX` attribute
	//
	IKEmulator::poleVectorX = fnNumericAttr.create("poleVectorX", "pvx", MFnNumericData::kDouble, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKEmulator::inputCategory));

	// Initialize `poleVectorY` attribute
	//
	IKEmulator::poleVectorY = fnNumericAttr.create("poleVectorY", "pvy", MFnNumericData::kDouble, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKEmulator::inputCategory));

	// Initialize `poleVectorZ` attribute
	//
	IKEmulator::poleVectorZ = fnNumericAttr.create("poleVectorZ", "pvz", MFnNumericData::kDouble, 1.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKEmulator::inputCategory));

	// Initialize `poleVector` attribute
	//
	IKEmulator::poleVector = fnNumericAttr.create("poleVector", "pv", IKEmulator::poleVectorX, IKEmulator::poleVectorY, IKEmulator::poleVectorZ, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKEmulator::inputCategory));

	// Initialize `poleMatrix` attribute
	//
	IKEmulator::poleMatrix = fnTypedAttr.create("poleMatrix", "pm", MFnData::kMatrix, IKEmulator::identityMatrixData, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKEmulator::inputCategory));

	// Initialize `twist` attribute
	//
	IKEmulator::twist = fnUnitAttr.create("twist", "twi", MFnUnitAttribute::kAngle, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.addToCategory(IKEmulator::inputCategory));

	// Initialize `stretch` attribute
	//
	IKEmulator::stretch = fnNumericAttr.create("stretch", "str", MFnNumericData::kDouble, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setMin(0.0));
	CHECK_MSTATUS(fnNumericAttr.setMax(1.0));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKEmulator::inputCategory));

	// Initialize `soften` attribute
	//
	IKEmulator::soften = fnNumericAttr.create("soften", "sft", MFnNumericData::kDouble, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setMin(0.0));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKEmulator::inputCategory));

	// Initialize `pin` attribute
	//
	IKEmulator::pin = fnNumericAttr.create("pin", "pn", MFnNumericData::kDouble, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setMin(0.0));
	CHECK_MSTATUS(fnNumericAttr.setMax(1.0));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKEmulator::inputCategory));

	// Initialize `goal` attribute
	//
	IKEmulator::goal = fnTypedAttr.create("goal", "gl", MFnData::kMatrix, IKEmulator::identityMatrixData, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnTypedAttr.addToCategory(IKEmulator::inputCategory));

	// Initialize `segmentScaleCompensate` attribute
	//
	IKEmulator::segmentScaleCompensate = fnNumericAttr.create("segmentScaleCompensate", "ssc", MFnNumericData::kBoolean, true, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKEmulator::inputCategory));

	// Initialize `parentInverseMatrix` attribute
	//
	IKEmulator::parentInverseMatrix = fnTypedAttr.create("parentInverseMatrix", "pim", MFnData::kMatrix, IKEmulator::identityMatrixData, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnTypedAttr.addToCategory(IKEmulator::inputCategory));

	// Output attributes:
	// Initialize `softOriginX` attribute
	//
	IKEmulator::softOriginX = fnUnitAttr.create("softOriginX", "sox", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(IKEmulator::outputCategory));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(IKEmulator::softputCategory));

	// Initialize `softOriginY` attribute
	//
	IKEmulator::softOriginY = fnUnitAttr.create("softOriginY", "soy", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(IKEmulator::outputCategory));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(IKEmulator::softputCategory));

	// Initialize `softOriginZ` attribute
	//
	IKEmulator::softOriginZ = fnUnitAttr.create("softOriginZ", "soz", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(IKEmulator::outputCategory));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(IKEmulator::softputCategory));

	// Initialize `softOrigin` attribute
	//
	IKEmulator::softOrigin = fnNumericAttr.create("softOrigin", "so", IKEmulator::softOriginX, IKEmulator::softOriginY, IKEmulator::softOriginZ, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setWritable(false));
	CHECK_MSTATUS(fnNumericAttr.setStorable(false));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKEmulator::outputCategory));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKEmulator::softputCategory));

	// Initialize `softGoalX` attribute
	//
	IKEmulator::softGoalX = fnUnitAttr.create("softGoalX", "sgx", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(IKEmulator::outputCategory));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(IKEmulator::softputCategory));

	// Initialize `softGoalY` attribute
	//
	IKEmulator::softGoalY = fnUnitAttr.create("softGoalY", "sgy", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(IKEmulator::outputCategory));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(IKEmulator::softputCategory));

	// Initialize `softGoalZ` attribute
	//
	IKEmulator::softGoalZ = fnUnitAttr.create("softGoalZ", "sgz", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(IKEmulator::outputCategory));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(IKEmulator::softputCategory));

	// Initialize `softGoal` attribute
	//
	IKEmulator::softGoal = fnNumericAttr.create("softGoal", "sg", IKEmulator::softGoalX, IKEmulator::softGoalY, IKEmulator::softGoalZ, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setWritable(false));
	CHECK_MSTATUS(fnNumericAttr.setStorable(false));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKEmulator::outputCategory));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKEmulator::softputCategory));

	// Initialize `softMatrix` attribute
	//
	IKEmulator::softMatrix = fnTypedAttr.create("softMatrix", "sm", MFnData::kMatrix, IKEmulator::identityMatrixData, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnTypedAttr.setWritable(false));
	CHECK_MSTATUS(fnTypedAttr.setStorable(false));
	CHECK_MSTATUS(fnTypedAttr.addToCategory(IKEmulator::outputCategory));
	CHECK_MSTATUS(fnTypedAttr.addToCategory(IKEmulator::softputCategory));

	// Initialize `softVectorX` attribute
	//
	IKEmulator::softVectorX = fnNumericAttr.create("softVectorX", "svx", MFnNumericData::kDouble, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setWritable(false));
	CHECK_MSTATUS(fnNumericAttr.setStorable(false));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKEmulator::outputCategory));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKEmulator::softputCategory));

	// Initialize `softVectorY` attribute
	//
	IKEmulator::softVectorY = fnNumericAttr.create("softVectorY", "svy", MFnNumericData::kDouble, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setWritable(false));
	CHECK_MSTATUS(fnNumericAttr.setStorable(false));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKEmulator::outputCategory));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKEmulator::softputCategory));

	// Initialize `softVectorZ` attribute
	//
	IKEmulator::softVectorZ = fnNumericAttr.create("softVectorZ", "svz", MFnNumericData::kDouble, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setWritable(false));
	CHECK_MSTATUS(fnNumericAttr.setStorable(false));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKEmulator::outputCategory));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKEmulator::softputCategory));

	// Initialize `softVector` attribute
	//
	IKEmulator::softVector = fnNumericAttr.create("softVector", "sv", IKEmulator::softVectorX, IKEmulator::softVectorY, IKEmulator::softVectorZ, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setWritable(false));
	CHECK_MSTATUS(fnNumericAttr.setStorable(false));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKEmulator::outputCategory));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKEmulator::softputCategory));

	// Initialize `softScale` attribute
	//
	IKEmulator::softDistance = fnNumericAttr.create("softDistance", "sd", MFnNumericData::kDouble, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setWritable(false));
	CHECK_MSTATUS(fnNumericAttr.setStorable(false));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKEmulator::outputCategory));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKEmulator::softputCategory));

	// Initialize `softScale` attribute
	//
	IKEmulator::softScale = fnNumericAttr.create("softScale", "ss", MFnNumericData::kDouble, 1.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setWritable(false));
	CHECK_MSTATUS(fnNumericAttr.setStorable(false));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKEmulator::outputCategory));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKEmulator::softputCategory));

	// Initialize `outMatrix` attribute
	//
	IKEmulator::outMatrix = fnTypedAttr.create("outMatrix", "om", MFnData::kMatrix, IKEmulator::identityMatrixData, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnTypedAttr.setWritable(false));
	CHECK_MSTATUS(fnTypedAttr.setStorable(false));
	CHECK_MSTATUS(fnTypedAttr.setArray(true));
	CHECK_MSTATUS(fnTypedAttr.setUsesArrayDataBuilder(true));
	CHECK_MSTATUS(fnTypedAttr.addToCategory(IKEmulator::outputCategory));

	// Initialize `outWorldMatrix` attribute
	//
	IKEmulator::outWorldMatrix = fnTypedAttr.create("outWorldMatrix", "owm", MFnData::kMatrix, IKEmulator::identityMatrixData, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnTypedAttr.setWritable(false));
	CHECK_MSTATUS(fnTypedAttr.setStorable(false));
	CHECK_MSTATUS(fnTypedAttr.setArray(true));
	CHECK_MSTATUS(fnTypedAttr.setUsesArrayDataBuilder(true));
	CHECK_MSTATUS(fnTypedAttr.addToCategory(IKEmulator::outputCategory));

	// Add attributes to node
	//
	CHECK_MSTATUS(IKEmulator::addAttribute(IKEmulator::envelope));
	CHECK_MSTATUS(IKEmulator::addAttribute(IKEmulator::forwardAxis));
	CHECK_MSTATUS(IKEmulator::addAttribute(IKEmulator::forwardAxisFlip));
	CHECK_MSTATUS(IKEmulator::addAttribute(IKEmulator::upAxis));
	CHECK_MSTATUS(IKEmulator::addAttribute(IKEmulator::upAxisFlip));
	CHECK_MSTATUS(IKEmulator::addAttribute(IKEmulator::restMatrix));
	CHECK_MSTATUS(IKEmulator::addAttribute(IKEmulator::parentRestMatrix));
	CHECK_MSTATUS(IKEmulator::addAttribute(IKEmulator::poleType));
	CHECK_MSTATUS(IKEmulator::addAttribute(IKEmulator::poleVector));
	CHECK_MSTATUS(IKEmulator::addAttribute(IKEmulator::poleMatrix));
	CHECK_MSTATUS(IKEmulator::addAttribute(IKEmulator::twist));
	CHECK_MSTATUS(IKEmulator::addAttribute(IKEmulator::stretch));
	CHECK_MSTATUS(IKEmulator::addAttribute(IKEmulator::soften));
	CHECK_MSTATUS(IKEmulator::addAttribute(IKEmulator::pin));
	CHECK_MSTATUS(IKEmulator::addAttribute(IKEmulator::goal));
	CHECK_MSTATUS(IKEmulator::addAttribute(IKEmulator::segmentScaleCompensate));
	CHECK_MSTATUS(IKEmulator::addAttribute(IKEmulator::parentInverseMatrix));

	CHECK_MSTATUS(IKEmulator::addAttribute(IKEmulator::softOrigin));
	CHECK_MSTATUS(IKEmulator::addAttribute(IKEmulator::softGoal));
	CHECK_MSTATUS(IKEmulator::addAttribute(IKEmulator::softMatrix));
	CHECK_MSTATUS(IKEmulator::addAttribute(IKEmulator::softVector));
	CHECK_MSTATUS(IKEmulator::addAttribute(IKEmulator::softDistance));
	CHECK_MSTATUS(IKEmulator::addAttribute(IKEmulator::softScale));
	CHECK_MSTATUS(IKEmulator::addAttribute(IKEmulator::outMatrix));
	CHECK_MSTATUS(IKEmulator::addAttribute(IKEmulator::outWorldMatrix));

	// Define attribute relationships
	//
	MObject softAttributes[6] = { IKEmulator::softOrigin, IKEmulator::softGoal, IKEmulator::softMatrix, IKEmulator::softVector, IKEmulator::softDistance, IKEmulator::softScale };

	for (MObject attribute : softAttributes)
	{

		CHECK_MSTATUS(IKEmulator::attributeAffects(IKEmulator::restMatrix, attribute));
		CHECK_MSTATUS(IKEmulator::attributeAffects(IKEmulator::parentRestMatrix, attribute));
		CHECK_MSTATUS(IKEmulator::attributeAffects(IKEmulator::stretch, attribute));
		CHECK_MSTATUS(IKEmulator::attributeAffects(IKEmulator::soften, attribute));
		CHECK_MSTATUS(IKEmulator::attributeAffects(IKEmulator::goal, attribute));
		CHECK_MSTATUS(IKEmulator::attributeAffects(IKEmulator::segmentScaleCompensate, attribute));

	}

	MObject solverAttributes[2] = { IKEmulator::outMatrix, IKEmulator::outWorldMatrix };

	for (MObject attribute : solverAttributes)
	{

		CHECK_MSTATUS(IKEmulator::attributeAffects(IKEmulator::envelope, attribute));
		CHECK_MSTATUS(IKEmulator::attributeAffects(IKEmulator::forwardAxis, attribute));
		CHECK_MSTATUS(IKEmulator::attributeAffects(IKEmulator::forwardAxisFlip, attribute));
		CHECK_MSTATUS(IKEmulator::attributeAffects(IKEmulator::upAxis, attribute));
		CHECK_MSTATUS(IKEmulator::attributeAffects(IKEmulator::upAxisFlip, attribute));
		CHECK_MSTATUS(IKEmulator::attributeAffects(IKEmulator::restMatrix, attribute));
		CHECK_MSTATUS(IKEmulator::attributeAffects(IKEmulator::parentRestMatrix, attribute));
		CHECK_MSTATUS(IKEmulator::attributeAffects(IKEmulator::poleType, attribute));
		CHECK_MSTATUS(IKEmulator::attributeAffects(IKEmulator::poleVector, attribute));
		CHECK_MSTATUS(IKEmulator::attributeAffects(IKEmulator::poleMatrix, attribute));
		CHECK_MSTATUS(IKEmulator::attributeAffects(IKEmulator::twist, attribute));
		CHECK_MSTATUS(IKEmulator::attributeAffects(IKEmulator::stretch, attribute));
		CHECK_MSTATUS(IKEmulator::attributeAffects(IKEmulator::soften, attribute));
		CHECK_MSTATUS(IKEmulator::attributeAffects(IKEmulator::pin, attribute));
		CHECK_MSTATUS(IKEmulator::attributeAffects(IKEmulator::goal, attribute));
		CHECK_MSTATUS(IKEmulator::attributeAffects(IKEmulator::segmentScaleCompensate, attribute));
		CHECK_MSTATUS(IKEmulator::attributeAffects(IKEmulator::parentInverseMatrix, attribute));

	}

	return status;

};