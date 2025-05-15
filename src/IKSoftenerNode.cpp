//
// File: IKSoftenerNode.cpp
//
// Dependency Graph Node: ikSoftener
//
// Author: Benjamin H. Singleton
//

#include "IKSoftenerNode.h"
#include "IKEmulatorNode.h"

MObject		IKSoftener::envelope;
MObject		IKSoftener::startMatrix;
MObject		IKSoftener::endMatrix;
MObject		IKSoftener::radius;
MObject		IKSoftener::chainLength;
MObject		IKSoftener::chainScaleCompensate;
MObject		IKSoftener::parentInverseMatrix;

MObject		IKSoftener::outPosition;
MObject		IKSoftener::outPositionX;
MObject		IKSoftener::outPositionY;
MObject		IKSoftener::outPositionZ;
MObject		IKSoftener::outWorldPosition;
MObject		IKSoftener::outWorldPositionX;
MObject		IKSoftener::outWorldPositionY;
MObject		IKSoftener::outWorldPositionZ;
MObject		IKSoftener::outVector;
MObject		IKSoftener::outVectorX;
MObject		IKSoftener::outVectorY;
MObject		IKSoftener::outVectorZ;
MObject		IKSoftener::outWorldVector;
MObject		IKSoftener::outWorldVectorX;
MObject		IKSoftener::outWorldVectorY;
MObject		IKSoftener::outWorldVectorZ;
MObject		IKSoftener::outMatrix;
MObject		IKSoftener::outWorldMatrix;
MObject		IKSoftener::softScale;
MObject		IKSoftener::softDistance;

MString		IKSoftener::inputCategory("Input");
MString		IKSoftener::outputCategory("Output");

MTypeId		IKSoftener::id(0x0013b1c4);


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


IKSoftener::IKSoftener() {}
IKSoftener::~IKSoftener() {}


MStatus IKSoftener::compute(const MPlug& plug, MDataBlock& data)
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

	// Check requested attribute
	//
	MObject attribute = plug.attribute(&status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	MFnAttribute fnAttribute(attribute, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	bool isOutput = fnAttribute.hasCategory(IKSoftener::outputCategory);

	if (isOutput)
	{
		
		// Get input data handles
		//
		MDataHandle envelopeHandle = data.inputValue(IKSoftener::envelope, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle startMatrixHandle = data.inputValue(IKSoftener::startMatrix, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle endMatrixHandle = data.inputValue(IKSoftener::endMatrix, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle radiusHandle = data.inputValue(IKSoftener::radius, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle chainLengthHandle = data.inputValue(IKSoftener::chainLength, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle chainScaleCompensateHandle = data.inputValue(IKSoftener::chainScaleCompensate, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle parentInverseMatrixHandle = data.inputValue(IKSoftener::parentInverseMatrix, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		
		// Get values from handles
		//
		double envelope = envelopeHandle.asDouble();

		MMatrix startMatrix = startMatrixHandle.asMatrix();
		MMatrix endMatrix = endMatrixHandle.asMatrix();
		MMatrix parentInverseMatrix = parentInverseMatrixHandle.asMatrix();

		double radius = radiusHandle.asDouble();
		double chainLength = chainLengthHandle.asDouble();
		bool chainScaleCompensate = chainScaleCompensateHandle.asBool();

		if (chainScaleCompensate)
		{

			MVector scale = IKEmulator::matrixToScale(startMatrix);
			chainLength *= scale.x;  // Most IK chains rely on x-forward so I'm leaving it as this for now.

		}

		// Solve soft IK
		//
		MPoint origin = IKEmulator::matrixToPosition(startMatrix);
		MPoint goal = IKEmulator::matrixToPosition(endMatrix);

		MPoint softGoal;
		MVector softVector;
		double softDistance, softScale;

		IKSoftener::solve(origin, goal, chainLength, radius, softGoal, softVector, softDistance, softScale);

		// Convert solution to local space
		//
		MPoint outWorldPosition = lerp(goal, softGoal, envelope);  // Lerp the two points using envelope
		MPoint outPosition = outWorldPosition * parentInverseMatrix;

		MVector outVector = softVector;
		MVector outWorldVector = outVector * parentInverseMatrix;

		MMatrix outMatrix = IKEmulator::createPositionMatrix(outWorldPosition);
		MMatrix outWorldMatrix = IKEmulator::createPositionMatrix(outPosition);

		// Get output data handles
		//
		MDataHandle outPositionHandle = data.outputValue(IKSoftener::outPosition, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle outPositionXHandle = outPositionHandle.child(IKSoftener::outPositionX);
		MDataHandle outPositionYHandle = outPositionHandle.child(IKSoftener::outPositionY);
		MDataHandle outPositionZHandle = outPositionHandle.child(IKSoftener::outPositionZ);

		MDataHandle outWorldPositionHandle = data.outputValue(IKSoftener::outWorldPosition, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle outWorldPositionXHandle = outWorldPositionHandle.child(IKSoftener::outWorldPositionX);
		MDataHandle outWorldPositionYHandle = outWorldPositionHandle.child(IKSoftener::outWorldPositionY);
		MDataHandle outWorldPositionZHandle = outWorldPositionHandle.child(IKSoftener::outWorldPositionZ);

		MDataHandle outVectorHandle = data.outputValue(IKSoftener::outVector, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle outVectorXHandle = outVectorHandle.child(IKSoftener::outVectorX);
		MDataHandle outVectorYHandle = outVectorHandle.child(IKSoftener::outVectorY);
		MDataHandle outVectorZHandle = outVectorHandle.child(IKSoftener::outVectorZ);

		MDataHandle outWorldVectorHandle = data.outputValue(IKSoftener::outWorldVector, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle outWorldVectorXHandle = outWorldVectorHandle.child(IKSoftener::outWorldVectorX);
		MDataHandle outWorldVectorYHandle = outWorldVectorHandle.child(IKSoftener::outWorldVectorY);
		MDataHandle outWorldVectorZHandle = outWorldVectorHandle.child(IKSoftener::outWorldVectorZ);

		MDataHandle softScaleHandle = data.outputValue(IKSoftener::softScale, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle softDistanceHandle = data.outputValue(IKSoftener::softDistance, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle outMatrixHandle = data.outputValue(IKSoftener::outMatrix, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle outWorldMatrixHandle = data.outputValue(IKSoftener::outWorldMatrix, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		// Set output handle values
		//
		MDistance::Unit distanceUnit = MDistance::uiUnit();

		outPositionXHandle.setMDistance(MDistance(outPosition.x, distanceUnit));
		outPositionYHandle.setMDistance(MDistance(outPosition.y, distanceUnit));
		outPositionZHandle.setMDistance(MDistance(outPosition.z, distanceUnit));
		outPositionHandle.setClean();

		outWorldPositionXHandle.setMDistance(MDistance(outWorldPosition.x, distanceUnit));
		outWorldPositionYHandle.setMDistance(MDistance(outWorldPosition.y, distanceUnit));
		outWorldPositionZHandle.setMDistance(MDistance(outWorldPosition.z, distanceUnit));
		outWorldPositionHandle.setClean();

		outVectorXHandle.setDouble(outVector.x);
		outVectorYHandle.setDouble(outVector.y);
		outVectorZHandle.setDouble(outVector.z);
		outVectorHandle.setClean();

		outWorldVectorXHandle.setDouble(outWorldVector.x);
		outWorldVectorYHandle.setDouble(outWorldVector.y);
		outWorldVectorZHandle.setDouble(outWorldVector.z);
		outWorldVectorHandle.setClean();

		outMatrixHandle.setMMatrix(outMatrix);
		outMatrixHandle.setClean();

		outWorldMatrixHandle.setMMatrix(outWorldMatrix);
		outWorldMatrixHandle.setClean();

		softDistanceHandle.setDouble(softDistance);
		softDistanceHandle.setClean();

		softScaleHandle.setDouble(softScale);
		softScaleHandle.setClean();

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


MPxNode::SchedulingType IKSoftener::schedulingType() const
/**
When overridden this method controls the degree of parallelism supported by the node during threaded evaluation.

@return: The scheduling type to be used for this node.
*/
{

	return MPxNode::SchedulingType::kParallel;

};


void IKSoftener::solve(const MPoint& origin, const MPoint& goal, const double chainLength, const double radius, MPoint& softGoal, MVector& softVector, double& softDistance, double& softScale)
/**
Solves for the soft IK position based on the supplied parameters.
See the following for details: http://www.softimageblog.com/archives/108

@param origin: The start position of the IK chain.
@param chainLength: The total length of the IK chain.
@param goal: The end goal of the IK chain.
@param radius: The softening radius relative from the goal.
@return: Void.
*/
{

	// Calculate soft distance and scale
	// 
	MVector aimVector = goal - origin;
	double distance = aimVector.length();

	double absoluteRadius = std::fabs(radius);
	double clampedRadius = (absoluteRadius > 0.0) ? radius : DBL_MIN;
	const double a = (chainLength - clampedRadius);

	softDistance = 0.0;

	if (0.0 <= distance && distance < a)
	{

		softDistance = distance;

	}
	else if (a <= distance)
	{

		softDistance = ((clampedRadius * (1.0 - std::pow(M_E, (-(distance - a) / clampedRadius)))) + a);

	}
	else;

	softScale = (softDistance > 0.0) ? (distance / softDistance) : 0.0;

	// Calculate soft goal
	//
	softVector = aimVector.normal();
	softGoal = origin + (softVector * softDistance);

};


void* IKSoftener::creator() 
/**
This function is called by Maya when a new instance is requested.
See pluginMain.cpp for details.

@return: IKSoftener
*/
{

	return new IKSoftener();

};


MStatus IKSoftener::initialize()
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
	MFnUnitAttribute fnUnitAttr;
	MFnMatrixAttribute fnMatrixAttr;
	MFnCompoundAttribute fnCompoundAttr;

	// Input attributes:
	// Initialize `envelope` attribute
	//
	IKSoftener::envelope = fnNumericAttr.create("envelope", "env", MFnNumericData::kDouble, 1.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setMin(0.0));
	CHECK_MSTATUS(fnNumericAttr.setMax(1.0));
	CHECK_MSTATUS(fnNumericAttr.setSoftMin(0.0));
	CHECK_MSTATUS(fnNumericAttr.setSoftMax(1.0));
	CHECK_MSTATUS(fnNumericAttr.setKeyable(true));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKSoftener::inputCategory));

	// Initialize `startMatrix` attribute
	//
	IKSoftener::startMatrix = fnMatrixAttr.create("startMatrix", "sm", MFnMatrixAttribute::kDouble, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnMatrixAttr.addToCategory(IKSoftener::inputCategory));

	// Initialize `endMatrix` attribute
	//
	IKSoftener::endMatrix = fnMatrixAttr.create("endMatrix", "em", MFnMatrixAttribute::kDouble, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnMatrixAttr.addToCategory(IKSoftener::inputCategory));

	// Initialize `radius` attribute
	//
	IKSoftener::radius = fnNumericAttr.create("radius", "rad", MFnNumericData::kDouble, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setMin(0.0));
	CHECK_MSTATUS(fnNumericAttr.setKeyable(true));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKSoftener::inputCategory));

	// Initialize `chainLength` attribute
	//
	IKSoftener::chainLength = fnNumericAttr.create("chainLength", "cl", MFnNumericData::kDouble, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setMin(0.0));
	CHECK_MSTATUS(fnNumericAttr.setKeyable(true));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKSoftener::inputCategory));

	// Initialize `chainScaleCompensate` attribute
	//
	IKSoftener::chainScaleCompensate = fnNumericAttr.create("chainScaleCompensate", "csc", MFnNumericData::kBoolean, false, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKSoftener::inputCategory));

	// Initialize `parentInverseMatrix` attribute
	//
	IKSoftener::parentInverseMatrix = fnMatrixAttr.create("parentInverseMatrix", "pim", MFnMatrixAttribute::kDouble, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnMatrixAttr.addToCategory(IKSoftener::inputCategory));

	// Output attributes:
	// Initialize `outPositionX` attribute
	//
	IKSoftener::outPositionX = fnUnitAttr.create("outPositionX", "opx", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(IKSoftener::outputCategory));

	// Initialize `outPositionY` attribute
	//
	IKSoftener::outPositionY = fnUnitAttr.create("outPositionY", "opy", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(IKSoftener::outputCategory));

	// Initialize `outPositionZ` attribute
	//
	IKSoftener::outPositionZ = fnUnitAttr.create("outPositionZ", "opz", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(IKSoftener::outputCategory));

	// Initialize `outPosition` attribute
	//
	IKSoftener::outPosition = fnNumericAttr.create("outPosition", "op", IKSoftener::outPositionX, IKSoftener::outPositionY, IKSoftener::outPositionZ, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setWritable(false));
	CHECK_MSTATUS(fnNumericAttr.setStorable(false));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKSoftener::outputCategory));

	// Initialize `outWorldPositionX` attribute
	//
	IKSoftener::outWorldPositionX = fnUnitAttr.create("outWorldPositionX", "owpx", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(IKSoftener::outputCategory));

	// Initialize `outWorldPositionY` attribute
	//
	IKSoftener::outWorldPositionY = fnUnitAttr.create("outWorldPositionY", "owpy", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(IKSoftener::outputCategory));

	// Initialize `outWorldPositionZ` attribute
	//
	IKSoftener::outWorldPositionZ = fnUnitAttr.create("outWorldPositionZ", "owpz", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(IKSoftener::outputCategory));

	// Initialize `outWorldPosition` attribute
	//
	IKSoftener::outWorldPosition = fnNumericAttr.create("outWorldPosition", "owp", IKSoftener::outWorldPositionX, IKSoftener::outWorldPositionY, IKSoftener::outWorldPositionZ, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setWritable(false));
	CHECK_MSTATUS(fnNumericAttr.setStorable(false));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKSoftener::outputCategory));

	// Initialize `outVectorX` attribute
	//
	IKSoftener::outVectorX = fnNumericAttr.create("outVectorX", "ovx", MFnNumericData::kDouble, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setWritable(false));
	CHECK_MSTATUS(fnNumericAttr.setStorable(false));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKSoftener::outputCategory));

	// Initialize `outVectorY` attribute
	//
	IKSoftener::outVectorY = fnNumericAttr.create("outVectorY", "ovy", MFnNumericData::kDouble, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setWritable(false));
	CHECK_MSTATUS(fnNumericAttr.setStorable(false));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKSoftener::outputCategory));

	// Initialize `outVectorZ` attribute
	//
	IKSoftener::outVectorZ = fnNumericAttr.create("outVectorZ", "ovz", MFnNumericData::kDouble, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setWritable(false));
	CHECK_MSTATUS(fnNumericAttr.setStorable(false));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKSoftener::outputCategory));

	// Initialize `outVector` attribute
	//
	IKSoftener::outVector = fnNumericAttr.create("outVector", "ov", IKSoftener::outVectorX, IKSoftener::outVectorY, IKSoftener::outVectorZ, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setWritable(false));
	CHECK_MSTATUS(fnNumericAttr.setStorable(false));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKSoftener::outputCategory));

	// Initialize `outWorldVectorX` attribute
	//
	IKSoftener::outWorldVectorX = fnNumericAttr.create("outWorldVectorX", "owvx", MFnNumericData::kDouble, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setWritable(false));
	CHECK_MSTATUS(fnNumericAttr.setStorable(false));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKSoftener::outputCategory));

	// Initialize `outWorldVectorY` attribute
	//
	IKSoftener::outWorldVectorY = fnNumericAttr.create("outWorldVectorY", "owvy", MFnNumericData::kDouble, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setWritable(false));
	CHECK_MSTATUS(fnNumericAttr.setStorable(false));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKSoftener::outputCategory));

	// Initialize `outWorldVectorZ` attribute
	//
	IKSoftener::outWorldVectorZ = fnNumericAttr.create("outWorldVectorZ", "owvz", MFnNumericData::kDouble, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setWritable(false));
	CHECK_MSTATUS(fnNumericAttr.setStorable(false));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKSoftener::outputCategory));

	// Initialize `outWorldVector` attribute
	//
	IKSoftener::outWorldVector = fnNumericAttr.create("outWorldVector", "owv", IKSoftener::outWorldVectorX, IKSoftener::outWorldVectorY, IKSoftener::outWorldVectorZ, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setWritable(false));
	CHECK_MSTATUS(fnNumericAttr.setStorable(false));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKSoftener::outputCategory));

	// Initialize `outMatrix` attribute
	//
	IKSoftener::outMatrix = fnMatrixAttr.create("outMatrix", "om", MFnMatrixAttribute::kDouble, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnMatrixAttr.setWritable(false));
	CHECK_MSTATUS(fnMatrixAttr.setStorable(false));
	CHECK_MSTATUS(fnMatrixAttr.addToCategory(IKSoftener::outputCategory));

	// Initialize `outWorldMatrix` attribute
	//
	IKSoftener::outWorldMatrix = fnMatrixAttr.create("outWorldMatrix", "owm", MFnMatrixAttribute::kDouble, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnMatrixAttr.setWritable(false));
	CHECK_MSTATUS(fnMatrixAttr.setStorable(false));
	CHECK_MSTATUS(fnMatrixAttr.addToCategory(IKSoftener::outputCategory));

	// Initialize `softScale` attribute
	//
	IKSoftener::softScale = fnNumericAttr.create("softScale", "ss", MFnNumericData::kDouble, 1.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setWritable(false));
	CHECK_MSTATUS(fnNumericAttr.setStorable(false));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKSoftener::outputCategory));

	// Initialize `distance` attribute
	//
	IKSoftener::softDistance = fnNumericAttr.create("softDistance", "sd", MFnNumericData::kDouble, 1.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setWritable(false));
	CHECK_MSTATUS(fnNumericAttr.setStorable(false));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKSoftener::outputCategory));

	// Add attributes to node
	//
	CHECK_MSTATUS(IKSoftener::addAttribute(IKSoftener::envelope));
	CHECK_MSTATUS(IKSoftener::addAttribute(IKSoftener::startMatrix));
	CHECK_MSTATUS(IKSoftener::addAttribute(IKSoftener::endMatrix));
	CHECK_MSTATUS(IKSoftener::addAttribute(IKSoftener::radius));
	CHECK_MSTATUS(IKSoftener::addAttribute(IKSoftener::chainLength));
	CHECK_MSTATUS(IKSoftener::addAttribute(IKSoftener::chainScaleCompensate));
	CHECK_MSTATUS(IKSoftener::addAttribute(IKSoftener::parentInverseMatrix));

	CHECK_MSTATUS(IKSoftener::addAttribute(IKSoftener::outPosition));
	CHECK_MSTATUS(IKSoftener::addAttribute(IKSoftener::outWorldPosition));
	CHECK_MSTATUS(IKSoftener::addAttribute(IKSoftener::outVector));
	CHECK_MSTATUS(IKSoftener::addAttribute(IKSoftener::outWorldVector));
	CHECK_MSTATUS(IKSoftener::addAttribute(IKSoftener::outMatrix));
	CHECK_MSTATUS(IKSoftener::addAttribute(IKSoftener::outWorldMatrix));
	CHECK_MSTATUS(IKSoftener::addAttribute(IKSoftener::softScale));
	CHECK_MSTATUS(IKSoftener::addAttribute(IKSoftener::softDistance));

	// Define attribute relationships
	//
	MObject attributes[8] = { IKSoftener::outPosition, IKSoftener::outWorldPosition, IKSoftener::outVector, IKSoftener::outWorldVector, IKSoftener::outMatrix, IKSoftener::outWorldMatrix, IKSoftener::softDistance, IKSoftener::softScale };
	
	for (MObject attribute : attributes)
	{

		CHECK_MSTATUS(IKSoftener::attributeAffects(IKSoftener::envelope, attribute));
		CHECK_MSTATUS(IKSoftener::attributeAffects(IKSoftener::startMatrix, attribute));
		CHECK_MSTATUS(IKSoftener::attributeAffects(IKSoftener::endMatrix, attribute));
		CHECK_MSTATUS(IKSoftener::attributeAffects(IKSoftener::radius, attribute));
		CHECK_MSTATUS(IKSoftener::attributeAffects(IKSoftener::chainLength, attribute));
		CHECK_MSTATUS(IKSoftener::attributeAffects(IKSoftener::chainScaleCompensate, attribute));
		CHECK_MSTATUS(IKSoftener::attributeAffects(IKSoftener::parentInverseMatrix, attribute));

	}

	return status;

};