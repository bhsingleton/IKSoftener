//
// File: SoftIKNode.cpp
//
// Dependency Graph Node: softIK
//
// Author: Benjamin H. Singleton
//

#include "IKSoftenerNode.h"

MObject		IKSoftener::envelope;
MObject		IKSoftener::startMatrix;
MObject		IKSoftener::endMatrix;
MObject		IKSoftener::radius;
MObject		IKSoftener::chainLength;
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

		MDataHandle parentInverseMatrixHandle = data.inputValue(IKSoftener::parentInverseMatrix, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		// Get values from handles
		//
		double envelope = envelopeHandle.asDouble();

		MMatrix startMatrix = startMatrixHandle.asMatrix();
		MMatrix endMatrix = endMatrixHandle.asMatrix();
		MMatrix parentInverseMatrix = parentInverseMatrixHandle.asMatrix();

		double radius = std::fabs(radiusHandle.asDouble());
		double chainLength = std::fabs(chainLengthHandle.asDouble());
		
		if (radius == 0.0)
		{

			radius = DBL_MIN;

		}
		
		// Calculate aim vector
		//
		MPoint startPoint = IKSoftener::matrixToPosition(startMatrix);
		MPoint endPoint = IKSoftener::matrixToPosition(endMatrix);

		MVector aimVector = MVector(endPoint - startPoint);
		double distance = aimVector.length();

		//Calculate soft distance
		// http://www.softimageblog.com/archives/108
		//
		const double a = (chainLength - radius);
		const double e = std::exp(1.0);

		double softDistance = 0.0;

		if (0.0 <= distance && distance < a) 
		{

			softDistance = distance;

		} else if (a <= distance) 
		{
				
			softDistance = ((radius * (1.0 - std::pow(e, (-(distance - a) / radius)))) + a);

		}
		else;

		double softScale = (softDistance > 0.0) ? (distance / softDistance) : 0.0;
		
		// Calculate soft end point
		//
		MVector worldVector = aimVector.normal();
		MVector vector = worldVector * parentInverseMatrix;

		MPoint softEndPoint = startPoint + (worldVector * softDistance);
		MPoint worldPosition = (endPoint * (1.0 - envelope)) + (softEndPoint * envelope); // Lerp the two points using the envelope
		MPoint position = worldPosition * parentInverseMatrix;

		MMatrix matrix = IKSoftener::createPositionMatrix(worldPosition);
		MMatrix worldMatrix = IKSoftener::createPositionMatrix(position);

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

		outPositionXHandle.setMDistance(MDistance(position.x, distanceUnit));
		outPositionYHandle.setMDistance(MDistance(position.y, distanceUnit));
		outPositionZHandle.setMDistance(MDistance(position.z, distanceUnit));
		outPositionHandle.setClean();

		outWorldPositionXHandle.setMDistance(MDistance(worldPosition.x, distanceUnit));
		outWorldPositionYHandle.setMDistance(MDistance(worldPosition.y, distanceUnit));
		outWorldPositionZHandle.setMDistance(MDistance(worldPosition.z, distanceUnit));
		outWorldPositionHandle.setClean();

		outVectorXHandle.setDouble(vector.x);
		outVectorYHandle.setDouble(vector.y);
		outVectorZHandle.setDouble(vector.z);
		outVectorHandle.setClean();

		outWorldVectorXHandle.setDouble(worldVector.x);
		outWorldVectorYHandle.setDouble(worldVector.y);
		outWorldVectorZHandle.setDouble(worldVector.z);
		outWorldVectorHandle.setClean();

		outMatrixHandle.setMMatrix(matrix);
		outMatrixHandle.setClean();

		outWorldMatrixHandle.setMMatrix(worldMatrix);
		outWorldMatrixHandle.setClean();

		softDistanceHandle.setDouble(distance);
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


MMatrix IKSoftener::createPositionMatrix(const MPoint& position)
/**
Returns a position matrix from the supplied vector.

@param position: The vector to convert.
@return: The new position matrix.
*/
{

	double rows[4][4] = {
	{ 1.0, 0.0, 0.0, 0.0 },
	{ 0.0, 1.0, 0.0, 0.0 },
	{ 0.0, 0.0, 1.0, 0.0 },
	{ position.x, position.y, position.z, 1.0 },
	};

	return MMatrix(rows);

};


MPoint IKSoftener::matrixToPosition(const MMatrix& matrix)
/**
Extracts the position component from the supplied transform matrix.

@param matrix: The transform matrix to extract from.
@return: The position value.
*/
{

	return MPoint(matrix[3]);

};


void* IKSoftener::creator() 
/**
This function is called by Maya when a new instance is requested.
See pluginMain.cpp for details.

@return: SoftIK
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
	// ".envelope" attribute
	//
	IKSoftener::envelope = fnNumericAttr.create("envelope", "env", MFnNumericData::kDouble, 1.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setMin(0.0));
	CHECK_MSTATUS(fnNumericAttr.setMax(1.0));
	CHECK_MSTATUS(fnNumericAttr.setSoftMin(0.0));
	CHECK_MSTATUS(fnNumericAttr.setSoftMax(1.0));
	CHECK_MSTATUS(fnNumericAttr.setKeyable(true));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKSoftener::inputCategory));

	// ".startMatrix" attribute
	//
	IKSoftener::startMatrix = fnMatrixAttr.create("startMatrix", "sm", MFnMatrixAttribute::kDouble, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnMatrixAttr.addToCategory(IKSoftener::inputCategory));

	// ".endMatrix" attribute
	//
	IKSoftener::endMatrix = fnMatrixAttr.create("endMatrix", "em", MFnMatrixAttribute::kDouble, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnMatrixAttr.addToCategory(IKSoftener::inputCategory));

	// ".radius" attribute
	//
	IKSoftener::radius = fnNumericAttr.create("radius", "rad", MFnNumericData::kDouble, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setMin(0.0));
	CHECK_MSTATUS(fnNumericAttr.setKeyable(true));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKSoftener::inputCategory));

	// ".chainLength" attribute
	//
	IKSoftener::chainLength = fnNumericAttr.create("chainLength", "cl", MFnNumericData::kDouble, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setMin(0.0));
	CHECK_MSTATUS(fnNumericAttr.setKeyable(true));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKSoftener::inputCategory));

	// ".parentInverseMatrix" attribute
	//
	IKSoftener::parentInverseMatrix = fnMatrixAttr.create("parentInverseMatrix", "pim", MFnMatrixAttribute::kDouble, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnMatrixAttr.addToCategory(IKSoftener::inputCategory));

	// Output attributes:
	// ".outPositionX" attribute
	//
	IKSoftener::outPositionX = fnUnitAttr.create("outPositionX", "opx", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(IKSoftener::outputCategory));

	// ".outPositionY" attribute
	//
	IKSoftener::outPositionY = fnUnitAttr.create("outPositionY", "opy", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(IKSoftener::outputCategory));

	// ".outPositionZ" attribute
	//
	IKSoftener::outPositionZ = fnUnitAttr.create("outPositionZ", "opz", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(IKSoftener::outputCategory));

	// ".outPosition" attribute
	//
	IKSoftener::outPosition = fnNumericAttr.create("outPosition", "op", IKSoftener::outPositionX, IKSoftener::outPositionY, IKSoftener::outPositionZ, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setWritable(false));
	CHECK_MSTATUS(fnNumericAttr.setStorable(false));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKSoftener::outputCategory));

	// ".outWorldPositionX" attribute
	//
	IKSoftener::outWorldPositionX = fnUnitAttr.create("outWorldPositionX", "owpx", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(IKSoftener::outputCategory));

	// ".outWorldPositionY" attribute
	//
	IKSoftener::outWorldPositionY = fnUnitAttr.create("outWorldPositionY", "owpy", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(IKSoftener::outputCategory));

	// ".outWorldPositionZ" attribute
	//
	IKSoftener::outWorldPositionZ = fnUnitAttr.create("outWorldPositionZ", "owpz", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(IKSoftener::outputCategory));

	// ".outWorldPosition" attribute
	//
	IKSoftener::outWorldPosition = fnNumericAttr.create("outWorldPosition", "owp", IKSoftener::outWorldPositionX, IKSoftener::outWorldPositionY, IKSoftener::outWorldPositionZ, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setWritable(false));
	CHECK_MSTATUS(fnNumericAttr.setStorable(false));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKSoftener::outputCategory));

	// ".outVectorX" attribute
	//
	IKSoftener::outVectorX = fnNumericAttr.create("outVectorX", "ovx", MFnNumericData::kDouble, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setWritable(false));
	CHECK_MSTATUS(fnNumericAttr.setStorable(false));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKSoftener::outputCategory));

	// ".outVectorY" attribute
	//
	IKSoftener::outVectorY = fnNumericAttr.create("outVectorY", "ovy", MFnNumericData::kDouble, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setWritable(false));
	CHECK_MSTATUS(fnNumericAttr.setStorable(false));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKSoftener::outputCategory));

	// ".outVectorZ" attribute
	//
	IKSoftener::outVectorZ = fnNumericAttr.create("outVectorZ", "ovz", MFnNumericData::kDouble, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setWritable(false));
	CHECK_MSTATUS(fnNumericAttr.setStorable(false));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKSoftener::outputCategory));

	// ".outVector" attribute
	//
	IKSoftener::outVector = fnNumericAttr.create("outVector", "ov", IKSoftener::outVectorX, IKSoftener::outVectorY, IKSoftener::outVectorZ, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setWritable(false));
	CHECK_MSTATUS(fnNumericAttr.setStorable(false));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKSoftener::outputCategory));

	// ".outWorldVectorX" attribute
	//
	IKSoftener::outWorldVectorX = fnNumericAttr.create("outWorldVectorX", "owvx", MFnNumericData::kDouble, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setWritable(false));
	CHECK_MSTATUS(fnNumericAttr.setStorable(false));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKSoftener::outputCategory));

	// ".outWorldVectorY" attribute
	//
	IKSoftener::outWorldVectorY = fnNumericAttr.create("outWorldVectorY", "owvy", MFnNumericData::kDouble, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setWritable(false));
	CHECK_MSTATUS(fnNumericAttr.setStorable(false));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKSoftener::outputCategory));

	// ".outWorldVectorZ" attribute
	//
	IKSoftener::outWorldVectorZ = fnNumericAttr.create("outWorldVectorZ", "owvz", MFnNumericData::kDouble, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setWritable(false));
	CHECK_MSTATUS(fnNumericAttr.setStorable(false));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKSoftener::outputCategory));

	// ".outWorldVector" attribute
	//
	IKSoftener::outWorldVector = fnNumericAttr.create("outWorldVector", "owv", IKSoftener::outWorldVectorX, IKSoftener::outWorldVectorY, IKSoftener::outWorldVectorZ, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setWritable(false));
	CHECK_MSTATUS(fnNumericAttr.setStorable(false));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKSoftener::outputCategory));

	// ".outMatrix" attribute
	//
	IKSoftener::outMatrix = fnMatrixAttr.create("outMatrix", "om", MFnMatrixAttribute::kDouble, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnMatrixAttr.setWritable(false));
	CHECK_MSTATUS(fnMatrixAttr.setStorable(false));
	CHECK_MSTATUS(fnMatrixAttr.addToCategory(IKSoftener::outputCategory));

	// ".outWorldMatrix" attribute
	//
	IKSoftener::outWorldMatrix = fnMatrixAttr.create("outWorldMatrix", "owm", MFnMatrixAttribute::kDouble, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnMatrixAttr.setWritable(false));
	CHECK_MSTATUS(fnMatrixAttr.setStorable(false));
	CHECK_MSTATUS(fnMatrixAttr.addToCategory(IKSoftener::outputCategory));

	// ".softScale" attribute
	//
	IKSoftener::softScale = fnNumericAttr.create("softScale", "ss", MFnNumericData::kDouble, 1.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setWritable(false));
	CHECK_MSTATUS(fnNumericAttr.setStorable(false));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(IKSoftener::outputCategory));

	// ".distance" attribute
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
		CHECK_MSTATUS(IKSoftener::attributeAffects(IKSoftener::parentInverseMatrix, attribute));

	}

	return status;

};