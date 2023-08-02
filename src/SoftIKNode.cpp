//
// File: SoftIKNode.cpp
//
// Dependency Graph Node: softIK
//
// Author: Benjamin H. Singleton
//

#include "SoftIKNode.h"

MObject		SoftIK::envelope;
MObject		SoftIK::startMatrix;
MObject		SoftIK::endMatrix;
MObject		SoftIK::softDistance;
MObject		SoftIK::chainLength;
MObject		SoftIK::parentInverseMatrix;

MObject		SoftIK::outputTranslate;
MObject		SoftIK::outputTranslateX;
MObject		SoftIK::outputTranslateY;
MObject		SoftIK::outputTranslateZ;
MObject		SoftIK::softScale;
MObject		SoftIK::distance;

MString		SoftIK::inputCategory("Input");
MString		SoftIK::outputCategory("Output");

MTypeId		SoftIK::id(0x0013b1c4);


SoftIK::SoftIK() {}
SoftIK::~SoftIK() {}


MStatus SoftIK::compute(const MPlug& plug, MDataBlock& data) 
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

	bool isOutput = fnAttribute.hasCategory(SoftIK::outputCategory);

	if (isOutput)
	{
		
		// Get input data handles
		//
		MDataHandle envelopeHandle = data.inputValue(SoftIK::envelope, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle startMatrixHandle = data.inputValue(SoftIK::startMatrix, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle endMatrixHandle = data.inputValue(SoftIK::endMatrix, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle softDistanceHandle = data.inputValue(SoftIK::softDistance, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle chainLengthHandle = data.inputValue(SoftIK::chainLength, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle parentInverseMatrixHandle = data.inputValue(SoftIK::parentInverseMatrix, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		// Get values from handles
		//
		double envelope = envelopeHandle.asDouble();

		MMatrix startMatrix = startMatrixHandle.asMatrix();
		MMatrix endMatrix = endMatrixHandle.asMatrix();
		MMatrix parentInverseMatrix = parentInverseMatrixHandle.asMatrix();

		double softDistance = softDistanceHandle.asDouble();
		double chainLength = fabs(chainLengthHandle.asDouble());

		if (softDistance <= 0.0)
		{

			softDistance = DBL_MIN;

		}
		
		// Calculate aim vector
		//
		MPoint startPoint = MPoint(startMatrix[3]);
		MPoint endPoint = MPoint(endMatrix[3]);

		MVector aimVector = MVector(endPoint - startPoint);
		double distance = aimVector.length();

		// Compute soft value
		// http://www.softimageblog.com/archives/108
		//
		double a = (chainLength - softDistance);
		double y = 0.0;

		if (0.0 <= distance && distance < a) 
		{

			y = distance;

		} else if (a <= distance) 
		{
				
			y = ((softDistance * (1.0 - pow(DBL_EPSILON, (-(distance - a) / softDistance)))) + a);

		}
		else;

		double softOffset = distance - y;
		double softScale = distance / y;

		// Calculate point in parent space
		//
		MVector forwardVector = aimVector.normal();
		MPoint softEndPoint = (endPoint + (forwardVector * -softOffset)) * parentInverseMatrix;

		MPoint point = (MPoint::origin * (1.0 - envelope)) + (softEndPoint * envelope); // Lerp the two points using the envelope

		// Get output data handles
		//
		MDataHandle outputTranslateXHandle = data.outputValue(SoftIK::outputTranslateX, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle outputTranslateYHandle = data.outputValue(SoftIK::outputTranslateY, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle outputTranslateZHandle = data.outputValue(SoftIK::outputTranslateZ, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle softScaleHandle = data.outputValue(SoftIK::softScale, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle distanceHandle = data.outputValue(SoftIK::distance, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		// Set output handle values
		//
		outputTranslateXHandle.setDouble(point.x);
		outputTranslateYHandle.setDouble(point.y);
		outputTranslateZHandle.setDouble(point.z);

		outputTranslateXHandle.setClean();
		outputTranslateYHandle.setClean();
		outputTranslateZHandle.setClean();

		softScaleHandle.setDouble(softScale);
		softScaleHandle.setClean();
		
		distanceHandle.setDouble(distance);
		distanceHandle.setClean();

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


void* SoftIK::creator() 
/**
This function is called by Maya when a new instance is requested.
See pluginMain.cpp for details.

@return: SoftIK
*/
{

	return new SoftIK();

};


MStatus SoftIK::initialize()
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
	SoftIK::envelope = fnNumericAttr.create("envelope", "env", MFnNumericData::kDouble, 1.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setMin(0.0));
	CHECK_MSTATUS(fnNumericAttr.setMin(1.0));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(SoftIK::inputCategory));

	// ".startMatrix" attribute
	//
	SoftIK::startMatrix = fnMatrixAttr.create("startMatrix", "sm", MFnMatrixAttribute::kDouble, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnMatrixAttr.addToCategory(SoftIK::inputCategory));

	// ".endMatrix" attribute
	//
	SoftIK::endMatrix = fnMatrixAttr.create("endMatrix", "em", MFnMatrixAttribute::kDouble, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnMatrixAttr.addToCategory(SoftIK::inputCategory));

	// ".softDistance" attribute
	//
	SoftIK::softDistance = fnNumericAttr.create("softDistance", "sd", MFnNumericData::kDouble, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.addToCategory(SoftIK::inputCategory));

	// ".chainLength" attribute
	//
	SoftIK::chainLength = fnNumericAttr.create("chainLength", "cl", MFnNumericData::kDouble, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.addToCategory(SoftIK::inputCategory));

	// ".parentInverseMatrix" attribute
	//
	SoftIK::parentInverseMatrix = fnMatrixAttr.create("parentInverseMatrix", "pim", MFnMatrixAttribute::kDouble, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnMatrixAttr.addToCategory(SoftIK::inputCategory));

	// Output attributes:
	// ".outputTranslateX" attribute
	//
	SoftIK::outputTranslateX = fnUnitAttr.create("outputTranslateX", "otx", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));

	CHECK_MSTATUS(fnUnitAttr.addToCategory(SoftIK::outputCategory));

	// ".outputTranslateY" attribute
	//
	SoftIK::outputTranslateY = fnUnitAttr.create("outputTranslateY", "oty", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));

	CHECK_MSTATUS(fnUnitAttr.addToCategory(SoftIK::outputCategory));

	// ".outputTranslateZ" attribute
	//
	SoftIK::outputTranslateZ = fnUnitAttr.create("outputTranslateZ", "otz", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));

	CHECK_MSTATUS(fnUnitAttr.addToCategory(SoftIK::outputCategory));

	// ".outputTranslate" attribute
	//
	SoftIK::outputTranslate = fnNumericAttr.create("outputTranslate", "ot", SoftIK::outputTranslateX, SoftIK::outputTranslateY, SoftIK::outputTranslateZ, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setWritable(false));
	CHECK_MSTATUS(fnNumericAttr.setStorable(false));

	CHECK_MSTATUS(fnNumericAttr.addToCategory(SoftIK::outputCategory));

	// ".softScale" attribute
	//
	SoftIK::softScale = fnNumericAttr.create("softScale", "ss", MFnNumericData::kDouble, 1.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setWritable(false));
	CHECK_MSTATUS(fnNumericAttr.setStorable(false));

	CHECK_MSTATUS(fnNumericAttr.addToCategory(SoftIK::outputCategory));

	// ".distance" attribute
	//
	SoftIK::distance = fnNumericAttr.create("distance", "d", MFnNumericData::kDouble, 1.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setWritable(false));
	CHECK_MSTATUS(fnNumericAttr.setStorable(false));

	CHECK_MSTATUS(fnNumericAttr.addToCategory(SoftIK::outputCategory));

	// Add attributes to node
	//
	CHECK_MSTATUS(SoftIK::addAttribute(SoftIK::envelope));
	CHECK_MSTATUS(SoftIK::addAttribute(SoftIK::startMatrix));
	CHECK_MSTATUS(SoftIK::addAttribute(SoftIK::endMatrix));
	CHECK_MSTATUS(SoftIK::addAttribute(SoftIK::softDistance));
	CHECK_MSTATUS(SoftIK::addAttribute(SoftIK::chainLength));
	CHECK_MSTATUS(SoftIK::addAttribute(SoftIK::parentInverseMatrix));

	CHECK_MSTATUS(SoftIK::addAttribute(SoftIK::outputTranslate));
	CHECK_MSTATUS(SoftIK::addAttribute(SoftIK::softScale));
	CHECK_MSTATUS(SoftIK::addAttribute(SoftIK::distance));

	// Define attribute relationships
	//
	CHECK_MSTATUS(SoftIK::attributeAffects(SoftIK::envelope, SoftIK::outputTranslate));
	CHECK_MSTATUS(SoftIK::attributeAffects(SoftIK::startMatrix, SoftIK::outputTranslate));
	CHECK_MSTATUS(SoftIK::attributeAffects(SoftIK::endMatrix, SoftIK::outputTranslate));
	CHECK_MSTATUS(SoftIK::attributeAffects(SoftIK::softDistance, SoftIK::outputTranslate));
	CHECK_MSTATUS(SoftIK::attributeAffects(SoftIK::chainLength, SoftIK::outputTranslate));
	CHECK_MSTATUS(SoftIK::attributeAffects(SoftIK::parentInverseMatrix, SoftIK::outputTranslate));

	CHECK_MSTATUS(SoftIK::attributeAffects(SoftIK::startMatrix, SoftIK::softScale));
	CHECK_MSTATUS(SoftIK::attributeAffects(SoftIK::endMatrix, SoftIK::softScale));
	CHECK_MSTATUS(SoftIK::attributeAffects(SoftIK::softDistance, SoftIK::softScale));
	CHECK_MSTATUS(SoftIK::attributeAffects(SoftIK::chainLength, SoftIK::softScale));

	CHECK_MSTATUS(SoftIK::attributeAffects(SoftIK::startMatrix, SoftIK::distance));
	CHECK_MSTATUS(SoftIK::attributeAffects(SoftIK::endMatrix, SoftIK::distance));

	return status;

};