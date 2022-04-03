#define MNoVersionString
#define MNoPluginEntry

#include "TangoNode.h"

MTypeId TangoNode::id(0x0);
MObject TangoNode::time;
MObject TangoNode::outputGeometry;

MStatus returnStatus;

#define McheckErr(stat,msg)			\
	if ( MS::kSuccess != stat ) {	\
		cerr << msg;				\
		return MS::kFailure;		\
	}

#define MAKE_INPUT(attr) \
CHECK_MSTATUS(attr.setKeyable(true)); \
CHECK_MSTATUS(attr.setStorable(true)); \
CHECK_MSTATUS(attr.setReadable(true)); \
CHECK_MSTATUS(attr.setWritable(true));

#define MAKE_OUTPUT(attr) \
CHECK_MSTATUS(attr.setKeyable(false)); \
CHECK_MSTATUS(attr.setStorable(false)); \
CHECK_MSTATUS(attr.setReadable(true)); \
CHECK_MSTATUS(attr.setWritable(false));


MStatus TangoNode::initialize()
{
	MFnUnitAttribute inputTimeAttr;
	MFnTypedAttribute outputGeomAttr;

	MStatus returnStatus;

	TangoNode::time = inputTimeAttr.create("time", "tm", MFnUnitAttribute::kTime, 0.0, &returnStatus);
	McheckErr(returnStatus, "ERROR creating TangoNode time attribute\n");
	TangoNode::outputGeometry = outputGeomAttr.create("outputMesh", "out", MFnData::kMesh, &returnStatus);
	McheckErr(returnStatus, "ERROR creating TangoNode output attribute\n");

	returnStatus = addAttribute(TangoNode::time);
	McheckErr(returnStatus, "ERROR adding time attribute\n");
	returnStatus = addAttribute(TangoNode::outputGeometry);
	McheckErr(returnStatus, "ERROR adding outputMesh attribute\n");

	returnStatus = attributeAffects(TangoNode::time, TangoNode::outputGeometry);
	McheckErr(returnStatus, "ERROR in time attributeAffects outputMesh\n");

	return MS::kSuccess;
}

void* TangoNode::creator()
{
	return new TangoNode;
}

MStatus TangoNode::compute(const MPlug& plug, MDataBlock& data)
{
	MStatus returnStatus;

	if (plug == outputGeometry) {
		MDataHandle time_handle = data.inputValue(time, &returnStatus);
		MTime time_value = time_handle.asTime();

		MDataHandle output_handle = data.outputValue(outputGeometry, &returnStatus);

		MFnMeshData mesh_data_creator;
		MObject output_object = mesh_data_creator.create(&returnStatus);

		output_handle.set(output_object);
		data.setClean(plug);
	}
	else {
		return MS::kUnknownParameter;
	}

	return MS::kSuccess;
}