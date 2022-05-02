#define MNoVersionString
#define MNoPluginEntry

#include "TangoNode.h"

MTypeId TangoNode::id(0x0);
MObject TangoNode::translate;
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
	MFnNumericAttribute translateAttr;
	MFnTypedAttribute outputGeomAttr;
	//MFnTypedAttribute translateAttr;

	MStatus returnStatus;

	//TangoNode::translate = translateAttr.create("translate", "tr", MFnNumericData::kFloat, 0.0, &returnStatus);
	//translateAttr.setArray(true);
	//translateAttr.setUsesArrayDataBuilder(true);

	TangoNode::translate = translateAttr.create("translate", "tr", MFnNumericData::k3Float, 0.0, &returnStatus);
	MAKE_INPUT(translateAttr);
	McheckErr(returnStatus, "ERROR creating TangoNode time attribute\n");

	//TangoNode::translate = translateAttr.create("translate", "tr", MFnData::kFloatArray);
	//McheckErr(returnStatus, "ERROR creating TangoNode time attribute\n");
	TangoNode::outputGeometry = outputGeomAttr.create("outputMesh", "out", MFnData::kMesh, &returnStatus);
	McheckErr(returnStatus, "ERROR creating TangoNode output attribute\n");

	returnStatus = addAttribute(TangoNode::translate);
	McheckErr(returnStatus, "ERROR adding translate attribute\n");
	returnStatus = addAttribute(TangoNode::outputGeometry);
	McheckErr(returnStatus, "ERROR adding outputMesh attribute\n");

	returnStatus = attributeAffects(TangoNode::translate, TangoNode::outputGeometry);
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
	
	//MDataHandle translate_handle = data.inputValue(translate, &returnStatus);
	//MFloatVector translate_value = translate_handle.asFloatVector();
	//std::cout << translate_value << std::endl;
	

	MFnMeshData mesh_data_creator;
	MObject output_object = mesh_data_creator.create(&returnStatus);
	MDataHandle output_handle = data.outputValue(outputGeometry, &returnStatus);
	output_handle.set(output_object);
	data.setClean(plug);


	MDataHandle translate_handle = data.inputValue(translate, &returnStatus);
	MFloatVector translate_value = translate_handle.asFloat3();
	std::cout << translate_value << std::endl;
	float temp = translate_value[1];
	std::string translatevalstr = std::to_string(temp);
	MGlobal::displayInfo("Implement Me!" + MString(translatevalstr.data()));
	// Create Target State
	MDagPath nodePath;
	MObject component;
	MSelectionList locatorList;
	MFnDagNode nodeFn;
	MGlobal::getActiveSelectionList(locatorList);
	if (locatorList.length() <= 0)
	{
		std::cout << "No selected locator" << std::endl;
		return MS::kSuccess;
	}
	locatorList.getDagPath(0, nodePath);
	MFnTransform transformFn(nodePath);
	MString name = transformFn.name();
	char* underscore = "_";
	MStringArray splittedString;
	name.split(*underscore, splittedString);
	//MGlobal::displayInfo("SplittedString Length" + splittedString.length());
	//unsigned int sptrLen = splittedString.length();
	if (splittedString.length() == 0) {
		std::cout << "No selected locator" << std::endl;
		return MS::kSuccess;
	}
	MString effectorName = "";
	for (int i = 1; i < splittedString.length() - 1; i++) {
		if (i != 1) {
			effectorName += "_";
		}
		effectorName += splittedString[i];
	}

	MString frameNumberString = splittedString[splittedString.length() - 1];
	MGlobal::displayInfo("Target State set");

	// Set target translation and framenumber
	MVector targetPos = transformFn.getTranslation(MSpace::kWorld);
	State targetState;
	targetState.point = vec3(targetPos[0], targetPos[1], targetPos[2]);
	targetState.frameNumber = frameNumberString.asInt();
	MGlobal::displayInfo("Target translation Frame number set");

	// Get Effector
	MSelectionList effectorList;
	
	MDagPath effectorPath;
	//MString effectorName = splittedString[1];
	MGlobal::displayInfo("effectorName: " + effectorName);
	effectorList.add(effectorName);
	if (effectorList.length() < 0)
	{
		std::cout << "No selected effector" << std::endl;
	}
	effectorList.getDagPath(0, effectorPath);
	MFnTransform effector(effectorPath);
	MPlugArray connections;
	effector.getConnections(connections);
	MString curves[] = { "translateX", "translateY", "translateZ", "rotateX", "rotateY", "rotateZ"};
	MGlobal::displayInfo("effector.Name(): " + effector.name());


	for (auto& curveName : curves)
	{
		// find connection with that curve name
		MPlug animCurvePlug;
		MGlobal::displayInfo("CurveName: " + curveName);
		int len = connections.length();
		MGlobal::displayInfo("Connections Length: " + connections.length());
		MGlobal::displayInfo("\n");
		int i = 0;
		for (; i < connections.length(); i++)
		{
			MPlugArray inputPlugs;
			connections[i].connectedTo(inputPlugs, true, false);
			if (inputPlugs.length() < 1)
			{
				continue;
			}
			animCurvePlug = inputPlugs[0];
			std::string curveNameStr = curveName.asChar();
			std::string plugApiType = animCurvePlug.node().apiTypeStr();
			std::string plugName = animCurvePlug.name().asChar();
			std::string effectorNameStr = effectorName.asChar();
			bool cond1 = (MString(animCurvePlug.node().apiTypeStr()) == MString("kAnimCurveTimeToDistance"));
			bool cond2 = (MString(animCurvePlug.node().apiTypeStr()) == MString("kAnimCurveTimeToAngular"));
			bool cond3 = (animCurvePlug.name() == effectorName + "_" + curveName + ".output");
			if ((cond1 || cond2) && cond3)
			{
				break;
			}
		}
		/*if (i == connections.length())
		{
			std::cout << "Connection with anim curve not found." << std::endl;
			break;
 		}*/
		MFnAnimCurve animCurve(animCurvePlug.node());
		// Get left and right key frame
		MItKeyframe keyFrameIterator(animCurvePlug.node());
		int leftKeyFrameNumber = -1;
		int rightKeyFrameNumber = -1;
		int leftKeyIndex = -1;
		while (keyFrameIterator.time().value() < targetState.frameNumber)
		{
			leftKeyIndex++;
			leftKeyFrameNumber = keyFrameIterator.time().value();
			keyFrameIterator.next();
		}
		rightKeyFrameNumber = keyFrameIterator.time().value();
		std::cout << "left and right keyframe numbers: " << leftKeyFrameNumber << " " << rightKeyFrameNumber << std::endl;

		MAngle leftIn, leftOut, rightIn, rightOut;
		double wtLeftIn, wtLeftOut, wtRightIn, wtRightOut;
		animCurve.getTangent(leftKeyIndex, leftIn, wtLeftIn, true);
		animCurve.getTangent(leftKeyIndex, leftOut, wtLeftOut, true);
		animCurve.getTangent(leftKeyIndex + 1, rightIn, wtRightIn, true);
		animCurve.getTangent(leftKeyIndex + 1, rightOut, wtRightOut, true);

		double leftInX = cos(leftIn.asRadians());
		double leftInY = sin(leftIn.asRadians());
		double leftOutX = cos(leftOut.asRadians());
		double leftOutY = sin(leftOut.asRadians());

		double rightInX = cos(rightIn.asRadians());
		double rightInY = sin(rightIn.asRadians());
		double rightOutX = cos(rightOut.asRadians());
		double rightOutY = sin(rightOut.asRadians());



	}
	
	return MS::kSuccess;
}