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
	// Enforce execution with mesh creation. In the future change to jobScript callback
	MStatus returnStatus;
	MFnMeshData mesh_data_creator;
	MObject output_object = mesh_data_creator.create(&returnStatus);
	MDataHandle output_handle = data.outputValue(outputGeometry, &returnStatus);
	output_handle.set(output_object);
	data.setClean(plug);

	// Print locator position
	MDataHandle translate_handle = data.inputValue(translate, &returnStatus);
	MFloatVector translate_value = translate_handle.asFloat3();
	std::string translateXYZ = std::to_string(translate_value[0]) + " " + std::to_string(translate_value[1]) + " " + std::to_string(translate_value[2]);
	MGlobal::displayInfo("Locator Translation: " + MString(translateXYZ.data()));
	
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
	std::string stringEffectorName = effectorName.asChar();
	MString frameNumberString = splittedString[splittedString.length() - 1];
	MGlobal::displayInfo("Target State set");

	// Declare Solver
	ASolver solver;

	// Set target translation and framenumber
	MVector targetPos = transformFn.getTranslation(MSpace::kWorld);
	vec3 targetPoint = vec3(targetPos[0], targetPos[1], targetPos[2]);
	State targetState(frameNumberString.asInt(), targetPoint, false);
	MGlobal::displayInfo("Target translation Frame number set");

	// Get Effector
	MSelectionList effectorList;
	MDagPath effectorPath;
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
	int j = 0;
	MString curves[] = { "translateX", "translateY", "translateZ", "rotateX", "rotateY", "rotateZ"};
	int componentsOfCurves[] = { 0, 1, 2, 0, 1, 2 };
	CurveType typesOfCurves[] = { TRANSLATION, TRANSLATION, TRANSLATION, ROTATION, ROTATION, ROTATION };
	MGlobal::displayInfo("effector.Name(): " + effector.name());
	
	// Calculate affacted curve segments by state
	int numberOfKeys;
	for (auto& curveName : curves)
	{
		// Find connection with that curve name
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
			if ((MString(animCurvePlug.node().apiTypeStr()) == MString("kAnimCurveTimeToDistance") || 
				MString(animCurvePlug.node().apiTypeStr()) == MString("kAnimCurveTimeToAngular")) 
				&& animCurvePlug.name() == effectorName + "_" + curveName + ".output")
			{
				break;
			}
		}
		MFnAnimCurve animCurve(animCurvePlug.node());
		
		// Get left and right key frame
		MItKeyframe keyFrameIterator(animCurvePlug.node());
		int leftKeyFrameNumber = -1;
		int rightKeyFrameNumber = -1;
		double leftKeyFrameValue;
		double rightKeyFrameValue;
		int leftKeyIndex = -1;
		while (keyFrameIterator.time().value() < targetState.frameNumber)
		{
			leftKeyIndex++;
			leftKeyFrameNumber = keyFrameIterator.time().value();
			leftKeyFrameValue = keyFrameIterator.value();
			keyFrameIterator.next();
		}
		rightKeyFrameNumber = keyFrameIterator.time().value();
		rightKeyFrameValue = keyFrameIterator.value();

		MAngle leftIn, leftOut, rightIn, rightOut;
		double wtLeftIn, wtLeftOut, wtRightIn, wtRightOut;
		animCurve.getTangent(leftKeyIndex, leftIn, wtLeftIn, true);
		animCurve.getTangent(leftKeyIndex, leftOut, wtLeftOut, true);
		animCurve.getTangent(leftKeyIndex + 1, rightIn, wtRightIn, true);
		animCurve.getTangent(leftKeyIndex + 1, rightOut, wtRightOut, true);

		// Insert values into solver
		CurveType type = typesOfCurves[j];
		int component = componentsOfCurves[j];
		Tangent leftTangentIn(cos(leftIn.asRadians()), sin(leftIn.asRadians()));
		Tangent leftTangentOut(cos(leftOut.asRadians()), sin(leftOut.asRadians()));
		Tangent rightTangentIn(cos(rightIn.asRadians()), sin(rightIn.asRadians()));
		Tangent rightTangentOut(cos(rightOut.asRadians()), sin(rightOut.asRadians()));
		std::string idLeft = ASolver::getKey(type, component, leftKeyFrameNumber);
		std::string idRight = ASolver::getKey(type, component, rightKeyFrameNumber);
		if (solver.keyFrames.find(idLeft) == solver.keyFrames.end())
		{
			solver.keyFrames.insert({ idLeft, std::make_unique<KeyFrame>(idLeft, leftKeyFrameValue, leftKeyFrameNumber,
																		leftKeyIndex, leftTangentIn, leftTangentOut) });
		}
		if (solver.keyFrames.find(idRight) == solver.keyFrames.end())
		{
			solver.keyFrames.insert({ idRight, std::make_unique<KeyFrame>(idRight, rightKeyFrameValue, rightKeyFrameNumber,
																		leftKeyIndex + 1, rightTangentIn, rightTangentOut) });
		}
		if (solver.curveSegments.find(idLeft) == solver.curveSegments.end())
		{
			solver.curveSegments.insert({ idLeft, std::make_unique<CurveSegment>(idLeft, type, component, stringEffectorName,
																					solver.keyFrames[idLeft].get(), solver.keyFrames[idRight].get()) });
		}
		targetState.orderedAffectedCurveSegments.push_back(solver.curveSegments[idLeft].get());
		numberOfKeys = animCurve.numKeys();
		j++;
	}
	solver.solve(targetState, numberOfKeys);

	
	return MS::kSuccess;
}