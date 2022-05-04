#define MNoVersionString
#define MNoPluginEntry

#include "TangoNode.h"

MTypeId TangoNode::id(0x0);
//MObject TangoNode::translate[] = {};
MObject TangoNode::translate0;
MObject TangoNode::translate1;
MObject TangoNode::translate2;
MObject TangoNode::translate3;
MObject TangoNode::translate4;
MObject TangoNode::translate5;
MObject TangoNode::translate6;
MObject TangoNode::translate7;
MObject TangoNode::translate8;
MObject TangoNode::translate9;
MObject TangoNode::outputGeometry;

const MString TangoNode::curves[] = { "translateX", "translateY", "translateZ", "rotateX", "rotateY", "rotateZ" };
const int TangoNode::componentsOfCurves[] = { 0, 1, 2, 0, 1, 2 };
const CurveType TangoNode::typesOfCurves[] = { TRANSLATION, TRANSLATION, TRANSLATION, ROTATION, ROTATION, ROTATION };

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
	TangoNode::outputGeometry = outputGeomAttr.create("outputMesh", "out", MFnData::kMesh, &returnStatus);
	McheckErr(returnStatus, "ERROR creating TangoNode output attribute\n");
	returnStatus = addAttribute(TangoNode::outputGeometry);
	McheckErr(returnStatus, "ERROR adding outputMesh attribute\n");

	TangoNode::translate0 = translateAttr.create("translate0", "tr0", MFnNumericData::k3Float, 0.0, &returnStatus);
	MAKE_INPUT(translateAttr);
	McheckErr(returnStatus, "ERROR creating TangoNode time attribute\n");
	returnStatus = addAttribute(TangoNode::translate0);
	McheckErr(returnStatus, "ERROR adding translate attribute\n");
	returnStatus = attributeAffects(TangoNode::translate0, TangoNode::outputGeometry);
	McheckErr(returnStatus, "ERROR in time attributeAffects outputMesh\n");

	TangoNode::translate1 = translateAttr.create("translate1", "tr1", MFnNumericData::k3Float, 0.0, &returnStatus);
	MAKE_INPUT(translateAttr);
	McheckErr(returnStatus, "ERROR creating TangoNode time attribute\n");
	returnStatus = addAttribute(TangoNode::translate1);
	McheckErr(returnStatus, "ERROR adding translate attribute\n");
	returnStatus = attributeAffects(TangoNode::translate1, TangoNode::outputGeometry);
	McheckErr(returnStatus, "ERROR in time attributeAffects outputMesh\n");

	TangoNode::translate2 = translateAttr.create("translate2", "tr2", MFnNumericData::k3Float, 0.0, &returnStatus);
	MAKE_INPUT(translateAttr);
	McheckErr(returnStatus, "ERROR creating TangoNode time attribute\n");
	returnStatus = addAttribute(TangoNode::translate2);
	McheckErr(returnStatus, "ERROR adding translate attribute\n");
	returnStatus = attributeAffects(TangoNode::translate2, TangoNode::outputGeometry);
	McheckErr(returnStatus, "ERROR in time attributeAffects outputMesh\n");

	TangoNode::translate3 = translateAttr.create("translate3", "tr3", MFnNumericData::k3Float, 0.0, &returnStatus);
	MAKE_INPUT(translateAttr);
	McheckErr(returnStatus, "ERROR creating TangoNode time attribute\n");
	returnStatus = addAttribute(TangoNode::translate3);
	McheckErr(returnStatus, "ERROR adding translate attribute\n");
	returnStatus = attributeAffects(TangoNode::translate3, TangoNode::outputGeometry);
	McheckErr(returnStatus, "ERROR in time attributeAffects outputMesh\n");

	TangoNode::translate4 = translateAttr.create("translate4", "tr4", MFnNumericData::k3Float, 0.0, &returnStatus);
	MAKE_INPUT(translateAttr);
	McheckErr(returnStatus, "ERROR creating TangoNode time attribute\n");
	returnStatus = addAttribute(TangoNode::translate4);
	McheckErr(returnStatus, "ERROR adding translate attribute\n");
	returnStatus = attributeAffects(TangoNode::translate4, TangoNode::outputGeometry);
	McheckErr(returnStatus, "ERROR in time attributeAffects outputMesh\n");

	TangoNode::translate5 = translateAttr.create("translate5", "tr5", MFnNumericData::k3Float, 0.0, &returnStatus);
	MAKE_INPUT(translateAttr);
	McheckErr(returnStatus, "ERROR creating TangoNode time attribute\n");
	returnStatus = addAttribute(TangoNode::translate5);
	McheckErr(returnStatus, "ERROR adding translate attribute\n");
	returnStatus = attributeAffects(TangoNode::translate5, TangoNode::outputGeometry);
	McheckErr(returnStatus, "ERROR in time attributeAffects outputMesh\n");

	TangoNode::translate6 = translateAttr.create("translate6", "tr6", MFnNumericData::k3Float, 0.0, &returnStatus);
	MAKE_INPUT(translateAttr);
	McheckErr(returnStatus, "ERROR creating TangoNode time attribute\n");
	returnStatus = addAttribute(TangoNode::translate6);
	McheckErr(returnStatus, "ERROR adding translate attribute\n");
	returnStatus = attributeAffects(TangoNode::translate6, TangoNode::outputGeometry);
	McheckErr(returnStatus, "ERROR in time attributeAffects outputMesh\n");

	TangoNode::translate7 = translateAttr.create("translate7", "tr7", MFnNumericData::k3Float, 0.0, &returnStatus);
	MAKE_INPUT(translateAttr);
	McheckErr(returnStatus, "ERROR creating TangoNode time attribute\n");
	returnStatus = addAttribute(TangoNode::translate7);
	McheckErr(returnStatus, "ERROR adding translate attribute\n");
	returnStatus = attributeAffects(TangoNode::translate7, TangoNode::outputGeometry);
	McheckErr(returnStatus, "ERROR in time attributeAffects outputMesh\n");

	TangoNode::translate8 = translateAttr.create("translate8", "tr8", MFnNumericData::k3Float, 0.0, &returnStatus);
	MAKE_INPUT(translateAttr);
	McheckErr(returnStatus, "ERROR creating TangoNode time attribute\n");
	returnStatus = addAttribute(TangoNode::translate8);
	McheckErr(returnStatus, "ERROR adding translate attribute\n");
	returnStatus = attributeAffects(TangoNode::translate8, TangoNode::outputGeometry);
	McheckErr(returnStatus, "ERROR in time attributeAffects outputMesh\n");

	TangoNode::translate9 = translateAttr.create("translate9", "tr9", MFnNumericData::k3Float, 0.0, &returnStatus);
	MAKE_INPUT(translateAttr);
	McheckErr(returnStatus, "ERROR creating TangoNode time attribute\n");
	returnStatus = addAttribute(TangoNode::translate9);
	McheckErr(returnStatus, "ERROR adding translate attribute\n");
	returnStatus = attributeAffects(TangoNode::translate9, TangoNode::outputGeometry);
	McheckErr(returnStatus, "ERROR in time attributeAffects outputMesh\n");

	return MS::kSuccess;
}

void* TangoNode::creator()
{
	return new TangoNode;
}


MStatus TangoNode::splitTransformName(MString name, MString& effectorName, int& frameNumber) {
	char* underscore = "_";
	MStringArray splittedString;
	name.split(*underscore, splittedString);
	if (splittedString.length() == 0) {
		std::cout << "No selected locator" << std::endl;
		return MS::kFailure;
	}
	for (int i = 1; i < splittedString.length() - 1; i++) {
		if (i != 1) {
			effectorName += "_";
		}
		effectorName += splittedString[i];
	}
	frameNumber = splittedString[splittedString.length() - 1].asInt();
	return MS::kSuccess;
}

MStatus TangoNode::getTargetParams(MString& effectorName, vec3& targetPoint, int& frameNumber) {
	MDagPath nodePath;
	MSelectionList locatorList;
	MGlobal::getActiveSelectionList(locatorList);
	if (locatorList.length() <= 0)
	{
		std::cout << "No selected locator" << std::endl;
		return MS::kFailure;
	}
	locatorList.getDagPath(0, nodePath);
	MFnTransform transformFn(nodePath);

	if (splitTransformName(transformFn.name(), effectorName, frameNumber) == MS::kFailure) {
		return MS::kFailure;
	}

	// Set target translation and framenumber
	MVector targetPos = transformFn.getTranslation(MSpace::kWorld);
	targetPoint = vec3(targetPos[0], targetPos[1], targetPos[2]);
	return MS::kSuccess;
}

void TangoNode::getKeyFrames(const int frameNumber, const MPlug animCurvePlug, double(&leftKey)[4], double(&rightKey)[4]) {
	MItKeyframe keyFrameIterator(animCurvePlug.node());
	int leftKeyIndex = -1;
	while (keyFrameIterator.time().value() < frameNumber)
	{
		leftKeyIndex++;
		leftKey[KNUM] = keyFrameIterator.time().value();
		leftKey[KTIME] = keyFrameIterator.time().asUnits(MTime::kSeconds);
		leftKey[KVAL] = keyFrameIterator.value();
		keyFrameIterator.next();
	}
	rightKey[KNUM] = keyFrameIterator.time().value();
	rightKey[KTIME] = keyFrameIterator.time().asUnits(MTime::kSeconds);
	rightKey[KVAL] = keyFrameIterator.value();
	leftKey[KIDX] = leftKeyIndex;
	rightKey[KIDX] = leftKeyIndex + 1;
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

	// Create Target State
	MString effectorName = "";
	int frameNumber = -1;
	vec3 targetPoint;
	if (getTargetParams(effectorName, targetPoint, frameNumber) == MS::kFailure) {
		return MS::kSuccess;
	}
	State targetState(frameNumber, targetPoint, false);
	MGlobal::displayInfo("Target State set");

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
	
	std::string effectorNameStr = effectorName.asChar();
	MGlobal::displayInfo("effector.Name(): " + effector.name());
	
	// Calculate affacted curve segments by state
	int numberOfKeys;
	int j = 0;
	for (auto& curveName : curves)
	{
		// Find connection with that curve name
		MPlug animCurvePlug;
		int len = connections.length();
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

		std::string animCurveName = animCurve.name().asChar();
		// Get left and right Keyframes and Tangents
		double leftKey[4];
		double rightKey[4];
		getKeyFrames(frameNumber, animCurvePlug, leftKey, rightKey);
		animCurve.setTangentsLocked(leftKey[KIDX], false);
		animCurve.setTangentsLocked(rightKey[KIDX], false);
		animCurve.setIsWeighted(true);
		animCurve.setWeightsLocked(leftKey[KIDX], false);
		animCurve.setWeightsLocked(rightKey[KIDX], false);
		Tangent leftTangentIn;
		Tangent leftTangentOut;
		Tangent rightTangentIn;
		Tangent rightTangentOut;
		animCurve.getTangent(leftKey[KIDX], leftTangentIn.x, leftTangentIn.y, true);
		animCurve.getTangent(leftKey[KIDX], leftTangentOut.x, leftTangentOut.y, false);
		animCurve.getTangent(rightKey[KIDX], rightTangentIn.x, rightTangentIn.y, true);
		animCurve.getTangent(rightKey[KIDX], rightTangentOut.x, rightTangentOut.y, false);
		// Insert values into mSolver
		CurveType type = typesOfCurves[j];
		int component = componentsOfCurves[j];
		std::string idLeft = ASolver::getKey(type, component, leftKey[KNUM]);
		std::string idRight = ASolver::getKey(type, component, rightKey[KNUM]);
		if (mSolver.keyFrames.find(idLeft) == mSolver.keyFrames.end())
		{
			mSolver.keyFrames.insert({ idLeft, std::make_unique<KeyFrame>(idLeft, leftKey[KVAL], leftKey[KNUM], leftKey[KTIME],
																		leftKey[KIDX], leftTangentIn, leftTangentOut) });
		}
		if (mSolver.keyFrames.find(idRight) == mSolver.keyFrames.end())
		{
			mSolver.keyFrames.insert({ idRight, std::make_unique<KeyFrame>(idRight, rightKey[KVAL], rightKey[KNUM], rightKey[KTIME],
																		rightKey[KIDX], rightTangentIn, rightTangentOut) });
		}
		if (mSolver.curveSegments.find(idLeft) == mSolver.curveSegments.end())
		{
			mSolver.curveSegments.insert({ idLeft, std::make_unique<CurveSegment>(idLeft, type, component, effectorNameStr,
																					mSolver.keyFrames[idLeft].get(), mSolver.keyFrames[idRight].get(), 
																						animCurveName) });
		}
		targetState.orderedAffectedCurveSegments.push_back(mSolver.curveSegments[idLeft].get());
		numberOfKeys = animCurve.numKeys();
		j++;
	}
	std::vector<CurveSegment*> C;
	mSolver.solve(targetState, C, numberOfKeys);
	MGlobal::displayInfo("Solver called");
	for (const auto& curveSegment : C)
	{
		MDagPath curveSegmentPath;
		MSelectionList curveFinderList;
		MString testMSTRING(curveSegment->mfnAnimCurveName.c_str());
		curveFinderList.add(MString(testMSTRING));
		if (curveFinderList.length() < 0)
		{
			std::cout << "Curve not found" << std::endl;
			return MS::kSuccess;
		}
		MObject objectCurve;
		curveFinderList.getDependNode(0, objectCurve);
		bool test = objectCurve.hasFn(MFn::kAnimCurve);
		int num = curveFinderList.length();
		std::string beforSTRingtest = objectCurve.apiTypeStr();
		MFnAnimCurve animCurve(objectCurve);
		std::string curveTestName = animCurve.name().asChar();
		animCurve.setTangent(curveSegment->keyLeft->keyFrameNumber,
			curveSegment->keyLeft->tangentPlus.x,
			curveSegment->keyLeft->tangentPlus.y,
			false);
		animCurve.setTangent(curveSegment->keyRight->keyFrameNumber,
			curveSegment->keyRight->tangentMinus.x,
			curveSegment->keyRight->tangentMinus.y,
			true);
	}
	MGlobal::displayInfo("Tangents updated.");
	
	//MGlobal::executeCommand("generateEditCurveTest();");
	return MS::kSuccess;
}

