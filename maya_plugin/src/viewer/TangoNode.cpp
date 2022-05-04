#define MNoVersionString
#define MNoPluginEntry

#include "TangoNode.h"

const MTypeId TangoNode::kNODE_ID(0x0);
const MString TangoNode::kNODE_NAME = "TangoNode";
const char* TangoNode::kIN_TRANSFORM_ATTR_NAME = "inTransform";
const char* TangoNode::kMSG_CXN_ATTR_NAME = "locatorCallBackAttr";
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
MObject TangoNode::inTransformAttr;

MCallbackIdArray TangoNode::callbacks;
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

	MFnMessageAttribute fnMsgAttr;
	TangoNode::inTransformAttr = fnMsgAttr.create(TangoNode::kIN_TRANSFORM_ATTR_NAME,
												  TangoNode::kIN_TRANSFORM_ATTR_NAME,
												  &returnStatus);
	CHECK_MSTATUS_AND_RETURN_IT(returnStatus);
	addAttribute(TangoNode::inTransformAttr);
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
	MStatus returnStatus;
	return MS::kSuccess;
}


void featureCallback(MNodeMessage::AttributeMessage msg,
	MPlug& plug,
	MPlug& otherPlug,
	void* data)
{
	if (msg != (MNodeMessage::kAttributeSet | MNodeMessage::kIncomingDirection)) {
		return;
	}
	const char* fullplugName = plug.name().asChar();
	if (strstr(fullplugName, "translateX") == NULL && strstr(fullplugName, "translateY") == NULL 
		&& strstr(fullplugName, "translateZ") == NULL) {
		return;
	}

	MStatus status;
	ASolver mSolver;
	State mTargetState;
	// Create Target State
	MString effectorName = "";
	int frameNumber = -1;
	vec3 targetPoint;
	if (TangoNode::getTargetParams(effectorName, targetPoint, frameNumber) == MS::kFailure) {
		return;
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
	for (auto& curveName : TangoNode::curves)
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
		TangoNode::getKeyFrames(frameNumber, animCurvePlug, leftKey, rightKey);
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
		CurveType type = TangoNode::typesOfCurves[j];
		int component = TangoNode::componentsOfCurves[j];
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
			return;
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
	
}

MStatus uninstallCallback()
{
	MStatus status = MMessage::removeCallbacks(TangoNode::callbacks);
	MGlobal::displayInfo("Removed feature!");
	return status;
}


void uninstallCallback(MObject& node, void* data)
{
	uninstallCallback();
}

void installCallback(MNodeMessage::AttributeMessage msg,
	MPlug& plug,
	MPlug& otherPlug,
	void* data)
{
	if (msg == (MNodeMessage::kConnectionBroken |
		MNodeMessage::kIncomingDirection |
		MNodeMessage::kOtherPlugSet)) {
		uninstallCallback();
	}
	if (msg != (MNodeMessage::kConnectionMade |
		MNodeMessage::kIncomingDirection |
		MNodeMessage::kOtherPlugSet)) {
		return;
	}
	// NOTE: (sonictk) We check if the node has its message connection connected
	// first to determine if we should install the real callback onto that node
	MObject callbackNode = plug.node();
	MFnDependencyNode fnNode(callbackNode);
	MPlug cxnPlug = fnNode.findPlug(TangoNode::kIN_TRANSFORM_ATTR_NAME);
	MPlugArray connectedPlugs;
	cxnPlug.connectedTo(connectedPlugs, true, false);
	if (connectedPlugs.length() != 1) {
		return;
	}
	MObject transformNode = connectedPlugs[0].node();
	if (!transformNode.hasFn(MFn::kTransform)) {
		return;
	}
	// NOTE: (sonictk) Install the callback onto the other node and add it to the
	// registry of callbacks to track
	MStatus status;
	MCallbackId featureCallbackId = MNodeMessage::addAttributeChangedCallback(transformNode,
		featureCallback,
		NULL,
		&status);
	if (status != MStatus::kSuccess) {
		return;
	}
	TangoNode::callbacks.append(featureCallbackId);
	MGlobal::displayInfo("Feature installed!");

}

// Registers this callback when this node is created
void TangoNode::postConstructor()
{
	MStatus status;
	MObject thisNode = thisMObject();

	MCallbackId installId = MNodeMessage::addAttributeChangedCallback(thisNode,
		installCallback,
		NULL,
		&status);
	if (status != MStatus::kSuccess) {
		MGlobal::displayError("Unable to install example feature!");
		uninstallCallback();
		return;
	}
	callbacks.append(installId);
	MNodeMessage::addNodePreRemovalCallback(thisNode,
		uninstallCallback,
		NULL,
		&status);
	if (status != MStatus::kSuccess) {
		MGlobal::displayError("Unable to install example feature!");
		uninstallCallback();
		return;
	}
}
