#ifndef TANGONODE_H
#define TANGONODE_H
#include <aSolver.h>
#include <maya/MPxNode.h>
#include <maya/MStatus.h>
#include <maya/MFnPlugin.h>
#include <maya/MTime.h>
#include <maya/MFnMesh.h>
#include <maya/MFnTransform.h>
#include <maya/MGlobal.h>
#include <maya/MPoint.h>
#include <maya/MFloatPoint.h>
#include <maya/MFloatPointArray.h>
#include <maya/MIntArray.h>
#include <maya/MDoubleArray.h>
#include <maya/MObject.h>
#include <maya/MPlug.h>
#include <maya/MDataBlock.h>
#include <maya/MFnMeshData.h>
#include <maya/MIOStream.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MDagPath.h>
#include <maya/MFnDagNode.h>
#include <maya/MSelectionList.h>
#include <maya/MFnIkEffector.h>
#include <maya/MFnAnimCurve.h>
#include <maya/MItKeyframe.h>
#include <maya/MFnHikEffector.h>



class TangoNode : public MPxNode
{
public:
	TangoNode() {};
	virtual ~TangoNode() {};
	static MStatus initialize();
	static void* creator();
	virtual MStatus compute(const MPlug& plug, MDataBlock& data);
	
	static MTypeId id;
	//static MObject translate[10];
	static MObject translate0, translate1, translate2, translate3, translate4, translate5, translate6, translate7, translate8, translate9;
	static MObject outputGeometry;

private:

	const enum keyData { KIDX, KNUM, KTIME, KVAL};
	const static MString curves[];
	const static int componentsOfCurves[];
	const static CurveType typesOfCurves[];

	ASolver mSolver;
	State mTargetState;
	MStatus splitTransformName(MString name, MString &effectorName, int &frameNumber);
	MStatus getTargetParams(MString &effectorName, vec3& targetPoint, int &frameNumber);
	void getKeyFrames(const int frameNumber, const MPlug animCurvePlug, double (&leftKey) [4], double (&rightKey) [4]);
};

#endif