#ifndef TANGONODE_H
#define TANGONODE_H
#include <maya/MPxNode.h>
#include <maya/MStatus.h>
#include <maya/MFnPlugin.h>
#include <maya/MTime.h>
#include <maya/MFnMesh.h>
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

class TangoNode : public MPxNode
{
public:
	TangoNode() {};
	virtual ~TangoNode() {};
	static MStatus initialize();
	static void* creator();
	virtual MStatus compute(const MPlug& plug, MDataBlock& data);
	
	static MTypeId id;
	static MObject time;
	static MObject outputGeometry;
};

#endif