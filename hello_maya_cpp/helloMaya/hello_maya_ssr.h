#pragma once
#include <maya/MArgList.h>
#include <maya/MObject.h>
#include <maya/MGlobal.h>
#include <maya/MPxCommand.h>
// custom Maya command
class helloMaya : public MPxCommand
{
public:
	helloMaya() {};
	virtual MStatus doIt(const MArgList& args);
	static void* creator();
	static MSyntax helloMayaSyntax();
};