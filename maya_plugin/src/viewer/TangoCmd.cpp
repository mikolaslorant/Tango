#include "TangoCmd.h"
#include <maya/MGlobal.h>
#include <list>
TangoCmd::TangoCmd() : MPxCommand()
{
}

TangoCmd::~TangoCmd()
{
}

MStatus TangoCmd::doIt(const MArgList& args)
{
	// message in Maya output window
	// cout<<"Implement Me!"<<endl;
	//std::cout.flush();

	// message in scriptor editor
	MGlobal::displayInfo("Implement Me!");	
	return MStatus::kSuccess;
}