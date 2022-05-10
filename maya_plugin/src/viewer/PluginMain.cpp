#include <maya/MPxCommand.h>
#include <maya/MFnPlugin.h>
#include <maya/MIOStream.h>
#include <maya/MString.h>
#include <maya/MArgList.h>
#include <maya/MGlobal.h>
#include <maya/MSimple.h>
#include <maya/MDoubleArray.h>
#include <maya/MPoint.h>
#include <maya/MPointArray.h>
#include <maya/MFnNurbsCurve.h>
#include <maya/MDGModifier.h>
#include <maya/MPlugArray.h>
#include <maya/MVector.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MStringArray.h>
#include <list>


#include "TangoNode.h"
#include "TangoCmd.h"

MStatus initializePlugin(MObject obj)
{
    MStatus   status = MStatus::kSuccess;
    MFnPlugin plugin(obj, "Tango", "1.0", "Any");

    // Register Command
    status = plugin.registerNode(TangoNode::kNODE_NAME, TangoNode::kNODE_ID, TangoNode::creator, TangoNode::initialize);
    if (!status) {
        status.perror("registerNode");
        return status;
    }
    MGlobal::displayInfo("Registerd Node");

    status = plugin.registerCommand(TangoCmd::kCOMMAND_NAME, TangoCmd::creator, TangoCmd::newSyntax);
    if (!status) {
        status.perror("registerCommand");
        return status;
    }
    MGlobal::displayInfo("Registerd Command");

    MGlobal::executeCommand("source \"" + plugin.loadPath() + "/tangoCurve.mel\"");
    
    CHECK_MSTATUS_AND_RETURN_IT(status);
    return status;
}

MStatus uninitializePlugin(MObject obj)
{
    MStatus   status = MStatus::kSuccess;
    MFnPlugin plugin(obj);
    uninstallCallback();
    status = plugin.deregisterCommand("TangoCmd");
    if (!status) {
        status.perror("deregisterCommand");
        return status;
    }
    status = plugin.deregisterNode(TangoNode::kNODE_ID);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    return status;
}