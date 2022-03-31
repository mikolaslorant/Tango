#include "hello_maya.h"
#include <maya/MFnPlugin.h>
// define EXPORT for exporting dll functions
#define EXPORT _declspec(dllexport)
// Maya Plugin creator function
void* helloMaya::creator()
{
	return new helloMaya;
}
// Plugin doIt function
MStatus helloMaya::doIt(const MArgList& argList)
{
	MStatus status;
	MGlobal::displayInfo("Hello World!");
	// <<<your code goes here>>>

	MString ma_name, ma_id;
	argList.get(argList.flagIndex("name") + 1, ma_name);
	argList.get(argList.flagIndex("id") + 1, ma_id);
	MString mel_cmd("confirmDialog -title \"Hello Maya\" -message \"Name: " + ma_name + "\\nID: " + ma_id + "\" - button \"OK\" - dismissString \"No\"; ");
	MGlobal::executeCommand(mel_cmd);
	return status;
}
// Initialize Maya Plugin upon loading
EXPORT MStatus initializePlugin(MObject obj)
{
	MStatus status;
	MFnPlugin plugin(obj, "CIS660", "1.0", "Any");
	status = plugin.registerCommand("helloMaya", helloMaya::creator);
	if (!status)
		status.perror("registerCommand failed");
	return status;
}
// Cleanup Plugin upon unloading
EXPORT MStatus uninitializePlugin(MObject obj)
{
	MStatus status;
	MFnPlugin plugin(obj);
	status = plugin.deregisterCommand("helloMaya");
	if (!status)
		status.perror("deregisterCommand failed");
	return status;
}