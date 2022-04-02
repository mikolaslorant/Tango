#include "hello_maya.h"
#include <maya/MFnPlugin.h>
#include <maya/MSyntax.h>
#include <maya/MArgDatabase.h>

// define EXPORT for exporting dll functions
#define EXPORT _declspec(dllexport)
// Maya Plugin creator function
void* helloMaya::creator()
{
	return new helloMaya;
}

//class helloMaya : public MPxCommand
//{
//public:
//	virtual MStatus doIt(const MArgList&);
//	static void* creator() { return new helloMaya; }
//	static MSyntax helloDialog();
//};

const char* nameFlag = "-n", * nameLongFlag = "-name";
const char* idFlag = "-i", * idLongFlag = "-id";

MSyntax helloMaya::helloMayaSyntax()
{
	MSyntax syntax;
	syntax.addFlag(nameFlag, nameLongFlag, MSyntax::kString);
	syntax.addFlag(idFlag, idLongFlag, MSyntax::kDouble);

	return syntax;
}

// Plugin doIt function
MStatus helloMaya::doIt(const MArgList& argList) {
	MStatus status = MS::kSuccess;
	MGlobal::displayInfo("Hello World!");

	// <<<your code goes here>>>

	MString ma_name = "John Doe";
	double ma_id = 0.0;
	MArgDatabase argData(helloMayaSyntax(), argList);

	if (argData.isFlagSet(nameFlag))
	{
		argData.getFlagArgument(nameFlag, 0, ma_name);
	}

	if (argData.isFlagSet(idFlag))
	{
		argData.getFlagArgument(idFlag, 0, ma_id);
	}

	//MString ma_name, ma_id;
	//argList.get(argList.flagIndex("name") + 1, ma_name);
	//argList.get(argList.flagIndex("id") + 1, ma_id);
	//MString mel_cmd("confirmDialog -title \"Hello Maya\" -message \"Name: " + ma_name + "\\nID: " + ma_id + "\" - button \"OK\" - dismissString \"No\"; ");

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
EXPORT MStatus uninitializePlugin(MObject obj) {
	MStatus status;
	MFnPlugin plugin(obj);
	status = plugin.deregisterCommand("helloMaya");
	if (!status)
		status.perror("deregisterCommand failed");
	return status;
}